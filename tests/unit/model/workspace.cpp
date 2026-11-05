#include "kstone/model/workspace.hpp"

#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"

#include <doctest/doctest.h>

#include <filesystem>
#include <functional>
#include <string>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

ModelPtr load(const ModelSource& source, ModelOptions options = {}) {
    auto m = Model::load(source, options, Context::defaults());
    REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
    return *m;
}

ModelPtr load_arm(ModelOptions options = {}) {
    return load(ModelSource::file((kFixtures / "arm.urdf").string()), options);
}

Eigen::VectorXd sample_q(const ModelPtr& m) {
    Eigen::VectorXd v(m->nv());
    for (Index i = 0; i < v.size(); ++i) v[i] = 0.3 - 0.17 * static_cast<double>(i);
    Eigen::VectorXd q(m->nq());
    REQUIRE(m->integrate(m->neutral(), v, q));
    return q;
}

// Central differences of a world-frame pose along each velocity direction: (linear; angular) per column.
Eigen::MatrixXd numeric_pose_jacobian(const ModelPtr& m, const Eigen::VectorXd& q,
                                      const std::function<Eigen::Isometry3d(const Eigen::VectorXd&)>& pose) {
    constexpr double h = 1e-6;
    Eigen::MatrixXd J(6, m->nv());
    Eigen::VectorXd qp(m->nq()), qm(m->nq());
    for (Index k = 0; k < m->nv(); ++k) {
        Eigen::VectorXd dv = Eigen::VectorXd::Zero(m->nv());
        dv[k] = h;
        REQUIRE(m->integrate(q, dv, qp));
        REQUIRE(m->integrate(q, -dv, qm));
        const Eigen::Isometry3d a = pose(qp), b = pose(qm);
        J.block<3, 1>(0, k) = (a.translation() - b.translation()) / (2 * h);
        const Eigen::AngleAxisd aa(a.linear() * b.linear().transpose());
        J.block<3, 1>(3, k) = aa.axis() * aa.angle() / (2 * h);
    }
    return J;
}
}  // namespace

TEST_CASE("workspace requires update and checks shapes") {
    const auto m = load_arm();
    Workspace ws(m);
    CHECK_FALSE(ws.updated());
    Eigen::Isometry3d pose;
    CHECK_FALSE(ws.frame_pose(0, pose));
    CHECK(ws.update(Eigen::VectorXd::Zero(2)).error().code == ErrorCode::invalid_argument);
    REQUIRE(ws.update(m->neutral()));
    CHECK(ws.updated());
    CHECK(ws.frame_pose(m->frames().size(), pose).error().code == ErrorCode::out_of_range);
    Eigen::MatrixXd J(6, 2);
    CHECK(ws.frame_jacobian(0, ReferenceFrame::world, J).error().code == ErrorCode::invalid_argument);
    CHECK_THROWS_AS(Workspace(nullptr), std::invalid_argument);
}

TEST_CASE("model without degrees of freedom has empty Jacobians") {
    const auto m = load(ModelSource::text(R"(<robot name="r"><link name="a"/><link name="b"/>
  <joint name="f" type="fixed"><parent link="a"/><child link="b"/></joint></robot>)",
                                          "urdf"));
    REQUIRE(m->nv() == 0);
    Workspace ws(m);
    REQUIRE(ws.update(Eigen::VectorXd(0)));
    Eigen::MatrixXd J(6, 0);
    for (FrameId f = 0; f < m->frames().size(); ++f) CHECK(ws.frame_jacobian(f, ReferenceFrame::local, J));
}

TEST_CASE("frame pose at neutral matches the URDF chain") {
    const auto m = load_arm();
    Workspace ws(m);
    REQUIRE(ws.update(m->neutral()));
    Eigen::Isometry3d tool;
    REQUIRE(ws.frame_pose(*m->frame_id("tool"), tool));
    // z: 0.1 + 0.2; x: 0.3 + 0.2 + 0.1; y: 0.02; yaw 0.3
    CHECK(tool.translation().isApprox(Eigen::Vector3d(0.6, 0.02, 0.3)));
    CHECK(tool.linear().isApprox(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix()));
    Eigen::Isometry3d j4;
    REQUIRE(ws.joint_pose(*m->joint_id("j4"), j4));
    CHECK(j4.translation().isApprox(Eigen::Vector3d(0.5, 0.0, 0.3)));
}

TEST_CASE("frame Jacobians match finite differences in every reference frame") {
    ModelOptions free;
    free.root = RootJoint::free_flyer;
    for (const auto& m : {load_arm(), load_arm(free)}) {
        Workspace ws(m);
        const FrameId tool = *m->frame_id("tool");
        const auto pose_of = [&](const Eigen::VectorXd& q) {
            Workspace tmp(m);
            REQUIRE(tmp.update(q));
            Eigen::Isometry3d T;
            REQUIRE(tmp.frame_pose(tool, T));
            return T;
        };
        const Eigen::VectorXd q = sample_q(m);
        REQUIRE(ws.update(q));
        Eigen::Isometry3d T;
        REQUIRE(ws.frame_pose(tool, T));

        Eigen::MatrixXd J_lwa(6, m->nv()), J_local(6, m->nv()), J_world(6, m->nv());
        REQUIRE(ws.frame_jacobian(tool, ReferenceFrame::local_world_aligned, J_lwa));
        REQUIRE(ws.frame_jacobian(tool, ReferenceFrame::local, J_local));
        REQUIRE(ws.frame_jacobian(tool, ReferenceFrame::world, J_world));

        const Eigen::MatrixXd J_num = numeric_pose_jacobian(m, q, pose_of);
        CHECK((J_lwa - J_num).cwiseAbs().maxCoeff() < 1e-6);

        const Eigen::Matrix3d Rt = T.linear().transpose();
        CHECK((J_local.topRows<3>() - Rt * J_lwa.topRows<3>()).cwiseAbs().maxCoeff() < 1e-9);
        CHECK((J_local.bottomRows<3>() - Rt * J_lwa.bottomRows<3>()).cwiseAbs().maxCoeff() < 1e-9);
        // World: same angular part; linear part is the velocity of the point at the world origin.
        CHECK((J_world.bottomRows<3>() - J_lwa.bottomRows<3>()).cwiseAbs().maxCoeff() < 1e-9);
        for (Index k = 0; k < m->nv(); ++k) {
            const Eigen::Vector3d w = J_lwa.block<3, 1>(3, k);
            const Eigen::Vector3d expect = J_lwa.block<3, 1>(0, k) - w.cross(T.translation());
            CHECK((J_world.block<3, 1>(0, k) - expect).cwiseAbs().maxCoeff() < 1e-9);
        }
    }
}

TEST_CASE("centre of mass includes base inertia and matches finite differences") {
    // 1 kg base at x = 0.5 and 1 kg link at 1.2 m from a z-axis joint located at x = 1.
    const std::string urdf = R"(<robot name="com">
      <link name="base"><inertial><origin xyz="0.5 0 0"/><mass value="1"/>
        <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
      <link name="arm"><inertial><origin xyz="0.2 0 0"/><mass value="1"/>
        <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
      <joint name="j" type="revolute"><parent link="base"/><child link="arm"/>
        <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="1" effort="1"/></joint>
    </robot>)";
    const auto m = load(ModelSource::text(urdf, "urdf"));
    CHECK(m->total_mass() == doctest::Approx(2.0));
    Workspace ws(m);

    Eigen::VectorXd q = m->neutral();
    REQUIRE(ws.update(q));
    Eigen::Vector3d com;
    REQUIRE(ws.center_of_mass(com));
    CHECK(com.isApprox(Eigen::Vector3d(0.85, 0.0, 0.0)));
    Eigen::MatrixXd Jc(3, 1);
    REQUIRE(ws.center_of_mass_jacobian(Jc));
    CHECK(Jc(1, 0) == doctest::Approx(0.1));  // half the mass, 0.2 m lever

    q[0] = 1.5707963267948966;  // pi / 2
    REQUIRE(ws.update(q));
    REQUIRE(ws.center_of_mass(com));
    CHECK(com.isApprox(Eigen::Vector3d(0.75, 0.1, 0.0)));

    Eigen::Vector3d sub;
    REQUIRE(ws.subtree_center_of_mass(1, sub));
    CHECK(sub.isApprox(Eigen::Vector3d(1.0, 0.2, 0.0)));
    Eigen::MatrixXd Js(3, 1);
    REQUIRE(ws.subtree_center_of_mass_jacobian(1, Js));
    CHECK(Js(0, 0) == doctest::Approx(-0.2));
    REQUIRE(ws.subtree_center_of_mass(0, sub));
    CHECK(sub.isApprox(com));
}

TEST_CASE("centre of mass Jacobian of the arm matches finite differences") {
    const auto m = load_arm();
    Workspace ws(m);
    const Eigen::VectorXd q = sample_q(m);
    REQUIRE(ws.update(q));
    Eigen::MatrixXd Jc(3, m->nv());
    REQUIRE(ws.center_of_mass_jacobian(Jc));

    constexpr double h = 1e-6;
    Workspace tmp(m);
    Eigen::VectorXd qp(m->nq()), qm(m->nq());
    for (Index k = 0; k < m->nv(); ++k) {
        Eigen::VectorXd dv = Eigen::VectorXd::Zero(m->nv());
        dv[k] = h;
        REQUIRE(m->integrate(q, dv, qp));
        REQUIRE(m->integrate(q, -dv, qm));
        Eigen::Vector3d cp, cm;
        REQUIRE(tmp.update(qp));
        REQUIRE(tmp.center_of_mass(cp));
        REQUIRE(tmp.update(qm));
        REQUIRE(tmp.center_of_mass(cm));
        CHECK(((cp - cm) / (2 * h) - Jc.col(k)).cwiseAbs().maxCoeff() < 1e-6);
    }
}

TEST_CASE("coupled mimic followers move with their leader") {
    ModelOptions opts;
    opts.mimic = MimicMode::coupled;
    const auto m = load_arm(opts);
    Eigen::VectorXd q = m->neutral();
    REQUIRE(m->set_joint_position(q, *m->joint_id("finger_left_joint"), 0.02));
    Workspace ws(m);
    REQUIRE(ws.update(q));
    Eigen::Isometry3d tool, left, right;
    REQUIRE(ws.frame_pose(*m->frame_id("tool"), tool));
    REQUIRE(ws.frame_pose(*m->frame_id("finger_left"), left));
    REQUIRE(ws.frame_pose(*m->frame_id("finger_right"), right));
    CHECK((tool.inverse() * left).translation().isApprox(Eigen::Vector3d(0, 0.02, 0)));
    CHECK((tool.inverse() * right).translation().isApprox(Eigen::Vector3d(0, -0.02, 0)));
}
