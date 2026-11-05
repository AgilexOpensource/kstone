#include "kstone/model/model.hpp"

#include "kstone/core/context.hpp"

#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <limits>
#include <string>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

ModelPtr load_arm(ModelOptions options = {}) {
    auto m = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), options, Context::defaults());
    REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
    return *m;
}

JointId jid(const ModelPtr& m, const char* name) { return m->joint_id(name).value(); }
}  // namespace

TEST_CASE("URDF topology, indices and masses") {
    const auto m = load_arm();
    CHECK(m->name() == "test_arm");
    CHECK(m->root() == RootJoint::fixed);
    // j1 j2 j3(cos,sin) j4 finger_left finger_right
    CHECK(m->nq() == 7);
    CHECK(m->nv() == 6);
    REQUIRE(m->joints().size() == 7);
    CHECK(m->joints()[0].type == JointType::universe);
    CHECK(*m->joint_id("universe") == 0);

    const auto& j3 = m->joints()[jid(m, "j3")];
    CHECK(j3.type == JointType::continuous);
    CHECK(j3.nq == 2);
    CHECK(j3.nv == 1);
    CHECK(m->joints()[jid(m, "j1")].type == JointType::revolute);
    CHECK(m->joints()[jid(m, "j4")].type == JointType::prismatic);
    CHECK(m->joint_id("nope").error().code == ErrorCode::not_found);

    REQUIRE(m->velocity_owner().size() == 6);
    CHECK(m->velocity_owner()[2] == jid(m, "j3"));

    CHECK(m->total_mass() == doctest::Approx(4.8));
    CHECK(m->subtree_mass(jid(m, "j2")) == doctest::Approx(1.8));
    CHECK(m->subtree_mass(0) == doctest::Approx(4.8));

    auto tool = m->frame_id("tool");
    REQUIRE(tool);
    CHECK(m->frames()[*tool].parent_joint == jid(m, "j4"));
    CHECK(m->frame_in_subtree(*tool, jid(m, "j2")));
    CHECK_FALSE(m->frame_in_subtree(*tool, jid(m, "finger_left_joint")));

    CHECK(m->is_ancestor(jid(m, "j1"), jid(m, "j4")));
    CHECK(m->is_ancestor(jid(m, "j4"), jid(m, "j4")));
    CHECK_FALSE(m->is_ancestor(jid(m, "j4"), jid(m, "j1")));
    auto chain = m->joint_chain(jid(m, "j1"), jid(m, "j4"));
    REQUIRE(chain);
    CHECK(*chain == std::vector<JointId>{jid(m, "j2"), jid(m, "j3"), jid(m, "j4")});
    CHECK_FALSE(m->joint_chain(jid(m, "j4"), jid(m, "j1")));

    const Mask sub = m->subtree_velocity_mask(jid(m, "j4"));
    CHECK(sub.indices() == std::vector<std::size_t>{3, 4, 5});
    CHECK(m->velocity_mask({jid(m, "j1"), jid(m, "j3")}).indices() == std::vector<std::size_t>{0, 2});
}

TEST_CASE("URDF limits") {
    const auto m = load_arm();
    CHECK(m->has_position_limit(jid(m, "j1")));
    CHECK_FALSE(m->has_position_limit(jid(m, "j3")));
    CHECK(m->upper_position_limit()[0] == doctest::Approx(2.0));
    CHECK(m->lower_position_limit()[1] == doctest::Approx(-1.5));
    CHECK(m->velocity_limit()[1] == doctest::Approx(1.2));
    CHECK(m->effort_limit()[3] == doctest::Approx(50.0));

    Eigen::VectorXd q = m->neutral();
    CHECK(m->check_position_limits(q));
    q[0] = 2.5;
    CHECK(m->check_position_limits(q).error().code == ErrorCode::out_of_range);
}

TEST_CASE("with_limits replaces position and velocity limits") {
    const auto m = load_arm();
    auto n = m->with_limits({{"j1", -1.0, 0.5, 0.8}, {"j3", {}, {}, 1.0}, {"j4", {}, 0.05, {}}});
    REQUIRE_MESSAGE(n, (n ? "" : n.error().message));
    const auto& t = *n;
    CHECK(t->lower_position_limit()[0] == doctest::Approx(-1.0));
    CHECK(t->upper_position_limit()[0] == doctest::Approx(0.5));
    CHECK(t->velocity_limit()[0] == doctest::Approx(0.8));
    CHECK(t->velocity_limit()[2] == doctest::Approx(1.0));
    CHECK(t->upper_position_limit()[4] == doctest::Approx(0.05));
    CHECK(t->lower_position_limit()[4] == doctest::Approx(0.0));
    CHECK(t->velocity_limit()[1] == doctest::Approx(1.2));
    CHECK(t->source().location == m->source().location);
    // The source model is untouched.
    CHECK(m->upper_position_limit()[0] == doctest::Approx(2.0));
    CHECK(m->velocity_limit()[0] == doctest::Approx(1.5));

    Eigen::VectorXd q = t->neutral();
    q[0] = 0.7;
    CHECK(t->check_position_limits(q).error().code == ErrorCode::out_of_range);

    auto fails = [&](const ModelPtr& model, JointLimitOverride o, const char* what) {
        auto r = model->with_limits({std::move(o)});
        REQUIRE_FALSE(r);
        CHECK(r.error().code == ErrorCode::invalid_argument);
        CHECK_MESSAGE(r.error().message.find(what) != std::string::npos, r.error().message);
    };
    fails(m, {"nope", {}, {}, 1.0}, "unknown joint");
    fails(m, {"j1", 0.5, -0.5, {}}, "empty");
    fails(m, {"j1", 2.5, {}, {}}, "empty");
    fails(m, {"j1", {}, std::numeric_limits<double>::infinity(), {}}, "not finite");
    fails(m, {"j1", {}, {}, 0.0}, "> 0");
    fails(m, {"j3", -1.0, 1.0, {}}, "revolute/prismatic");
    fails(m, {"tool_joint", {}, {}, 1.0}, "unknown joint");
    CHECK_FALSE(m->with_limits({{"j1", {}, {}, 1.0}, {"j1", {}, {}, 0.5}}));

    ModelOptions opts;
    opts.mimic = MimicMode::coupled;
    fails(load_arm(opts), {"finger_right_joint", {}, {}, 0.1}, "coupled mimic");
    CHECK(m->with_limits({{"finger_right_joint", {}, {}, 0.1}}));

    auto wide = m->with_limits({{"j1", -3.0, 3.0, 4.0}});
    REQUIRE(wide);
    CHECK((*wide)->lower_position_limit()[0] == doctest::Approx(-3.0));
    CHECK((*wide)->upper_position_limit()[0] == doctest::Approx(3.0));
    CHECK((*wide)->velocity_limit()[0] == doctest::Approx(4.0));
}

TEST_CASE("configuration space operations") {
    const auto m = load_arm();
    Eigen::VectorXd q = m->neutral();
    CHECK(m->validate(q));
    CHECK(m->validate(Eigen::VectorXd::Zero(3)).error().code == ErrorCode::invalid_argument);

    REQUIRE(m->set_joint_position(q, jid(m, "j3"), 0.7));
    CHECK(*m->joint_position(q, jid(m, "j3")) == doctest::Approx(0.7));
    REQUIRE(m->set_joint_position(q, jid(m, "j1"), -0.4));
    CHECK(*m->joint_position(q, jid(m, "j1")) == doctest::Approx(-0.4));
    CHECK_FALSE(m->joint_position(q, 0));

    Eigen::VectorXd bad = q;
    bad[2] = 3.0;  // breaks cos^2 + sin^2 = 1 of j3
    CHECK_FALSE(m->validate(bad));

    Eigen::VectorXd v(6);
    v << 0.1, -0.2, 0.3, 0.01, 0.002, -0.001;
    Eigen::VectorXd q1(m->nq()), dv(m->nv());
    REQUIRE(m->integrate(q, v, q1));
    REQUIRE(m->difference(q, q1, dv));
    CHECK((dv - v).cwiseAbs().maxCoeff() < 1e-12);
    CHECK(*m->joint_position(q1, jid(m, "j3")) == doctest::Approx(1.0));

    Eigen::VectorXd wrong(3);
    CHECK(m->integrate(q, v, wrong).error().code == ErrorCode::invalid_argument);
    CHECK(m->root_pose(q).value().matrix().isIdentity());
    CHECK(m->set_root_pose(q, Eigen::Isometry3d::Identity()).error().code == ErrorCode::unsupported);
}

TEST_CASE("NameMap reorders scalar joints") {
    const auto m = load_arm();
    auto map = NameMap::create(m, {"j3", "j1"});
    REQUIRE(map);
    Eigen::VectorXd q = m->neutral();
    Eigen::Vector2d ext(0.5, -0.25);
    REQUIRE(map->scatter_positions(ext, q));
    CHECK(*m->joint_position(q, jid(m, "j3")) == doctest::Approx(0.5));
    CHECK(q[0] == doctest::Approx(-0.25));
    Eigen::Vector2d back;
    REQUIRE(map->gather_positions(q, back));
    CHECK((back - ext).cwiseAbs().maxCoeff() < 1e-12);

    Eigen::VectorXd v = Eigen::VectorXd::Zero(6);
    REQUIRE(map->scatter_velocities(Eigen::Vector2d(1.0, 2.0), v));
    CHECK(v[2] == 1.0);
    CHECK(v[0] == 2.0);
    CHECK(map->scatter_positions(Eigen::Vector3d::Zero(), q).error().code == ErrorCode::invalid_argument);

    CHECK(NameMap::create(m, {"j1", "j1"}).error().code == ErrorCode::invalid_argument);
    CHECK(NameMap::create(m, {"ghost"}).error().code == ErrorCode::not_found);
    CHECK(NameMap::create(m, {"universe"}).error().code == ErrorCode::unsupported);
}

TEST_CASE("mimic relations") {
    const auto indep = load_arm();
    REQUIRE(indep->mimics().size() == 1);
    CHECK(indep->mimics()[0].follower == "finger_right_joint");
    CHECK(indep->mimics()[0].leader == "finger_left_joint");
    CHECK(indep->mimics()[0].multiplier == -1.0);
    CHECK(indep->joints()[jid(indep, "finger_right_joint")].type == JointType::prismatic);

    ModelOptions opts;
    opts.mimic = MimicMode::coupled;
    const auto coupled = load_arm(opts);
    CHECK(coupled->nv() == 5);
    CHECK(coupled->mimics().size() == 1);
    const auto& follower = coupled->joints()[jid(coupled, "finger_right_joint")];
    CHECK(follower.type == JointType::mimic);
    CHECK(follower.nv == 0);
}

// urdfdom orders sibling joints by name, so "a_follower" is visited before its leader "b_leader".
TEST_CASE("inverted URDF position limits are rejected") {
    const std::string urdf = R"(<robot name="inv"><link name="a"/><link name="b"/>
  <joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/>
    <limit lower="1" upper="-1" effort="1" velocity="1"/></joint></robot>)";
    auto m = Model::load(ModelSource::text(urdf, "urdf"), {}, Context::defaults());
    REQUIRE_FALSE(m);
    CHECK(m.error().message.find("joint j") != std::string::npos);
}

TEST_CASE("mimic follower before its leader") {
    const std::string urdf = R"(<robot name="order">
  <link name="base"/><link name="la"/><link name="lb"/>
  <joint name="a_follower" type="revolute"><parent link="base"/><child link="la"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/><mimic joint="b_leader" multiplier="2" offset="0.1"/></joint>
  <joint name="b_leader" type="revolute"><parent link="base"/><child link="lb"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>)";
    const Context ctx = Context::defaults();
    for (MimicMode mode : {MimicMode::independent, MimicMode::coupled}) {
        CAPTURE(static_cast<int>(mode));
        ModelOptions opts;
        opts.mimic = mode;
        auto m = Model::load(ModelSource::text(urdf, "urdf"), opts, ctx);
        REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
        REQUIRE((*m)->mimics().size() == 1);
        CHECK((*m)->mimics()[0].leader == "b_leader");
        CHECK((*m)->nv() == (mode == MimicMode::coupled ? 1 : 2));
        if (mode == MimicMode::coupled) {
            Eigen::VectorXd q = (*m)->neutral();
            REQUIRE(q.size() == (*m)->nq());
            CHECK((*m)->joints()[jid(*m, "a_follower")].type == JointType::mimic);
        }
    }
}

TEST_CASE("root joints") {
    ModelOptions unnamed;
    unnamed.root = RootJoint::free_flyer;
    CHECK(load_arm(unnamed)->joints()[1].name == "root_joint");

    ModelOptions free;
    free.root = RootJoint::free_flyer;
    free.root_joint_name = "base";
    const auto f = load_arm(free);
    CHECK(f->root() == RootJoint::free_flyer);
    CHECK(f->nv() == 12);
    CHECK(f->joint_id("base"));
    Eigen::VectorXd q = f->neutral();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translate(Eigen::Vector3d(0.1, 0.2, 0.3));
    pose.rotate(Eigen::AngleAxisd(1.1, Eigen::Vector3d(1, 2, 3).normalized()));
    REQUIRE(f->set_root_pose(q, pose));
    CHECK(f->validate(q));
    CHECK(f->root_pose(q).value().isApprox(pose));
    CHECK(f->subtree_velocity_mask(jid(f, "base")).count() == 12);

    Eigen::VectorXd drift = q;
    drift.segment<4>(3) *= 1.0 + 1e-6;
    const auto invalid = f->validate(drift);
    REQUIRE_FALSE(invalid);
    CHECK(invalid.error().message.find("joint base has a quaternion of norm 1.000001 at q[3]") !=
          std::string::npos);
    REQUIRE(f->normalize(drift));
    CHECK(f->validate(drift));
    CHECK(drift.isApprox(q));
    Eigen::VectorXd zero = q;
    zero.segment<4>(3).setZero();
    CHECK(f->normalize(zero).error().code == ErrorCode::invalid_argument);
    CHECK(zero.segment<4>(3).isZero());
    CHECK_FALSE(f->normalize(drift, -1.0));
}

TEST_CASE("load from text, MJCF and error paths") {
    const Context ctx = Context::defaults();
    auto text = read_text_file(kFixtures / "arm.urdf");
    REQUIRE(text);
    auto m = Model::load(ModelSource::text(*text, "urdf"), {}, ctx);
    REQUIRE(m);
    CHECK((*m)->nq() == 7);

    CHECK(Model::load(ModelSource::text(*text, ""), {}, ctx).error().code == ErrorCode::invalid_argument);
    CHECK(Model::load(ModelSource::text(*text, "sdf"), {}, ctx).error().code == ErrorCode::unsupported);
    CHECK(Model::load(ModelSource::text("<robot", "urdf"), {}, ctx).error().code == ErrorCode::parse_error);
    CHECK(Model::load(ModelSource::file("missing.urdf"), {}, ctx).error().code == ErrorCode::not_found);

    auto mj = Model::load(ModelSource::file((kFixtures / "pendulum.xml").string()), {}, ctx);
    REQUIRE_MESSAGE(mj, (mj ? "" : mj.error().message));
    CHECK((*mj)->nq() == 2);
    CHECK((*mj)->total_mass() == doctest::Approx(1.5));
    CHECK((*mj)->has_position_limit(jid(*mj, "hinge1")));
    CHECK((*mj)->upper_position_limit()[0] == doctest::Approx(1.0));

    auto mj_text = read_text_file(kFixtures / "pendulum.xml");
    REQUIRE(mj_text);
    ModelOptions free;
    free.root = RootJoint::free_flyer;
    // The top body already owns a joint, so the parser cannot float it: reported, not silently ignored.
    CHECK(Model::load(ModelSource::text(*mj_text, "mjcf"), free, ctx).error().code == ErrorCode::unsupported);

    const std::string floating = R"(<mujoco model="floating"><worldbody>
      <body name="base"><inertial pos="0 0 0" mass="2" diaginertia="0.1 0.1 0.1"/>
        <body name="link1"><joint name="hinge1" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 -0.25" mass="1" diaginertia="0.01 0.01 0.001"/></body></body>
    </worldbody></mujoco>)";
    auto mj2 = Model::load(ModelSource::text(floating, "mjcf"), free, ctx);
    REQUIRE_MESSAGE(mj2, (mj2 ? "" : mj2.error().message));
    CHECK((*mj2)->nv() == 7);
    CHECK((*mj2)->root() == RootJoint::free_flyer);
    CHECK((*mj2)->total_mass() == doctest::Approx(3.0));
}
