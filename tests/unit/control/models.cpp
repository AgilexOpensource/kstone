// Coverage on free_flyer-rooted models: omni base, differential chassis (no nonholonomic
// constraint) and a free-floating base.
#include "kstone/control/constraints.hpp"
#include "kstone/control/solver.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"

#include <Eigen/Geometry>
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

struct Case {
    const char* file;
    RootJoint root;
    const char* tool;
    Index nq, nv;
};

const Case kOmni{"omni_dual_arm_gripper.urdf", RootJoint::free_flyer, "left_tool_frame", 41, 36};
const Case kDiff{"diff_chassis.urdf", RootJoint::free_flyer, "tool", 15, 12};
const Case kFloat{"floating_base.urdf", RootJoint::free_flyer, "tool", 11, 10};

ModelPtr load(const Case& c) {
    ModelOptions o;
    o.root = c.root;
    auto m = Model::load(ModelSource::file((kFixtures / c.file).string()), o, Context::defaults());
    REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
    return *m;
}

Solver make_solver(const ModelPtr& m, SolverOptions o = {}) {
    auto s = Solver::create(m, std::move(o));
    REQUIRE_MESSAGE(s, (s ? "" : s.error().message));
    return std::move(*s);
}

Eigen::Isometry3d pose_at(const ModelPtr& m, const Eigen::VectorXd& q, const std::string& frame) {
    Workspace ws(m);
    REQUIRE(ws.update(q));
    auto id = m->frame_id(frame);
    REQUIRE(id);
    Eigen::Isometry3d pose;
    REQUIRE(ws.frame_pose(*id, pose));
    return pose;
}

// A valid configuration away from neutral: random velocity integrated from the middle of the limits.
Eigen::VectorXd random_q(const ModelPtr& m, std::mt19937& rng) {
    Eigen::VectorXd q = m->neutral();
    for (const JointInfo& j : m->joints()) {
        if (j.nq == 1 && j.nv == 1 && m->has_position_limit(j.id)) {
            const double lo = m->lower_position_limit()[j.idx_q];
            const double hi = m->upper_position_limit()[j.idx_q];
            q[j.idx_q] = 0.5 * (lo + hi);
        }
    }
    std::uniform_real_distribution<double> u(-0.3, 0.3);
    Eigen::VectorXd v(m->nv());
    for (Index i = 0; i < m->nv(); ++i) v[i] = u(rng);
    Eigen::VectorXd out(m->nq());
    REQUIRE(m->integrate(q, v, out));
    // Narrow joints (grippers) can overshoot; keep a margin so finite differences stay inside.
    for (const JointInfo& j : m->joints()) {
        if (j.nq == 1 && j.nv == 1 && m->has_position_limit(j.id)) {
            const double lo = m->lower_position_limit()[j.idx_q];
            const double hi = m->upper_position_limit()[j.idx_q];
            const double margin = 0.1 * (hi - lo);
            out[j.idx_q] = std::clamp(out[j.idx_q], lo + margin, hi - margin);
        }
    }
    REQUIRE(m->check_position_limits(out));
    return out;
}

Eigen::Vector3d rotation_delta(const Eigen::Matrix3d& from, const Eigen::Matrix3d& to) {
    const Eigen::AngleAxisd aa(to * from.transpose());
    return aa.angle() * aa.axis();
}

// Posture and damping at level 1, below the frame tasks. Weighted mode ignores levels, so there the
// posture weight decides how much it pulls against the frame tasks.
void add_secondary(Solver& solver, const Eigen::VectorXd& q0, double posture_weight = 1.0) {
    TaskOptions low;
    low.level = 1;
    auto posture = std::make_shared<PostureTask>(q0);
    REQUIRE(posture->set_weight(posture_weight));
    REQUIRE(solver.add_task("posture", posture, low));
}

// Drives `tool` to `target` from q0 and returns the final configuration.
Eigen::VectorXd converge(Solver& solver, Eigen::VectorXd q, int steps) {
    SolveResult r;
    int degraded = 0;  // held, infeasible or failed stages
    int inexact = 0;   // solved, but a settled level kept the previous stage's x (does not fail solve())
    for (int i = 0; i < steps; ++i) {
        const Status s = solver.solve(q, 0.01, r);
        REQUIRE_MESSAGE(s, "step " << i << ": " << (s ? "" : s.error().message));
        for (const auto& st : r.stages) {
            degraded +=
                st.held || (st.status != qp::SolveStatus::optimal && st.status != qp::SolveStatus::inexact);
            inexact += st.status == qp::SolveStatus::inexact;
        }
        q = r.q_next;
    }
    CHECK(degraded == 0);
    CHECK(inexact * 100 <= steps);
    return q;
}

void check_reached(const ModelPtr& m, const Eigen::VectorXd& q, const char* tool,
                   const Eigen::Isometry3d& target) {
    const Eigen::Isometry3d reached = pose_at(m, q, tool);
    CHECK((reached.translation() - target.translation()).norm() < 1e-3);
    CHECK(rotation_delta(reached.linear(), target.linear()).norm() < 1e-2);
    CHECK(m->validate(q));
    CHECK(m->check_position_limits(q));
}

}  // namespace

TEST_CASE("models: dimensions and root types") {
    for (const Case* c : {&kOmni, &kDiff, &kFloat}) {
        INFO(std::string(c->file));
        auto m = load(*c);
        CHECK(m->root() == c->root);
        CHECK(m->nq() == c->nq);
        CHECK(m->nv() == c->nv);
        CHECK(m->frame_id(c->tool));
        CHECK(m->validate(m->neutral()));
    }
    CHECK(load(kOmni)->mimics().size() == 4);
}

TEST_CASE("models: root pose round trip") {
    for (const Case* c : {&kOmni, &kFloat}) {
        INFO(std::string(c->file));
        auto m = load(*c);
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() << 0.7, -0.4, 0.3;
        pose.linear() = Eigen::AngleAxisd(1.1, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
        Eigen::VectorXd q = m->neutral();
        REQUIRE(m->set_root_pose(q, pose));
        CHECK(m->validate(q));
        auto back = m->root_pose(q);
        REQUIRE(back);
        CHECK((back->translation() - pose.translation()).norm() < 1e-12);
        CHECK((back->linear() - pose.linear()).norm() < 1e-12);
    }
}

TEST_CASE("models: frame Jacobian matches finite differences") {
    std::mt19937 rng(7);
    constexpr double eps = 1e-6;
    for (const Case* c : {&kOmni, &kDiff, &kFloat}) {
        INFO(std::string(c->file));
        auto m = load(*c);
        const Eigen::VectorXd q = random_q(m, rng);
        Workspace ws(m);
        REQUIRE(ws.update(q));
        auto id = m->frame_id(c->tool);
        REQUIRE(id);
        Eigen::MatrixXd J(6, m->nv());
        REQUIRE(ws.frame_jacobian(*id, ReferenceFrame::local_world_aligned, J));
        Eigen::Isometry3d p0;
        REQUIRE(ws.frame_pose(*id, p0));
        Eigen::VectorXd qe(m->nq());
        for (Index i = 0; i < m->nv(); ++i) {
            CAPTURE(i);
            const Eigen::VectorXd dv = Eigen::VectorXd::Unit(m->nv(), i) * eps;
            REQUIRE(m->integrate(q, dv, qe));
            const Eigen::Isometry3d p1 = pose_at(m, qe, c->tool);
            const Eigen::Vector3d lin = (p1.translation() - p0.translation()) / eps;
            const Eigen::Vector3d ang = rotation_delta(p0.linear(), p1.linear()) / eps;
            CHECK((lin - J.block<3, 1>(0, i)).norm() < 1e-4);
            CHECK((ang - J.block<3, 1>(3, i)).norm() < 1e-4);
        }
    }
}

TEST_CASE("models: far target moves the base and is reached") {
    for (const Case* c : {&kOmni, &kDiff, &kFloat}) {
        INFO(std::string(c->file));
        auto m = load(*c);
        const Eigen::VectorXd q0 = m->neutral();
        Eigen::Isometry3d target = pose_at(m, q0, c->tool);
        // Beyond the reach of any arm: only the root can cover it.
        target.translation() += Eigen::Vector3d(1.5, -1.0, c->root == RootJoint::free_flyer ? 0.8 : 0.0);
        target.linear() =
            Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()).toRotationMatrix() * target.linear();

        Solver solver = make_solver(m);
        REQUIRE(solver.add_task("tool", std::make_shared<FrameTask>(c->tool, target)));
        add_secondary(solver, q0);
        const Eigen::VectorXd q = converge(solver, q0, 600);
        check_reached(m, q, c->tool, target);
        auto base = m->root_pose(q);
        REQUIRE(base);
        CHECK(base->translation().head<2>().norm() > 1.0);
    }
}

namespace {

// Both tool frames at `arm_level` (right arm one level lower when split), secondary tasks below.
void run_dual_arm(SolveMode mode, bool split_levels) {
    INFO(std::string(to_string(mode)));
    auto m = load(kOmni);
    const Eigen::VectorXd q0 = m->neutral();
    // Targets from forward kinematics of a valid configuration with a displaced base: reachable by construction.
    std::mt19937 rng(11);
    Eigen::VectorXd q_goal = random_q(m, rng);
    Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
    base.translation() << 0.6, 0.2, 0.0;
    base.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    REQUIRE(m->set_root_pose(q_goal, base));
    const Eigen::Isometry3d left = pose_at(m, q_goal, "left_tool_frame");
    const Eigen::Isometry3d right = pose_at(m, q_goal, "right_tool_frame");
    SolverOptions o;
    o.mode = mode;
    Solver solver = make_solver(m, o);
    TaskOptions left_options;
    left_options.level = 0;
    TaskOptions right_options;
    right_options.level = split_levels ? 1 : 0;
    REQUIRE(solver.add_task("left", std::make_shared<FrameTask>("left_tool_frame", left), left_options));
    REQUIRE(solver.add_task("right", std::make_shared<FrameTask>("right_tool_frame", right), right_options));
    // Weighted and layered_coupled sum posture with the frame tasks; keep it a weak preference.
    add_secondary(solver, q0, 1e-3);
    if (split_levels) {
        TaskOptions lowest;
        lowest.level = 2;
        REQUIRE(solver.set_task_options("posture", lowest));
    }
    const Eigen::VectorXd q = converge(solver, q0, 1000);
    check_reached(m, q, "left_tool_frame", left);
    check_reached(m, q, "right_tool_frame", right);
}

}  // namespace

TEST_CASE("models: omni dual arm, weighted and layered modes") {
    for (SolveMode mode : {SolveMode::weighted, SolveMode::layered_independent, SolveMode::layered_coupled})
        run_dual_arm(mode, false);
}

// Lexicographic mode: settled levels of two 6D frame tasks are near-degenerate, which can make DAQP
// report a feasible stage infeasible; the backend retries from a cold working set and the
// lexicographic solve otherwise keeps the last feasible x as inexact (rare, bounded above).
TEST_CASE("models: omni dual arm, lexicographic with one frame task per level") {
    run_dual_arm(SolveMode::lexicographic, true);
}

TEST_CASE("models: omni dual arm, lexicographic with both frame tasks on one level") {
    run_dual_arm(SolveMode::lexicographic, false);
}
