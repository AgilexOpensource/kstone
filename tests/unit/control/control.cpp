#include "kstone/control/constraints.hpp"
#include "kstone/control/solver.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"
#include "kstone/qp/backend.hpp"

#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

ModelPtr load_arm() {
    auto m = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
    return *m;
}

Solver make_solver(const ModelPtr& m, SolverOptions o = {}) {
    auto s = Solver::create(m, std::move(o));
    REQUIRE_MESSAGE(s, (s ? "" : s.error().message));
    return std::move(*s);
}

JointInfo joint(const ModelPtr& m, const std::string& name) {
    auto id = m->joint_id(name);
    REQUIRE(id);
    return m->joints()[*id];
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

std::shared_ptr<JointTask> joint_task(const std::string& name, double target, double gain = 1.0) {
    auto t =
        std::make_shared<JointTask>(std::vector<std::string>{name}, Eigen::VectorXd::Constant(1, target));
    REQUIRE(t->set_gain(gain));
    return t;
}

// Reports success with a NaN step.
class NanBackend final : public qp::Backend {
  public:
    const char* name() const noexcept override { return "nan"; }
    Status solve(const qp::Problem& problem, qp::Solution& out) override {
        out.x.setConstant(problem.variables(), std::numeric_limits<double>::quiet_NaN());
        out.status = qp::SolveStatus::optimal;
        return {};
    }
    void reset() noexcept override {}
};

// Reports a cycling failure on every problem, like DAQP on a degenerate active set.
class CyclingBackend final : public qp::Backend {
  public:
    const char* name() const noexcept override { return "cycling"; }
    Status solve(const qp::Problem& problem, qp::Solution& out) override {
        out.x.setZero(problem.variables());
        out.status = qp::SolveStatus::failed;
        out.exitflag = -2;
        return {};
    }
    void reset() noexcept override {}
};

// Lands just outside the upper box, as a backend within its primal tolerance may.
class OvershootBackend final : public qp::Backend {
  public:
    const char* name() const noexcept override { return "overshoot"; }
    Status solve(const qp::Problem& problem, qp::Solution& out) override {
        out.x.setZero(problem.variables());
        for (Index i = 0; i < problem.variables(); ++i)
            if (std::isfinite(problem.ub[i])) out.x[i] = problem.ub[i] + 1e-6;
        out.status = qp::SolveStatus::optimal;
        out.exitflag = 1;
        return {};
    }
    void reset() noexcept override {}
};

}  // namespace

TEST_CASE("solver: frame task converges to a reachable pose") {
    auto m = load_arm();
    Eigen::VectorXd v = Eigen::VectorXd::Zero(m->nv());
    v[joint(m, "j1").idx_v] = 0.4;
    v[joint(m, "j2").idx_v] = -0.3;
    v[joint(m, "j3").idx_v] = 0.5;
    v[joint(m, "j4").idx_v] = 0.05;
    Eigen::VectorXd goal(m->nq());
    REQUIRE(m->integrate(m->neutral(), v, goal));
    const Eigen::Isometry3d target = pose_at(m, goal, "tool");

    Solver solver = make_solver(m);
    auto task = std::make_shared<FrameTask>("tool", target);
    REQUIRE(solver.add_task("tool", task));
    Eigen::VectorXd q = m->neutral();
    SolveResult r;
    for (int i = 0; i < 400; ++i) {
        REQUIRE(solver.solve(q, 0.01, r));
        q = r.q_next;
    }
    const Eigen::Isometry3d reached = pose_at(m, q, "tool");
    CHECK((reached.translation() - target.translation()).norm() < 1e-4);
    CHECK((reached.linear() - target.linear()).norm() < 1e-3);
    REQUIRE(r.residuals.size() == 1);
    CHECK(r.residuals[0].name == "tool");
}

TEST_CASE("solver: velocity limits from the model and the options") {
    auto m = load_arm();
    const Index j1 = joint(m, "j1").idx_v;
    SolveResult r;
    {
        Solver solver = make_solver(m);
        REQUIRE(solver.add_task("j1", joint_task("j1", 1.9)));
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        CHECK(r.v[j1] == doctest::Approx(1.5).epsilon(1e-6));
    }
    {
        SolverOptions o;
        o.velocity_limit = Eigen::VectorXd::Constant(m->nv(), std::nan(""));
        o.velocity_limit[j1] = 0.5;
        Solver solver = make_solver(m, o);
        REQUIRE(solver.add_task("j1", joint_task("j1", 1.9)));
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        CHECK(r.v[j1] == doctest::Approx(0.5).epsilon(1e-6));
    }
    {
        Solver solver = make_solver(m);
        REQUIRE(solver.add_constraint(
            "vel", std::make_shared<JointVelocityConstraint>(std::vector<std::string>{"j1"},
                                                             Eigen::VectorXd::Constant(1, 0.25))));
        REQUIRE(solver.add_task("j1", joint_task("j1", 1.9)));
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        CHECK(r.v[j1] == doctest::Approx(0.25).epsilon(1e-6));
    }
}

TEST_CASE("solver: position limits are never crossed") {
    auto m = load_arm();
    const JointInfo& j4 = joint(m, "j4");
    Solver solver = make_solver(m);
    REQUIRE(solver.add_task("j4", joint_task("j4", 0.5)));
    Eigen::VectorXd q = m->neutral();
    SolveResult r;
    for (int i = 0; i < 300; ++i) {
        REQUIRE(solver.solve(q, 0.01, r));
        q = r.q_next;
        REQUIRE(q[j4.idx_q] <= 0.1 + 1e-9);
    }
    CHECK(q[j4.idx_q] == doctest::Approx(0.1).epsilon(1e-4));

    SolverOptions o;
    o.upper_position_limit = Eigen::VectorXd::Constant(m->nq(), std::numeric_limits<double>::infinity());
    o.upper_position_limit[j4.idx_q] = 0.05;
    REQUIRE(solver.set_options(o));
    for (int i = 0; i < 300; ++i) {
        REQUIRE(solver.solve(q, 0.01, r));
        q = r.q_next;
    }
    // Starting above the tightened limit: moves back inside, never pulled further out.
    CHECK(q[j4.idx_q] <= 0.05 + 1e-6);
}

TEST_CASE("solver: mimic followers track their leader") {
    auto m = load_arm();
    const JointInfo& left = joint(m, "finger_left_joint");
    const JointInfo& right = joint(m, "finger_right_joint");
    Solver solver = make_solver(m);
    REQUIRE(solver.add_task("grip", joint_task("finger_left_joint", 0.03)));
    Eigen::VectorXd q = m->neutral();
    SolveResult r;
    for (int i = 0; i < 100; ++i) {
        REQUIRE(solver.solve(q, 0.01, r));
        q = r.q_next;
        REQUIRE(q[right.idx_q] == doctest::Approx(-q[left.idx_q]).epsilon(1e-9));
    }
    CHECK(q[left.idx_q] == doctest::Approx(0.03).epsilon(1e-3));
    CHECK(r.v[right.idx_v] == doctest::Approx(-r.v[left.idx_v]).epsilon(1e-6));
}

TEST_CASE("solver: locked coordinates and locked tasks hold still") {
    auto m = load_arm();
    const Index j1 = joint(m, "j1").idx_v;
    const Index j2 = joint(m, "j2").idx_v;
    Eigen::VectorXd target = m->neutral();
    target[joint(m, "j1").idx_q] = 1.0;
    target[joint(m, "j2").idx_q] = 1.0;
    auto posture = std::make_shared<PostureTask>(target);

    SolverOptions o;
    o.locked = Mask(static_cast<std::size_t>(m->nv()), false);
    o.locked.set(static_cast<std::size_t>(j2));
    Solver solver = make_solver(m, o);
    REQUIRE(solver.add_task("posture", posture));
    SolveResult r;
    REQUIRE(solver.solve(m->neutral(), 0.01, r));
    CHECK(r.v[j2] == 0.0);
    CHECK(r.v[j1] > 0.1);

    REQUIRE(solver.set_options({}));
    auto hold = joint_task("j1", 0.0);
    Mask mask(static_cast<std::size_t>(m->nv()), false);
    mask.set(static_cast<std::size_t>(j1));
    REQUIRE(hold->set_mask(mask));
    REQUIRE(solver.add_task("hold", hold, TaskOptions{0, true, true}));
    REQUIRE(solver.solve(m->neutral(), 0.01, r));
    CHECK(r.v[j1] == 0.0);
    CHECK(r.v[j2] > 0.1);
    CHECK(r.residuals.size() == 1);
}

TEST_CASE("solver: every mode runs and reports its stages") {
    auto m = load_arm();
    const Eigen::Isometry3d target = pose_at(m, m->neutral(), "tool");
    Eigen::VectorXd rest = m->neutral();
    rest[joint(m, "j3").idx_q] = std::cos(0.3);
    rest[joint(m, "j3").idx_q + 1] = std::sin(0.3);
    struct Case {
        SolveMode mode;
        std::size_t stages;
    };
    for (Case c : {Case{SolveMode::weighted, 1}, Case{SolveMode::lexicographic, 1},
                   Case{SolveMode::layered_independent, 2}, Case{SolveMode::layered_coupled, 2}}) {
        CAPTURE(to_string(c.mode));
        SolverOptions o;
        o.mode = c.mode;
        Solver solver = make_solver(m, o);
        REQUIRE(solver.add_task("tool", std::make_shared<FrameTask>("tool", target), TaskOptions{0}));
        auto posture = std::make_shared<PostureTask>(m->neutral());
        REQUIRE(posture->set_weight(0.1));
        REQUIRE(solver.add_task("posture", posture, TaskOptions{1}));
        SolveResult r;
        const Status s = solver.solve(m->neutral(), 0.01, r);
        REQUIRE_MESSAGE(s, (s ? "" : s.error().message));
        CHECK(r.stages.size() == c.stages);
        for (const StageReport& st : r.stages) CHECK(st.solve_s > 0.0);
        CHECK(r.residuals.size() == 2);
        CHECK(r.v.allFinite());
        CHECK_FALSE(r.held);
        CHECK(r.quality == solve_quality(r));

        // A closed-loop constraint with feedback away from q evaluates every later stage at the advanced feedback.
        auto loose = std::make_shared<JointVelocityConstraint>(std::vector<std::string>{"j1"},
                                                               Eigen::VectorXd::Constant(1, 10.0));
        loose->set_closed_loop(true);
        REQUIRE(solver.add_constraint("loose", loose));
        const Eigen::VectorXd feedback = rest;
        const Status fs = solver.solve(m->neutral(), &feedback, 0.01, r);
        REQUIRE_MESSAGE(fs, (fs ? "" : fs.error().message));
        CHECK(r.stages.size() == c.stages);
        CHECK(r.v.allFinite());
    }
    // layered_independent runs level 0 first, like every other mode.
    SolverOptions o;
    o.mode = SolveMode::layered_independent;
    Solver solver = make_solver(m, o);
    REQUIRE(solver.add_task("a", joint_task("j1", 0.5), TaskOptions{0}));
    REQUIRE(solver.add_task("b", joint_task("j2", 0.5), TaskOptions{3}));
    SolveResult r;
    REQUIRE(solver.solve(m->neutral(), 0.01, r));
    REQUIRE(r.stages.size() == 2);
    CHECK(r.stages[0].level == 0);
    CHECK(r.stages[1].level == 3);
}

TEST_CASE("solver: every mode applies each task gain once per step") {
    auto m = load_arm();
    const double target = 1e-3, gain = 0.2, dt = 0.01;
    for (SolveMode mode : {SolveMode::weighted, SolveMode::lexicographic, SolveMode::layered_independent,
                           SolveMode::layered_coupled}) {
        CAPTURE(to_string(mode));
        SolverOptions o;
        o.mode = mode;
        Solver solver = make_solver(m, o);
        int level = 0;
        for (const char* name : {"j1", "j2", "j4"})
            REQUIRE(solver.add_task(name, joint_task(name, target, gain), TaskOptions{level++}));
        SolveResult r;
        REQUIRE(solver.solve(m->neutral(), dt, r));
        for (const char* name : {"j1", "j2", "j4"}) {
            CAPTURE(name);
            CHECK(r.v[joint(m, name).idx_v] * dt / target == doctest::Approx(gain).epsilon(1e-3));
        }
    }
}

TEST_CASE("solver: layered_independent removes drift from earlier stages in full") {
    // j2 (level 0) tilts the arm; the tool holds its world z with the j4 slide alone (level 1).
    auto m = load_arm();
    Eigen::VectorXd q0 = m->neutral();
    q0[joint(m, "j2").idx_q] = 0.5;
    q0[joint(m, "j4").idx_q] = 0.05;
    const Eigen::Isometry3d hold = pose_at(m, q0, "tool");
    const auto only = [&](const char* name) {
        Mask mask(static_cast<std::size_t>(m->nv()), false);
        mask.set(static_cast<std::size_t>(joint(m, name).idx_v));
        return mask;
    };
    const auto z_error = [&](const Eigen::VectorXd& q) {
        return std::abs(pose_at(m, q, "tool").translation().z() - hold.translation().z());
    };
    const auto step = [&](SolveMode mode, double lm, double& drift) {
        SolverOptions o;
        o.mode = mode;
        Solver solver = make_solver(m, o);
        auto lead = joint_task("j2", 0.502);
        REQUIRE(lead->set_mask(only("j2")));
        REQUIRE(solver.add_task("lead", lead, TaskOptions{0}));
        auto tool = std::make_shared<FrameTask>("tool", hold);
        tool->set_world_axes(true);
        REQUIRE(tool->set_gain(0.2));
        REQUIRE(tool->set_lm_damping(lm));
        Eigen::VectorXd w = Eigen::VectorXd::Zero(6);
        w[2] = 1.0;
        REQUIRE(tool->set_weight(w));
        REQUIRE(tool->set_mask(only("j4")));
        tool->set_mask_columns(false);  // as the frame module: the row sees j2 but frees only j4
        REQUIRE(solver.add_task("tool", tool, TaskOptions{1}));
        SolveResult r;
        REQUIRE(solver.solve(q0, 0.01, r));
        Eigen::VectorXd lead_only = q0;
        lead_only[joint(m, "j2").idx_q] += r.v[joint(m, "j2").idx_v] * 0.01;
        drift = z_error(lead_only);
        return z_error(r.q_next);
    };
    // lm: damping must follow the error at q, not the drift it is removing.
    for (const double lm : {0.0, 50.0}) {
        CAPTURE(lm);
        double drift = 0.0, unused = 0.0;
        const double layered = step(SolveMode::layered_independent, lm, drift);
        const double strict = step(SolveMode::lexicographic, lm, unused);
        REQUIRE(drift > 1e-4);
        // Without the feedforward the tool keeps about (1 - gain) of the drift.
        CHECK(layered < 0.05 * drift);
        CHECK(layered < strict + 0.05 * drift);
    }
}

TEST_CASE("solver: unleveled tasks join only the last layered stage") {
    auto m = load_arm();
    const auto v_of = [&](const char* name) { return static_cast<Index>(joint(m, name).idx_v); };
    const auto masked = [&](const char* name, double target) {
        auto t = joint_task(name, target);
        Mask mask(static_cast<std::size_t>(m->nv()), false);
        mask.set(static_cast<std::size_t>(v_of(name)));
        REQUIRE(t->set_mask(mask));
        return t;
    };
    Eigen::VectorXd rest = m->neutral();
    rest[joint(m, "j3").idx_q] = std::cos(0.3);
    rest[joint(m, "j3").idx_q + 1] = std::sin(0.3);
    for (SolveMode mode : {SolveMode::layered_independent, SolveMode::layered_coupled}) {
        CAPTURE(to_string(mode));
        SolverOptions o;
        o.mode = mode;
        Solver solver = make_solver(m, o);
        REQUIRE(solver.add_task("a", masked("j1", 0.5), TaskOptions{0}));
        REQUIRE(solver.add_task("b", masked("j2", 0.5), TaskOptions{2}));
        REQUIRE(solver.add_task("free", masked("j2", 0.0)));
        // Unmasked and unleveled beside leveled tasks: frees no joint, so j3 stays.
        REQUIRE(solver.add_task("posture", std::make_shared<PostureTask>(rest)));
        SolveResult r;
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        REQUIRE(r.stages.size() == 2);
        CHECK(r.stages[1].level == 2);
        CHECK(r.v[v_of("j3")] == 0.0);
        for (const TaskResidual& res : r.residuals)
            if (res.name == "free" || res.name == "posture") CHECK(res.level == -1);
    }
    // With no leveled task there is one stage over all joints.
    SolverOptions o;
    o.mode = SolveMode::layered_independent;
    Solver solver = make_solver(m, o);
    REQUIRE(solver.add_task("posture", std::make_shared<PostureTask>(rest)));
    SolveResult r;
    REQUIRE(solver.solve(m->neutral(), 0.01, r));
    REQUIRE(r.stages.size() == 1);
    CHECK(r.v[v_of("j3")] != 0.0);
}

// State row q_a + q_b <= 0, relinearized at each stage's configuration like collision rows.
class SumBelowZero final : public Constraint {
  public:
    SumBelowZero(const JointInfo& a, const JointInfo& b) : Constraint(1), a_(a), b_(b) {}
    bool defers_to_later_stages() const noexcept override { return true; }

  private:
    Status on_bind(const Model&) override { return {}; }
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd>,
                   Eigen::Ref<Eigen::VectorXd> hi) override {
        const Eigen::VectorXd& q = tick.workspace.q();
        A(0, a_.idx_v) = 1.0;
        A(0, b_.idx_v) = 1.0;
        hi[0] = -(q[a_.idx_q] + q[b_.idx_q]) / tick.dt;
        return {};
    }

    JointInfo a_, b_;
};

TEST_CASE("solver: layered_independent leaves a shared row to the later stage") {
    auto m = load_arm();
    const JointInfo j1 = joint(m, "j1");
    const JointInfo j2 = joint(m, "j2");
    const auto masked = [&](const char* name, Index v, double target) {
        auto t = joint_task(name, target);
        Mask mask(static_cast<std::size_t>(m->nv()), false);
        mask.set(static_cast<std::size_t>(v));
        REQUIRE(t->set_mask(mask));
        return t;
    };
    SolverOptions o;
    o.mode = SolveMode::layered_independent;
    Solver solver = make_solver(m, o);
    REQUIRE(solver.add_constraint("row", std::make_shared<SumBelowZero>(j1, j2)));
    REQUIRE(solver.add_task("first", masked("j1", j1.idx_v, 0.5), TaskOptions{0}));
    REQUIRE(solver.add_task("second", masked("j2", j2.idx_v, 0.0), TaskOptions{1}));
    SolveResult r;
    REQUIRE(solver.solve(m->neutral(), 0.01, r));
    // The j1 stage moves freely; the j2 stage yields to keep the row.
    CHECK(r.v[j1.idx_v] > 0.1);
    CHECK(r.v[j2.idx_v] < -0.1);
    CHECK(r.q_next[j1.idx_q] + r.q_next[j2.idx_q] <= 1e-6);
}

TEST_CASE("solver: layered_independent keeps a shared external row in every stage") {
    auto m = load_arm();
    const JointInfo j1 = joint(m, "j1");
    const JointInfo j2 = joint(m, "j2");
    const auto masked = [&](const char* name, Index v, double target) {
        auto t = joint_task(name, target);
        Mask mask(static_cast<std::size_t>(m->nv()), false);
        mask.set(static_cast<std::size_t>(v));
        REQUIRE(t->set_mask(mask));
        return t;
    };
    SolverOptions o;
    o.mode = SolveMode::layered_independent;
    Solver solver = make_solver(m, o);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(1, m->nv());
    A(0, j1.idx_v) = 1.0;
    A(0, j2.idx_v) = 1.0;
    auto c = std::make_shared<ExternalConstraint>();
    REQUIRE(c->set(A, Eigen::VectorXd::Constant(1, -std::numeric_limits<double>::infinity()),
                   Eigen::VectorXd::Zero(1)));
    REQUIRE(solver.add_constraint("row", c));
    REQUIRE(solver.add_task("first", masked("j1", j1.idx_v, 0.5), TaskOptions{0}));
    REQUIRE(solver.add_task("second", masked("j2", j2.idx_v, 0.0), TaskOptions{1}));
    SolveResult r;
    REQUIRE(solver.solve(m->neutral(), 0.01, r));
    // Constant rows are not re-evaluated per stage, so no stage may spend another's share.
    CHECK(r.v[j1.idx_v] + r.v[j2.idx_v] <= 1e-6);
}

TEST_CASE("solver: a scope-only mask lets other free coordinates serve the task") {
    auto m = load_arm();
    const auto v1 = static_cast<Index>(joint(m, "j1").idx_v);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(m->nv());
    v[v1] = 0.4;
    Eigen::VectorXd goal(m->nq());
    REQUIRE(m->integrate(m->neutral(), v, goal));
    Mask chain(static_cast<std::size_t>(m->nv()), false), base = chain;
    for (const char* j : {"j2", "j3", "j4"}) chain.set(static_cast<std::size_t>(joint(m, j).idx_v));
    base.set(static_cast<std::size_t>(v1));
    struct Case {
        SolveMode mode;
        bool zero;
        bool moves;
    };
    // Only a stage that frees j1 beside the tool lets the tool pull it, and only without zeroed columns.
    for (Case c : {Case{SolveMode::weighted, true, false}, Case{SolveMode::weighted, false, true},
                   Case{SolveMode::layered_coupled, false, true},
                   Case{SolveMode::layered_independent, false, false}}) {
        CAPTURE(to_string(c.mode));
        CAPTURE(c.zero);
        SolverOptions o;
        o.mode = c.mode;
        Solver solver = make_solver(m, o);
        auto tool = std::make_shared<FrameTask>("tool", pose_at(m, goal, "tool"));
        REQUIRE(tool->set_mask(chain));
        tool->set_mask_columns(c.zero);
        CHECK(tool->mask_columns() == c.zero);
        auto hold = joint_task("j1", 0.0);
        REQUIRE(hold->set_weight(0.01));
        REQUIRE(hold->set_mask(base));
        REQUIRE(solver.add_task("tool", tool, TaskOptions{0}));
        REQUIRE(solver.add_task("base", hold, TaskOptions{1}));
        SolveResult r;
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        CHECK((std::abs(r.v[v1]) > 1e-3) == c.moves);
    }
}

TEST_CASE("solver: lexicographic mode enforces strict priority") {
    auto m = load_arm();
    const JointInfo& j1 = joint(m, "j1");
    Eigen::VectorXd q = m->neutral();
    const auto run = [&](SolveMode mode) {
        SolverOptions o;
        o.mode = mode;
        Solver solver = make_solver(m, o);
        REQUIRE(solver.add_task("high", joint_task("j1", 0.1, 0.1), TaskOptions{0}));
        REQUIRE(solver.add_task("low", joint_task("j1", -0.1, 0.1), TaskOptions{1}));
        SolveResult r;
        REQUIRE(solver.solve(q, 0.01, r));
        return r;
    };
    const SolveResult h = run(SolveMode::lexicographic);
    CHECK(h.v[j1.idx_v] * 0.01 == doctest::Approx(0.01).epsilon(1e-4));
    CHECK(h.residuals[0].residual < 1e-6);
    const SolveResult w = run(SolveMode::weighted);
    CHECK(std::abs(w.v[j1.idx_v]) < 1e-3);
}

TEST_CASE("solver: null-space posture leaves the primary task untouched") {
    auto m = load_arm();
    const Index j1 = joint(m, "j1").idx_v;
    const auto run = [&](bool with_null_space) {
        Solver solver = make_solver(m);
        auto primary = joint_task("j1", 0.5, 0.01);
        REQUIRE(solver.add_task("primary", primary));
        if (with_null_space) {
            Eigen::VectorXd target = m->neutral();
            target[joint(m, "j1").idx_q] = -1.0;
            target[joint(m, "j2").idx_q] = 0.8;
            REQUIRE(solver.add_task("rest", std::make_shared<NullSpacePostureTask>(primary, target)));
        }
        SolveResult r;
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        return r.v;
    };
    const Eigen::VectorXd alone = run(false);
    const Eigen::VectorXd both = run(true);
    CHECK(alone[j1] == doctest::Approx(0.5).epsilon(1e-2));
    CHECK(both[j1] == doctest::Approx(alone[j1]).epsilon(1e-4));
    CHECK(both[joint(m, "j2").idx_v] > 1e-3);
}

TEST_CASE("solver: hold retry and infeasibility") {
    auto m = load_arm();
    const Index j1 = joint(m, "j1").idx_v;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(1, m->nv());
    A(0, j1) = 1.0;
    SolveResult r;
    {
        Solver solver = make_solver(m);
        REQUIRE(solver.add_task("j1", joint_task("j1", 1.0)));
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        CHECK(r.quality == SolveQuality::nominal);
        CHECK(std::string(to_string(r.quality)) == "nominal");
    }
    {
        // v_j1 >= 2 conflicts with the 1.5 rad/s limit; the recovery scaled to v_j1 >= 1 is admissible.
        Solver solver = make_solver(m);
        auto c = std::make_shared<ExternalConstraint>();
        REQUIRE(c->set(A, Eigen::VectorXd::Constant(1, 2.0), Eigen::VectorXd::Constant(1, 3.0)));
        REQUIRE(solver.add_constraint("push", c));
        REQUIRE(solver.add_task("j1", joint_task("j1", 1.0)));
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        CHECK(r.held);
        CHECK(r.quality == SolveQuality::degraded);
        CHECK(std::string(to_string(r.quality)) == "degraded");
        CHECK(r.v[j1] >= 1.0 - 1e-6);
        CHECK(r.v[j1] <= 1.5 + 1e-6);

        SolverOptions o;
        o.hold_retry = false;
        REQUIRE(solver.set_options(o));
        const Status s = solver.solve(m->neutral(), 0.01, r);
        REQUIRE_FALSE(s);
        CHECK(s.error().code == ErrorCode::infeasible);
        CHECK(r.quality == SolveQuality::nominal);
        CHECK(r.v.isZero(0.0));
        CHECK(r.q_next == m->neutral());
    }
}

TEST_CASE("solver: a backend failure falls back to hold when x = 0 is admissible") {
    auto m = load_arm();
    const Index j1 = joint(m, "j1").idx_v;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(1, m->nv());
    A(0, j1) = 1.0;
    SolverOptions o;
    o.backend = [] { return std::make_unique<CyclingBackend>(); };
    SolveResult r;
    {
        Solver solver = make_solver(m, o);
        REQUIRE(solver.add_task("j1", joint_task("j1", 1.0)));
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        CHECK(r.held);
        CHECK(r.quality == SolveQuality::degraded);
        CHECK(r.v.isZero(0.0));
        CHECK(r.q_next == m->neutral());

        o.hold_retry = false;
        REQUIRE(solver.set_options(o));
        const Status s = solver.solve(m->neutral(), 0.01, r);
        REQUIRE_FALSE(s);
        CHECK(s.error().code == ErrorCode::internal);
        o.hold_retry = true;
    }
    {
        // An equality v_j1 = 0.5 survives widening, so hold is not admissible.
        Solver solver = make_solver(m, o);
        auto c = std::make_shared<ExternalConstraint>();
        REQUIRE(c->set(A, Eigen::VectorXd::Constant(1, 0.5), Eigen::VectorXd::Constant(1, 0.5)));
        REQUIRE(solver.add_constraint("pin", c));
        REQUIRE(solver.add_task("j1", joint_task("j1", 1.0)));
        REQUIRE_FALSE(solver.solve(m->neutral(), 0.01, r));
    }
}

TEST_CASE("solver: the commanded velocity never exceeds the velocity limit") {
    auto m = load_arm();
    const Index j1 = joint(m, "j1").idx_v;
    SolverOptions o;
    o.backend = [] { return std::make_unique<OvershootBackend>(); };
    o.check_feasibility = false;
    Solver solver = make_solver(m, o);
    REQUIRE(solver.add_task("j1", joint_task("j1", 1.0)));
    SolveResult r;
    REQUIRE(solver.solve(m->neutral(), 0.01, r));
    CHECK(r.v[j1] > 1.5 - 1e-9);
    CHECK(r.v[j1] <= 1.5);
}

TEST_CASE("solver: a stage with no free joint still checks its constraints") {
    auto m = load_arm();
    const Index j1 = joint(m, "j1").idx_v;
    const Index j2 = joint(m, "j2").idx_v;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(1, m->nv());
    A(0, j2) = 1.0;
    const auto push = [&] {
        auto c = std::make_shared<ExternalConstraint>();
        REQUIRE(c->set(A, Eigen::VectorXd::Constant(1, 0.5), Eigen::VectorXd::Constant(1, 1.0)));
        return c;
    };
    SolveResult r;
    {
        // An unmasked task frees every joint, so the constraint moves j2.
        Solver solver = make_solver(m);
        REQUIRE(solver.add_task("posture", std::make_shared<PostureTask>(m->neutral())));
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        CHECK(r.v.isZero(1e-9));
        REQUIRE(solver.add_constraint("push", push()));
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        CHECK_FALSE(r.held);
        CHECK(r.v[j2] >= 0.5 - 1e-6);
    }
    {
        // The only task moves j1, which is locked: nothing is free and v_j2 >= 0.5 fails at x = 0.
        SolverOptions o;
        o.locked = Mask(static_cast<std::size_t>(m->nv()), false);
        o.locked.set(static_cast<std::size_t>(j1));
        Solver solver = make_solver(m, o);
        auto task = joint_task("j1", 1.0);
        Mask mask(static_cast<std::size_t>(m->nv()), false);
        mask.set(static_cast<std::size_t>(j1));
        REQUIRE(task->set_mask(mask));
        REQUIRE(solver.add_task("j1", task));
        REQUIRE(solver.add_constraint("push", push()));
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        CHECK(r.held);
        CHECK(r.quality == SolveQuality::degraded);
        CHECK(r.v.isZero(0.0));

        o.hold_retry = false;
        REQUIRE(solver.set_options(o));
        const Status s = solver.solve(m->neutral(), 0.01, r);
        REQUIRE_FALSE(s);
        CHECK(s.error().code == ErrorCode::infeasible);
    }
}

TEST_CASE("solver: argument errors") {
    auto m = load_arm();
    CHECK_FALSE(Solver::create(nullptr));
    SolverOptions bad;
    bad.damping = 0;
    CHECK_FALSE(Solver::create(m, bad));
    bad = {};
    bad.velocity_limit = Eigen::VectorXd::Ones(2);
    CHECK_FALSE(Solver::create(m, bad));
    bad = {};
    bad.backend = [] { return std::unique_ptr<qp::Backend>(); };
    CHECK_FALSE(Solver::create(m, bad));
    auto rigid = Model::load(ModelSource::text(R"(<robot name="r"><link name="a"/></robot>)", "urdf"), {},
                             Context::defaults());
    REQUIRE(rigid);
    CHECK_FALSE(Solver::create(*rigid));

    Solver solver = make_solver(m);
    CHECK_FALSE(solver.add_task("x", nullptr));
    REQUIRE(solver.add_task("x", joint_task("j1", 0.0)));
    CHECK_FALSE(solver.add_task("x", joint_task("j1", 0.0)));
    CHECK_FALSE(solver.add_task("y", joint_task("nope", 0.0)));
    CHECK_FALSE(solver.add_task("z", joint_task("j1", 0.0), TaskOptions{-1}));
    CHECK(solver.task("x") != nullptr);
    CHECK(solver.task("missing") == nullptr);
    CHECK(solver.remove_task("missing").error().code == ErrorCode::not_found);

    SolveResult r;
    CHECK(solver.solve(m->neutral(), 0.0, r).error().code == ErrorCode::invalid_argument);
    CHECK(solver.solve(Eigen::VectorXd::Zero(2), 0.01, r).error().code == ErrorCode::invalid_argument);
    REQUIRE(solver.remove_task("x"));
    REQUIRE(solver.solve(m->neutral(), 0.01, r));
    CHECK(r.v.isZero(0.0));
}

TEST_CASE("solver: a non-finite QP solution fails before integration") {
    // arm.urdf has a continuous (SO(2)) joint, whose integrate asserts on NaN in debug pinocchio.
    auto m = load_arm();
    for (bool check : {true, false}) {
        CAPTURE(check);
        SolverOptions o;
        o.backend = [] { return std::make_unique<NanBackend>(); };
        o.check_feasibility = check;
        Solver solver = make_solver(m, o);
        REQUIRE(solver.add_task("j1", joint_task("j1", 1.0)));
        SolveResult r;
        const Eigen::VectorXd q = m->neutral();
        Status s = solver.solve(q, 0.01, r);
        REQUIRE_FALSE(s);
        CHECK(s.error().code == ErrorCode::internal);
        CHECK(s.error().message.find("non-finite velocity step (joint j1 = nan)") != std::string::npos);
        CHECK(r.q_next == q);
        CHECK(r.v.isZero());
    }
}

TEST_CASE("support polygon: validation and inscribed radius") {
    using V = Eigen::Vector2d;
    auto square =
        SupportPolygon::create("base_link", {V(-0.1, -0.1), V(0.1, -0.1), V(0.1, 0.1), V(-0.1, 0.1)});
    REQUIRE(square);
    CHECK(square->edges() == 4);
    CHECK(square->inset_limit() == doctest::Approx(0.1));
    auto clockwise =
        SupportPolygon::create("base_link", {V(-0.1, 0.1), V(0.1, 0.1), V(0.1, -0.1), V(-0.1, -0.1)});
    REQUIRE(clockwise);
    CHECK(clockwise->inset_limit() == doctest::Approx(0.1));
    Eigen::Matrix2Xd inset(2, 4);
    clockwise->inset_vertices(0.04, inset);
    for (Eigen::Index i = 0; i < 4; ++i) CHECK(inset.col(i).cwiseAbs().isApprox(V(0.06, 0.06)));
    // Counter-clockwise: consecutive vertices turn left.
    const Eigen::Vector2d e0 = inset.col(1) - inset.col(0), e1 = inset.col(2) - inset.col(1);
    CHECK(e0.x() * e1.y() - e0.y() * e1.x() > 0.0);
    auto triangle = SupportPolygon::create("base_link", {V(0, 0), V(1, 0), V(0, 1)});
    REQUIRE(triangle);
    Eigen::Matrix2Xd tri(2, 3);
    triangle->inset_vertices(0.0, tri);
    CHECK(tri.col(0).isApprox(V(0, 0)));
    CHECK(tri.col(1).isApprox(V(1, 0)));
    CHECK(tri.col(2).isApprox(V(0, 1)));
    CHECK_FALSE(SupportPolygon::create("base_link", {V(0, 0), V(1, 0)}));
    CHECK_FALSE(SupportPolygon::create("base_link", {V(0, 0), V(1, 0), V(0.2, 0.2), V(0, 1)}));  // concave

    CenterOfMassConstraint com(*square);
    CHECK_FALSE(com.set_margin(0.2));
    CHECK(com.set_margin(0.05));
    CHECK_FALSE(com.set_payloads({Payload{"tool", -1.0}}));
}

TEST_CASE("support polygon: centre of mass Jacobian matches finite differences") {
    using V = Eigen::Vector2d;
    auto m = load_arm();
    auto polygon = SupportPolygon::create("base_link", {V(-1, -1), V(1, -1), V(1, 1), V(-1, 1)});
    REQUIRE(polygon);
    const std::vector<Payload> payloads{Payload{"tool", 0.5, Eigen::Vector3d(0.02, 0.0, 0.05)}};
    Eigen::VectorXd v0 = Eigen::VectorXd::Zero(m->nv());
    v0[joint(m, "j1").idx_v] = 0.4;
    v0[joint(m, "j2").idx_v] = -0.6;
    v0[joint(m, "j3").idx_v] = 0.9;
    v0[joint(m, "j4").idx_v] = 0.05;
    v0[joint(m, "finger_left_joint").idx_v] = 0.02;
    v0[joint(m, "finger_right_joint").idx_v] = -0.02;
    Eigen::VectorXd q(m->nq());
    REQUIRE(m->integrate(m->neutral(), v0, q));
    Workspace ws(m);
    REQUIRE(ws.update(q));
    SupportPolygon::Evaluation e;
    CHECK_FALSE(polygon->evaluate(ws, payloads, e));
    CHECK_FALSE(polygon->bind(*m, {Payload{"missing", 1.0}}));
    CHECK_FALSE(polygon->bound());
    REQUIRE(polygon->bind(*m, payloads));
    REQUIRE(polygon->evaluate(ws, payloads, e));
    constexpr double h = 1e-6;
    Eigen::VectorXd qp(m->nq()), qm(m->nq());
    for (Index i = 0; i < m->nv(); ++i) {
        Eigen::VectorXd d = Eigen::VectorXd::Zero(m->nv());
        d[i] = h;
        REQUIRE(m->integrate(q, d, qp));
        REQUIRE(m->integrate(q, -d, qm));
        SupportPolygon::Evaluation ep, em;
        REQUIRE(ws.update(qp));
        REQUIRE(polygon->evaluate(ws, payloads, ep));
        REQUIRE(ws.update(qm));
        REQUIRE(polygon->evaluate(ws, payloads, em));
        const Eigen::Vector2d numeric = (ep.com - em.com) / (2 * h);
        CHECK((numeric - e.jacobian.col(i)).norm() < 1e-6);
    }

    // As a constraint it keeps the solve feasible.
    Solver solver = make_solver(m);
    REQUIRE(solver.add_constraint("com", std::make_shared<CenterOfMassConstraint>(*polygon)));
    REQUIRE(solver.add_task("j1", joint_task("j1", 1.0)));
    SolveResult r;
    const Status s = solver.solve(q, 0.01, r);
    REQUIRE_MESSAGE(s, (s ? "" : s.error().message));

    // Recovery columns: the recovery joints' rows, plus the other joints' rows that may not push outward.
    CenterOfMassConstraint com(*polygon);
    REQUIRE(com.bind(m));
    const auto j2 = static_cast<std::size_t>(joint(m, "j2").idx_v);
    const auto nv = static_cast<std::size_t>(m->nv());
    const Index n = polygon->edges();
    CHECK(com.rows() == n);
    CHECK_FALSE(com.set_columns(Mask(nv + 1)));
    REQUIRE(com.set_columns(Mask::from_indices(nv, {j2})));
    REQUIRE(com.rows() == 2 * n);
    REQUIRE(ws.update(q));
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(com.rows(), m->nv());
    Eigen::VectorXd lo = Eigen::VectorXd::Constant(com.rows(), -INFINITY);
    Eigen::VectorXd hi = Eigen::VectorXd::Constant(com.rows(), INFINITY);
    REQUIRE(com.evaluate(Tick{ws, 0.01}, A, lo, hi));
    CHECK(A.topRows(n).col(static_cast<Index>(j2)).norm() > 1e-6);
    CHECK(A.bottomRows(n).col(static_cast<Index>(j2)).norm() == 0.0);
    CenterOfMassConstraint plain(*polygon);
    REQUIRE(plain.bind(m));
    Eigen::MatrixXd full = Eigen::MatrixXd::Zero(n, m->nv());
    Eigen::VectorXd plo = Eigen::VectorXd::Constant(n, -INFINITY),
                    phi = Eigen::VectorXd::Constant(n, INFINITY);
    REQUIRE(plain.evaluate(Tick{ws, 0.01}, full, plo, phi));
    CHECK(hi.head(n) == phi);
    for (Index i = 0; i < m->nv(); ++i) {
        if (static_cast<std::size_t>(i) == j2) continue;
        CHECK(A.topRows(n).col(i).norm() == 0.0);
        CHECK((A.bottomRows(n).col(i) - full.col(i)).norm() < 1e-12);
    }
    for (Index i = 0; i < n; ++i) {
        CHECK(hi[n + i] >= 0.0);
        CHECK(hi[n + i] >= hi[i]);
    }
    REQUIRE(com.set_columns(Mask()));
    CHECK(com.rows() == n);
}

TEST_CASE("centre of mass: the waist compensates for the arm, and the arm only stops when it cannot") {
    using V = Eigen::Vector2d;
    auto m = load_arm();
    const auto j1 = static_cast<std::size_t>(joint(m, "j1").idx_v);
    const auto nv = static_cast<std::size_t>(m->nv());
    const auto masked = [&](const char* name, double target) {
        auto t = joint_task(name, target);
        Mask mask(nv, false);
        mask.set(static_cast<std::size_t>(joint(m, name).idx_v));
        REQUIRE(t->set_mask(mask));
        return t;
    };
    const auto com_y = [&](const SupportPolygon& p, const Eigen::VectorXd& q) {
        Workspace ws(m);
        REQUIRE(ws.update(q));
        SupportPolygon::Evaluation e;
        REQUIRE(p.evaluate(ws, {}, e));
        return e.com.y();
    };
    // j1 (yaw) is the waist and j4 (extension) the arm; extending the arm moves the CoM sideways.
    struct Case {
        double waist;
        bool upper;
    };
    for (const Case c : {Case{0.5, true}, Case{-1.995, false}}) {
        const bool saturated = !c.upper;
        for (SolveMode mode :
             {SolveMode::layered_independent, SolveMode::layered_coupled, SolveMode::weighted}) {
            CAPTURE(c.waist);
            CAPTURE(to_string(mode));
            Eigen::VectorXd q = m->neutral();
            q[joint(m, "j1").idx_q] = c.waist;
            auto probe = SupportPolygon::create("base_link", {V(-1, -1), V(1, -1), V(1, 1), V(-1, 1)});
            REQUIRE(probe);
            REQUIRE(probe->bind(*m, {}));
            const double y0 = com_y(*probe, q);
            const double edge = c.upper ? y0 + 1e-3 : y0 - 1e-3;
            const double lo = c.upper ? -1.0 : edge, hi = c.upper ? edge : 1.0;
            auto polygon = SupportPolygon::create("base_link", {V(-1, lo), V(1, lo), V(1, hi), V(-1, hi)});
            REQUIRE(polygon);
            REQUIRE(polygon->bind(*m, {}));
            auto com = std::make_shared<CenterOfMassConstraint>(*polygon);
            REQUIRE(com->set_columns(Mask::from_indices(nv, {j1})));
            SolverOptions o;
            o.mode = mode;
            Solver solver = make_solver(m, o);
            REQUIRE(solver.add_constraint("com", com));
            REQUIRE(solver.add_task("waist", masked("j1", c.waist), TaskOptions{0}));
            REQUIRE(solver.add_task("arm", masked("j4", 0.1), TaskOptions{1}));
            double worst = 0.0, arm = 0.0;
            bool retracted = false;
            for (int k = 0; k < 400; ++k) {
                SolveResult r;
                REQUIRE(solver.solve(q, 0.01, r));
                Eigen::VectorXd next(m->nq());
                REQUIRE(m->integrate(q, r.v * 0.01, next));
                q = next;
                const double out = c.upper ? com_y(*polygon, q) - edge : edge - com_y(*polygon, q);
                worst = std::max(worst, out);
                const double a = q[joint(m, "j4").idx_q];
                retracted = retracted || a < arm - 1e-9;
                arm = a;
            }
            CHECK(worst < 2e-3);
            CHECK_FALSE(retracted);
            if (saturated) {
                CHECK(arm < 0.09);
            } else {
                CHECK(arm == doctest::Approx(0.1).epsilon(1e-3));
                CHECK(std::abs(q[joint(m, "j1").idx_q] - c.waist) > 1e-3);
            }
        }
    }
}

TEST_CASE("solver: hold keeps a violated row from growing but lets it recover") {
    auto m = load_arm();
    const Index j2 = joint(m, "j2").idx_v;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, m->nv());
    A(0, j2) = 1.0;
    A(1, j2) = 1.0;
    Eigen::VectorXd lo(2), hi(2);
    lo << 2.0, -1.0;  // v_j2 >= 2 and v_j2 <= 0.3 contradict, also with the recovery scaled to 0.5
    hi << 3.0, 0.3;
    for (const double target : {1.0, -1.0}) {
        CAPTURE(target);
        Solver solver = make_solver(m);
        REQUIRE(solver.add_task("j2", joint_task("j2", target)));
        auto c = std::make_shared<ExternalConstraint>();
        REQUIRE(c->set(A, lo, hi));
        REQUIRE(solver.add_constraint("rows", c));
        SolveResult r;
        REQUIRE(solver.solve(m->neutral(), 0.01, r));
        CHECK(r.held);
        CHECK(r.quality == SolveQuality::degraded);
        CHECK(r.v[j2] >= -1e-9);       // the violated row v_j2 >= 0.5 may not get worse
        CHECK(r.v[j2] <= 0.3 + 1e-9);  // the satisfied row keeps its bound
        if (target > 0)
            CHECK(r.v[j2] == doctest::Approx(0.3).epsilon(1e-6));  // recovering is allowed
        else
            CHECK(std::abs(r.v[j2]) <= 1e-9);
    }
}

namespace {
// Admits ends with q_j1 <= cap and estimates the admitted fraction as for a linear distance.
class CapConstraint final : public Constraint {
  public:
    CapConstraint(Index j1, double cap, bool estimate) : j1_(j1), cap_(cap), estimate_(estimate) {}
    bool checks_end(const Eigen::VectorXd&, const Eigen::VectorXd&) const override { return true; }
    bool admits(const Tick& end) override {
        ++calls;
        const double v = end.workspace.q()[j1_];
        fraction_ = v > 0.0 ? std::clamp(cap_ / v, 0.0, 1.0) : 1.0;
        return v <= cap_;
    }
    double admitted_fraction() const noexcept override {
        return estimate_ ? fraction_ : Constraint::admitted_fraction();
    }
    int calls{0};

  protected:
    Status on_bind(const Model&) override { return {}; }
    Status compute(const Tick&, Eigen::Ref<Eigen::MatrixXd>, Eigen::Ref<Eigen::VectorXd>,
                   Eigen::Ref<Eigen::VectorXd>) override {
        return {};
    }

  private:
    Index j1_;
    double cap_, fraction_{1.0};
    bool estimate_;
};
}  // namespace

TEST_CASE("solver: endpoint clip shrinks toward the admitted fraction") {
    auto m = load_arm();
    const Index j1 = joint(m, "j1").idx_q;
    const double dt = 0.01;
    SolveResult free;
    {
        Solver solver = make_solver(m);
        REQUIRE(solver.add_task("j1", joint_task("j1", 1.0)));
        REQUIRE(solver.solve(m->neutral(), dt, free));
    }
    const double full = free.q_next[j1];
    REQUIRE(full > 0.0);
    for (const bool estimate : {true, false}) {
        CAPTURE(estimate);
        Solver solver = make_solver(m);
        REQUIRE(solver.add_task("j1", joint_task("j1", 1.0)));
        auto cap = std::make_shared<CapConstraint>(j1, 0.8 * full, estimate);
        REQUIRE(solver.add_constraint("cap", cap));
        // An accepting constraint keeps its default fraction out of the estimate.
        REQUIRE(solver.add_constraint("open", std::make_shared<CapConstraint>(j1, 1e9, false)));
        SolveResult r;
        REQUIRE(solver.solve(m->neutral(), dt, r));
        CHECK(r.q_next[j1] <= 0.8 * full + 1e-12);
        CHECK(r.step_scale > 0.0);
        if (estimate) {
            CHECK(cap->calls == 2);  // one rejected end, one accepted at 0.9 x the estimate
            CHECK(r.step_scale == doctest::Approx(0.72));
        } else {
            CHECK(r.step_scale == doctest::Approx(0.45));
        }
    }
    {
        // An end admitted at no scale holds the start after a single rejection.
        Solver solver = make_solver(m);
        REQUIRE(solver.add_task("j1", joint_task("j1", 1.0)));
        auto cap = std::make_shared<CapConstraint>(j1, -1.0, true);
        REQUIRE(solver.add_constraint("cap", cap));
        SolveResult r;
        REQUIRE(solver.solve(m->neutral(), dt, r));
        CHECK(r.step_scale == 0.0);
        CHECK(r.q_next[j1] == doctest::Approx(m->neutral()[j1]));
        CHECK(cap->calls == 1);
    }
}
