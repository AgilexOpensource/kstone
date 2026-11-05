#include "kstone/control/tasks.hpp"
#include "kstone/module/yaml.hpp"
#include "kstone/qp/backend.hpp"
#include "kstone/runtime/config.hpp"
#include "kstone/runtime/engine.hpp"

#include <doctest/doctest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;
constexpr double kDt = 0.01;

template <typename T>
std::string message(const Result<T>& r) {
    return r ? std::string{} : r.error().message;
}

bool contains(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }

// A QP backend that reports success with a NaN step.
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

// Drives `joint` toward its target at full gain; optionally overwrites one q coordinate in prepare().
class Probe final : public Module {
  public:
    explicit Probe(std::string joint = "j1") : Module("probe"), joint_(std::move(joint)) {}
    const char* type() const noexcept override { return "probe"; }
    Status configure(ModuleSetup& setup) override {
        task = std::make_shared<JointTask>(std::vector<std::string>{joint_}, Eigen::VectorXd::Zero(1));
        if (auto s = task->set_gain(1.0); !s) return s;
        auto h = setup.add_task(joint_, task);
        if (!h) return std::move(h).error();
        return {};
    }
    Status prepare(Cycle& cycle) override {
        if (write_index >= 0) cycle.q[write_index] = write_value;
        last_now = cycle.now;
        ++steps;
        return {};
    }
    Status reset(const Cycle&) override {
        ++resets;
        return {};
    }
    std::shared_ptr<JointTask> task;
    double last_now{-1.0};
    int steps{0};
    int resets{0};
    Eigen::Index write_index{-1};
    double write_value{0.0};

  private:
    std::string joint_;
};

// Loopback that records every written command.
class RecordingRobot final : public RobotInterface {
  public:
    RecordingRobot(Eigen::VectorXd q, Eigen::Index nv) : inner_(std::move(q), nv) {}
    Status read(RobotState& state) override { return inner_.read(state); }
    Status write(const RobotCommand& command) override {
        ++writes;
        last = command;
        return inner_.write(command);
    }
    int writes{0};
    RobotCommand last;

  private:
    LoopbackRobot inner_;
};

struct Fixture {
    ModelPtr model;
    std::shared_ptr<Probe> probe;
    std::shared_ptr<RecordingRobot> robot;
    Engine engine;

    Eigen::Index iq(const char* joint) const { return model->joints()[*model->joint_id(joint)].idx_q; }
    Eigen::Index iv(const char* joint) const { return model->joints()[*model->joint_id(joint)].idx_v; }
    void target_j1(double q) {
        Eigen::VectorXd t(1);
        t << q;
        REQUIRE(probe->task->set_target(t));
    }
};

// arm.urdf: j1 is revolute in [-2, 2] with a velocity limit of 1.5 rad/s.
// solver_velocity > 0 lets the solver exceed the model velocity limit so the guard sees it.
Fixture make(CommandGuardOptions guard, double solver_velocity = 0.0, double j1 = 0.0,
             const Context& context = Context::defaults()) {
    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE_MESSAGE(model, message(model));
    SolverOptions so;
    if (solver_velocity > 0.0) so.velocity_limit = Eigen::VectorXd::Constant((*model)->nv(), solver_velocity);
    auto set = ModuleSet::create(*model, so);
    REQUIRE(set);
    auto probe = std::make_shared<Probe>();
    REQUIRE(set->add(probe));
    Eigen::VectorXd q0 = (*model)->neutral();
    q0[(*model)->joints()[*(*model)->joint_id("j1")].idx_q] = j1;
    auto robot = std::make_shared<RecordingRobot>(q0, (*model)->nv());
    EngineOptions options;
    options.rate_hz = 1.0 / kDt;
    options.realtime = false;
    options.safety = guard;
    auto e = Engine::create(std::move(*set), robot, options, context);
    REQUIRE_MESSAGE(e, message(e));
    return {*model, probe, robot, std::move(*e)};
}

CommandGuardOptions mode(GuardMode m) {
    CommandGuardOptions g;
    g.mode = m;
    return g;
}

}  // namespace

TEST_CASE("guard: defaults reject and keep the fixture usable") {
    CHECK(EngineOptions{}.safety.mode == GuardMode::reject);
    CHECK(EngineOptions{}.safety.velocity_scale == 1.0);
    CHECK(EngineOptions{}.safety.tolerance == 1e-6);

    // Within the model limits the guard changes nothing.
    Fixture f = make({});
    f.target_j1(1.0);
    for (int i = 0; i < 50; ++i) REQUIRE(f.engine.step(kDt * (i + 1)));
    CHECK(f.robot->writes == 50);
    CHECK(f.engine.stats().guard_rejects == 0);
    CHECK(f.engine.stats().guard_clamps == 0);
    CHECK(f.robot->last.v == f.engine.result().v);
    CHECK(f.robot->last.q == f.engine.result().q_next);
}

TEST_CASE("guard: reject a velocity violation and write nothing") {
    Fixture f = make({}, 1000.0);
    f.target_j1(1.0);
    Status s = f.engine.step(kDt);
    REQUIRE_FALSE(s);
    CHECK(s.error().code == ErrorCode::out_of_range);
    INFO(s.error().message);
    CHECK(contains(s.error().message, "engine guard: joint j1 velocity "));
    CHECK(contains(s.error().message, "exceeds limit 1.500000"));
    CHECK(f.robot->writes == 0);
    CHECK(f.engine.stats().guard_rejects == 1);
    CHECK(f.engine.stats().cycles == 0);
    CHECK(f.engine.result().v[f.iv("j1")] > 1.5);  // the raw solve is still visible
}

TEST_CASE("guard: each rejected cycle is logged once and run() stops after max_failed_cycles") {
    std::vector<std::string> logged;
    Context context = Context::defaults();
    context.log = Logger(make_callback_sink([&](const LogRecord& r) { logged.emplace_back(r.message); }),
                         LogLevel::error);
    Fixture f = make({}, 1000.0, 0.0, context);
    f.target_j1(1.0);
    Status s = f.engine.run(1.0);
    REQUIRE_FALSE(s);
    REQUIRE(logged.size() == 5);
    CHECK(logged.back() == s.error().message);
    CHECK(f.robot->writes == 0);
    CHECK(f.engine.stats().guard_rejects == 5);
    CHECK(f.engine.stats().failed_cycles == 5);
}

TEST_CASE("guard: velocity_scale tightens the limit") {
    CommandGuardOptions g;
    g.velocity_scale = 0.5;
    Fixture f = make(g);
    f.target_j1(1.0);
    Status s = f.engine.step(kDt);
    REQUIRE_FALSE(s);
    CHECK(contains(s.error().message, "exceeds limit 0.750000"));
}

TEST_CASE("guard: reject a position violation and write nothing") {
    // Measured j1 = 1.99; prepare() moves it to 2.003, the solver pulls back only halfway (limit_gain 0.5).
    Fixture f = make({}, 0.0, 1.99);
    f.target_j1(2.003);
    f.probe->write_index = f.iq("j1");
    f.probe->write_value = 2.003;
    Status s = f.engine.step(kDt);
    REQUIRE_FALSE(s);
    INFO(s.error().message);
    CHECK(contains(s.error().message, "engine guard: joint j1 position 2.0015"));
    CHECK(contains(s.error().message, "outside [-2.000000, 2.000000]"));
    CHECK(f.robot->writes == 0);
    CHECK(f.engine.stats().guard_rejects == 1);
}

TEST_CASE("guard: a joint measured outside its range may move back") {
    Fixture f = make({}, 0.0, 2.05);
    f.target_j1(0.0);
    REQUIRE(f.engine.step(kDt));
    CHECK(f.robot->last.q[f.iq("j1")] < 2.05);
    CHECK(f.engine.stats().guard_rejects == 0);
}

TEST_CASE("guard: values within the tolerance pass") {
    CommandGuardOptions g;
    g.tolerance = 1e-3;
    Fixture f = make(g, 1.5005);
    f.target_j1(1.0);
    REQUIRE(f.engine.step(kDt));
    CHECK(f.robot->last.v[f.iv("j1")] == doctest::Approx(1.5005));

    g.tolerance = 1e-4;
    Fixture tight = make(g, 1.5005);
    tight.target_j1(1.0);
    CHECK_FALSE(tight.engine.step(kDt));
}

TEST_CASE("guard: clamp velocity and re-integrate q") {
    Fixture f = make(mode(GuardMode::clamp), 1000.0);
    f.target_j1(1.0);
    const Eigen::VectorXd q0 = f.engine.state().q;
    REQUIRE(f.engine.step(kDt));
    CHECK(f.engine.result().v[f.iv("j1")] > 1.5);
    const RobotCommand& c = f.robot->last;
    CHECK(c.v[f.iv("j1")] == doctest::Approx(1.5));
    Eigen::VectorXd expect(f.model->nq());
    REQUIRE(f.model->integrate(q0, c.v * kDt, expect));
    CHECK((c.q - expect).cwiseAbs().maxCoeff() < 1e-12);
    CHECK(f.engine.command().q == c.q);
    CHECK(f.engine.stats().guard_clamps == 1);
    REQUIRE(f.engine.step(2 * kDt));
    CHECK(f.engine.stats().guard_clamps == 2);
    CHECK(f.engine.stats().guard_rejects == 0);
}

TEST_CASE("guard: clamp position keeps q_next and v consistent") {
    Fixture f = make(mode(GuardMode::clamp), 0.0, 1.99);
    f.target_j1(2.003);
    f.probe->write_index = f.iq("j1");
    f.probe->write_value = 2.003;
    REQUIRE(f.engine.step(kDt));
    const RobotCommand& c = f.robot->last;
    CHECK(c.q[f.iq("j1")] == doctest::Approx(2.0));
    // Relative to the q prepare() wrote, which the solver integrated from.
    CHECK(2.003 + c.v[f.iv("j1")] * kDt == doctest::Approx(2.0));
    CHECK(f.engine.stats().guard_clamps == 1);
}

TEST_CASE("guard: non-finite steps are rejected before the guard even in clamp mode") {
    auto model =
        Model::load(ModelSource::file((kFixtures / "rotor_velocity.urdf").string()), {}, Context::defaults());
    REQUIRE_MESSAGE(model, message(model));
    SolverOptions so;
    so.backend = [] { return std::make_unique<NanBackend>(); };
    so.check_feasibility = false;
    auto set = ModuleSet::create(*model, so);
    REQUIRE(set);
    REQUIRE(set->add(std::make_shared<Probe>("yaw")));
    auto robot = std::make_shared<RecordingRobot>((*model)->neutral(), (*model)->nv());
    EngineOptions options;
    options.rate_hz = 1.0 / kDt;
    options.realtime = false;
    options.safety = mode(GuardMode::clamp);
    auto e = Engine::create(std::move(*set), robot, options);
    REQUIRE_MESSAGE(e, message(e));

    Status s = e->step(kDt);
    REQUIRE_FALSE(s);
    INFO(s.error().message);
    CHECK(s.error().code == ErrorCode::internal);
    CHECK(contains(s.error().message, "non-finite velocity step (joint yaw = nan)"));
    CHECK(robot->writes == 0);
    CHECK(e->stats().guard_rejects == 0);
}

TEST_CASE("guard: a non-finite QP step is a solver error, also with the guard off") {
    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE_MESSAGE(model, message(model));
    SolverOptions so;
    so.backend = [] { return std::make_unique<NanBackend>(); };
    auto set = ModuleSet::create(*model, so);
    REQUIRE(set);
    REQUIRE(set->add(std::make_shared<Probe>()));
    auto robot = std::make_shared<RecordingRobot>((*model)->neutral(), (*model)->nv());
    EngineOptions options;
    options.rate_hz = 1.0 / kDt;
    options.realtime = false;
    options.safety = mode(GuardMode::off);
    std::vector<std::string> logged;
    Context context = Context::defaults();
    context.log = Logger(make_callback_sink([&](const LogRecord& r) { logged.emplace_back(r.message); }),
                         LogLevel::error);
    auto e = Engine::create(std::move(*set), robot, options, context);
    REQUIRE_MESSAGE(e, message(e));

    Status s = e->step(kDt);
    REQUIRE(logged.size() == 1);
    CHECK(logged[0] == s.error().message);
    REQUIRE_FALSE(s);
    INFO(s.error().message);
    CHECK(s.error().code == ErrorCode::internal);
    CHECK(contains(s.error().message, "non-finite velocity step (joint j1 = nan)"));
    CHECK(robot->writes == 0);
    CHECK_FALSE(e->run(1.0));
    CHECK(robot->writes == 0);
    CHECK(e->stats().cycles == 0);
}

TEST_CASE("guard: off writes the solver output unchanged") {
    Fixture f = make(mode(GuardMode::off), 1000.0);
    f.target_j1(1.0);
    REQUIRE(f.engine.step(kDt));
    CHECK(f.robot->last.v[f.iv("j1")] > 1.5);
    CHECK(f.robot->last.v == f.engine.result().v);
    CHECK(f.engine.stats().guard_rejects == 0);
    CHECK(f.engine.stats().guard_clamps == 0);
}

TEST_CASE("guard: invalid options") {
    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE(model);
    auto check = [&](CommandGuardOptions g, const char* field) {
        auto set = ModuleSet::create(*model);
        REQUIRE(set);
        EngineOptions options;
        options.safety = g;
        auto e = Engine::create(
            std::move(*set), std::make_shared<LoopbackRobot>((*model)->neutral(), (*model)->nv()), options);
        REQUIRE_FALSE(e);
        CHECK(contains(e.error().message, field));
    };
    CommandGuardOptions g;
    g.velocity_scale = 0.0;
    check(g, "velocity_scale");
    g = {};
    g.tolerance = std::numeric_limits<double>::quiet_NaN();
    check(g, "tolerance");
}

namespace {

std::string build_error(const std::string& yaml) {
    auto doc = parse_yaml(yaml, "t.yaml");
    REQUIRE_MESSAGE(doc, message(doc));
    auto e = build_engine(*doc, kFixtures);
    REQUIRE_FALSE(e);
    return e.error().message;
}

}  // namespace

TEST_CASE("guard: yaml safety section") {
    auto doc = parse_yaml(
        "model: {file: arm.urdf}\nrealtime: false\n"
        "safety: {mode: clamp, velocity_scale: 0.5, tolerance: 1e-5}\n",
        "t.yaml");
    REQUIRE_MESSAGE(doc, message(doc));
    auto e = build_engine(*doc, kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    const CommandGuardOptions& g = e->options().safety;
    CHECK(g.mode == GuardMode::clamp);
    CHECK(g.velocity_scale == 0.5);
    CHECK(g.tolerance == 1e-5);

    const std::string head = "model: {file: arm.urdf}\n";
    std::string err = build_error(head + "safety: {velocity_scale: 0}\n");
    CHECK_MESSAGE(contains(err, "t.yaml:2:26: safety.velocity_scale: must be > 0"), err);
    err = build_error(head + "safety: {tolerance: -1}\n");
    CHECK_MESSAGE(contains(err, "t.yaml:2:21: safety.tolerance: must be >= 0"), err);
    err = build_error(head + "safety: {mode: fast}\n");
    CHECK_MESSAGE(contains(err, "t.yaml:2:16: safety.mode"), err);
    err = build_error(head + "safety: {limit: 1}\n");
    CHECK_MESSAGE(contains(err, "t.yaml:2:17: safety.limit"), err);
}

TEST_CASE("pause: paused cycles neither solve nor write, step_paused does both without a resync") {
    Fixture f = make({});
    f.target_j1(1.5);
    for (int k = 1; k <= 30; ++k) REQUIRE(f.engine.step(k * kDt));
    REQUIRE(f.robot->last.v[f.iv("j1")] > 0.5);
    const int steps = f.probe->steps;
    const int writes = f.robot->writes;
    const int resets = f.probe->resets;
    const double module_now = f.probe->last_now;

    f.engine.set_paused(true);
    CHECK(f.engine.paused());
    for (int k = 31; k <= 130; ++k) REQUIRE(f.engine.step(k * kDt));
    CHECK(f.robot->writes == writes);
    CHECK(f.probe->steps == steps);
    CHECK(f.engine.stats().held_cycles == 100);
    CHECK(f.engine.stats().cycles == 130);

    // Each requested step solves and writes once from the ideal state; module time skips the held span.
    f.engine.step_paused(2);
    REQUIRE(f.engine.step(131 * kDt));
    CHECK(f.probe->last_now == doctest::Approx(module_now + kDt));
    REQUIRE(f.engine.step(132 * kDt));
    REQUIRE(f.engine.step(133 * kDt));
    CHECK(f.probe->steps == steps + 2);
    CHECK(f.robot->writes == writes + 2);
    CHECK(f.probe->resets == resets);
    CHECK(f.engine.stats().held_cycles == 101);

    // Resuming resyncs the ideal state to the measured one and resets the modules.
    const auto resyncs = f.engine.stats().resyncs;
    f.engine.step_paused(5);
    f.engine.set_paused(false);
    CHECK_FALSE(f.engine.paused());
    REQUIRE(f.engine.step(134 * kDt));
    CHECK(f.probe->resets == resets + 1);
    CHECK(f.engine.stats().resyncs == resyncs + 1);
    CHECK(f.probe->last_now == doctest::Approx(module_now + 3 * kDt));
    CHECK(f.robot->writes == writes + 3);
    CHECK(f.engine.stats().held_cycles == 101);
}

TEST_CASE("pause: set_paused and step_paused from another thread") {
    Fixture f = make({});
    f.target_j1(1.0);
    std::atomic<bool> stop{false};
    std::thread control([&] {
        for (int k = 0; k < 20; ++k) {
            f.engine.set_paused(k % 2 == 0);
            f.engine.step_paused(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        stop = true;
    });
    const Status s = f.engine.run(0.0, &stop);
    control.join();
    CHECK(s);
    CHECK(f.engine.stats().cycles > 0);
}
