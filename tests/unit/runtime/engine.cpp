#include "kstone/runtime/engine.hpp"

#include "kstone/core/clock.hpp"
#include "kstone/core/resources.hpp"
#include "kstone/model/workspace.hpp"
#include "kstone/module/builtins.hpp"
#include "kstone/module/registry.hpp"
#include "kstone/module/yaml.hpp"
#include "kstone/motion/joint_trajectory.hpp"
#include "kstone/runtime/config.hpp"
#include "kstone/runtime/snapshot.hpp"

#include <doctest/doctest.h>

#include <unistd.h>
#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <sys/wait.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

template <typename T>
std::string message(const Result<T>& r) {
    return r ? std::string{} : r.error().message;
}

Engine load_fixture() {
    auto e = load_engine(kFixtures / "arm_loopback.yaml");
    REQUIRE_MESSAGE(e, message(e));
    return std::move(*e);
}

}  // namespace

TEST_CASE("load_engine: includes and variables") {
    EngineLoadOptions options;
    options.yaml.vars["RATE"] = "200";
    auto e = load_engine(kFixtures / "arm_include.yaml", options);
    REQUIRE_MESSAGE(e, message(e));
    CHECK(e->options().rate_hz == 200.0);
    CHECK(e->modules().modules().size() == 2);
}

namespace {

double joint_q(Engine& e, const Eigen::VectorXd& q, const char* name) {
    const Model& m = *e.modules().model();
    return *m.joint_position(q, *m.joint_id(name));
}

std::string build_error(const std::string& yaml) {
    auto doc = parse_yaml(yaml, "t.yaml");
    REQUIRE_MESSAGE(doc, message(doc));
    auto e = build_engine(*doc, kFixtures);
    REQUIRE_FALSE(e);
    return e.error().message;
}

// Fails reads or writes on demand.
class FlakyRobot final : public RobotInterface {
  public:
    FlakyRobot(Eigen::VectorXd q, Eigen::Index nv) : inner_(std::move(q), nv) {}
    Status read(RobotState& state) override {
        if (fail_read) return make_error(ErrorCode::io_error, "read failed");
        return inner_.read(state);
    }
    Status write(const RobotCommand& command) override {
        ++writes;
        if (fail_write) return make_error(ErrorCode::io_error, "write failed");
        return inner_.write(command);
    }
    bool fail_read{false};
    bool fail_write{false};
    int writes{0};

  private:
    LoopbackRobot inner_;
};
}  // namespace

TEST_CASE("engine: config fixture drives a joint target to convergence") {
    Engine e = load_fixture();
    CHECK(e.dt() == doctest::Approx(0.01));
    CHECK_FALSE(e.options().realtime);
    CHECK(e.modules().modules().size() == 3);

    auto arm = std::dynamic_pointer_cast<JointTargetModule>(e.modules().module("arm"));
    REQUIRE(arm);
    REQUIRE(e.run(0.05));
    CHECK(e.stats().cycles == 5);
    CHECK(e.stats().time == doctest::Approx(0.05));

    Eigen::VectorXd target(3);
    target << 0.5, -0.3, 1.0;
    REQUIRE(arm->set_target(target));
    for (int i = 0; i < 2000; ++i) REQUIRE(e.step(e.stats().time + e.dt()));
    CHECK(e.stats().cycles == 2005);
    CHECK(e.stats().time == doctest::Approx(20.05));
    CHECK(arm->settled());
    const Eigen::VectorXd& q = e.state().q;
    CHECK(joint_q(e, q, "j1") == doctest::Approx(0.5).epsilon(1e-3));
    CHECK(joint_q(e, q, "j2") == doctest::Approx(-0.3).epsilon(1e-3));
    CHECK_FALSE(e.modules().module("tool")->running());  // the claim outlives arrival
    CHECK(e.stats().overruns == 0);
}

TEST_CASE("engine: a joint target ramps up instead of stepping to the velocity limit") {
    Engine e = load_fixture();
    auto arm = std::dynamic_pointer_cast<JointTargetModule>(e.modules().module("arm"));
    REQUIRE(arm);
    REQUIRE(e.run(0.02));
    Eigen::VectorXd target(3);
    target << 1.0, 0.0, 0.0;
    REQUIRE(arm->set_target(target));
    double before = joint_q(e, e.state().q, "j1");
    double v_prev = 0.0;
    for (int i = 0; i < 5; ++i) {
        REQUIRE(e.run(0.01));
        const double now = joint_q(e, e.state().q, "j1");
        const double v = (now - before) / 0.01;
        CAPTURE(i);
        CHECK(std::abs(v - v_prev) / 0.01 <= 2.0 * 1.5 * 1.01);  // acceleration_factor * velocity limit
        before = now;
        v_prev = v;
    }
}

TEST_CASE("engine: all-continuous arm crosses +-pi on the short arc") {
    auto doc = parse_yaml(R"(
model: {file: continuous_arm.urdf}
rate_hz: 100
feedback_timeout_s: 0
robot: {type: loopback}
modules:
  tool:
    type: frame_target
    frame: tool
    trajectory:
      position: {velocity: 0.2, acceleration: 1, jerk: 10}
      orientation: {velocity: 1, acceleration: 2, jerk: 20}
  ns: {type: null_space, primary: tool}
  joints:
    type: joint_target
    joints: [c1, c2, c3, c4]
    priority: 1
    home: [3.0, 3.0, 3.0, 3.0]
)",
                          "t.yaml");
    REQUIRE_MESSAGE(doc, message(doc));
    auto built = build_engine(*doc, kFixtures);
    REQUIRE_MESSAGE(built, message(built));
    Engine& e = *built;
    const ModelPtr& model = e.modules().model();
    const char* names[] = {"c1", "c2", "c3", "c4"};
    for (const char* n : names) CHECK_FALSE(model->has_position_limit(*model->joint_id(n)));

    // Unbounded joints: every null-space preference still configures and switches.
    auto ns = std::dynamic_pointer_cast<NullSpaceModule>(e.modules().module("ns"));
    REQUIRE(ns);
    CHECK(ns->preference() == NullSpacePreference::posture);
    CHECK(ns->tune(ConfigNode::map({{"preference", "manipulability"}})));
    CHECK(ns->preference() == NullSpacePreference::manipulability);
    CHECK(ns->tune(ConfigNode::map({{"preference", "posture"}})));

    auto joints = std::dynamic_pointer_cast<JointTargetModule>(e.modules().module("joints"));
    REQUIRE(joints);
    double now = 0.0;
    // Per-joint signed travel, summed from wrapped per-cycle increments; checks each increment's speed.
    const auto drive = [&](auto&& busy) {
        Eigen::Vector4d travel = Eigen::Vector4d::Zero();
        for (int k = 0; k < 2000 && busy(); ++k) {
            Eigen::Vector4d before;
            for (int i = 0; i < 4; ++i) before[i] = joint_q(e, e.state().q, names[i]);
            now += 0.01;
            REQUIRE(e.step(now));
            for (int i = 0; i < 4; ++i) {
                const double d = std::remainder(joint_q(e, e.state().q, names[i]) - before[i], 2 * M_PI);
                const JointInfo& j = model->joints()[static_cast<std::size_t>(*model->joint_id(names[i]))];
                CHECK(std::abs(d) <= model->velocity_limit()[j.idx_v] * 0.01 + 1e-9);
                travel[i] += d;
            }
        }
        CHECK_FALSE(busy());
        return travel;
    };
    const auto at = [&](double angle) {
        for (const char* n : names)
            CHECK(std::abs(std::remainder(joint_q(e, e.state().q, n) - angle, 2 * M_PI)) < 1e-3);
    };

    REQUIRE(e.step(now));
    REQUIRE(joints->set_target(Eigen::Vector4d::Constant(3.0)));
    drive([&] { return !joints->settled(); });
    at(3.0);

    // 3.0 -> -3.0 through pi: +0.28 rad, not -6 rad back through 0.
    REQUIRE(joints->set_target(Eigen::Vector4d::Constant(-3.0)));
    Eigen::Vector4d travel = drive([&] { return !joints->settled(); });
    at(-3.0);
    for (int i = 0; i < 4; ++i) CHECK(travel[i] == doctest::Approx(2 * M_PI - 6.0).epsilon(1e-3));

    // A target a full turn away is the same posture: no motion.
    REQUIRE(joints->set_target(Eigen::Vector4d::Constant(-3.0 + 2 * M_PI)));
    travel = drive([&] { return !joints->settled(); });
    CHECK(travel.cwiseAbs().maxCoeff() < 1e-3);

    // Home at 3.0 crosses pi back.
    REQUIRE(joints->request_home());
    travel = drive([&] { return joints->home_status() != HomeStatus::succeeded; });
    at(3.0);
    for (int i = 0; i < 4; ++i) CHECK(travel[i] == doctest::Approx(6.0 - 2 * M_PI).epsilon(1e-3));

    // Cartesian target solved over the unbounded joints; the goal is a reachable pose across pi.
    auto tool = std::dynamic_pointer_cast<FrameTargetModule>(e.modules().module("tool"));
    REQUIRE(tool);
    Workspace ws(model);
    const FrameId frame = *model->frame_id("tool");
    Eigen::VectorXd q_goal(model->nq());
    REQUIRE(model->integrate(e.state().q, Eigen::Vector4d(0.3, -0.2, 0.25, 0.3), q_goal));
    Eigen::Isometry3d goal;
    REQUIRE(ws.update(q_goal));
    REQUIRE(ws.frame_pose(frame, goal));
    REQUIRE(tool->set_target(goal));
    // An idle joint_target still holds its posture; release it so only the tool and ns remain.
    joints->set_enabled(false);
    for (int k = 0; k < 500; ++k) REQUIRE(e.step(now += 0.01));
    Eigen::Isometry3d reached;
    REQUIRE(ws.update(e.state().q));
    REQUIRE(ws.frame_pose(frame, reached));
    CHECK((reached.translation() - goal.translation()).norm() < 1e-3);

    CHECK(e.stats().guard_rejects == 0);
    CHECK(e.stats().guard_clamps == 0);
}

TEST_CASE("engine: revolute joint limited beyond +-pi moves multi-turn without wrapping") {
    auto doc = parse_yaml(R"(
model: {file: multiturn_joint.urdf}
rate_hz: 100
feedback_timeout_s: 0
robot: {type: loopback}
modules:
  joint: {type: joint_target, joints: [m1]}
)",
                          "t.yaml");
    REQUIRE_MESSAGE(doc, message(doc));
    auto built = build_engine(*doc, kFixtures);
    REQUIRE_MESSAGE(built, message(built));
    Engine& e = *built;
    const ModelPtr& model = e.modules().model();
    const JointId m1 = *model->joint_id("m1");
    CHECK(model->joints()[static_cast<std::size_t>(m1)].nq == 1);
    CHECK(model->has_position_limit(m1));

    auto joint = std::dynamic_pointer_cast<JointTargetModule>(e.modules().module("joint"));
    REQUIRE(joint);
    double now = 0.0;
    // Scalar q is not wrapped: travel is the plain difference and must match the target exactly.
    const auto drive = [&](double target) {
        REQUIRE(joint->set_target(Eigen::VectorXd::Constant(1, target)));
        double travel = 0.0;
        for (int k = 0; k < 2000 && !joint->settled(); ++k) {
            const double before = joint_q(e, e.state().q, "m1");
            REQUIRE(e.step(now += 0.01));
            const double d = joint_q(e, e.state().q, "m1") - before;
            CHECK(std::abs(d) <= 2.0 * 0.01 + 1e-9);
            travel += d;
        }
        CHECK(joint->settled());
        CHECK(joint_q(e, e.state().q, "m1") == doctest::Approx(target).epsilon(1e-4));
        return travel;
    };

    REQUIRE(e.step(now));
    // 0 -> 5 goes the long way through pi (a continuous joint would take -1.28 rad).
    CHECK(drive(5.0) == doctest::Approx(5.0).epsilon(1e-3));
    // 5 -> -5 is a full -10 rad, not the 2.57 rad short arc.
    CHECK(drive(-5.0) == doctest::Approx(-10.0).epsilon(1e-3));
    Eigen::VectorXd ext(1);
    auto map = NameMap::create(model, {"m1"});
    REQUIRE(map);
    REQUIRE(map->gather_positions(e.state().q, ext));
    CHECK(ext[0] == doctest::Approx(-5.0).epsilon(1e-4));
    CHECK(e.stats().guard_rejects == 0);
    CHECK(e.stats().guard_clamps == 0);
}

TEST_CASE("engine: stop flag ends an unbounded run") {
    Engine e = load_fixture();
    std::atomic<bool> stop{true};
    REQUIRE(e.run(0.0, &stop));
    CHECK(e.stats().cycles == 0);
    CHECK_FALSE(e.run(0.0));
}

TEST_CASE("engine: observers run around every step on the calling thread") {
    struct Counter final : EngineObserver {
        void before_step(Engine& e) override {
            ++before;
            seen_cycles = e.stats().cycles;
            if (before == 3) e.modules().module("tool")->set_enabled(false);
        }
        void after_step(const Engine& e) override {
            ++after;
            CHECK(e.stats().cycles == seen_cycles + 1);
        }
        int before{0}, after{0};
        std::uint64_t seen_cycles{0};
    };
    Engine e = load_fixture();
    auto counter = std::make_shared<Counter>();
    e.add_observer(counter);
    e.add_observer(nullptr);
    REQUIRE(e.run(0.05));
    CHECK(counter->before == 5);
    CHECK(counter->after == 5);
    CHECK_FALSE(e.modules().module("tool")->running());
}

#if defined(__linux__)
namespace {
std::vector<int> allowed_cpus() {
    cpu_set_t set;
    CPU_ZERO(&set);
    REQUIRE(pthread_getaffinity_np(pthread_self(), sizeof(set), &set) == 0);
    std::vector<int> out;
    for (int c = 0; c < CPU_SETSIZE; ++c)
        if (CPU_ISSET(static_cast<std::size_t>(c), &set)) out.push_back(c);
    return out;
}

int current_policy() {
    int policy = -1;
    sched_param p{};
    REQUIRE(pthread_getschedparam(pthread_self(), &policy, &p) == 0);
    return policy;
}
}  // namespace

TEST_CASE("engine: thread.cpus pins run() and restores the affinity") {
    struct Probe final : EngineObserver {
        void before_step(Engine&) override { during = allowed_cpus(); }
        std::vector<int> during;
    };
    const std::vector<int> before = allowed_cpus();
    REQUIRE_FALSE(before.empty());
    auto doc = parse_yaml(
        "model: {file: arm.urdf}\nrealtime: true\nthread: {cpus: [" + std::to_string(before.back()) + "]}\n",
        "t.yaml");
    REQUIRE(doc);
    auto e = build_engine(*doc, kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    CHECK(e->options().thread.cpus == std::vector<int>{before.back()});
    auto probe = std::make_shared<Probe>();
    e->add_observer(probe);
    REQUIRE(e->run(0.03));
    CHECK(probe->during == std::vector<int>{before.back()});
    CHECK(allowed_cpus() == before);
}

TEST_CASE("engine: thread options are ignored when not realtime") {
    struct Probe final : EngineObserver {
        void before_step(Engine&) override { policy = current_policy(); }
        int policy{-1};
    };
    std::vector<std::string> warnings;
    Context context = Context::defaults();
    context.log = Logger(make_callback_sink([&](const LogRecord& r) {
                             if (r.level == LogLevel::warn) warnings.emplace_back(r.message);
                         }),
                         LogLevel::warn);
    const int before = current_policy();
    auto doc = parse_yaml("model: {file: arm.urdf}\nrealtime: false\nthread: {priority: 10}\n", "t.yaml");
    REQUIRE(doc);
    auto e = build_engine(*doc, kFixtures, {}, context);
    REQUIRE_MESSAGE(e, message(e));
    auto probe = std::make_shared<Probe>();
    e->add_observer(probe);
    REQUIRE(e->run(0.03));
    CHECK(probe->policy == before);
    CHECK(warnings.empty());
}

TEST_CASE("engine: elevate_thread_limits is a no-op without real-time needs and rejects bad options") {
    CHECK(elevate_thread_limits(ThreadOptions{}, Logger{}));
    ThreadOptions pinned;
    pinned.cpus = {0};
    CHECK(elevate_thread_limits(pinned, Logger{}));
    ThreadOptions bad;
    bad.priority = 1000;
    const Status s = elevate_thread_limits(bad, Logger{});
    REQUIRE_FALSE(s);
    CHECK(s.error().code == ErrorCode::invalid_argument);
}

TEST_CASE("engine: thread.priority switches run() to SCHED_FIFO or warns and keeps running") {
    struct Probe final : EngineObserver {
        void before_step(Engine&) override { policy = current_policy(); }
        int policy{-1};
    };
    std::vector<std::string> warnings;
    Context context = Context::defaults();
    context.log = Logger(make_callback_sink([&](const LogRecord& r) {
                             if (r.level == LogLevel::warn) warnings.emplace_back(r.message);
                         }),
                         LogLevel::warn);
    const int before = current_policy();
    auto doc = parse_yaml("model: {file: arm.urdf}\nrealtime: true\nthread: {priority: 10}\n", "t.yaml");
    REQUIRE(doc);
    auto rt = build_engine(*doc, kFixtures, {}, context);
    REQUIRE_MESSAGE(rt, message(rt));
    auto probe = std::make_shared<Probe>();
    rt->add_observer(probe);
    REQUIRE(rt->run(0.03));
    CHECK(rt->stats().cycles > 0);
    if (warnings.empty()) {
        CHECK(probe->policy == SCHED_FIFO);
    } else {
        CHECK(probe->policy == before);
        CHECK_MESSAGE(warnings[0].find("ulimit -r") != std::string::npos, warnings[0]);
    }
    CHECK(current_policy() == before);
}

#if !defined(__SANITIZE_ADDRESS__) && !defined(__SANITIZE_THREAD__)
// Sanitizer shadow memory makes mlockall unusable, and fork is unreliable under tsan.
long locked_kb() {
    std::ifstream status("/proc/self/status");
    for (std::string line; std::getline(status, line);)
        if (line.rfind("VmLck:", 0) == 0) return std::stol(line.substr(6));
    return -1;
}

TEST_CASE("engine: thread.lock_memory locks the process or warns and keeps running") {
    // mlockall outlives run(), so it runs in a child process. Exit 0 = locked, 1 = warned, others = failure.
    const pid_t pid = fork();
    REQUIRE(pid >= 0);
    if (pid == 0) {
        std::vector<std::string> warnings;
        Context context = Context::defaults();
        context.log = Logger(make_callback_sink([&](const LogRecord& r) {
                                 if (r.level == LogLevel::warn) warnings.emplace_back(r.message);
                             }),
                             LogLevel::warn);
        auto doc =
            parse_yaml("model: {file: arm.urdf}\nrealtime: true\nthread: {lock_memory: true}\n", "t.yaml");
        if (!doc) _exit(10);
        auto rt = build_engine(*doc, kFixtures, {}, context);
        if (!rt || locked_kb() != 0) _exit(11);
        if (!rt->run(0.03) || rt->stats().cycles == 0) _exit(12);
        if (warnings.empty()) _exit(locked_kb() > 0 ? 0 : 13);
        _exit(warnings[0].find("ulimit -l") != std::string::npos && locked_kb() == 0 ? 1 : 14);
    }
    int status = 0;
    REQUIRE(waitpid(pid, &status, 0) == pid);
    REQUIRE(WIFEXITED(status));
    const int code = WEXITSTATUS(status);
    MESSAGE((code == 0 ? "memory locked" : "lock refused, warned"));
    CHECK_MESSAGE((code == 0 || code == 1), "child exit ", code);
}
#endif
#endif

TEST_CASE("engine: invalid thread options are rejected") {
    std::string m = build_error("model: {file: arm.urdf}\nthread: {priority: 100}\n");
    CHECK_MESSAGE(m.find("thread.priority: must be <= 99") != std::string::npos, m);
    m = build_error("model: {file: arm.urdf}\nthread: {cpus: [-1]}\n");
    CHECK_MESSAGE(m.find("thread.cpus[0]: must be >= 0") != std::string::npos, m);
#if defined(__linux__)
    m = build_error("model: {file: arm.urdf}\nthread: {cpus: [0, 0]}\n");
    CHECK_MESSAGE(m.find("thread.cpus lists 0 twice") != std::string::npos, m);
    m = build_error("model: {file: arm.urdf}\nthread: {cpus: [100000]}\n");
    CHECK_MESSAGE(m.find("outside [0, ") != std::string::npos, m);
#else
    m = build_error("model: {file: arm.urdf}\nthread: {cpus: [0]}\n");
    CHECK_MESSAGE(m.find("only supported on Linux") != std::string::npos, m);
#endif
}

TEST_CASE("engine: set_rate_hz switches dt at the next cycle and keeps time continuous") {
    struct Switch final : EngineObserver {
        void before_step(Engine& e) override {
            if (++n == 2) CHECK(e.set_rate_hz(50.0));
        }
        void after_step(const Engine& e) override { times.push_back(e.stats().time); }
        int n{0};
        std::vector<double> times;
    };
    Engine e = load_fixture();
    REQUIRE(e.options().rate_hz == 100.0);
    CHECK_FALSE(e.set_rate_hz(0.0));
    CHECK_FALSE(e.set_rate_hz(std::nan("")));
    auto sw = std::make_shared<Switch>();
    e.add_observer(sw);
    REQUIRE(e.run(0.1));
    CHECK(e.options().rate_hz == 50.0);
    CHECK(e.dt() == doctest::Approx(0.02));
    CHECK(e.stats().cycles == 6);  // 0.01, 0.02, then 0.04 .. 0.1
    CHECK(e.stats().time == doctest::Approx(0.1));
    REQUIRE(sw->times.size() == 6);
    CHECK(sw->times[2] == doctest::Approx(0.04));
    REQUIRE(e.set_rate_hz(200.0));
    REQUIRE(e.run(0.01));
    CHECK(e.stats().cycles == 8);
    CHECK(e.stats().time == doctest::Approx(0.11));
}

TEST_CASE("engine: run() paces cycles to the clock") {
    auto doc = load_yaml(kFixtures / "arm_loopback.yaml");
    REQUIRE(doc);
    ConfigNode cfg = *doc;
    cfg.set("rate_hz", 200.0);
    auto e = build_engine(cfg, kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    const auto start = std::chrono::steady_clock::now();
    REQUIRE(e->run(0.1));
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    CHECK(e->stats().cycles == 20);
    // Lower bound is strict; the upper bound only guards against runaway sleeps on loaded CI machines.
    CHECK(elapsed >= 0.095);
    CHECK(elapsed < 2.0);
    CHECK(e->stats().max_cycle_s > 0.0);
}

TEST_CASE("engine: a stop request interrupts a long pacing sleep") {
    auto doc = load_yaml(kFixtures / "arm_loopback.yaml");
    REQUIRE(doc);
    ConfigNode cfg = *doc;
    cfg.set("rate_hz", 0.01);
    auto e = build_engine(cfg, kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    std::atomic<bool> stop{false};
    const auto start = std::chrono::steady_clock::now();
    std::thread stopper([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        stop = true;
    });
    const Status s = e->run(0.0, &stop);
    stopper.join();
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    REQUIRE(s);
    CHECK(e->stats().cycles == 1);
    CHECK(elapsed < 2.0);
}

TEST_CASE("engine: robot errors abort the cycle without writing") {
    auto doc = load_yaml(kFixtures / "arm_loopback.yaml");
    REQUIRE(doc);
    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE(model);
    auto robot = std::make_shared<FlakyRobot>((*model)->neutral(), (*model)->nv());
    EngineLoadOptions options;
    options.robot = robot;
    auto e = build_engine(*doc, kFixtures, options);
    REQUIRE_MESSAGE(e, message(e));
    REQUIRE(e->run(0.02));
    CHECK(robot->writes == 2);
    robot->fail_read = true;
    Status s = e->run(1.0);
    REQUIRE_FALSE(s);
    CHECK(s.error().code == ErrorCode::io_error);
    CHECK(robot->writes == 2);
    CHECK(e->stats().cycles == 2);
    CHECK(e->stats().failed_cycles == 5);

    auto wrong = std::make_shared<LoopbackRobot>(Eigen::VectorXd::Zero(3), 3);
    auto set = ModuleSet::create(*model);
    REQUIRE(set);
    auto bad = Engine::create(std::move(*set), wrong);
    REQUIRE_FALSE(bad);
    CHECK(bad.error().message.find("nq=3") != std::string::npos);
}

TEST_CASE("engine: the chassis odometry timeout follows feedback_timeout_s") {
    ModelOptions free;
    free.root = RootJoint::free_flyer;
    auto model =
        Model::load(ModelSource::file((kFixtures / "diff_chassis.urdf").string()), free, Context::defaults());
    REQUIRE(model);
    auto set = ModuleSet::create(*model);
    REQUIRE(set);
    ChassisModuleConfig cc;
    cc.period_s = 0.01;
    auto ch = std::make_shared<ChassisModule>("chassis", cc);
    REQUIRE(set->add(ch));
    EngineOptions options;
    options.feedback_timeout_s = 0.4;
    auto e = Engine::create(std::move(*set),
                            std::make_shared<LoopbackRobot>((*model)->neutral(), (*model)->nv()), options);
    REQUIRE_MESSAGE(e, message(e));
    CHECK(ch->controller()->config().odom_timeout_s == 0.4);
}

TEST_CASE("engine: robot quaternions are normalized within 1e-3, rejected beyond") {
    ModelOptions free;
    free.root = RootJoint::free_flyer;
    auto model = Model::load(ModelSource::file((kFixtures / "floating_base.urdf").string()), free,
                             Context::defaults());
    REQUIRE(model);
    for (const double scale : {1.0 + 1e-6, 1.0 - 5e-4, 1.5}) {
        Eigen::VectorXd q = (*model)->neutral();
        q.segment<4>(3) *= scale;
        auto set = ModuleSet::create(*model);
        REQUIRE(set);
        EngineOptions options;
        auto e = Engine::create(std::move(*set), std::make_shared<LoopbackRobot>(q, (*model)->nv()), options);
        if (scale == 1.5) {
            REQUIRE_FALSE(e);
            CHECK(e.error().message.find("robot state: q: joint root_joint has a quaternion of norm 1.5") !=
                  std::string::npos);
            continue;
        }
        REQUIRE_MESSAGE(e, message(e));
        CHECK(std::abs(e->state().q.segment<4>(3).norm() - 1.0) < 1e-12);
        REQUIRE(e->step(0.01));
        CHECK((*model)->validate(e->ideal()));
    }
}

namespace {
// Fails the reads whose 1-based index is listed; read 1 is Engine::create's.
class ScriptedRobot final : public RobotInterface {
  public:
    ScriptedRobot(Eigen::VectorXd q, Eigen::Index nv, std::vector<int> failing)
        : inner_(std::move(q), nv), failing_(std::move(failing)) {}
    Status read(RobotState& state) override {
        ++reads;
        if (std::find(failing_.begin(), failing_.end(), reads) != failing_.end())
            return make_error(ErrorCode::io_error, "read failed");
        return inner_.read(state);
    }
    Status write(const RobotCommand& command) override {
        ++writes;
        return inner_.write(command);
    }
    int reads{0};
    int writes{0};

  private:
    LoopbackRobot inner_;
    std::vector<int> failing_;
};

Result<Engine> scripted_engine(std::shared_ptr<ScriptedRobot>& robot, std::vector<int> failing,
                               int max_failed) {
    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE(model);
    auto set = ModuleSet::create(*model);
    REQUIRE(set);
    robot = std::make_shared<ScriptedRobot>((*model)->neutral(), (*model)->nv(), std::move(failing));
    EngineOptions options;
    options.rate_hz = 100.0;
    options.max_failed_cycles = max_failed;
    return Engine::create(std::move(*set), robot, options);
}
}  // namespace

TEST_CASE("engine: run() continues past isolated failed cycles") {
    std::shared_ptr<ScriptedRobot> robot;
    auto e = scripted_engine(robot, {2, 3, 5, 6, 7, 8}, 5);
    REQUIRE_MESSAGE(e, message(e));
    REQUIRE(e->run(0.1));
    CHECK(robot->reads == 11);
    CHECK(robot->writes == 4);
    CHECK(e->stats().cycles == 4);
    CHECK(e->stats().failed_cycles == 6);
}

TEST_CASE("engine: run() stops after max_failed_cycles consecutive failures") {
    std::shared_ptr<ScriptedRobot> robot;
    auto e = scripted_engine(robot, {2, 4, 5, 6}, 3);
    REQUIRE_MESSAGE(e, message(e));
    Status s = e->run(1.0);
    REQUIRE_FALSE(s);
    CHECK(s.error().code == ErrorCode::io_error);
    CHECK(robot->reads == 6);
    CHECK(e->stats().failed_cycles == 4);

    auto first = scripted_engine(robot, {2}, 1);
    REQUIRE_MESSAGE(first, message(first));
    REQUIRE_FALSE(first->run(1.0));
    CHECK(robot->reads == 2);

    auto bad = scripted_engine(robot, {}, 0);
    REQUIRE_FALSE(bad);
    CHECK(bad.error().message.find("max_failed_cycles") != std::string::npos);
}

TEST_CASE("engine: failed cycles do not advance module trajectories") {
    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE(model);
    auto set = ModuleSet::create(*model);
    REQUIRE(set);
    JointTargetModuleConfig jc;
    jc.joints = {"j1", "j2", "j3"};
    jc.trajectory.trigger = JointTrigger::continuous;
    auto arm = std::make_shared<JointTargetModule>("arm", jc);
    REQUIRE(set->add(arm));
    auto robot = std::make_shared<FlakyRobot>((*model)->neutral(), (*model)->nv());
    EngineOptions options;
    options.feedback_timeout_s = 0.0;  // steps use virtual time; attach timing would follow the wall clock
    auto e = Engine::create(std::move(*set), robot, options);
    REQUIRE_MESSAGE(e, message(e));
    REQUIRE(arm->set_target(Eigen::Vector3d::Constant(1.0)));
    for (int k = 0; k < 20; ++k) REQUIRE(e->step(k * e->dt()));
    const double before = joint_q(*e, e->command().q, "j1");
    REQUIRE(e->step(20 * e->dt()));
    const double last = joint_q(*e, e->command().q, "j1");
    const double nominal = last - before;
    REQUIRE(nominal > 1e-4);

    robot->fail_write = true;
    for (int k = 21; k < 31; ++k) CHECK_FALSE(e->step(k * e->dt()));
    robot->fail_write = false;
    REQUIRE(e->step(31 * e->dt()));
    // Resuming continues from the last written command instead of jumping by the missed cycles.
    CHECK(std::abs(joint_q(*e, e->command().q, "j1") - last) <= 1.5 * nominal);
}

namespace {

// Loopback with a hand-set stamp; counts writes.
class StampedRobot final : public RobotInterface {
  public:
    StampedRobot(Eigen::VectorXd q, Eigen::Index nv) : inner_(std::move(q), nv) {}
    Status read(RobotState& state) override {
        if (auto s = inner_.read(state); !s) return s;
        state.stamp_ns = stamp_ns;
        if (group_stamps.empty())
            std::fill(state.group_stamps_ns.begin(), state.group_stamps_ns.end(), stamp_ns);
        else
            state.group_stamps_ns = group_stamps;
        if (offset.size() == state.q.size()) state.q += offset;
        return {};
    }
    Status write(const RobotCommand& command) override {
        ++writes;
        if (fail_write) return make_error(ErrorCode::io_error, "simulated write failure");
        last_fresh = command.fresh;
        last_groups = command.group_attached;
        return inner_.write(command);
    }
    std::int64_t stamp_ns{-1};
    std::vector<std::int64_t> group_stamps;  ///< Per feedback group when set; else every group gets stamp_ns.
    std::vector<std::uint8_t> last_groups;
    int writes{0};
    bool last_fresh{false};
    bool fail_write{false};
    Eigen::VectorXd offset;  ///< Added to the reported q (simulated drift).

  private:
    LoopbackRobot inner_;
};

struct StampedFixture {
    std::shared_ptr<ManualClock> clock;
    std::shared_ptr<StampedRobot> robot;
    Engine engine;
};

StampedFixture make_stamped(double feedback_timeout_s, std::vector<std::string> feedback_joints = {},
                            bool stamped = true, std::vector<FeedbackGroup> groups = {}, bool outside = false,
                            bool frame = false, bool coupled = false, double velocity_scale = 1.0) {
    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE(model);
    auto set = ModuleSet::create(*model);
    REQUIRE(set);
    JointTargetModuleConfig jc;
    jc.joints = {"j1", "j2", "j3"};
    if (frame) {
        FrameTargetModuleConfig fc;
        fc.frame = "tool";
        fc.joints = jc.joints;
        REQUIRE(set->add(std::make_shared<FrameTargetModule>("arm", fc)));
    } else {
        REQUIRE(set->add(std::make_shared<JointTargetModule>("arm", jc)));
    }
    if (coupled) {
        jc.on_idle = OnIdle::claim;
        REQUIRE(set->add(std::make_shared<JointTargetModule>("arm_idle", jc)));
    }
    jc.joints = {"j4"};
    REQUIRE(set->add(std::make_shared<JointTargetModule>("wrist", jc)));
    auto clock = std::make_shared<ManualClock>(1'000'000'000);
    Eigen::VectorXd initial = (*model)->neutral();
    if (outside) {
        initial[(*model)->joints()[*(*model)->joint_id("j1")].idx_q] = 2.05;
        initial[(*model)->joints()[*(*model)->joint_id("j2")].idx_q] = -1.55;
        initial[(*model)->joints()[*(*model)->joint_id("j4")].idx_q] = -0.02;
    }
    auto robot = std::make_shared<StampedRobot>(initial, (*model)->nv());
    robot->stamp_ns = stamped ? clock->now_ns() : -1;
    Context context = Context::defaults();
    context.clock = clock;
    EngineOptions options;
    options.feedback_timeout_s = feedback_timeout_s;
    options.feedback_joints = std::move(feedback_joints);
    options.feedback_groups = std::move(groups);
    options.safety.velocity_scale = velocity_scale;
    auto e = Engine::create(std::move(*set), robot, options, context);
    REQUIRE_MESSAGE(e, message(e));
    return {clock, robot, std::move(*e)};
}

}  // namespace

TEST_CASE("engine: stale or unstamped feedback still writes, flagged not fresh") {
    StampedFixture f = make_stamped(0.05);
    REQUIRE(f.engine.step(0.01));
    CHECK(f.robot->writes == 1);
    CHECK(f.engine.fresh());
    CHECK_FALSE(f.robot->last_fresh);
    CHECK(f.engine.measured() == nullptr);

    f.clock->advance_ns(50'000'000);
    f.robot->stamp_ns = f.clock->now_ns();
    REQUIRE(f.engine.step(0.02));
    CHECK(f.robot->last_fresh);
    CHECK(f.engine.measured() != nullptr);

    f.clock->advance_ns(60'000'000);
    REQUIRE(f.engine.step(0.03));
    CHECK(f.engine.written());
    CHECK_FALSE(f.engine.fresh());
    CHECK_FALSE(f.engine.command().fresh);
    CHECK_FALSE(f.robot->last_fresh);
    CHECK(f.engine.measured() == nullptr);
    CHECK(f.robot->writes == 3);
    CHECK(f.engine.stats().stale_cycles == 1);

    f.robot->stamp_ns = -1;
    REQUIRE(f.engine.step(0.04));
    CHECK(f.robot->writes == 4);
    CHECK_FALSE(f.robot->last_fresh);
    CHECK(f.engine.stats().stale_cycles == 2);

    // Fresh again, but not flagged fresh until feedback has stayed fresh for the whole timeout.
    f.robot->stamp_ns = f.clock->now_ns() - 10'000'000;
    REQUIRE(f.engine.step(0.05));
    CHECK(f.robot->writes == 5);
    CHECK(f.engine.fresh());
    CHECK_FALSE(f.robot->last_fresh);
    f.clock->advance_ns(50'000'000);
    f.robot->stamp_ns = f.clock->now_ns();
    REQUIRE(f.engine.step(0.06));
    CHECK(f.robot->last_fresh);
}

namespace {
Eigen::Index iq(const Engine& e, const char* joint) {
    const Model& m = *e.modules().model();
    return m.joints()[*m.joint_id(joint)].idx_q;
}
}  // namespace

namespace {
Eigen::VectorXd clipped_feedback(const Engine& engine) {
    const auto& model = *engine.modules().model();
    Eigen::VectorXd q = engine.state().q;
    for (const auto& joint : model.joints())
        if (joint.nq == 1 && joint.nv == 1 && model.has_position_limit(joint.id))
            q[joint.idx_q] = std::clamp(q[joint.idx_q], model.lower_position_limit()[joint.idx_q],
                                        model.upper_position_limit()[joint.idx_q]);
    return q;
}
void check_clipped_targets(const Engine& engine) {
    const Eigen::VectorXd clipped = clipped_feedback(engine);
    CHECK(engine.command().q.isApprox(clipped, 1e-12));
    CHECK(engine.ideal().isApprox(clipped, 1e-12));
    CHECK(engine.command().v.isZero(1e-12));
    if (auto frame = std::dynamic_pointer_cast<FrameTargetModule>(engine.modules().module("arm"))) {
        Workspace at(engine.modules().model());
        REQUIRE(at.update(clipped));
        Eigen::Isometry3d expected;
        REQUIRE(at.frame_pose(frame->frame_id(), expected));
        CHECK(frame->target().matrix().isApprox(expected.matrix(), 1e-12));
        CHECK(frame->trajectory()->ramp_pose().matrix().isApprox(expected.matrix(), 1e-12));
    } else {
        auto arm = std::dynamic_pointer_cast<JointTargetModule>(engine.modules().module("arm"));
        REQUIRE(arm);
        CHECK(arm->target()[0] == doctest::Approx(clipped[iq(engine, "j1")]));
        CHECK(arm->target()[1] == doctest::Approx(clipped[iq(engine, "j2")]));
    }
}
}  // namespace

TEST_CASE("engine: synchronization transition starts raw and follows actual Ruckig to completion") {
    for (const bool frame : {false, true}) {
        for (const double scale : {1.0, 0.25}) {
            CAPTURE(frame);
            CAPTURE(scale);
            StampedFixture f = make_stamped(0.0, {}, true, {}, true, frame, false, scale);
            const auto& model = *f.engine.modules().model();
            const Eigen::VectorXd raw = f.engine.state().q;
            const Eigen::VectorXd target = clipped_feedback(f.engine);
            REQUIRE_FALSE(raw.isApprox(target, 1e-12));
            CHECK(f.engine.ideal().isApprox(raw, 1e-12));
            std::vector<Index> qs, vs;
            for (const char* name : {"j1", "j2", "j4", "finger_left_joint"}) {
                const auto& joint = model.joints()[*model.joint_id(name)];
                qs.push_back(joint.idx_q);
                vs.push_back(joint.idx_v);
            }
            auto trajectory = JointTrajectory::create(4);
            REQUIRE(trajectory);
            Eigen::VectorXd seed(4), goal(4), limits(4);
            for (std::size_t k = 0; k < 4; ++k) {
                const Index i = static_cast<Index>(k);
                seed[i] = raw[qs[k]];
                goal[i] = target[qs[k]];
                limits[i] = model.velocity_limit()[vs[k]] * scale;
            }
            const Eigen::VectorXd zero = Eigen::VectorXd::Zero(model.nv());
            const Eigen::VectorXd trajectory_zero = Eigen::VectorXd::Zero(4);
            REQUIRE(f.engine.step(0.01));
            CHECK(f.engine.state().q.isApprox(raw, 1e-12));
            CHECK(f.engine.command().q.isApprox(raw, 1e-12));
            CHECK(f.engine.command().v.isZero(1e-12));
            Eigen::VectorXd previous = raw, previous_v = zero, previous_a = zero;
            for (int cycle = 2; cycle <= 400; ++cycle) {
                REQUIRE_MESSAGE(trajectory->step(seed, goal, trajectory_zero, limits, 0.01), "Ruckig oracle");
                REQUIRE_MESSAGE(f.engine.step(cycle * 0.01), "transition cycle failed");
                const auto& q = f.engine.command().q;
                const auto& v = f.engine.command().v;
                CHECK(f.engine.ideal().isApprox(q, 1e-12));
                const Eigen::VectorXd a = (v - previous_v) / 0.01;
                for (std::size_t k = 0; k < 4; ++k) {
                    const Index i = static_cast<Index>(k);
                    const Index iq = qs[k], iv = vs[k];
                    CHECK(q[iq] == doctest::Approx(trajectory->reference()[i]).epsilon(1e-10));
                    CHECK(v[iv] == doctest::Approx(trajectory->velocity()[i]).epsilon(1e-10));
                    CHECK(std::abs(q[iq] - previous[iq]) <= limits[i] * 0.01 + 1e-10);
                    CHECK(std::abs(v[iv]) <= limits[i] + 1e-10);
                    CHECK(std::abs(a[iv]) <= 2.0 * limits[i] + 1e-10);
                    CHECK(std::abs(a[iv] - previous_a[iv]) <= 20.0 * limits[i] * 0.01 + 1e-9);
                    CHECK(std::abs(q[iq] - target[iq]) <= std::abs(previous[iq] - target[iq]) + 1e-10);
                }
                previous = q;
                previous_v = v;
                previous_a = a;
            }
            CHECK(previous.isApprox(target, 1e-10));
            CHECK(previous_v.isZero(1e-10));
            check_clipped_targets(f.engine);
            CHECK(f.engine.stats().guard_rejects == 0);
        }
    }
}

TEST_CASE("engine: unattached settling writes do not consume the first attached transition command") {
    StampedFixture f = make_stamped(0.25, {}, true, {}, true);
    const Eigen::VectorXd initial = f.engine.state().q;
    // The adapter accepts internal writes but forwards only attached ones, just like the ROS node.
    for (int k = 1; k < 25; ++k) {
        f.clock->advance_ns(10'000'000);
        f.robot->stamp_ns = f.clock->now_ns();
        REQUIRE(f.engine.step(k * 0.01));
        CHECK_FALSE(f.engine.attached());
        CHECK_FALSE(f.robot->last_fresh);
        CHECK(f.engine.command().q.isApprox(initial, 1e-12));
        CHECK(f.engine.command().v.isZero(1e-12));
    }
    // Latest feedback, not the first sample of the settling window, seeds delivery.
    f.robot->offset = Eigen::VectorXd::Zero(initial.size());
    f.robot->offset[iq(f.engine, "j1")] = 0.01;
    f.clock->advance_ns(10'000'000);
    f.robot->stamp_ns = f.clock->now_ns();
    REQUIRE(f.engine.step(0.25));
    REQUIRE(f.engine.attached());
    CHECK(f.robot->last_fresh);
    const Eigen::VectorXd raw = f.engine.state().q;
    CHECK(raw[iq(f.engine, "j1")] == doctest::Approx(initial[iq(f.engine, "j1")] + 0.01));
    CHECK(f.engine.command().q.isApprox(raw, 1e-12));
    CHECK(f.engine.command().v.isZero(1e-12));
    f.robot->offset.setZero();
    const Eigen::VectorXd goal = clipped_feedback(f.engine);
    auto trajectory = JointTrajectory::create(4);
    REQUIRE(trajectory);
    const auto& model = *f.engine.modules().model();
    const auto& joint = model.joints()[*model.joint_id("j1")];
    Eigen::VectorXd seed(4), target(4), velocity(4);
    int coordinate = 0;
    for (const char* name : {"j1", "j2", "j4", "finger_left_joint"}) {
        const auto& j = model.joints()[*model.joint_id(name)];
        seed[coordinate] = raw[j.idx_q];
        target[coordinate] = goal[j.idx_q];
        velocity[coordinate++] = model.velocity_limit()[j.idx_v];
    }
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(4);
    for (int k = 26; k <= 225; ++k) {
        f.clock->advance_ns(10'000'000);
        f.robot->stamp_ns = f.clock->now_ns();
        REQUIRE(f.engine.step(k * 0.01));
        REQUIRE(trajectory->step(seed, target, zero, velocity, 0.01));
        CHECK(f.engine.command().q[joint.idx_q] ==
              doctest::Approx(trajectory->reference()[0]).epsilon(1e-10));
        CHECK(f.engine.command().v[joint.idx_v] == doctest::Approx(trajectory->velocity()[0]).epsilon(1e-10));
    }
    CHECK(f.engine.command().q.isApprox(goal, 1e-9));
}

TEST_CASE("engine: synchronization rebases held FK targets and releases coupled idle claims coherently") {
    StampedFixture f = make_stamped(0.0, {}, true, {}, true, true, true);
    for (int k = 1; k <= 300; ++k) REQUIRE(f.engine.step(k * 0.01));
    auto frame = std::dynamic_pointer_cast<FrameTargetModule>(f.engine.modules().module("arm"));
    REQUIRE(frame);
    CHECK_FALSE(frame->running());
    Workspace at(f.engine.modules().model());
    REQUIRE(at.update(clipped_feedback(f.engine)));
    Eigen::Isometry3d expected;
    REQUIRE(at.frame_pose(frame->frame_id(), expected));
    CHECK(frame->target().matrix().isApprox(expected.matrix(), 1e-12));
    auto goal = expected;
    goal.translation().x() += 0.02;
    REQUIRE(frame->set_target(goal));
    auto idle = std::dynamic_pointer_cast<JointTargetModule>(f.engine.modules().module("arm_idle"));
    REQUIRE(idle->set_target(idle->target()));
    REQUIRE(f.engine.step(3.01));
    CHECK_FALSE(frame->running());
    CHECK(frame->has_target());
    f.engine.modules().resynchronize(true);
    REQUIRE(f.engine.step(3.02));
    CHECK_FALSE(frame->running());
    CHECK_FALSE(frame->has_target());
    CHECK(frame->target().matrix().isApprox(expected.matrix(), 1e-12));
}

TEST_CASE("engine: synchronization transition reattach and pause resume start raw and do not stall") {
    for (const bool frame : {false, true}) {
        CAPTURE(frame);
        StampedFixture f = make_stamped(0.05, {}, true, {}, true, frame);
        double time = 0.0;
        const auto step = [&](std::int64_t ns, bool fresh) {
            f.clock->advance_ns(ns);
            if (fresh) f.robot->stamp_ns = f.clock->now_ns();
            time += 0.01;
            REQUIRE(f.engine.step(time));
        };
        step(0, true);
        CHECK(f.engine.command().q.isApprox(f.engine.state().q, 1e-12));
        const auto first = f.engine.command().q.eval();
        step(50'000'000, true);
        REQUIRE(f.engine.attached());
        CHECK(f.engine.command().q.isApprox(first, 1e-12));
        CHECK(f.engine.command().v.isZero(1e-12));
        step(10'000'000, true);
        CHECK_FALSE(f.engine.command().q.isApprox(first, 1e-12));
        auto arm = f.engine.modules().module("arm");
        if (auto ft = std::dynamic_pointer_cast<FrameTargetModule>(arm)) {
            auto goal = ft->target();
            goal.translation().x() += 0.02;
            REQUIRE(ft->set_target(goal));
        } else {
            REQUIRE(std::dynamic_pointer_cast<JointTargetModule>(arm)->set_target(
                {"j1"}, Eigen::VectorXd::Constant(1, 1.0)));
        }
        step(10'000'000, true);
        step(60'000'000, false);
        CHECK_FALSE(f.engine.attached());
        f.robot->offset = Eigen::VectorXd::Zero(f.engine.state().q.size());
        f.robot->offset[iq(f.engine, "j1")] = 0.2;
        f.robot->offset[iq(f.engine, "j2")] = -0.2;
        f.robot->offset[iq(f.engine, "j4")] = -0.03;
        step(0, true);
        CHECK_FALSE(f.engine.state().q.isApprox(clipped_feedback(f.engine), 1e-12));
        CHECK(f.engine.command().q.isApprox(f.engine.state().q, 1e-12));
        const auto reattach = f.engine.command().q.eval();
        f.robot->offset.setZero();
        step(50'000'000, true);
        REQUIRE(f.engine.attached());
        CHECK(f.engine.command().q.isApprox(reattach, 1e-12));
        CHECK(f.engine.command().v.isZero(1e-12));
        step(10'000'000, true);
        CHECK_FALSE(f.engine.command().q.isApprox(reattach, 1e-12));
        if (auto ft = std::dynamic_pointer_cast<FrameTargetModule>(arm)) CHECK_FALSE(ft->has_target());
        f.engine.set_paused(true);
        f.robot->offset[iq(f.engine, "j1")] = 0.2;
        f.robot->offset[iq(f.engine, "j2")] = -0.2;
        step(10'000'000, true);
        f.engine.set_paused(false);
        step(0, true);
        CHECK_FALSE(f.engine.state().q.isApprox(clipped_feedback(f.engine), 1e-12));
        CHECK(f.engine.command().q.isApprox(f.engine.state().q, 1e-12));
        f.robot->offset.setZero();
        REQUIRE(f.engine.set_rate_hz(50));
        for (int k = 0; k < 200; ++k) step(20'000'000, true);
        check_clipped_targets(f.engine);
    }
}

TEST_CASE("engine: synchronization transition derives mimic position and velocity through completion") {
    auto f = make_stamped(0.0);
    REQUIRE(f.engine.step(0.01));
    f.engine.set_paused(true);
    f.robot->offset = Eigen::VectorXd::Zero(f.engine.state().q.size());
    const Index leader = iq(f.engine, "finger_left_joint"), follower = iq(f.engine, "finger_right_joint");
    f.robot->offset[leader] = 0.07;
    f.robot->offset[follower] = 0.3;
    REQUIRE(f.engine.step(0.02));
    f.engine.set_paused(false);
    REQUIRE(f.engine.step(0.03));
    CHECK(f.engine.command().q[leader] == doctest::Approx(0.07));
    CHECK(f.engine.command().q[follower] == doctest::Approx(-0.07));
    CHECK(f.engine.state().q[follower] == doctest::Approx(0.3));
    f.robot->offset.setZero();
    const auto& m = *f.engine.modules().model();
    const Index lv = m.joints()[*m.joint_id("finger_left_joint")].idx_v;
    const Index fv = m.joints()[*m.joint_id("finger_right_joint")].idx_v;
    double previous = 0.07;
    for (int k = 4; k <= 200; ++k) {
        REQUIRE(f.engine.step(k * 0.01));
        const auto& c = f.engine.command();
        CHECK(c.q[follower] == doctest::Approx(-c.q[leader]));
        CHECK(c.v[fv] == doctest::Approx(-c.v[lv]));
        INFO("cycle=", k, ", delta=", c.q[leader] - previous, ", remaining=", previous - 0.04);
        CHECK(c.q[leader] <= previous + 1e-8);  // Ruckig's numerical convergence near the endpoint.
        CHECK(std::abs(c.q[leader] - previous) <= 0.2 * 0.01 + 1e-10);
        previous = c.q[leader];
    }
    CHECK(previous == doctest::Approx(0.04));
}

TEST_CASE("engine: synchronization transition write failure rolls back Ruckig without hidden advancement") {
    auto f = make_stamped(0.0, {}, true, {}, true);
    auto reference = make_stamped(0.0, {}, true, {}, true);
    f.robot->fail_write = true;
    CHECK_FALSE(f.engine.step(0.01));
    CHECK_FALSE(f.engine.written());
    f.robot->fail_write = false;
    REQUIRE(f.engine.step(0.01));
    REQUIRE(reference.engine.step(0.01));
    CHECK(f.engine.command().q == reference.engine.command().q);
    CHECK(f.engine.command().v.isZero(1e-12));
    const auto before = f.engine.ideal().eval();
    f.robot->fail_write = true;
    CHECK_FALSE(f.engine.step(0.02));
    CHECK_FALSE(f.engine.written());
    CHECK(f.engine.ideal().isApprox(before, 1e-12));
    f.robot->fail_write = false;
    for (int k = 2; k <= 200; ++k) {
        REQUIRE(f.engine.step(k * 0.01));
        REQUIRE(reference.engine.step(k * 0.01));
        CHECK(f.engine.command().q.isApprox(reference.engine.command().q, 1e-12));
        CHECK(f.engine.command().v.isApprox(reference.engine.command().v, 1e-12));
    }
}

TEST_CASE("engine: synchronization transition leaves unrelated modules and feedback groups running") {
    for (bool separate_groups : {false, true}) {
        const std::vector<FeedbackGroup> groups =
            separate_groups ? std::vector<FeedbackGroup>{{"arm", {"j1", "j2", "j3"}}, {"wrist", {"j4"}}}
                            : std::vector<FeedbackGroup>{};
        auto f = make_stamped(0.0, {}, true, groups);
        REQUIRE(f.engine.step(0.01));
        auto arm = std::dynamic_pointer_cast<JointTargetModule>(f.engine.modules().module("arm"));
        REQUIRE(arm);
        REQUIRE(arm->set_target({"j1"}, Eigen::VectorXd::Constant(1, 0.5)));
        f.engine.set_paused(true);
        f.robot->offset = Eigen::VectorXd::Zero(f.engine.state().q.size());
        f.robot->offset[iq(f.engine, "j4")] = -0.1;
        REQUIRE(f.engine.step(0.02));
        f.engine.set_paused(false);
        REQUIRE(f.engine.step(0.03));
        CHECK(f.engine.command().q[iq(f.engine, "j4")] == doctest::Approx(-0.1));
        f.robot->offset.setZero();
        for (int k = 4; k <= 30; ++k) REQUIRE(f.engine.step(k * 0.01));
        CHECK(arm->running());
        CHECK(f.engine.command().q[iq(f.engine, "j1")] > 0.001);
        CHECK(f.engine.command().q[iq(f.engine, "j4")] > -0.1);
    }
}

TEST_CASE("engine: stale feedback detaches, reattaching holds then syncs again after the timeout") {
    StampedFixture f = make_stamped(0.05);
    auto arm = std::dynamic_pointer_cast<JointTargetModule>(f.engine.modules().module("arm"));
    REQUIRE(arm);
    Eigen::VectorXd target(3);
    target << 1.0, 0.0, 0.0;
    REQUIRE(arm->set_target(target));
    CHECK_FALSE(f.engine.attached());
    for (int k = 1; k <= 5; ++k) REQUIRE(f.engine.step(0.01 * k));
    CHECK_FALSE(f.engine.attached());
    CHECK(f.engine.stats().attaches == 0);
    f.clock->advance_ns(50'000'000);
    f.robot->stamp_ns = f.clock->now_ns();
    REQUIRE(f.engine.step(0.06));
    CHECK(f.engine.attached());
    CHECK(f.engine.stats().attaches == 1);
    CHECK(arm->target()[0] == doctest::Approx(1.0));
    const auto resyncs = f.engine.stats().resyncs;

    f.clock->advance_ns(60'000'000);
    for (int k = 7; k <= 11; ++k) REQUIRE(f.engine.step(0.01 * k));
    CHECK(f.engine.stats().stale_cycles == 5);
    CHECK_FALSE(f.engine.attached());
    CHECK(f.engine.written());

    // The robot drifted while stale; the first fresh cycle holds where it is and drops the earlier target.
    const Eigen::Index j1 = iq(f.engine, "j1"), j2 = iq(f.engine, "j2");
    f.robot->offset = Eigen::VectorXd::Zero(f.engine.state().q.size());
    f.robot->offset[j2] = 0.2;
    f.robot->stamp_ns = f.clock->now_ns();
    REQUIRE(f.engine.step(0.12));
    CHECK(f.engine.fresh());
    CHECK_FALSE(f.engine.attached());
    CHECK_FALSE(f.engine.command().fresh);
    CHECK(f.engine.measured() == nullptr);
    CHECK(f.engine.stats().resyncs == resyncs + 1);
    const Eigen::VectorXd held = f.engine.state().q;
    CHECK(arm->target()[0] == doctest::Approx(held[j1]));
    CHECK(f.engine.ideal()[j1] == doctest::Approx(held[j1]));
    CHECK(f.engine.ideal()[j2] == doctest::Approx(held[j2]));

    // Every fresh cycle within the window syncs to the latest measurement; a target set meanwhile is kept.
    REQUIRE(arm->set_target(target));
    f.robot->offset[j2] = 0.1;
    f.clock->advance_ns(20'000'000);
    f.robot->stamp_ns = f.clock->now_ns();
    Eigen::VectorXd sent = f.engine.command().q;
    REQUIRE(f.engine.step(0.125));
    CHECK_FALSE(f.engine.attached());
    CHECK_FALSE(f.engine.command().fresh);
    CHECK(f.engine.stats().resyncs == resyncs + 1);
    CHECK(f.engine.state().q[j2] - sent[j2] == doctest::Approx(0.1));
    CHECK(std::abs(f.engine.command().q[j2] - f.engine.state().q[j2]) < 0.05);

    f.robot->offset[j2] = 0.3;
    f.clock->advance_ns(30'000'000);
    f.robot->stamp_ns = f.clock->now_ns();
    sent = f.engine.command().q;
    REQUIRE(f.engine.step(0.13));
    CHECK(f.engine.attached());
    CHECK(f.engine.command().fresh);
    CHECK(f.engine.stats().attaches == 2);
    CHECK(f.engine.stats().resyncs == resyncs + 1);
    const Eigen::VectorXd latest = f.engine.state().q;
    CHECK(latest[j2] - sent[j2] == doctest::Approx(0.3));
    CHECK(std::abs(f.engine.command().q[j2] - latest[j2]) < 0.05);
    CHECK(f.engine.ideal()[j1] > latest[j1]);
    f.robot->offset.setZero();

    // Paused cycles detach on stale feedback and never sync; the first unpaused settled cycle holds and
    // attaches with a single sync.
    f.engine.set_paused(true);
    f.clock->advance_ns(60'000'000);
    REQUIRE(f.engine.step(0.14));
    CHECK_FALSE(f.engine.attached());
    f.robot->stamp_ns = f.clock->now_ns();
    REQUIRE(f.engine.step(0.15));
    f.clock->advance_ns(50'000'000);
    f.robot->stamp_ns = f.clock->now_ns();
    REQUIRE(f.engine.step(0.16));
    CHECK_FALSE(f.engine.attached());
    f.engine.set_paused(false);
    REQUIRE(f.engine.step(0.17));
    CHECK(f.engine.attached());
    CHECK(f.engine.stats().attaches == 3);
    CHECK(f.engine.stats().resyncs == resyncs + 2);
}

TEST_CASE("resolve_feedback_groups: configured groups first, then default, mimics follow their leader") {
    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE(model);
    const Model& m = **model;
    const auto group_of = [&](const FeedbackLayout& l, const char* joint) {
        return l.joint_group[*m.joint_id(joint)];
    };

    auto none = resolve_feedback_groups(m, {});
    REQUIRE(none);
    CHECK(none->names == std::vector<std::string>{"default"});
    CHECK(group_of(*none, "j3") == 0);
    CHECK(group_of(*none, "universe") == 1);

    auto two = resolve_feedback_groups(m, {{"hand", {"finger_left_joint"}}, {"base", {"j[12]"}}});
    REQUIRE(two);
    CHECK(two->names == std::vector<std::string>{"hand", "base", "default"});
    CHECK(group_of(*two, "finger_left_joint") == 0);
    CHECK(group_of(*two, "finger_right_joint") == 0);
    CHECK(group_of(*two, "j2") == 1);
    CHECK(group_of(*two, "j4") == 2);

    auto all = resolve_feedback_groups(m, {{"all", {"*"}}});
    REQUIRE(all);
    CHECK(all->names == std::vector<std::string>{"all"});

    const auto fails = [&](std::vector<FeedbackGroup> groups, const std::string& text) {
        auto r = resolve_feedback_groups(m, groups);
        REQUIRE_FALSE(r);
        CHECK(r.error().code == ErrorCode::invalid_argument);
        CHECK_MESSAGE(r.error().message.find(text) != std::string::npos, r.error().message);
    };
    fails({{"", {"j1"}}}, "name is empty");
    fails({{"default", {"j1"}}}, "'default' is reserved");
    fails({{"a", {"j1"}}, {"a", {"j2"}}}, "declared twice");
    fails({{"a", {}}}, "has no joints");
    fails({{"a", {"nope*"}}}, "matches no joint");
    fails({{"a", {"finger_right_joint"}}}, "mimic follower");
    fails({{"a", {"j1"}}, {"b", {"j*"}}}, "is in feedback groups 'a' and 'b'");
}

TEST_CASE("engine: feedback groups attach on their own and a stale group detaches only itself") {
    StampedFixture f = make_stamped(0.05, {}, false, {{"wrist", {"j4"}}});
    Engine& e = f.engine;
    REQUIRE(e.group_count() == 2);
    CHECK(e.group_name(0) == "wrist");
    CHECK(e.group_name(1) == "default");
    CHECK(e.group_name(2).empty());
    auto arm = std::dynamic_pointer_cast<JointTargetModule>(e.modules().module("arm"));
    auto wrist = std::dynamic_pointer_cast<JointTargetModule>(e.modules().module("wrist"));
    REQUIRE(arm);
    REQUIRE(wrist);

    // Only the default group falls back to RobotState::stamp_ns; an unstamped configured group stays stale.
    const auto stamp = [&](bool wrist_fresh) {
        f.robot->stamp_ns = f.clock->now_ns();
        f.robot->group_stamps = {wrist_fresh ? f.clock->now_ns() : -1, -1};
    };
    stamp(false);
    REQUIRE(e.step(0.01));
    CHECK_FALSE(e.group_fresh(0));
    CHECK(e.group_fresh(1));
    CHECK_FALSE(e.fresh());
    f.clock->advance_ns(50'000'000);
    stamp(false);
    REQUIRE(e.step(0.02));
    CHECK_FALSE(e.group_attached(0));
    CHECK(e.group_attached(1));
    CHECK_FALSE(e.attached());
    CHECK(e.measured() != nullptr);
    CHECK(e.stats().attaches == 1);
    CHECK(f.robot->last_groups == std::vector<std::uint8_t>{0, 1});
    CHECK_FALSE(f.robot->last_fresh);

    Eigen::VectorXd target(3);
    target << 1.0, 0.0, 0.0;
    REQUIRE(arm->set_target(target));
    REQUIRE(wrist->set_target(Eigen::VectorXd::Constant(1, 0.08)));

    // The wrist turns fresh away from where it was commanded: its sync drops only the wrist target.
    const Eigen::Index j4 = iq(e, "j4"), j1 = iq(e, "j1");
    f.robot->offset = Eigen::VectorXd::Zero(e.state().q.size());
    f.robot->offset[j4] = 0.05;
    stamp(true);
    REQUIRE(e.step(0.03));
    CHECK(e.group_fresh(0));
    CHECK_FALSE(e.group_attached(0));
    CHECK(wrist->target()[0] == doctest::Approx(e.state().q[j4]));
    CHECK(arm->target()[0] == doctest::Approx(1.0));
    CHECK(e.ideal()[j4] == doctest::Approx(e.state().q[j4]));
    CHECK(e.ideal()[j1] > 0.0);
    f.robot->offset.setZero();
    f.clock->advance_ns(50'000'000);
    stamp(true);
    REQUIRE(e.step(0.04));
    CHECK(e.attached());
    CHECK(e.command().fresh);
    CHECK(e.stats().attaches == 2);
    CHECK(f.robot->last_groups == std::vector<std::uint8_t>{1, 1});

    const auto stale = e.stats().stale_cycles;
    f.clock->advance_ns(10'000'000);
    stamp(false);
    REQUIRE(e.step(0.05));
    CHECK_FALSE(e.group_attached(0));
    CHECK(e.group_attached(1));
    CHECK_FALSE(e.command().fresh);
    CHECK(f.robot->last_groups == std::vector<std::uint8_t>{0, 1});
    CHECK(e.stats().stale_cycles == stale + 1);
}

TEST_CASE("engine: a driver that resizes group_stamps_ns fails the cycle") {
    StampedFixture f = make_stamped(0.0, {}, true, {{"wrist", {"j4"}}});
    f.robot->group_stamps = {f.clock->now_ns()};
    auto s = f.engine.step(0.01);
    REQUIRE_FALSE(s);
    CHECK(s.error().message.find("resized group_stamps_ns") != std::string::npos);
}

TEST_CASE("engine: a stale or unstamped state at create starts detached") {
    StampedFixture f = make_stamped(0.05, {}, false);
    CHECK_FALSE(f.engine.attached());
    auto arm = std::dynamic_pointer_cast<JointTargetModule>(f.engine.modules().module("arm"));
    REQUIRE(arm);
    Eigen::VectorXd target(3);
    target << 1.0, 0.0, 0.0;
    REQUIRE(arm->set_target(target));
    REQUIRE(f.engine.step(0.01));
    CHECK_FALSE(f.engine.attached());
    CHECK_FALSE(f.robot->last_fresh);

    const Eigen::Index j1 = iq(f.engine, "j1");
    f.robot->offset = Eigen::VectorXd::Zero(f.engine.state().q.size());
    f.robot->offset[j1] = 0.3;
    f.robot->stamp_ns = f.clock->now_ns();
    REQUIRE(f.engine.step(0.02));
    CHECK_FALSE(f.engine.attached());
    CHECK(arm->target()[0] == doctest::Approx(f.engine.state().q[j1]));
    CHECK(f.engine.command().q[j1] == doctest::Approx(f.engine.state().q[j1]));
    f.robot->offset.setZero();

    // A stale read within the window restarts it.
    f.clock->advance_ns(30'000'000);
    f.robot->stamp_ns = f.clock->now_ns();
    REQUIRE(f.engine.step(0.03));
    CHECK_FALSE(f.engine.attached());
    f.robot->stamp_ns = -1;
    REQUIRE(f.engine.step(0.04));
    f.robot->stamp_ns = f.clock->now_ns();
    REQUIRE(f.engine.step(0.05));
    f.clock->advance_ns(30'000'000);
    f.robot->stamp_ns = f.clock->now_ns();
    REQUIRE(f.engine.step(0.06));
    CHECK_FALSE(f.engine.attached());
    CHECK_FALSE(f.robot->last_fresh);
    f.clock->advance_ns(20'000'000);
    f.robot->stamp_ns = f.clock->now_ns();
    REQUIRE(f.engine.step(0.07));
    CHECK(f.engine.attached());
    CHECK(f.robot->last_fresh);
    CHECK(f.engine.stats().attaches == 1);
    CHECK(f.engine.command().q[j1] == doctest::Approx(f.engine.state().q[j1]));

    StampedFixture always = make_stamped(0.0, {}, false);
    CHECK(always.engine.attached());
    REQUIRE(always.engine.step(0.01));
    CHECK(always.robot->last_fresh);
    CHECK(always.engine.stats().attaches == 0);
}

TEST_CASE("engine: feedback_joints selects the joints whose error is estimated") {
    StampedFixture f = make_stamped(0.0, {"j1"});
    REQUIRE(f.engine.step(0.01));
    const Eigen::VectorXd ideal = f.engine.ideal();
    const Eigen::Index j1 = iq(f.engine, "j1"), j2 = iq(f.engine, "j2");
    f.robot->offset = Eigen::VectorXd::Zero(ideal.size());
    f.robot->offset[j1] = 0.1;
    f.robot->offset[j2] = 0.1;
    f.robot->stamp_ns += 1;
    REQUIRE(f.engine.step(0.02));
    REQUIRE(f.engine.measured());
    CHECK((*f.engine.measured())[j1] - ideal[j1] == doctest::Approx(0.1));
    CHECK((*f.engine.measured())[j2] == doctest::Approx(ideal[j2]));

    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE(model);
    for (const char* name :
         {"nope", "finger_right_joint", "universe", "tool_joint", "nope*", "tool*", "j[!1-4]"}) {
        auto set = ModuleSet::create(*model);
        REQUIRE(set);
        EngineOptions options;
        options.feedback_joints = {name};
        auto e = Engine::create(
            std::move(*set), std::make_shared<LoopbackRobot>((*model)->neutral(), (*model)->nv()), options);
        INFO(name);
        CHECK_FALSE(e);
    }
}

TEST_CASE("engine: feedback_joints accepts fnmatch patterns over eligible joints") {
    const auto selected = [](std::vector<std::string> patterns) {
        StampedFixture f = make_stamped(0.0, std::move(patterns));
        REQUIRE(f.engine.step(0.01));
        const Eigen::VectorXd ideal = f.engine.ideal();
        const Model& m = *f.engine.modules().model();
        Eigen::VectorXd drifted(ideal.size());
        REQUIRE(m.integrate(ideal, Eigen::VectorXd::Constant(m.nv(), 0.01), drifted));
        f.robot->offset = drifted - ideal;
        f.robot->stamp_ns += 1;
        REQUIRE(f.engine.step(0.02));
        REQUIRE(f.engine.measured());
        std::string out;
        for (const char* j : {"j1", "j2", "j3", "j4", "finger_left_joint"}) {
            const JointId id = *m.joint_id(j);
            const double d = *m.joint_position(*f.engine.measured(), id) - *m.joint_position(ideal, id);
            if (std::abs(d) > 1e-6) out += std::string(out.empty() ? "" : ",") + j;
        }
        return out;
    };
    CHECK(selected({"j3"}) == "j3");
    CHECK(selected({}) == "j1,j2,j3,j4,finger_left_joint");
    CHECK(selected({"j[13]"}) == "j1,j3");
    CHECK(selected({"j?", "finger_*"}) == "j1,j2,j3,j4,finger_left_joint");
    CHECK(selected({"*_joint"}) ==
          "finger_left_joint");  // the mimic follower and fixed tool_joint are skipped
    CHECK(selected({"j[^1-3]", "j2"}) == "j2,j4");
}

TEST_CASE("engine: disabled feedback timeout accepts any stamp") {
    StampedFixture f = make_stamped(0.0);
    f.clock->advance_ns(10'000'000'000);
    REQUIRE(f.engine.step(0.01));
    f.robot->stamp_ns = -1;
    REQUIRE(f.engine.step(0.02));
    CHECK(f.robot->writes == 2);

    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE(model);
    auto set = ModuleSet::create(*model);
    REQUIRE(set);
    EngineOptions bad;
    bad.feedback_timeout_s = -1.0;
    auto e = Engine::create(std::move(*set),
                            std::make_shared<LoopbackRobot>((*model)->neutral(), (*model)->nv()), bad);
    REQUIRE_FALSE(e);
    CHECK(e.error().code == ErrorCode::invalid_argument);
}

TEST_CASE("engine: repeated stamps are counted") {
    StampedFixture f = make_stamped(0.0);
    // create() already read the initial stamp once.
    REQUIRE(f.engine.step(0.01));
    REQUIRE(f.engine.step(0.02));
    CHECK(f.engine.stats().repeated_reads == 2);
    f.robot->stamp_ns += 1;
    REQUIRE(f.engine.step(0.03));
    CHECK(f.engine.stats().repeated_reads == 2);
    f.robot->stamp_ns = -1;
    REQUIRE(f.engine.step(0.04));
    REQUIRE(f.engine.step(0.05));
    CHECK(f.engine.stats().repeated_reads == 2);
    CHECK(f.robot->writes == 5);
}

TEST_CASE("engine: loopback stamps reads with the context clock") {
    auto clock = std::make_shared<ManualClock>(42);
    LoopbackRobot robot(Eigen::VectorXd::Zero(2), 2, clock);
    RobotState state;
    REQUIRE(robot.read(state));
    CHECK(state.stamp_ns == 42);

    Engine e = load_fixture();
    CHECK(e.state().stamp_ns >= 0);
    REQUIRE(e.run(0.03));
    CHECK(e.stats().repeated_reads == 0);
    CHECK(e.stats().max_lateness_s == 0.0);
    CHECK(e.result().quality == SolveQuality::nominal);
}

TEST_CASE("engine: instances sharing a model run independently on separate threads") {
    auto doc = load_yaml(kFixtures / "arm_loopback.yaml");
    REQUIRE(doc);
    const ConfigNode* modules = doc->find("modules");
    REQUIRE(modules);
    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE(model);
    const auto make = [&](double j1) {
        const Context context = Context::defaults();
        auto set = ModuleSet::create(*model, {}, context);
        REQUIRE(set);
        REQUIRE(ModuleRegistry::with_builtins().populate(*set, *modules, context));
        EngineOptions options;
        options.feedback_timeout_s = 0.0;  // attach timing would follow the wall clock
        auto e = Engine::create(std::move(*set),
                                std::make_shared<LoopbackRobot>((*model)->neutral(), (*model)->nv()), options,
                                context);
        REQUIRE_MESSAGE(e, message(e));
        auto arm = std::dynamic_pointer_cast<JointTargetModule>(e->modules().module("arm"));
        REQUIRE(arm);
        Eigen::VectorXd target(3);
        target << j1, -0.3, 0.6;
        const Status s = arm->set_target(target);
        REQUIRE_MESSAGE(s, (s ? "" : s.error().message));
        return std::move(*e);
    };
    constexpr double kDuration = 0.3;

    Engine seq_a = make(0.5);
    Engine seq_b = make(-0.4);
    REQUIRE(seq_a.run(kDuration));
    REQUIRE(seq_b.run(kDuration));

    Engine par_a = make(0.5);
    Engine par_b = make(-0.4);
    Status status_a, status_b;
    std::thread ta([&] { status_a = par_a.run(kDuration); });
    std::thread tb([&] { status_b = par_b.run(kDuration); });
    ta.join();
    tb.join();
    REQUIRE(status_a);
    REQUIRE(status_b);
    CHECK(par_a.stats().cycles == seq_a.stats().cycles);
    CHECK(par_b.stats().cycles == seq_b.stats().cycles);
    CHECK(par_a.state().q == seq_a.state().q);
    CHECK(par_a.state().v == seq_a.state().v);
    CHECK(par_b.state().q == seq_b.state().q);
    CHECK(par_b.state().v == seq_b.state().v);
    CHECK(par_a.state().q != par_b.state().q);
}

TEST_CASE("engine: configuration errors carry location and path") {
    std::string m = build_error("model: {file: arm.urdf}\nfeedback_timeout_s: -0.5\n");
    CHECK_MESSAGE(m.find("t.yaml:2:21: feedback_timeout_s: must be >= 0") == 0, m);
    {
        auto doc = parse_yaml("model: {file: arm.urdf}\nfeedback_timeout_s: 0.25\n", "t.yaml");
        REQUIRE(doc);
        auto e = build_engine(*doc, kFixtures);
        REQUIRE_MESSAGE(e, message(e));
        CHECK(e->options().feedback_timeout_s == 0.25);
        REQUIRE(e->step(0.01));
    }

    m = build_error("model: {file: arm.urdf}\nrate_hz: 0\n");
    CHECK_MESSAGE(m.find("t.yaml:2:10: rate_hz: must be > 0") == 0, m);

    {
        auto doc = parse_yaml(
            "model: {file: arm.urdf}\nfeedback_timeout_s: 0\n"
            "feedback_groups: [{name: wrist, joints: [j4]}]\n",
            "t.yaml");
        REQUIRE(doc);
        auto e = build_engine(*doc, kFixtures);
        REQUIRE_MESSAGE(e, message(e));
        REQUIRE(e->options().feedback_groups.size() == 1);
        CHECK(e->options().feedback_groups[0].joints == std::vector<std::string>{"j4"});
        CHECK(e->group_count() == 2);
    }
    m = build_error("model: {file: arm.urdf}\nfeedback_groups: [{name: wrist}]\n");
    CHECK_MESSAGE(m.find("joints") != std::string::npos, m);
    m = build_error("model: {file: arm.urdf}\nfeedback_groups: [{name: default, joints: [j4]}]\n");
    CHECK_MESSAGE(m.find("'default' is reserved") != std::string::npos, m);

    m = build_error("model: {file: arm.urdf}\nrate: 100\n");
    CHECK_MESSAGE(m.find("rate: unknown key") != std::string::npos, m);
    CHECK_MESSAGE(m.find("robot, modules)") != std::string::npos, m);
    m = build_error("model: {file: arm.urdf, bogus: 1}\n");
    CHECK_MESSAGE(m.find("mimic, joint_limits, locked_joints)") != std::string::npos, m);
    m = build_error("model: {file: arm.urdf}\nrobot: {bogus: 1}\n");
    CHECK_MESSAGE(m.find("library, params)") != std::string::npos, m);
    for (const char* id : {"a.b", "a/b", "' a'", "1a"}) {
        m = build_error(std::string("model: {file: arm.urdf}\nmodules:\n  ") + id +
                        ": {type: joint_target, joints: [j1]}\n");
        CHECK_MESSAGE(m.find("invalid module id") != std::string::npos, m);
    }
    m = build_error(
        "model: {file: arm.urdf}\nmodules:\n  jt: {type: joint_target, joints: [j1], home: [5]}\n");
    CHECK_MESSAGE(m.find("home 5 of j1 is outside [-2, 2]") != std::string::npos, m);
    CHECK_MESSAGE(m.find(":3:7: modules.jt: ") != std::string::npos, m);
    CHECK_MESSAGE(m.find("[KST-E206]") != std::string::npos, m);
    m = build_error("model: arm.urdf\n");
    CHECK_MESSAGE(m.find("write model: {file: arm.urdf}") != std::string::npos, m);
    {
        const std::string tool =
            "model: {file: arm.urdf}\nmodules:\n  tool: {type: frame_target, frame: tool, trajectory: "
            "{position: {velocity: 0.3, acceleration: 1, jerk: 10}, orientation: {velocity: 1, "
            "acceleration: 2, jerk: 20}}}\n  ns: {type: null_space, primary: tool, ";
        m = build_error(tool + "reference: custom, posture: [0, 0, 0]}\n");
        CHECK_MESSAGE(m.find("needs 4 values (j1, j2, j3, j4), got 3") != std::string::npos, m);
        m = build_error(tool + "reference: custom, posture: [0, 0, 0, 0.5]}\n");
        CHECK_MESSAGE(m.find("posture 0.5 of j4 is outside [0, 0.1]") != std::string::npos, m);
        m = build_error(tool + "posture: [0, 0, 0, 0]}\n");
        CHECK_MESSAGE(m.find("only with reference: custom") != std::string::npos, m);
        m = build_error(tool + "joints: [finger_left_joint]}\n");
        CHECK_MESSAGE(m.find("not driven by primary 'tool'") != std::string::npos, m);
    }

    m = build_error("rate_hz: 100\n");
    CHECK_MESSAGE(m.find("model: required key is missing") != std::string::npos, m);

    m = build_error("model: {file: missing.urdf}\n");
    CHECK_MESSAGE(m.find("t.yaml:1:15: model.file: ") == 0, m);

    m = build_error("model: {file: arm.urdf, root: wheeled}\n");
    CHECK_MESSAGE(m.find("model.root") != std::string::npos, m);

    m = build_error("model: {file: arm.urdf}\nsolver: {mode: fastest}\n");
    CHECK_MESSAGE(m.find("solver.mode") != std::string::npos, m);

    m = build_error("model: {file: arm.urdf}\nrobot: {initial: [0, 0]}\n");
    CHECK_MESSAGE(m.find("t.yaml:2:18: robot.initial: expected 7 values, got 2") == 0, m);

    m = build_error("model: {file: arm.urdf}\nmodules:\n  arm: {type: joint_target, joints: [nope]}\n");
    CHECK_MESSAGE(m.find("nope") != std::string::npos, m);

    m = build_error("- 1\n");
    CHECK_MESSAGE(m.find("expected a map") != std::string::npos, m);
}

TEST_CASE("engine: robot.initial by joint name, mimic followers and root pose") {
    auto doc = parse_yaml(
        "model: {file: arm.urdf}\nrobot: {initial: {j1: 0.5, j3: 3.0, finger_left_joint: 0.03}}\n", "t.yaml");
    REQUIRE(doc);
    auto e = build_engine(*doc, kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    RobotState state;
    REQUIRE(e->robot().read(state));
    const Eigen::VectorXd& q = state.q;
    CHECK(joint_q(*e, q, "j1") == doctest::Approx(0.5));
    CHECK(joint_q(*e, q, "j2") == doctest::Approx(0.0));
    CHECK(joint_q(*e, q, "j3") == doctest::Approx(3.0));
    CHECK(joint_q(*e, q, "finger_right_joint") == doctest::Approx(-0.03));

    std::string m = build_error("model: {file: arm.urdf}\nrobot: {initial: {nope: 1}}\n");
    CHECK_MESSAGE(m.find("t.yaml:2:25: robot.initial.nope: unknown joint") == 0, m);
    m = build_error("model: {file: arm.urdf}\nrobot: {initial: {j1: 2.5}}\n");
    CHECK_MESSAGE(m.find("robot.initial.j1: 2.5") != std::string::npos, m);
    m = build_error("model: {file: arm.urdf}\nrobot: {initial: {finger_left_joint: 0.05}}\n");
    CHECK_MESSAGE(m.find("robot.initial.finger_left_joint") != std::string::npos, m);
    m = build_error("model: {file: arm.urdf}\nrobot: {initial: {tool_joint: 0}}\n");
    CHECK_MESSAGE(m.find("robot.initial.tool_joint") != std::string::npos, m);
    m = build_error("model: {file: arm.urdf}\nrobot: {initial: {j1: x}}\n");
    CHECK_MESSAGE(m.find("robot.initial.j1: expected a number") != std::string::npos, m);
    m = build_error("model: {file: arm.urdf}\nrobot: {initial_root: {position: [1, 0, 0]}}\n");
    CHECK_MESSAGE(m.find("robot.initial_root: model root is fixed") != std::string::npos, m);
    m = build_error(
        "model: {file: floating_base.urdf, root: free_flyer}\n"
        "robot: {initial: [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0], initial_root: {position: [1, 0, 0]}}\n");
    CHECK_MESSAGE(m.find("robot.initial_root: not combinable") != std::string::npos, m);
    m = build_error(
        "model: {file: arm.urdf}\nmodules:\n  g: {type: joint_target, joints: [finger_left_joint, "
        "finger_right_joint]}\n");
    CHECK_MESSAGE(m.find("are coupled by a mimic; list only one") != std::string::npos, m);
    m = build_error(
        "model: {file: arm.urdf, mimic: coupled}\nmodules:\n  g: {type: joint_target, joints: "
        "[finger_right_joint]}\n");
    CHECK_MESSAGE(m.find("coupled mimic follower; use its leader finger_left_joint") != std::string::npos, m);

    doc = parse_yaml(
        "model: {file: floating_base.urdf, root: free_flyer}\n"
        "robot: {initial: {joint2: 0.2}, initial_root: {position: [1, 2, 3], rpy: [0, 0, 0.5]}}\n",
        "t.yaml");
    REQUIRE(doc);
    e = build_engine(*doc, kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    const Model& model = *e->modules().model();
    REQUIRE(e->robot().read(state));
    auto root = model.root_pose(state.q);
    REQUIRE(root);
    CHECK(root->translation().isApprox(Eigen::Vector3d(1, 2, 3)));
    CHECK(Eigen::AngleAxisd(root->rotation()).angle() == doctest::Approx(0.5));
    CHECK(joint_q(*e, state.q, "joint2") == doctest::Approx(0.2));
}

TEST_CASE("engine: model.joint_limits replaces the loaded model limits") {
    auto doc = parse_yaml(
        "model:\n"
        "  file: arm.urdf\n"
        "  joint_limits:\n"
        "    velocity: {j2: 0.4, j3: 3.0}\n"
        "    position: {lower: {j1: -0.5}, upper: {j1: 0.5, j4: 0.05}}\n"
        "robot: {initial: {j1: 0.4}}\n",
        "t.yaml");
    REQUIRE(doc);
    auto e = build_engine(*doc, kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    const Model& m = *e->modules().model();
    CHECK(m.lower_position_limit()[0] == doctest::Approx(-0.5));
    CHECK(m.upper_position_limit()[0] == doctest::Approx(0.5));
    CHECK(m.upper_position_limit()[4] == doctest::Approx(0.05));
    // j2 tightened, j3 widened, the rest keep the URDF values.
    Eigen::VectorXd v(6);
    v << 1.5, 0.4, 3.0, 0.5, 0.2, 0.2;
    CHECK(m.velocity_limit().isApprox(v));

    doc = parse_yaml(
        "model: {file: arm.urdf, joint_limits: {position: {upper: {j1: 2.8}}}}\n"
        "robot: {initial: {j1: 2.5}}\n",
        "t.yaml");
    REQUIRE(doc);
    auto wide = build_engine(*doc, kFixtures);
    REQUIRE_MESSAGE(wide, message(wide));
    RobotState state;
    REQUIRE(wide->robot().read(state));
    CHECK(joint_q(*wide, state.q, "j1") == doctest::Approx(2.5));

    std::string err = build_error(
        "model: {file: arm.urdf, joint_limits: {position: {upper: {j1: 0.5}}}}\n"
        "robot: {initial: {j1: 0.8}}\n");
    CHECK_MESSAGE(err.find("robot.initial.j1: 0.8") != std::string::npos, err);
    err = build_error("model: {file: arm.urdf, joint_limits: {position: {upper: {j1: -3.0}}}}\n");
    CHECK_MESSAGE(
        err.find("t.yaml:1:63: model.joint_limits.position.upper.j1: joint j1: position range is empty") == 0,
        err);
    err = build_error("model: {file: arm.urdf, joint_limits: {velocity: {nope: 1}}}\n");
    CHECK_MESSAGE(
        err.find("model.joint_limits.velocity.nope: joint nope: unknown joint") != std::string::npos, err);
    err = build_error("model: {file: arm.urdf, joint_limits: {velocity: {j1: x}}}\n");
    CHECK_MESSAGE(err.find("model.joint_limits.velocity.j1: expected a number") != std::string::npos, err);
    err = build_error("model: {file: arm.urdf, joint_limits: {velocity: {j1: 0}}}\n");
    CHECK_MESSAGE(
        err.find("model.joint_limits.velocity.j1: joint j1: velocity limit must be finite and > 0") !=
            std::string::npos,
        err);
    err = build_error("model: {file: arm.urdf, joint_limits: {effort: {j1: 1}}}\n");
    CHECK_MESSAGE(err.find("model.joint_limits.effort: unknown key") != std::string::npos, err);
    err = build_error("model: {file: arm.urdf, joint_limits: {position: {middle: {j1: 1}}}}\n");
    CHECK_MESSAGE(err.find("model.joint_limits.position.middle: unknown key") != std::string::npos, err);
    err = build_error("model: {file: arm.urdf, joint_limits: [1]}\n");
    CHECK_MESSAGE(err.find("model.joint_limits: expected a map") != std::string::npos, err);
}

TEST_CASE("engine: model.locked_joints holds joints at their initial value") {
    auto doc = parse_yaml(
        "model: {file: arm.urdf, locked_joints: [j2, finger_left_joint]}\n"
        "feedback_timeout_s: 0\n"
        "robot: {initial: {j2: 0.2, finger_left_joint: 0.01}}\n"
        "modules:\n  arm: {type: joint_target, joints: [j1, j2, j3]}\n",
        "t.yaml");
    REQUIRE(doc);
    auto e = build_engine(*doc, kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    auto arm = std::dynamic_pointer_cast<JointTargetModule>(e->modules().module("arm"));
    REQUIRE(arm);
    Eigen::VectorXd target(3);
    target << 0.5, -0.3, 1.0;
    REQUIRE(arm->set_target(target));
    for (int i = 0; i < 300; ++i) REQUIRE(e->step(e->stats().time + e->dt()));
    const Eigen::VectorXd& q = e->state().q;
    CHECK(joint_q(*e, q, "j1") == doctest::Approx(0.5).epsilon(1e-3));
    CHECK(joint_q(*e, q, "j2") == doctest::Approx(0.2));
    CHECK(joint_q(*e, q, "finger_left_joint") == doctest::Approx(0.01));
    CHECK(joint_q(*e, q, "finger_right_joint") == doctest::Approx(-0.01));

    std::string err = build_error("model: {file: arm.urdf, locked_joints: [nope]}\n");
    CHECK_MESSAGE(err.find("t.yaml:1:41: model.locked_joints[0]: unknown joint 'nope'") == 0, err);
    err = build_error("model: {file: arm.urdf, locked_joints: [j1, j1]}\n");
    CHECK_MESSAGE(err.find("model.locked_joints[1]: duplicate joint 'j1'") != std::string::npos, err);
    err = build_error("model: {file: arm.urdf, locked_joints: [tool_joint]}\n");
    CHECK_MESSAGE(err.find("model.locked_joints[0]: unknown joint 'tool_joint'") != std::string::npos, err);
    err = build_error("model: {file: arm.urdf, mimic: coupled, locked_joints: [finger_right_joint]}\n");
    CHECK_MESSAGE(err.find("'finger_right_joint' has no velocity") != std::string::npos, err);
    err = build_error("model: {file: floating_base.urdf, root: free_flyer, locked_joints: [root_joint]}\n");
    CHECK_MESSAGE(err.find("'root_joint' is the model root") != std::string::npos, err);
    err = build_error("model: {file: arm.urdf, locked_joints: j1}\n");
    CHECK_MESSAGE(err.find("model.locked_joints: expected a list") != std::string::npos, err);
    err = build_error("model: {file: arm.urdf, locked_joints: [1]}\n");
    CHECK_MESSAGE(err.find("model.locked_joints[0]: expected a string") != std::string::npos, err);
    err = build_error("model: {file: arm.urdf, nope: 1}\n");
    CHECK_MESSAGE(err.find("joint_limits, locked_joints)") != std::string::npos, err);
}

TEST_CASE("engine: model.package_dirs resolves package:// before the host handler") {
    const fs::path root = fs::temp_directory_path() / ("kstone-package-dirs-" + std::to_string(::getpid()));
    fs::create_directories(root / "pkgs/arm_description/urdf");
    fs::copy_file(kFixtures / "arm.urdf", root / "pkgs/arm_description/urdf/arm.urdf",
                  fs::copy_options::overwrite_existing);
    auto doc = parse_yaml("model: {file: package://arm_description/urdf/arm.urdf, package_dirs: [pkgs]}\n",
                          "t.yaml");
    REQUIRE(doc);
    auto e = build_engine(*doc, root);
    CHECK_MESSAGE(e, message(e));

    // A host handler still serves packages that package_dirs lacks.
    Context context = Context::defaults();
    context.resources.add_scheme("package", make_directory_scheme({root / "pkgs"}));
    doc = parse_yaml("model: {file: package://arm_description/urdf/arm.urdf, package_dirs: [elsewhere]}\n",
                     "t.yaml");
    REQUIRE(doc);
    e = build_engine(*doc, root, {}, context);
    CHECK_MESSAGE(e, message(e));

    doc = parse_yaml("model: {file: package://arm_description/urdf/arm.urdf, package_dirs: [elsewhere]}\n",
                     "t.yaml");
    REQUIRE(doc);
    e = build_engine(*doc, root);
    CHECK_FALSE(e);
    std::error_code ec;
    fs::remove_all(root, ec);
}

TEST_CASE("engine: latency histograms and reset_stats") {
    Engine e = load_fixture();
    REQUIRE(e.run(0.1));
    const EngineStats& s = e.stats();
    CHECK(s.cycle_hist.count() == 10);
    CHECK(s.cycle_hist.quantile(0.5) <= s.cycle_hist.quantile(0.99));
    CHECK(s.cycle_hist.quantile(1.0) == doctest::Approx(s.max_cycle_s));
    e.reset_stats();
    CHECK(e.stats().cycles == 0);
    CHECK(e.stats().max_cycle_s == 0.0);
    CHECK(e.stats().cycle_hist.count() == 0);
    CHECK(e.stats().time == doctest::Approx(0.1));
    REQUIRE(e.run(0.05));
    CHECK(e.stats().cycles == 5);
    CHECK(e.stats().cycle_hist.count() == 5);
    CHECK(e.stats().time == doctest::Approx(0.15));
}

TEST_CASE("engine: telemetry sink receives throttled snapshots") {
    struct Recorder final : TelemetrySink {
        void publish(const EngineSnapshot& s) override {
            times.push_back(s.stats.time);
            last = s;
        }
        std::vector<double> times;
        EngineSnapshot last;
    };
    CHECK(make_telemetry_observer(nullptr) == nullptr);
    Engine e = load_fixture();
    auto every = std::make_shared<Recorder>();
    auto half = std::make_shared<Recorder>();
    e.add_observer(make_telemetry_observer(every));
    e.add_observer(make_telemetry_observer(half, 50.0));
    REQUIRE(e.run(0.1));
    CHECK(every->times.size() == 10);
    REQUIRE(half->times.size() == 5);
    CHECK(half->times[1] - half->times[0] == doctest::Approx(0.02));

    const EngineSnapshot& s = every->last;
    CHECK(s.schema_version == snapshot_schema_version);
    CHECK(s.stats.cycles == 10);
    CHECK(s.cycle_p99_s == doctest::Approx(e.stats().cycle_hist.quantile(0.99)));
    CHECK(s.written);
    CHECK(s.fresh);
    CHECK_FALSE(s.paused);
    CHECK(s.command.q.isApprox(e.command().q));
    CHECK(s.state.q.isApprox(e.state().q));
    CHECK(s.ideal.isApprox(e.ideal()));
    CHECK(s.has_measured == (e.measured() != nullptr));
    CHECK(s.residuals.size() == e.result().residuals.size());
    REQUIRE(s.modules.size() == 3);
    CHECK(s.modules[0].id == "tool");
    CHECK(s.modules[0].type == std::string(e.modules().modules()[0]->type()));
    CHECK(s.modules[0].held_by == -1);
    CHECK(s.modules[1].ports.size() == e.modules().modules()[1]->ports().size());

    e.set_paused(true);
    REQUIRE(e.run(0.02));
    CHECK(every->times.size() == 12);
    CHECK(every->last.paused);
    CHECK_FALSE(every->last.written);
}

TEST_CASE("engine: self_collision drops pairs penetrating at robot.initial") {
    const fs::path dir = fs::temp_directory_path() / ("kstone-initial-pairs-" + std::to_string(::getpid()));
    fs::create_directories(dir);
    // c folds back onto a at q = 0 and swings away at j2 = pi.
    std::ofstream(dir / "fold.urdf") << R"(<robot name="fold">
  <link name="base"><collision><origin xyz="-1 0 0"/><geometry><sphere radius="0.05"/></geometry></collision></link>
  <link name="a"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <link name="m"/>
  <link name="b"/>
  <link name="c"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="a"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" velocity="1" effort="1"/></joint>
  <joint name="jm" type="revolute"><parent link="a"/><child link="m"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
  <joint name="j2" type="revolute"><parent link="m"/><child link="b"/><origin xyz="0.3 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.2" upper="3.2" velocity="1" effort="1"/></joint>
  <joint name="j3" type="fixed"><parent link="b"/><child link="c"/><origin xyz="-0.15 0 0"/></joint>
</robot>)";
    const auto pairs = [&](const std::string& robot) -> std::size_t {
        auto doc = parse_yaml("model: {file: fold.urdf}\nrobot: " + robot +
                                  "\nmodules:\n  collision: {type: self_collision, urdf: " +
                                  (dir / "fold.urdf").string() + "}\n",
                              "t.yaml");
        REQUIRE_MESSAGE(doc, message(doc));
        auto e = build_engine(*doc, dir);
        REQUIRE_MESSAGE(e, message(e));
        auto module = std::dynamic_pointer_cast<CollisionModuleBase>(e->modules().module("collision"));
        REQUIRE(module);
        return module->constraint()->query().model()->pairs().size();
    };
    CHECK(pairs("{type: loopback}") == 1);
    CHECK(pairs("{type: loopback, initial: {j2: 3.14}}") == 2);
    fs::remove_all(dir);
}
