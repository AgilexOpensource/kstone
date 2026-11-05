// Zero-allocation check for Solver::solve after warm-up. The glibc malloc family is interposed in this
// executable, so allocations from kstone, Pinocchio, Eigen, DAQP and operator new are all counted.
// Set KSTONE_ALLOC_TRACE=1 to print a backtrace for each counted allocation.
#include "kstone/chassis/chassis.hpp"
#include "kstone/collision/constraint.hpp"
#include "kstone/control/constraints.hpp"
#include "kstone/control/solver.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"
#include "kstone/module/builtins.hpp"
#include "kstone/motion/joint_trajectory.hpp"
#include "kstone/motion/pose_trajectory.hpp"
#include "kstone/runtime/debug_server.hpp"
#include "kstone/runtime/engine.hpp"
#include "kstone/runtime/robot.hpp"
#include "kstone/runtime/snapshot.hpp"

#include <doctest/doctest.h>

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <execinfo.h>
#include <filesystem>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

extern "C" {
void* __libc_malloc(std::size_t);
void* __libc_calloc(std::size_t, std::size_t);
void* __libc_realloc(void*, std::size_t);
void* __libc_memalign(std::size_t, std::size_t);
void __libc_free(void*);
}

namespace {
thread_local bool g_armed = false;  // per thread: only the armed thread is counted
bool g_trace = false;
bool g_split_coal = false;  // count allocations made inside libcoal separately
std::size_t g_count = 0;
std::size_t g_coal_count = 0;

bool inside_coal() {
    void* frames[32];
    const int n = backtrace(frames, 32);
    for (int i = 0; i < n; ++i) {
        Dl_info info;
        if (dladdr(frames[i], &info) != 0 && info.dli_fname != nullptr &&
            std::strstr(info.dli_fname, "libcoal"))
            return true;
    }
    return false;
}

void note() {
    if (!g_armed) return;
    if (g_split_coal) {
        g_armed = false;
        const bool coal = inside_coal();
        g_armed = true;
        if (coal) {
            ++g_coal_count;
            return;
        }
    }
    ++g_count;
    if (g_trace) {
        g_armed = false;
        void* frames[64];
        const int n = backtrace(frames, 64);
        const char sep[] = "---- allocation\n";
        (void)!write(2, sep, sizeof(sep) - 1);
        backtrace_symbols_fd(frames, n, 2);
        g_armed = true;
    }
}
}  // namespace

extern "C" {
void* malloc(std::size_t n) {
    note();
    return __libc_malloc(n);
}
void* calloc(std::size_t a, std::size_t b) {
    note();
    return __libc_calloc(a, b);
}
void* realloc(void* p, std::size_t n) {
    note();
    return __libc_realloc(p, n);
}
void* memalign(std::size_t a, std::size_t n) {
    note();
    return __libc_memalign(a, n);
}
void* aligned_alloc(std::size_t a, std::size_t n) {
    note();
    return __libc_memalign(a, n);
}
int posix_memalign(void** out, std::size_t a, std::size_t n) {
    note();
    void* p = __libc_memalign(a, n);
    if (!p) return 12;  // ENOMEM
    *out = p;
    return 0;
}
void free(void* p) { __libc_free(p); }
}

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

ModelPtr load_omni() {
    ModelOptions o;
    o.root = RootJoint::free_flyer;
    auto m = Model::load(ModelSource::file((kFixtures / "omni_dual_arm_gripper.urdf").string()), o,
                         Context::defaults());
    REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
    return *m;
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

// Allocations counted over `steps` solves after `warmup` solves.
std::size_t count_allocations(SolveMode mode, bool split_levels) {
    auto m = load_omni();
    SolverOptions o;
    o.mode = mode;
    auto created = Solver::create(m, o);
    REQUIRE(created);
    Solver solver = std::move(*created);

    const Eigen::VectorXd q0 = m->neutral();
    Eigen::Isometry3d left = pose_at(m, q0, "left_tool_frame");
    Eigen::Isometry3d right = pose_at(m, q0, "right_tool_frame");
    left.translation() += Eigen::Vector3d(0.3, 0.1, 0.05);
    right.translation() += Eigen::Vector3d(0.3, -0.1, 0.05);

    TaskOptions right_opt, low;
    right_opt.level = split_levels ? 1 : 0;
    low.level = split_levels ? 2 : 1;
    REQUIRE(solver.add_task("left", std::make_shared<FrameTask>("left_tool_frame", left)));
    REQUIRE(solver.add_task("right", std::make_shared<FrameTask>("right_tool_frame", right), right_opt));
    REQUIRE(solver.add_task("posture", std::make_shared<PostureTask>(q0), low));
    REQUIRE(
        solver.add_constraint("vel", std::make_shared<JointVelocityConstraint>(
                                         std::vector<std::string>{"left_arm_joint_1", "right_arm_joint_1"},
                                         Eigen::VectorXd::Constant(1, 0.5))));

    SolveResult r;
    Eigen::VectorXd q = q0;
    for (int i = 0; i < 20; ++i) {
        REQUIRE(solver.solve(q, 0.01, r));
        q = r.q_next;
    }
    Eigen::VectorXd next(q.size());
    g_count = 0;
    g_trace = std::getenv("KSTONE_ALLOC_TRACE") != nullptr;
    bool ok = true;
    for (int i = 0; i < 10; ++i) {
        g_armed = true;
        ok = ok && static_cast<bool>(solver.solve(q, 0.01, r));
        g_armed = false;
        next = r.q_next;
        q.swap(next);
    }
    REQUIRE(ok);
    return g_count;
}
}  // namespace

TEST_CASE("alloc: solve does not allocate after warm-up") {
    struct Row {
        const char* name;
        SolveMode mode;
        bool split;
    };
    for (const Row& row : {Row{"weighted", SolveMode::weighted, false},
                           Row{"layered_independent", SolveMode::layered_independent, true},
                           Row{"layered_coupled", SolveMode::layered_coupled, true},
                           Row{"lexicographic", SolveMode::lexicographic, true}}) {
        CAPTURE(std::string(row.name));
        const std::size_t n = count_allocations(row.mode, row.split);
        MESSAGE(std::string(row.name) << ": " << n << " allocations in 10 solves");
        CHECK(n == 0);
    }
}

// Collision rows active against a moving obstacle (set_pose keeps the layout), plus a self pair.
TEST_CASE("alloc: solve with a collision constraint does not allocate after warm-up") {
    auto loaded = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), ModelOptions{},
                              Context::defaults());
    REQUIRE(loaded);
    ModelPtr m = *loaded;
    std::vector<CollisionGeometry> geometries;
    for (const char* frame : {"link2", "link4", "tool"}) {
        CollisionGeometry g;
        g.name = std::string(frame) + "_box";
        g.frame = frame;
        g.shape = Shape::box(0.05, 0.04, 0.08);
        geometries.push_back(g);
    }
    auto collision = CollisionModel::create(m, geometries, CollisionModelOptions{}, Context::defaults());
    REQUIRE_MESSAGE(collision, (collision ? "" : collision.error().message));
    REQUIRE_FALSE((*collision)->pairs().empty());
    auto world = std::make_shared<CollisionWorld>();
    auto post = world->add("post", Shape::capsule(0.05, 0.5), Eigen::Isometry3d::Identity());
    REQUIRE(post);
    CollisionConstraintOptions co;
    co.activation_margin = 10.0;  // every pair yields a row
    co.max_rows = 2;              // and some are dropped
    auto con = CollisionConstraint::create(*collision, world, co);
    REQUIRE(con);

    auto created = Solver::create(m);
    REQUIRE(created);
    Solver solver = std::move(*created);
    Eigen::VectorXd target = m->neutral();
    REQUIRE(m->integrate(m->neutral(), Eigen::VectorXd::Constant(m->nv(), 0.3), target));
    REQUIRE(solver.add_task("posture", std::make_shared<PostureTask>(target)));
    REQUIRE(solver.add_constraint("collision", *con));

    const Eigen::VectorXd q0 = m->neutral();
    const Eigen::Isometry3d tool = pose_at(m, q0, "tool");
    auto place = [&](int i) {
        Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
        p.translation() = tool.translation() + Eigen::Vector3d(0.3 + 0.05 * std::sin(0.1 * i), 0.0, 0.0);
        return world->set_pose(*post, p);
    };
    SolveResult r;
    Eigen::VectorXd q = q0;
    for (int i = 0; i < 20; ++i) {
        REQUIRE(place(i));
        REQUIRE(solver.solve(q, 0.01, r));
        q = r.q_next;
    }
    Eigen::VectorXd next(q.size());
    g_count = 0;
    g_coal_count = 0;
    g_trace = std::getenv("KSTONE_ALLOC_TRACE") != nullptr;
    g_split_coal = true;
    bool ok = true;
    for (int i = 20; i < 30; ++i) {
        g_armed = true;
        ok = ok && static_cast<bool>(place(i));
        ok = ok && static_cast<bool>(solver.solve(q, 0.01, r));
        g_armed = false;
        next = r.q_next;
        q.swap(next);
    }
    g_split_coal = false;
    REQUIRE(ok);
    CHECK((*con)->report().rows == 2);
    CHECK((*con)->report().dropped > 0);
    // Coal 3.0.x builds without COAL_TURN_ASSERT_INTO_EXCEPTION still construct the exception of
    // every COAL_ASSERT (fwd.hh), i.e. a std::string per check per distance call. That is inside
    // libcoal and reported, not hidden; everything outside it must stay allocation free.
    MESSAGE("collision: " << g_count << " allocations outside libcoal, " << g_coal_count
                          << " inside libcoal in 10 solves");
    CHECK(g_count == 0);
}

// Pose tracking through Ruckig with the command lease refreshed every cycle.
TEST_CASE("alloc: chassis step does not allocate after warm-up") {
    for (ChassisKinematics kind : {ChassisKinematics::omni, ChassisKinematics::differential}) {
        CAPTURE(std::string(to_string(kind)));
        ChassisConfig cfg;
        cfg.kinematics = kind;
        auto created = ChassisController::create(cfg);
        REQUIRE(created);
        ChassisController ctrl = std::move(*created);
        const double dt = 0.02;
        REQUIRE(ctrl.set_scheduler_period(dt));
        REQUIRE(ctrl.start(0.0));
        const Pose2D goal{1.0, 0.5, 0.8};
        double now = 0.0;
        for (int i = 0; i < 5; ++i) {
            now += dt;
            REQUIRE(ctrl.submit_pose(goal, now));
            REQUIRE(ctrl.step(now).reason == ChassisReason::tracking);
        }
        g_count = 0;
        g_trace = std::getenv("KSTONE_ALLOC_TRACE") != nullptr;
        bool ok = true;
        for (int i = 0; i < 200; ++i) {
            now += dt;
            g_armed = true;
            auto accepted = ctrl.submit_pose(goal, now);
            ok = ok && accepted && *accepted;
            ok = ok && ctrl.step(now).state == ChassisState::active;
            g_armed = false;
        }
        CHECK(ok);
        CHECK(g_count == 0);
    }
}

// Ruckig replans every cycle; the target moves so no cycle short-circuits.
TEST_CASE("alloc: joint trajectory step does not allocate after warm-up") {
    for (JointSync sync : {JointSync::time, JointSync::phase, JointSync::none}) {
        CAPTURE(std::string(to_string(sync)));
        JointTrajectoryConfig cfg;
        cfg.trigger = JointTrigger::continuous;
        cfg.sync = sync;
        cfg.velocity_cap = Eigen::VectorXd::Constant(6, 0.5);
        auto created = JointTrajectory::create(6, cfg, {false, false, false, false, false, true});
        REQUIRE(created);
        JointTrajectory traj = std::move(*created);
        const Eigen::VectorXd limit = Eigen::VectorXd::Ones(6);
        Eigen::VectorXd position = Eigen::VectorXd::Zero(6), target(6);
        const double dt = 0.01;
        auto cycle = [&](int i) {
            for (Eigen::Index j = 0; j < 6; ++j)
                target[j] = std::sin(0.01 * i + static_cast<double>(j)) * (1.0 + static_cast<double>(j));
            const bool stepped = static_cast<bool>(traj.step(position, target, traj.velocity(), limit, dt));
            position = traj.reference();
            return stepped;
        };
        for (int i = 0; i < 5; ++i) REQUIRE(cycle(i));
        g_count = 0;
        g_trace = std::getenv("KSTONE_ALLOC_TRACE") != nullptr;
        bool ok = true;
        for (int i = 5; i < 205; ++i) {
            g_armed = true;
            ok = cycle(i) && ok;
            g_armed = false;
        }
        CHECK(ok);
        CHECK(traj.active());
        CHECK(g_count == 0);
    }
}

// Moving target forces a replan every cycle; the blocked frame exercises the leash.
TEST_CASE("alloc: pose trajectory step does not allocate after warm-up") {
    for (PoseSync sync : {PoseSync::axis_phase, PoseSync::axis_time}) {
        CAPTURE(std::string(to_string(sync)));
        PoseTrajectoryConfig cfg;
        cfg.sync = sync;
        cfg.translation = {0.3, 1.0, 0.0, 0.0, 8.0};
        cfg.rotation = {1.0, 2.0, 0.0, 0.0, 10.0};
        cfg.lead_speed_scale = 2.0;
        auto created = PoseTrajectory::create(cfg);
        REQUIRE(created);
        PoseTrajectory traj = std::move(*created);
        const Eigen::Isometry3d observed = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
        auto cycle = [&](int i) {
            const double t = 0.02 * i;
            target.translation() << 0.5 + 0.1 * std::sin(t), 0.2 * std::cos(t), 0.1;
            target.linear() =
                Eigen::AngleAxisd(0.5 + 0.2 * std::sin(t), Eigen::Vector3d::UnitZ()).toRotationMatrix();
            return static_cast<bool>(traj.step(observed, target, 0.02));
        };
        for (int i = 0; i < 5; ++i) REQUIRE(cycle(i));
        g_count = 0;
        g_trace = std::getenv("KSTONE_ALLOC_TRACE") != nullptr;
        bool ok = true;
        for (int i = 5; i < 205; ++i) {
            g_armed = true;
            ok = cycle(i) && ok;
            g_armed = false;
        }
        CHECK(ok);
        CHECK(g_count == 0);
    }
}

// Chassis + two frame tasks + a joint target that claims the left arm (holding the left frame task) and releases it.
TEST_CASE("alloc: module set step does not allocate after warm-up") {
    auto m = load_omni();
    auto created = ModuleSet::create(m);
    REQUIRE(created);
    ModuleSet set = std::move(*created);

    ChassisModuleConfig cc;
    cc.period = 0.01;
    auto chassis = std::make_shared<ChassisModule>("chassis", cc);
    PoseTrajectoryConfig pc;
    pc.translation = {0.3, 1.0, 0.0, 0.0, 10.0};
    pc.rotation = {1.0, 3.0, 0.0, 0.0, 20.0};
    FrameTaskModuleConfig lc;
    lc.frame = "left_tool_frame";
    lc.trajectory = pc;
    FrameTaskModuleConfig rc = lc;
    rc.frame = "right_tool_frame";
    lc.on_held = OnHeld::abort;  // covers on_preempted()
    auto left = std::make_shared<FrameTaskModule>("left", lc);
    auto right = std::make_shared<FrameTaskModule>("right", rc);
    JointTargetModuleConfig jc;
    for (int i = 1; i <= 7; ++i) jc.joints.push_back("left_arm_joint_" + std::to_string(i));
    auto arm = std::make_shared<JointTargetModule>("left_arm", jc);
    for (const auto& mod : std::vector<std::shared_ptr<Module>>{chassis, left, right, arm})
        REQUIRE(set.add(mod));
    REQUIRE(set.configure());

    const Eigen::VectorXd q0 = m->neutral();
    Eigen::Isometry3d lt = pose_at(m, q0, "left_tool_frame");
    Eigen::Isometry3d rt = pose_at(m, q0, "right_tool_frame");
    Eigen::VectorXd arm_goal = Eigen::VectorXd::Constant(7, 0.2);
    Eigen::VectorXd q = q0, next(q0.size()), v = Eigen::VectorXd::Zero(m->nv());
    SolveResult r;
    double now = 0.0;
    auto cycle = [&](int i) {
        const double t = 0.01 * i;
        now += 0.01;
        bool ok = static_cast<bool>(chassis->controller()->submit_pose({0.2 * std::sin(t), 0.1, 0.1}, now));
        lt.translation().x() += 1e-4;
        rt.translation().y() -= 1e-4;
        ok = ok && left->set_target(lt) && right->set_target(rt);
        if (i % 60 == 0) {
            arm_goal *= -1.0;
            ok = ok && arm->set_target(arm_goal);
        }
        ok = ok && set.step(q, v, now, 0.01, r);
        v = r.v;
        return ok;
    };
    for (int i = 0; i < 30; ++i) {
        REQUIRE(cycle(i));
        q = r.q_next;
    }
    int held = 0, resumed = 0;
    bool was_running = left->running();
    g_count = 0;
    g_trace = std::getenv("KSTONE_ALLOC_TRACE") != nullptr;
    bool ok = true;
    for (int i = 30; i < 330; ++i) {
        g_armed = true;
        ok = cycle(i) && ok;
        g_armed = false;
        next = r.q_next;
        q.swap(next);
        if (was_running && !left->running()) ++held;
        if (!was_running && left->running()) ++resumed;
        was_running = left->running();
    }
    CHECK(ok);
    MESSAGE("module set: left held " << held << "x, resumed " << resumed << "x in 300 steps");
    CHECK(held > 0);
    CHECK(resumed > 0);
    CHECK(left->preempted() >= static_cast<std::uint64_t>(held));
    CHECK(g_count == 0);
}

// Gaze over both hands, null-space preference switching on the right arm, centre-of-mass support and periodic homing.
TEST_CASE("alloc: gaze, null-space preference, centre of mass and homing do not allocate after warm-up") {
    auto m = load_omni();
    auto created = ModuleSet::create(m);
    REQUIRE(created);
    ModuleSet set = std::move(*created);

    PoseTrajectoryConfig pc;
    pc.translation = {0.3, 1.0, 0.0, 0.0, 10.0};
    pc.rotation = {1.0, 3.0, 0.0, 0.0, 20.0};
    FrameTaskModuleConfig lc;
    lc.frame = "left_tool_frame";
    lc.trajectory = pc;
    FrameTaskModuleConfig rc = lc;
    rc.frame = "right_tool_frame";
    rc.task.level = 0;
    for (int i = 1; i <= 7; ++i) {
        lc.joints.push_back("left_arm_joint_" + std::to_string(i));
        rc.joints.push_back("right_arm_joint_" + std::to_string(i));
    }
    auto left = std::make_shared<FrameTaskModule>("left", lc);
    auto right = std::make_shared<FrameTaskModule>("right", rc);
    GazeModuleConfig gc;
    gc.frame = "head_yaw_link";
    gc.joints = {"head_pitch_joint", "head_yaw_joint"};
    gc.frames = {"left_tool_frame", "right_tool_frame"};
    gc.max_velocity = 1.0;
    gc.target_idle_s = 0.3;
    auto gaze = std::make_shared<GazeModule>("gaze", gc);
    NullSpacePreferenceModuleConfig nc;
    nc.primary = right;
    auto ns = std::make_shared<NullSpacePreferenceModule>("ns", nc);
    CenterOfMassModuleConfig cc;
    cc.frame = "base_link";
    cc.polygon = {Eigen::Vector2d(-1, -1), Eigen::Vector2d(1, -1), Eigen::Vector2d(1, 1),
                  Eigen::Vector2d(-1, 1)};
    cc.recovery_tasks = {left, right};
    auto com = std::make_shared<CenterOfMassModule>("com", cc);
    for (const auto& mod : std::vector<std::shared_ptr<Module>>{left, right, gaze, ns, com})
        REQUIRE(set.add(mod));
    const Status configured = set.configure();
    REQUIRE_MESSAGE(configured, (configured ? "" : configured.error().message));

    const Eigen::VectorXd q0 = m->neutral();
    Eigen::Isometry3d lt = pose_at(m, q0, "left_tool_frame");
    Eigen::Isometry3d rt = pose_at(m, q0, "right_tool_frame");
    Eigen::VectorXd q = q0, next(q0.size()), v = Eigen::VectorXd::Zero(m->nv());
    SolveResult r;
    double now = 0.0;
    int homes = 0;
    auto cycle = [&](int i) {
        now += 0.01;
        bool ok = true;
        const int phase = i % 100;
        if (phase < 50) {  // move both hands, then let them idle so gaze drops them and homes
            lt.translation().x() += 1e-4;
            rt.translation().y() -= 1e-4;
            const bool homing =
                left->home_status() == HomeStatus::waiting || left->home_status() == HomeStatus::executing;
            ok = ok && (homing || left->set_target(lt)) && right->set_target(rt);
        } else if (phase == 60) {
            ok = ok && left->request_home();
            ++homes;
        }
        if (i % 40 == 0) ns->set_preference(static_cast<NullSpacePreference>((i / 40) % 3));
        ok = ok && set.step(q, v, now, 0.01, r);
        v = r.v;
        return ok;
    };
    for (int i = 0; i < 100; ++i) {
        REQUIRE(cycle(i));
        q = r.q_next;
    }
    int gaze_switches = 0, home_done = 0;
    std::size_t last_frames = gaze->active_frames();
    g_count = 0;
    g_trace = std::getenv("KSTONE_ALLOC_TRACE") != nullptr;
    bool ok = true;
    for (int i = 100; i < 400; ++i) {
        g_armed = true;
        ok = cycle(i) && ok;
        g_armed = false;
        next = r.q_next;
        q.swap(next);
        if (gaze->active_frames() != last_frames) ++gaze_switches;
        last_frames = gaze->active_frames();
        if (left->home_status() == HomeStatus::succeeded) ++home_done;
    }
    CHECK(ok);
    MESSAGE("gaze track changes " << gaze_switches << ", homes requested " << homes);
    CHECK(gaze_switches > 0);
    CHECK(home_done > 0);
    CHECK(g_count == 0);
}

// A clamping velocity guard inside a full engine cycle.
TEST_CASE("alloc: engine step with a clamping guard does not allocate after warm-up") {
    auto loaded = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), ModelOptions{},
                              Context::defaults());
    REQUIRE(loaded);
    ModelPtr m = *loaded;
    SolverOptions so;
    so.velocity_limit = Eigen::VectorXd::Constant(m->nv(), 1000.0);  // let the solver exceed the model limit
    auto created = ModuleSet::create(m, so);
    REQUIRE(created);
    JointTargetModuleConfig jc;
    jc.joints = {"j1"};
    jc.velocity_limit = Eigen::VectorXd::Constant(1, 100.0);
    auto arm = std::make_shared<JointTargetModule>("arm", jc);
    REQUIRE(created->add(arm));
    auto robot = std::make_shared<LoopbackRobot>(m->neutral(), m->nv());
    EngineOptions eo;
    eo.rate_hz = 100.0;
    eo.realtime = false;
    eo.safety.mode = GuardMode::clamp;
    auto engine = Engine::create(std::move(*created), robot, eo);
    REQUIRE(engine);

    Eigen::VectorXd goal = Eigen::VectorXd::Constant(1, 1.9);
    REQUIRE(arm->set_target(goal));
    bool ok = true;
    auto cycle = [&](int i) {
        if (i % 80 == 0) {
            goal *= -1.0;
            ok = static_cast<bool>(arm->set_target(goal)) && ok;
        }
        ok = static_cast<bool>(engine->step(0.01 * i)) && ok;
    };
    for (int i = 1; i < 30; ++i) cycle(i);
    REQUIRE(ok);
    const std::uint64_t clamps = engine->stats().guard_clamps;
    g_count = 0;
    g_trace = std::getenv("KSTONE_ALLOC_TRACE") != nullptr;
    for (int i = 30; i < 330; ++i) {
        g_armed = true;
        cycle(i);
        g_armed = false;
    }
    CHECK(ok);
    MESSAGE("engine: " << engine->stats().guard_clamps - clamps << " clamped cycles in 300 steps");
    CHECK(engine->stats().guard_clamps > clamps);
    CHECK(engine->stats().guard_rejects == 0);
    CHECK(g_count == 0);
}

// Module display outputs recomputed by an attached debug server (no client), inside the engine cycle.
TEST_CASE("alloc: engine step with debug server displays does not allocate after warm-up") {
    auto loaded = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), ModelOptions{},
                              Context::defaults());
    REQUIRE(loaded);
    ModelPtr m = *loaded;
    std::vector<CollisionGeometry> geometries;
    for (const char* frame : {"link2", "link4", "tool"}) {
        CollisionGeometry g;
        g.name = std::string(frame) + "_box";
        g.frame = frame;
        g.shape = Shape::box(0.05, 0.04, 0.08);
        geometries.push_back(g);
    }
    auto collision = CollisionModel::create(m, geometries, CollisionModelOptions{}, Context::defaults());
    REQUIRE(collision);
    CollisionModelOptions no_self;
    no_self.self_collision = false;
    auto robot_geometry = CollisionModel::create(m, geometries, no_self, Context::defaults());
    REQUIRE(robot_geometry);
    auto created = ModuleSet::create(m);
    REQUIRE(created);
    SelfCollisionModuleConfig sc;
    sc.model = *collision;
    sc.constraint.activation_margin = 10.0;  // every pair is in the band
    WorldCollisionModuleConfig wc;
    wc.model = *robot_geometry;
    WorldObstacle post;
    post.name = "post";
    post.shape = Shape::capsule(0.05, 0.5);
    wc.obstacles = {post};
    wc.constraint.activation_margin = 10.0;
    auto world = std::make_shared<WorldCollisionModule>("world", wc);
    CenterOfMassModuleConfig cc;
    cc.frame = "base_link";
    cc.margin = 0.05;
    cc.polygon = {Eigen::Vector2d(-1, -1), Eigen::Vector2d(1, -1), Eigen::Vector2d(1, 1),
                  Eigen::Vector2d(-1, 1)};
    JointTargetModuleConfig jc;
    jc.joints = {"j1"};
    auto arm = std::make_shared<JointTargetModule>("arm", jc);
    REQUIRE(created->add(std::make_shared<SelfCollisionModule>("collision", sc)));
    REQUIRE(created->add(world));
    REQUIRE(created->add(std::make_shared<CenterOfMassModule>("com", cc)));
    REQUIRE(created->add(arm));
    auto robot = std::make_shared<LoopbackRobot>(m->neutral(), m->nv());
    EngineOptions eo;
    eo.rate_hz = 100.0;
    eo.realtime = false;
    auto engine = Engine::create(std::move(*created), robot, eo);
    REQUIRE(engine);
    DebugServerOptions dso;
    dso.port = 0;
    dso.publish_hz = 1000.0;  // recompute the displays on nearly every cycle
    dso.serve_assets = false;
    auto server = DebugServer::start(*engine, dso);
    REQUIRE_MESSAGE(server, (server ? std::string{} : server.error().message));

    Eigen::VectorXd goal = Eigen::VectorXd::Constant(1, 0.8);
    bool ok = static_cast<bool>(arm->set_target(goal));
    auto cycle = [&](int i) {
        if (i % 80 == 0) {
            goal *= -1.0;
            ok = static_cast<bool>(arm->set_target(goal)) && ok;
        }
        PortValue pv;
        pv.pose.translation().x() = 0.3 + 0.001 * (i % 50);
        pv.flag = i % 40 < 30;
        ok = static_cast<bool>(world->write_port(0, pv)) && static_cast<bool>(world->write_port(1, pv)) && ok;
        ok = static_cast<bool>(engine->step(0.01 * i)) && ok;
    };
    for (int i = 1; i < 30; ++i) {
        cycle(i);
        usleep(2000);  // let the IO thread rotate through every snapshot buffer
    }
    REQUIRE(ok);
    g_count = 0;
    g_coal_count = 0;
    g_trace = std::getenv("KSTONE_ALLOC_TRACE") != nullptr;
    g_split_coal = true;
    for (int i = 30; i < 230; ++i) {
        g_armed = true;
        cycle(i);
        g_armed = false;
        if (i % 20 == 0) usleep(1000);  // let the publish period elapse
    }
    g_split_coal = false;
    CHECK(ok);
    MESSAGE("displays: " << g_count << " allocations outside libcoal, " << g_coal_count << " inside libcoal");
    CHECK(g_count == 0);
}

TEST_CASE("alloc: telemetry snapshots do not allocate after warm-up") {
    struct Sink final : TelemetrySink {
        void publish(const EngineSnapshot& s) override {
            ++count;
            ports += s.modules.empty() ? 0 : s.modules[0].ports.size();
        }
        std::size_t count{0}, ports{0};
    };
    auto loaded = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), ModelOptions{},
                              Context::defaults());
    REQUIRE(loaded);
    ModelPtr m = *loaded;
    auto created = ModuleSet::create(m);
    REQUIRE(created);
    JointTargetModuleConfig jc;
    jc.joints = {"j1", "j2"};
    auto arm = std::make_shared<JointTargetModule>("arm", jc);
    REQUIRE(created->add(arm));
    auto robot = std::make_shared<LoopbackRobot>(m->neutral(), m->nv());
    EngineOptions eo;
    eo.realtime = false;
    auto engine = Engine::create(std::move(*created), robot, eo);
    REQUIRE(engine);
    auto sink = std::make_shared<Sink>();
    engine->add_observer(make_telemetry_observer(sink));

    Eigen::VectorXd goal = Eigen::VectorXd::Constant(2, 0.8);
    bool ok = static_cast<bool>(arm->set_target(goal));
    for (int i = 1; i < 30; ++i) ok = static_cast<bool>(engine->step(0.01 * i)) && ok;
    REQUIRE(ok);
    g_count = 0;
    g_trace = std::getenv("KSTONE_ALLOC_TRACE") != nullptr;
    for (int i = 30; i < 230; ++i) {
        g_armed = true;
        ok = static_cast<bool>(engine->step(0.01 * i)) && ok;
        g_armed = false;
    }
    CHECK(ok);
    CHECK(sink->count == 229);
    CHECK(sink->ports > 0);
    CHECK(g_count == 0);
}
