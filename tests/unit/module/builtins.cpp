#include "kstone/module/builtins.hpp"

#include "kstone/collision/model.hpp"
#include "kstone/collision/world.hpp"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

ModelPtr load(const char* file, ModelOptions o = {}) {
    auto m = Model::load(ModelSource::file((kFixtures / file).string()), o, Context::defaults());
    REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
    return *m;
}

ModuleSet make_set(const ModelPtr& m) {
    auto s = ModuleSet::create(m);
    REQUIRE_MESSAGE(s, (s ? "" : s.error().message));
    return std::move(*s);
}

PoseTrajectoryConfig pose_limits() {
    PoseTrajectoryConfig c;
    c.translation = {0.3, 1.0, 0.0, 0.0, 10.0};
    c.rotation = {1.0, 3.0, 0.0, 0.0, 20.0};
    return c;
}

Eigen::Isometry3d pose_at(const ModelPtr& m, const Eigen::VectorXd& q, const std::string& frame) {
    Workspace ws(m);
    REQUIRE(ws.update(q));
    Eigen::Isometry3d p;
    REQUIRE(ws.frame_pose(m->frame_id(frame).value(), p));
    return p;
}

double joint_q(const ModelPtr& m, const Eigen::VectorXd& q, const std::string& name) {
    return m->joint_position(q, m->joint_id(name).value()).value();
}

struct Loop {
    Loop(ModelPtr m, ModuleSet& s, Eigen::VectorXd q0) : model(std::move(m)), set(s), q(std::move(q0)) {}

    ModelPtr model;
    ModuleSet& set;
    Eigen::VectorXd q;
    SolveResult out;
    double now{0.0};
    double dt{0.01};

    void step(int n = 1) {
        for (int i = 0; i < n; ++i) {
            now += dt;
            const Eigen::VectorXd v = out.v.size() ? out.v : Eigen::VectorXd::Zero(model->nv());
            Status s = set.step(q, v, now, dt, out);
            REQUIRE_MESSAGE(s, (s ? "" : s.error().message));
            q = out.q_next;
        }
    }
};
}  // namespace

TEST_CASE("homing: state machine, timeout, preemption and skipped steps") {
    ModelPtr m = load("arm.urdf");
    Workspace ws(m);
    Eigen::VectorXd q = m->neutral();
    const Eigen::VectorXd v = Eigen::VectorXd::Zero(m->nv());
    auto at = [&](std::uint64_t index, double now) { return Cycle{now, 0.01, index, q, v, ws}; };
    const Mask joints(static_cast<std::size_t>(m->nv()), true);

    CHECK_FALSE(Homing::validate({0.0, 1.0}));
    CHECK_FALSE(Homing::validate({0.5, 0.0}));
    CHECK_FALSE(Homing::validate({0.5, -1.0}));
    CHECK_FALSE(Homing::validate({1.5, 1.0}));
    CHECK_FALSE(Homing::validate({0.5, std::nan("")}));

    {
        Homing h;
        REQUIRE(h.set_config({0.5, 1.0}));
        CHECK(h.claim(joints).joints == nullptr);
        CHECK_FALSE(h.start());
        h.request();
        CHECK(h.status() == HomeStatus::waiting);
        CHECK(h.claim(joints).joints == &joints);
        CHECK(h.claim(joints).priority == priority::preemptive);
        CHECK_FALSE(h.update(at(1, 0.0)));
        CHECK_FALSE(h.update(at(2, 5.0)));  // waiting never times out
        CHECK(h.start());
        CHECK(h.status() == HomeStatus::executing);
        CHECK(h.claim(joints).on_held == OnHeld::wait);  // preemptive: never aborts when held
        CHECK_FALSE(h.update(at(3, 6.0)));
        CHECK(h.update(at(4, 7.5)));
        CHECK(h.status() == HomeStatus::timed_out);
        CHECK(h.claim(joints).joints == nullptr);
        h.request();
        CHECK_FALSE(h.update(at(5, 8.0)));
        CHECK(h.start());
        h.finish(HomeStatus::succeeded);
        CHECK(h.status() == HomeStatus::succeeded);
    }
    {
        Homing h;
        REQUIRE(h.set_config({0.5, 1.0}));
        h.request();
        CHECK_FALSE(h.update(at(1, 0.0)));
        CHECK(h.start());
        // Steady progress keeps it running past timeout_s with no total cap.
        double error = 1.0;
        std::uint64_t index = 2;
        for (double now = 0.5; now < 10.0; now += 0.5, ++index) {
            CHECK_FALSE(h.update(at(index, now)));
            h.progress(now, error, false);
            error -= 0.05;
        }
        CHECK(h.status() == HomeStatus::executing);
        // Stalled, jittering below epsilon or NaN: no progress, so the clock runs out.
        const double anchor = error + 0.05;
        h.progress(10.0, anchor + 1e-3, false);
        h.progress(10.4, anchor - Homing::progress_epsilon / 2, false);
        h.progress(10.6, std::nan(""), false);
        CHECK_FALSE(h.update(at(index++, 10.4)));
        CHECK(h.update(at(index++, 10.9)));
        CHECK(h.status() == HomeStatus::timed_out);
    }
    {
        Homing h;
        REQUIRE(h.set_config({0.5, 1.0}));
        h.request();
        CHECK_FALSE(h.update(at(1, 0.0)));
        CHECK(h.start());
        CHECK_FALSE(h.update(at(2, 0.1)));
        h.progress(0.1, 0.5, false);
        h.progress(0.9, 0.5, true);  // arrived but still settling counts as progress
        CHECK_FALSE(h.update(at(3, 1.8)));
        CHECK(h.update(at(4, 2.0)));
    }
    {
        Homing h;
        h.request();
        CHECK_FALSE(h.update(at(1, 0.0)));
        CHECK(h.update(at(5, 0.1)));
        CHECK(h.status() == HomeStatus::aborted);
        CHECK(std::string(to_string(HomeStatus::aborted)) == "aborted");
    }
}

TEST_CASE("frame_task module: holds its pose, then tracks a target through the trajectory") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig c;
    c.frame = "tool";
    c.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", c);
    REQUIRE(set.add(ft));
    REQUIRE(set.configure());

    Eigen::VectorXd q0 = m->neutral();
    q0[m->joints()[m->joint_id("j2").value()].idx_q] = 0.3;
    Loop loop{m, set, q0};
    const Eigen::Isometry3d start = pose_at(m, q0, "tool");
    loop.step(50);
    CHECK((pose_at(m, loop.q, "tool").translation() - start.translation()).norm() < 1e-6);
    CHECK(ft->target().isApprox(start, 1e-9));
    CHECK_FALSE(ft->task()->mask_columns());  // joints scope the task; the mode decides the coupling

    Eigen::VectorXd q_goal = q0;
    for (const char* j : {"j1", "j2", "j3", "j4"}) {
        const JointId id = m->joint_id(j).value();
        REQUIRE(m->set_joint_position(q_goal, id, m->joint_position(q_goal, id).value() + 0.05));
    }
    const Eigen::Isometry3d goal = pose_at(m, q_goal, "tool");
    Eigen::Isometry3d bad = goal;
    bad.linear() *= 2.0;
    CHECK_FALSE(ft->set_target(bad));
    Status accepted = ft->set_target(goal);
    REQUIRE_MESSAGE(accepted, (accepted ? "" : accepted.error().message));
    loop.step(2);
    // The ramp moves the task target gradually.
    CHECK((ft->task()->target().translation() - goal.translation()).norm() > 1e-4);
    Workspace ws(m);
    REQUIRE(ws.update(loop.q));
    REQUIRE(ft->outputs().size() == 6);
    REQUIRE(ft->update_outputs(ws));
    OutputValue planned, lag, speed, error, angular_error;
    REQUIRE(ft->read_output(0, planned));
    REQUIRE(ft->read_output(1, lag));
    REQUIRE(ft->read_output(2, speed));
    REQUIRE(ft->read_output(4, error));
    CHECK(error.number ==
          doctest::Approx((goal.translation() - pose_at(m, loop.q, "tool").translation()).norm()));
    CHECK((planned.points.col(0) - ft->task()->target().translation()).norm() < 1e-12);
    CHECK(lag.number > 1e-4);
    CHECK(speed.number > 0.0);
    CHECK(set.task_levels("tool") == std::vector<int>{0});
    loop.step(600);
    REQUIRE(ws.update(loop.q));
    REQUIRE(ft->update_outputs(ws));
    REQUIRE(ft->read_output(1, lag));
    CHECK(lag.number < 1e-6);
    REQUIRE(ft->read_output(4, error));
    REQUIRE(ft->read_output(5, angular_error));
    CHECK(error.number < 1e-4);
    CHECK(angular_error.number < 1e-3);
    const Eigen::Isometry3d reached = pose_at(m, loop.q, "tool");
    CHECK((reached.translation() - goal.translation()).norm() < 1e-4);
    CHECK((reached.linear() - goal.linear()).norm() < 1e-3);
}

TEST_CASE("frame_task module: per-axis weights and lm_damping reach the task and tune") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig c;
    c.frame = "tool";
    c.trajectory = pose_limits();
    c.position_weight = Eigen::Vector3d(2.0, 0.0, 1.0);
    c.weight = 3.0;
    c.lm_damping = 0.5;
    auto ft = std::make_shared<FrameTaskModule>("tool", c);
    REQUIRE(set.add(ft));
    REQUIRE(set.configure());
    Eigen::VectorXd expected(6);
    expected << 6.0, 0.0, 3.0, 3.0, 3.0, 3.0;
    CHECK(ft->task()->weights().isApprox(expected));
    CHECK(ft->task()->lm_damping() == 0.5);

    auto vec = [](double x, double y, double z) { return ConfigNode::list({x, y, z}); };
    REQUIRE(ft->tune(ConfigNode::map({{"orientation_weight", vec(1.0, 0.0, 0.5)}, {"lm_damping", 1.5}})));
    expected << 6.0, 0.0, 3.0, 3.0, 0.0, 1.5;
    CHECK(ft->task()->weights().isApprox(expected));
    CHECK(ft->task()->lm_damping() == 1.5);
    CHECK_FALSE(ft->tune(ConfigNode::map({{"position_weight", vec(1.0, -1.0, 1.0)}})));
    CHECK_FALSE(ft->tune(ConfigNode::map({{"lm_damping", -1.0}})));
    CHECK(ft->task()->weights().isApprox(expected));
}

TEST_CASE("frame_task module: explicit joints must move the frame relative to its reference") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig c;
    c.frame = "tool";
    c.reference = "tool";
    c.joints = {"j1", "j2"};
    c.trajectory = pose_limits();
    REQUIRE(set.add(std::make_shared<FrameTaskModule>("tool", c)));
    Status s = set.configure();
    REQUIRE_FALSE(s);
    CHECK(s.error().message.find("none of the joints moves") != std::string::npos);
}

TEST_CASE("frame_task module: locked freezes the chain, unlocking holds the current pose") {
    ModelPtr m = load("arm.urdf");
    {
        ModuleSet set = make_set(m);
        FrameTaskModuleConfig c;
        c.frame = "tool";
        c.trajectory = pose_limits();
        c.locked = true;
        REQUIRE(set.add(std::make_shared<FrameTaskModule>("tool", c)));
        CHECK_FALSE(set.configure());  // locking needs explicit joints
    }
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig c;
    c.frame = "tool";
    c.joints = {"j1", "j2", "j3", "j4"};
    c.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", c);
    REQUIRE(set.add(ft));
    REQUIRE(set.configure());
    CHECK(ft->tunables().find("locked")->as_bool() == false);

    Eigen::VectorXd q0 = m->neutral();
    q0[m->joints()[m->joint_id("j2").value()].idx_q] = 0.3;
    Loop loop{m, set, q0};
    loop.step(5);
    Eigen::VectorXd q_goal = loop.q;
    for (const std::string& j : c.joints) {
        const JointId id = m->joint_id(j).value();
        REQUIRE(m->set_joint_position(q_goal, id, m->joint_position(q_goal, id).value() + 0.2));
    }
    const Eigen::Isometry3d goal = pose_at(m, q_goal, "tool");
    REQUIRE(ft->set_target(goal));
    loop.step(20);
    REQUIRE(ft->tune(ConfigNode::map({{"locked", true}})));
    CHECK(ft->tunables().find("locked")->as_bool());
    loop.step(1);
    const Eigen::VectorXd frozen = loop.q;
    loop.step(100);
    CHECK((loop.q - frozen).norm() < 1e-9);

    REQUIRE(ft->tune(ConfigNode::map({{"locked", false}})));
    loop.step(100);
    // The goal set before locking is dropped: the frame holds where it was frozen.
    CHECK((loop.q - frozen).norm() < 1e-6);
    CHECK(ft->target().isApprox(pose_at(m, frozen, "tool"), 1e-6));
    CHECK((pose_at(m, loop.q, "tool").translation() - goal.translation()).norm() > 1e-3);
}

TEST_CASE("joint_target module: claims past arrival until a pose target takes over") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    JointTargetModuleConfig jc;
    jc.joints = {"j1", "j2", "j3"};
    jc.priority = 1;
    jc.release_after_s = 0.0;  // immediate handover; the watchdog has its own test
    auto jt = std::make_shared<JointTargetModule>("joints", jc);
    REQUIRE(set.add(ft));
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());

    Loop loop{m, set, m->neutral()};
    loop.step(5);
    CHECK(ft->running());
    CHECK(jt->running());
    CHECK_FALSE(jt->claiming());

    CHECK_FALSE(jt->set_target(Eigen::VectorXd::Zero(2)));
    Eigen::VectorXd target(3);
    target << 0.5, -0.3, 3.0;
    REQUIRE(jt->set_target(target));
    loop.step();
    CHECK(jt->claiming());
    CHECK_FALSE(ft->running());
    REQUIRE(ft->held_by());
    CHECK(*ft->held_by() == "joints");

    int steps = 0;
    while (!jt->settled() && steps < 2000) {
        loop.step();
        ++steps;
    }
    INFO("steps " << steps);
    CHECK(jt->settled());
    CHECK(joint_q(m, loop.q, "j1") == doctest::Approx(0.5).epsilon(1e-3));
    CHECK(joint_q(m, loop.q, "j2") == doctest::Approx(-0.3).epsilon(1e-3));
    CHECK(std::abs(std::remainder(joint_q(m, loop.q, "j3") - 3.0, 2 * M_PI)) < 1e-3);
    // Arrival keeps the claim: no input, no handover.
    loop.step(100);
    CHECK(jt->claiming());
    CHECK_FALSE(ft->running());

    // A pose and a joint target in the same cycle: the joint target wins.
    Eigen::VectorXd q_goal = loop.q;
    q_goal[m->joints()[m->joint_id("j2").value()].idx_q] += 0.1;
    const Eigen::Isometry3d goal = pose_at(m, q_goal, "tool");
    REQUIRE(ft->set_target(goal));
    REQUIRE(jt->set_target(target));
    loop.step();
    CHECK(jt->claiming());
    CHECK_FALSE(ft->running());

    // A pose alone takes the joints over this cycle and is tracked.
    REQUIRE(ft->set_target(goal));
    loop.step();
    CHECK_FALSE(jt->claiming());
    CHECK(ft->running());
    CHECK(ft->target().isApprox(goal, 1e-9));
    loop.step(800);
    CHECK((pose_at(m, loop.q, "tool").translation() - goal.translation()).norm() < 1e-4);
    CHECK_FALSE(jt->claiming());

    // A new joint target claims again.
    REQUIRE(jt->set_target(target));
    loop.step();
    CHECK(jt->claiming());
    CHECK_FALSE(ft->running());
}

TEST_CASE("joint_target module: release_after_s defers a request until joint input stops") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    JointTargetModuleConfig jc;
    jc.joints = {"j1", "j2", "j3"};
    jc.priority = 1;
    jc.release_after_s = 0.5;
    auto jt = std::make_shared<JointTargetModule>("joints", jc);
    REQUIRE(set.add(ft));
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());

    Loop loop{m, set, m->neutral()};
    loop.step(5);
    Eigen::VectorXd target(3);
    target << 0.3, -0.2, 0.1;
    REQUIRE(jt->set_target(target));
    loop.step(10);
    Eigen::VectorXd q_goal = loop.q;
    q_goal[m->joints()[m->joint_id("j2").value()].idx_q] += 0.1;
    const Eigen::Isometry3d goal = pose_at(m, q_goal, "tool");
    // 0.1 s after the joint input: the pose request waits.
    REQUIRE(ft->set_target(goal));
    loop.step(20);
    CHECK(jt->claiming());
    CHECK_FALSE(ft->running());
    // Joint input inside the window restarts it and cancels the pending release.
    REQUIRE(jt->set_target(target));
    loop.step(40);
    CHECK(jt->claiming());
    CHECK_FALSE(ft->running());
    // A request now waits out the rest of the window, then the joints are released without new input.
    REQUIRE(ft->set_target(goal));
    loop.step();
    CHECK(jt->claiming());
    loop.step(15);
    CHECK_FALSE(jt->claiming());
    CHECK(ft->running());
    CHECK(ft->target().isApprox(goal, 1e-9));
    loop.step(800);
    CHECK((pose_at(m, loop.q, "tool").translation() - goal.translation()).norm() < 1e-4);

    // Past the window a request takes the joints at once.
    REQUIRE(jt->set_target(target));
    loop.step(60);
    CHECK(jt->claiming());
    REQUIRE(ft->set_target(goal));
    loop.step();
    CHECK_FALSE(jt->claiming());
    CHECK(ft->running());
}

TEST_CASE("joint_target module: hold_idle false leaves idle joints to other tasks") {
    ModelPtr m = load("arm.urdf");
    Eigen::VectorXd q0 = m->neutral();
    q0[m->joints()[m->joint_id("j2").value()].idx_q] = 0.3;
    Eigen::VectorXd q_goal = q0;
    for (const char* j : {"j1", "j2", "j3", "j4"}) {
        const JointId id = m->joint_id(j).value();
        REQUIRE(m->set_joint_position(q_goal, id, m->joint_position(q_goal, id).value() + 0.1));
    }
    const Eigen::Isometry3d goal = pose_at(m, q_goal, "tool");

    auto run = [&](bool hold_idle) {
        ModuleSet set = make_set(m);
        FrameTaskModuleConfig fc;
        fc.frame = "tool";
        fc.trajectory = pose_limits();
        auto ft = std::make_shared<FrameTaskModule>("tool", fc);
        JointTargetModuleConfig jc;
        jc.joints = {"j1", "j2", "j3"};
        jc.hold_idle = hold_idle;
        auto jt = std::make_shared<JointTargetModule>("joints", jc);
        REQUIRE(set.add(ft));
        REQUIRE(set.add(jt));
        REQUIRE(set.configure());
        Loop loop{m, set, q0};
        loop.step(5);
        REQUIRE(ft->set_target(goal));
        loop.step(600);
        const double error = (pose_at(m, loop.q, "tool").translation() - goal.translation()).norm();

        Eigen::VectorXd target(3);
        target << 0.2, 0.1, -0.2;
        REQUIRE(jt->set_target(target));
        int steps = 0;
        while (!jt->settled() && steps < 2000) {
            loop.step();
            ++steps;
        }
        CHECK(jt->settled());
        CHECK(joint_q(m, loop.q, "j1") == doctest::Approx(0.2).epsilon(1e-3));
        return error;
    };
    // Holding fights the frame task; released, the frame task reaches its goal.
    CHECK(run(true) > 1e-3);
    CHECK(run(false) < 1e-4);
}

TEST_CASE("joint_target module: claim_idle holds the joints, tunes off and on, and syncs on enable") {
    ModelPtr m = load("arm.urdf");
    Eigen::VectorXd q0 = m->neutral();
    q0[m->joints()[m->joint_id("j2").value()].idx_q] = 0.3;
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    JointTargetModuleConfig jc;
    jc.joints = {"j1", "j2", "j3"};
    jc.hold_idle = false;
    jc.claim_idle = true;
    auto jt = std::make_shared<JointTargetModule>("joints", jc);
    REQUIRE(set.add(ft));
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());
    Loop loop{m, set, q0};
    loop.step(5);
    CHECK(ft->held_by() != nullptr);
    Eigen::VectorXd target(3);
    target << 0.2, 0.1, -0.2;
    REQUIRE(jt->set_target(target));
    loop.step(1000);
    CHECK(jt->settled());
    CHECK(ft->held_by() != nullptr);
    CHECK(joint_q(m, loop.q, "j1") == doctest::Approx(0.2).epsilon(1e-3));

    // Disabled: the frame task moves the joints; re-enabled: the target restarts from them.
    jt->set_enabled(false);
    Eigen::Isometry3d goal = pose_at(m, loop.q, "tool");
    goal.translation().z() -= 0.05;
    loop.step(2);
    CHECK(ft->held_by() == nullptr);
    REQUIRE(ft->set_target(goal));
    loop.step(600);
    const double moved = joint_q(m, loop.q, "j2");
    jt->set_enabled(true);
    loop.step(5);
    CHECK(ft->held_by() != nullptr);
    CHECK(jt->target()[1] == doctest::Approx(moved).epsilon(1e-3));

    // Tuned off: released at once; tuned on: holds where the joints are.
    ConfigNode off = ConfigNode::map();
    off.set("claim_idle", false);
    REQUIRE(jt->tune(off));
    loop.step(2);
    CHECK(ft->held_by() == nullptr);
    ConfigNode on = ConfigNode::map();
    on.set("claim_idle", true);
    REQUIRE(jt->tune(on));
    loop.step(2);
    CHECK(ft->held_by() != nullptr);
    CHECK(jt->target()[1] == doctest::Approx(joint_q(m, loop.q, "j2")).epsilon(1e-3));

    // A pose target takes the idle claim over; the next joint target claims again.
    {
        Eigen::VectorXd q_goal = loop.q;
        q_goal[m->joints()[m->joint_id("j2").value()].idx_q] += 0.1;
        const Eigen::Isometry3d reach = pose_at(m, q_goal, "tool");
        REQUIRE(ft->set_target(reach));
        loop.step();
        CHECK(ft->running());
        loop.step(600);
        CHECK((pose_at(m, loop.q, "tool").translation() - reach.translation()).norm() < 1e-4);
        CHECK(ft->running());
        REQUIRE(jt->set_target(target));
        loop.step();
        CHECK(ft->held_by() != nullptr);
        loop.step(1000);
        CHECK(joint_q(m, loop.q, "j1") == doctest::Approx(0.2).epsilon(1e-3));
    }
}

namespace {
Eigen::VectorXd arm_pose(const ModelPtr& m, double d);
}  // namespace

TEST_CASE("on_held: frame_task and joint_target resume the target with wait and drop it with abort") {
    for (const OnHeld policy : {OnHeld::wait, OnHeld::abort}) {
        CAPTURE(to_string(policy));
        ModelPtr m = load("arm.urdf");
        ModuleSet set = make_set(m);
        FrameTaskModuleConfig fc;
        fc.frame = "tool";
        fc.trajectory = pose_limits();
        fc.on_held = policy;
        auto ft = std::make_shared<FrameTaskModule>("tool", fc);
        JointTargetModuleConfig lc;
        lc.joints = {"j2", "j3"};
        lc.on_held = policy;
        auto low = std::make_shared<JointTargetModule>("low", lc);
        JointTargetModuleConfig hc;
        hc.joints = {"j1", "j2"};
        hc.priority = 5;
        auto high = std::make_shared<JointTargetModule>("high", hc);
        REQUIRE(set.add(ft));
        REQUIRE(set.add(low));
        REQUIRE(set.add(high));
        REQUIRE(set.configure());

        // frame_task: preempted half-way to a target by a joint claim. Idle joint modules would hold
        // their joints against it, so only the active ones stay enabled.
        low->set_enabled(false);
        high->set_enabled(false);
        Loop loop{m, set, m->neutral()};
        loop.step(5);
        const Eigen::Isometry3d goal = pose_at(m, arm_pose(m, 0.2), "tool");
        REQUIRE(ft->set_target(goal));
        loop.step(20);
        REQUIRE(ft->running());
        high->set_enabled(true);
        REQUIRE(high->set_target(Eigen::Vector2d(0.05, 0.05)));
        loop.step();
        REQUIRE(ft->held_by());
        CHECK(*ft->held_by() == "high");
        CHECK(ft->preempted() == 1);
        while (!high->settled()) loop.step();
        high->set_enabled(false);
        loop.step();
        REQUIRE(ft->running());
        if (policy == OnHeld::wait) {
            CHECK(ft->has_target());
            CHECK(ft->target().isApprox(goal, 1e-9));
            loop.step(800);
            CHECK((pose_at(m, loop.q, "tool").translation() - goal.translation()).norm() < 1e-4);
        } else {
            CHECK_FALSE(ft->has_target());
            const Eigen::Isometry3d hold = pose_at(m, loop.q, "tool");
            CHECK(ft->target().isApprox(hold, 1e-3));
            loop.step(100);
            CHECK((pose_at(m, loop.q, "tool").translation() - hold.translation()).norm() < 1e-3);
        }

        // joint_target: a lower-priority claim preempted by a higher one on a shared joint.
        ft->set_enabled(false);
        low->set_enabled(true);
        loop.step();
        const double j3_start = joint_q(m, loop.q, "j3");
        REQUIRE(low->set_target(Eigen::Vector2d(joint_q(m, loop.q, "j2"), j3_start + 1.0)));
        loop.step(10);
        REQUIRE(low->running());
        REQUIRE(low->claiming());
        high->set_enabled(true);
        REQUIRE(high->set_target(Eigen::Vector2d(0.0, 0.0)));
        loop.step();
        REQUIRE(low->held_by());
        CHECK(*low->held_by() == "high");
        CHECK(low->claiming() == (policy == OnHeld::wait));
        while (!high->settled()) loop.step();
        high->set_enabled(false);
        loop.step();
        if (policy == OnHeld::wait) {
            int steps = 0;
            while (!low->settled() && steps++ < 3000) loop.step();
            CHECK(low->settled());
            CHECK(joint_q(m, loop.q, "j3") == doctest::Approx(j3_start + 1.0).epsilon(1e-3));
        } else {
            CHECK(low->running());
            CHECK_FALSE(low->claiming());
            const double j3_hold = joint_q(m, loop.q, "j3");
            CHECK(std::abs(j3_hold - (j3_start + 1.0)) > 0.05);  // did not continue to the dropped target
            loop.step(100);
            CHECK(joint_q(m, loop.q, "j3") == doctest::Approx(j3_hold).epsilon(1e-2));
        }
    }
}

TEST_CASE("module ports: frame_task pose and joint_target vector round-trip through set_target") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    JointTargetModuleConfig jc;
    jc.joints = {"j1", "j3"};
    auto jt = std::make_shared<JointTargetModule>("joints", jc);
    REQUIRE(set.add(ft));
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());

    REQUIRE(ft->ports().size() == 1);
    const PortInfo& fp = ft->ports()[0];
    CHECK(fp.name == "target");
    CHECK(fp.type == PortType::pose);
    CHECK(fp.frame == "tool");
    CHECK(fp.size() == 1);
    CHECK(ft->port_index("target") == 0);
    CHECK(ft->port_index("nope") == -1);

    REQUIRE(jt->ports().size() == 1);
    const PortInfo& jp = jt->ports()[0];
    CHECK(jp.type == PortType::vector);
    CHECK(jp.labels == std::vector<std::string>{"j1", "j3"});
    REQUIRE(jp.size() == 2);
    REQUIRE(jp.lower.size() == 2);
    const JointInfo& j1 = m->joints()[m->joint_id("j1").value()];
    CHECK(jp.lower[0] == m->lower_position_limit()[j1.idx_q]);
    CHECK(jp.upper[0] == m->upper_position_limit()[j1.idx_q]);
    CHECK(std::isinf(jp.lower[1]));  // j3 is continuous

    PortValue v;
    REQUIRE(ft->read_port(0, v));
    CHECK(v.pose.isApprox(pose_at(m, m->neutral(), "tool"), 1e-9));
    REQUIRE(jt->read_port(0, v));
    CHECK(v.vector.isZero());

    Loop loop{m, set, m->neutral()};
    loop.step();
    REQUIRE(ft->read_port(0, v));
    CHECK(v.pose.isApprox(pose_at(m, loop.q, "tool"), 1e-9));
    v.pose.translation().z() -= 0.01;
    REQUIRE(ft->write_port(0, v));
    CHECK(ft->target().isApprox(v.pose));
    v.pose.linear() *= 2.0;
    CHECK_FALSE(ft->write_port(0, v));
    CHECK(ft->read_port(1, v).error().code == ErrorCode::out_of_range);

    v.vector = Eigen::Vector2d(0.2, 0.1);
    REQUIRE(jt->write_port(0, v));
    CHECK(jt->claiming());
    PortValue r;
    REQUIRE(jt->read_port(0, r));
    CHECK(r.vector.isApprox(v.vector));
    v.vector = Eigen::Vector2d(1e308, -1e308);
    REQUIRE(jt->write_port(0, v));
    REQUIRE(jt->read_port(0, r));
    CHECK(r.vector[0] == jp.upper[0]);
    CHECK(r.vector[1] == -1e308);  // continuous joint: no limit to clamp to
    v.vector = Eigen::VectorXd::Zero(3);
    CHECK_FALSE(jt->write_port(0, v));
    CHECK(std::string(to_string(PortType::vector)) == "vector");
}

TEST_CASE("joint_target module: rejects non-scalar joints and missing limits") {
    ModelOptions o;
    o.root = RootJoint::free_flyer;
    ModelPtr m = load("diff_chassis.urdf", o);
    {
        ModuleSet set = make_set(m);
        JointTargetModuleConfig jc;
        jc.joints = {"root_joint"};
        REQUIRE(set.add(std::make_shared<JointTargetModule>("j", jc)));
        CHECK_FALSE(set.configure());
    }
    {
        ModuleSet set = make_set(m);
        JointTargetModuleConfig jc;
        jc.joints = {"lift_joint"};
        jc.velocity_limit = Eigen::VectorXd::Constant(1, -1.0);
        REQUIRE(set.add(std::make_shared<JointTargetModule>("j", jc)));
        auto s = set.configure();
        REQUIRE_FALSE(s);
        CHECK(s.error().code == ErrorCode::invalid_argument);
    }
}

WorldObstacle ball_at(double x) {
    WorldObstacle o;
    o.name = "ball";
    o.shape = Shape::sphere(0.1);
    o.pose.translation().x() = x;
    return o;
}

TEST_CASE("world_collision module: stops a joint target at d_min until disabled") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelOptions co;
    co.self_collision = false;
    auto geometry = CollisionModel::from_urdf(
        m, ModelSource::file((kFixtures / "slider_collision.urdf").string()), co, Context::defaults());
    REQUIRE_MESSAGE(geometry, (geometry ? "" : geometry.error().message));
    const WorldObstacle ball = ball_at(0.5);

    ModuleSet set = make_set(m);
    WorldCollisionModuleConfig sc;
    sc.model = *geometry;
    sc.obstacles = {ball};
    auto col = std::make_shared<WorldCollisionModule>("collision", sc);
    JointTargetModuleConfig jc;
    jc.joints = {"slide"};
    auto jt = std::make_shared<JointTargetModule>("slide", jc);
    REQUIRE(set.add(col));
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());
    CHECK(col->constraint() != nullptr);

    Loop loop{m, set, m->neutral()};
    REQUIRE(jt->set_target(Eigen::VectorXd::Constant(1, 0.55)));
    loop.step(500);
    // Gap = 0.5 - x - 0.2 >= d_min (0.03).
    const double x = loop.q[0];
    CHECK(x <= 0.27 + 1e-4);
    CHECK(x > 0.25);
    CHECK(jt->claiming());

    col->set_enabled(false);
    loop.step(500);
    CHECK(loop.q[0] == doctest::Approx(0.55).epsilon(1e-3));
    CHECK(jt->settled());
}

TEST_CASE("world_collision module: closed_loop evaluates the distance at the measured configuration") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelOptions co;
    co.self_collision = false;
    auto geometry = CollisionModel::from_urdf(
        m, ModelSource::file((kFixtures / "slider_collision.urdf").string()), co, Context::defaults());
    REQUIRE(geometry);
    const WorldObstacle ball = ball_at(0.5);

    for (const bool closed : {false, true}) {
        ModuleSet set = make_set(m);
        WorldCollisionModuleConfig sc;
        sc.model = *geometry;
        sc.obstacles = {ball};
        sc.closed_loop = closed;
        JointTargetModuleConfig jc;
        jc.joints = {"slide"};
        auto jt = std::make_shared<JointTargetModule>("slide", jc);
        REQUIRE(set.add(std::make_shared<WorldCollisionModule>("collision", sc)));
        REQUIRE(set.add(jt));
        REQUIRE(set.configure());
        REQUIRE(jt->set_target(Eigen::VectorXd::Constant(1, 0.55)));
        Eigen::VectorXd q = m->neutral(), v = Eigen::VectorXd::Zero(m->nv());
        SolveResult out;
        // The robot is measured 0.1 ahead of the ideal state.
        for (int k = 1; k <= 500; ++k) {
            Eigen::VectorXd measured = q;
            measured[0] += 0.1;
            REQUIRE(set.step(q, v, &measured, 0.01 * k, 0.01, out));
            q = out.q_next;
            v = out.v;
        }
        INFO("closed_loop " << closed);
        CHECK(q[0] == doctest::Approx(closed ? 0.17 : 0.27).epsilon(1e-3));
    }

    // A robot lagging 0.1 behind: closed loop still bounds the command itself.
    ModuleSet set = make_set(m);
    WorldCollisionModuleConfig sc;
    sc.model = *geometry;
    sc.obstacles = {ball};
    sc.closed_loop = true;
    JointTargetModuleConfig jc;
    jc.joints = {"slide"};
    auto jt = std::make_shared<JointTargetModule>("slide", jc);
    REQUIRE(set.add(std::make_shared<WorldCollisionModule>("collision", sc)));
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());
    REQUIRE(jt->set_target(Eigen::VectorXd::Constant(1, 0.55)));
    Eigen::VectorXd q = m->neutral(), v = Eigen::VectorXd::Zero(m->nv());
    SolveResult out;
    for (int k = 1; k <= 500; ++k) {
        Eigen::VectorXd measured = q;
        measured[0] -= 0.1;
        REQUIRE(set.step(q, v, &measured, 0.01 * k, 0.01, out));
        q = out.q_next;
        v = out.v;
    }
    CHECK(q[0] == doctest::Approx(0.27).epsilon(1e-3));
}

TEST_CASE("world_collision module: visuals and display outputs") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelOptions co;
    co.self_collision = false;
    auto geometry = CollisionModel::from_urdf(
        m, ModelSource::file((kFixtures / "slider_collision.urdf").string()), co, Context::defaults());
    REQUIRE(geometry);
    const WorldObstacle ball = ball_at(0.5);
    ModuleSet set = make_set(m);
    WorldCollisionModuleConfig sc;
    sc.model = *geometry;
    sc.obstacles = {ball};
    auto col = std::make_shared<WorldCollisionModule>("collision", sc);
    REQUIRE(set.add(col));
    REQUIRE(set.configure());

    // Robot geometries are self_collision's visuals; only the obstacle is drawn here.
    REQUIRE(col->visuals().size() == 1);
    CHECK(col->visuals().back().name == "ball");
    CHECK(col->visuals().back().frame.empty());
    CHECK(col->visuals().back().placement.translation().x() == doctest::Approx(0.5));
    REQUIRE(col->outputs().size() == 7);
    CHECK(col->outputs()[3].size == 1);
    CHECK(col->outputs()[4].type == OutputType::text);

    Workspace ws(m);
    OutputValue v;
    REQUIRE(ws.update(m->neutral()));
    REQUIRE(col->update_outputs(ws));
    REQUIRE(col->read_output(0, v));
    CHECK(v.number == doctest::Approx(0.3));
    REQUIRE(col->read_output(1, v));
    REQUIRE(v.points.cols() == 2);
    CHECK(v.points(0, 0) == doctest::Approx(0.1));
    CHECK(v.points(0, 1) == doctest::Approx(0.4));
    REQUIRE(col->read_output(2, v));
    CHECK(v.visuals.empty());
    REQUIRE(col->read_output(3, v));
    CHECK(v.visuals.empty());

    // Gap 0.035 is inside d_min + activation_margin = 0.04.
    Eigen::VectorXd q = m->neutral();
    q[0] = 0.265;
    REQUIRE(ws.update(q));
    REQUIRE(col->update_outputs(ws));
    REQUIRE(col->read_output(2, v));
    CHECK(v.visuals == std::vector<int>{0});
    REQUIRE(col->read_output(3, v));
    CHECK(v.visuals == std::vector<int>{0});
    REQUIRE(col->read_output(4, v));
    CHECK(v.text.size() > 7);
    CHECK(v.text.compare(v.text.size() - 7, 7, " / ball") == 0);
    REQUIRE(col->read_output(5, v));
    CHECK(v.number == 1.0);
    REQUIRE(col->read_output(6, v));
    CHECK(v.number == 0.0);
    CHECK_FALSE(col->read_output(7, v));
}

TEST_CASE("world_collision module: obstacle ports move and disable the obstacle") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelOptions co;
    co.self_collision = false;
    auto geometry = CollisionModel::from_urdf(
        m, ModelSource::file((kFixtures / "slider_collision.urdf").string()), co, Context::defaults());
    REQUIRE(geometry);
    ModuleSet set = make_set(m);
    WorldCollisionModuleConfig sc;
    sc.model = *geometry;
    sc.obstacles = {ball_at(0.5)};
    auto col = std::make_shared<WorldCollisionModule>("collision", sc);
    JointTargetModuleConfig jc;
    jc.joints = {"slide"};
    auto jt = std::make_shared<JointTargetModule>("slide", jc);
    REQUIRE(set.add(col));
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());

    REQUIRE(col->ports().size() == 2);
    CHECK(col->ports()[0].name == "ball");
    CHECK(col->ports()[0].type == PortType::pose);
    CHECK(col->ports()[0].reference.empty());
    CHECK(col->ports()[1].name == "ball/enabled");
    CHECK(col->ports()[1].type == PortType::flag);
    PortValue pv;
    REQUIRE(col->read_port(0, pv));
    CHECK(pv.pose.translation().x() == doctest::Approx(0.5));
    REQUIRE(col->read_port(1, pv));
    CHECK(pv.flag);
    CHECK_FALSE(col->read_port(2, pv));
    CHECK_FALSE(col->write_port(2, pv));

    Eigen::Isometry3d placement;
    REQUIRE(col->visual_placement(0, placement));
    CHECK(placement.translation().x() == doctest::Approx(0.5));

    // Moving the ball to 0.4 tightens the bound to 0.4 - 0.2 - 0.03.
    pv.pose = Eigen::Isometry3d::Identity();
    pv.pose.translation().x() = 0.4;
    REQUIRE(col->write_port(0, pv));
    REQUIRE(col->visual_placement(0, placement));
    CHECK(placement.translation().x() == doctest::Approx(0.4));
    Loop loop{m, set, m->neutral()};
    REQUIRE(jt->set_target(Eigen::VectorXd::Constant(1, 0.55)));
    loop.step(500);
    CHECK(loop.q[0] <= 0.17 + 1e-4);
    CHECK(loop.q[0] > 0.15);

    pv.flag = false;
    REQUIRE(col->write_port(1, pv));
    REQUIRE(col->read_port(1, pv));
    CHECK_FALSE(pv.flag);
    CHECK_FALSE(col->visual_placement(0, placement));
    CHECK_FALSE(col->visual_placement(1, placement));
    loop.step(500);
    CHECK(loop.q[0] == doctest::Approx(0.55).epsilon(1e-3));

    Workspace ws(m);
    OutputValue v;
    REQUIRE(ws.update(loop.q));
    REQUIRE(col->update_outputs(ws));
    REQUIRE(col->read_output(3, v));
    CHECK(v.visuals.empty());
}

TEST_CASE("world_collision module: estimates obstacle velocity from pose writes") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelOptions co;
    co.self_collision = false;
    auto geometry = CollisionModel::from_urdf(
        m, ModelSource::file((kFixtures / "slider_collision.urdf").string()), co, Context::defaults());
    REQUIRE(geometry);
    ModuleSet set = make_set(m);
    WorldCollisionModuleConfig sc;
    sc.model = *geometry;
    sc.obstacles = {ball_at(0.6)};
    sc.constraint.gain = 20.0;
    auto col = std::make_shared<WorldCollisionModule>("collision", sc);
    JointTargetModuleConfig jc;
    jc.joints = {"slide"};
    auto jt = std::make_shared<JointTargetModule>("slide", jc);
    REQUIRE(set.add(col));
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());
    Loop loop{m, set, m->neutral()};
    REQUIRE(jt->set_target(Eigen::VectorXd::Zero(1)));

    // The ball closes at 1 m/s, written once per step; the slider is pushed back without sinking in.
    PortValue pv;
    pv.pose = Eigen::Isometry3d::Identity();
    double x = 0.6, min_margin = 1.0;
    for (int i = 0; i < 40; ++i) {
        x -= 0.01;
        pv.pose.translation().x() = x;
        REQUIRE(col->write_port(0, pv));
        loop.step();
        min_margin = std::min(min_margin, x - loop.q[0] - 0.2 - sc.constraint.d_min);
    }
    CHECK(col->world()->linear_velocity(0).isApprox(Eigen::Vector3d(-1.0, 0, 0), 1e-9));
    CHECK(loop.q[0] < -0.03);  // pushed back past 0
    CHECK(min_margin > -1e-3);

    // No writes for longer than velocity_timeout: static again.
    loop.step(19);
    CHECK_FALSE(col->world()->linear_velocity(0).isZero());
    loop.step(2);
    CHECK(col->world()->linear_velocity(0).isZero());

    // A write after the gap is a jump, not motion; 0 disables estimation.
    pv.pose.translation().x() = 0.5;
    REQUIRE(col->write_port(0, pv));
    loop.step();
    CHECK(col->world()->linear_velocity(0).isZero());
    REQUIRE(col->tune(ConfigNode::map({{"velocity_timeout", 0.0}})));
    pv.pose.translation().x() = 0.45;
    REQUIRE(col->write_port(0, pv));
    loop.step();
    CHECK(col->world()->linear_velocity(0).isZero());
    CHECK_FALSE(col->tune(ConfigNode::map({{"velocity_timeout", -1.0}})));
}

TEST_CASE("world_collision module: rejects empty, duplicate and slashed obstacles") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelOptions co;
    co.self_collision = false;
    auto geometry = CollisionModel::from_urdf(
        m, ModelSource::file((kFixtures / "slider_collision.urdf").string()), co, Context::defaults());
    REQUIRE(geometry);
    for (const std::size_t count : {std::size_t{0}, std::size_t{2}, std::size_t{1}}) {
        ModuleSet set = make_set(m);
        WorldCollisionModuleConfig sc;
        sc.model = *geometry;
        sc.obstacles.assign(count, ball_at(0.5));
        if (count == 1) sc.obstacles[0].name = "a/b";
        REQUIRE(set.add(std::make_shared<WorldCollisionModule>("collision", sc)));
        auto s = set.configure();
        REQUIRE_FALSE(s);
        CHECK(s.error().code == ErrorCode::invalid_argument);
    }
}

TEST_CASE("chassis module: locks the root and writes the controller pose") {
    ModelOptions o;
    o.root = RootJoint::free_flyer;
    ModelPtr m = load("diff_chassis.urdf", o);
    ModuleSet set = make_set(m);
    ChassisModuleConfig cc;
    cc.chassis.kinematics = ChassisKinematics::differential;
    cc.period = 0.02;
    auto ch = std::make_shared<ChassisModule>("chassis", cc);
    CHECK(ch->controller() == nullptr);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.joints = {"lift_joint", "shoulder_yaw_joint", "shoulder_pitch_joint", "elbow_joint"};
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    REQUIRE(set.add(ch));
    REQUIRE(set.add(ft));
    REQUIRE(set.configure());
    REQUIRE(ch->controller());
    for (std::size_t i = 0; i < 6; ++i) CHECK(set.solver().options().locked.test(i));

    Loop loop{m, set, m->neutral()};
    loop.dt = 0.02;
    const Pose2D goal{0.8, 0.3, 0.4};
    for (int i = 0; i < 1500; ++i) {
        REQUIRE(ch->controller()->submit_pose(goal, loop.now));
        loop.step();
        REQUIRE(loop.out.v.head<6>().norm() < 1e-9);
        if (i == 0) CHECK(ch->controller()->state() == ChassisState::active);
    }
    const Pose2D p = ch->controller()->pose();
    auto root = m->root_pose(loop.q);
    REQUIRE(root);
    CHECK(root->isApprox(chassis_root_pose(p), 1e-9));
    CHECK(std::hypot(p.x - goal.x, p.y - goal.y) < 1.5 * ChassisConfig{}.arrival_position_tolerance);
    CHECK(ch->output().state == ChassisState::active);

    // Skipped cycles pause and resynchronise instead of integrating the gap.
    ch->set_enabled(false);
    loop.step(10);
    ch->set_enabled(true);
    REQUIRE(ch->controller()->submit_pose(goal, loop.now));
    loop.step();
    CHECK(ch->controller()->state() == ChassisState::active);

    ModelPtr fixed = load("arm.urdf");
    ModuleSet bad = make_set(fixed);
    REQUIRE(bad.add(std::make_shared<ChassisModule>("chassis", cc)));
    CHECK_FALSE(bad.configure());
}

TEST_CASE("chassis module: follows the engine dt and recovers a fault on resynchronisation") {
    ModelOptions o;
    o.root = RootJoint::free_flyer;
    ModelPtr m = load("diff_chassis.urdf", o);
    ModuleSet set = make_set(m);
    ChassisModuleConfig cc;
    cc.chassis.kinematics = ChassisKinematics::differential;
    cc.period = 0.02;
    auto ch = std::make_shared<ChassisModule>("chassis", cc);
    REQUIRE(set.add(ch));
    REQUIRE(set.configure());

    // 0.15 s exceeds max(max_dt_s, 2 * 0.02) for the configured period but not for the engine dt.
    Loop loop{m, set, m->neutral()};
    loop.dt = 0.15;
    for (int i = 0; i < 20; ++i) {
        REQUIRE(ch->controller()->submit_velocity({0.1, 0.0, 0.0}, loop.now));
        loop.step();
        REQUIRE(ch->controller()->state() == ChassisState::active);
    }
    CHECK(ch->controller()->scheduler_period() == doctest::Approx(0.15));
    CHECK(ch->controller()->pose().x > 0.1);

    ch->controller()->step(loop.now + 10.0);
    REQUIRE(ch->controller()->state() == ChassisState::fault);
    ch->set_enabled(false);
    loop.step(3);
    ch->set_enabled(true);
    loop.step();
    CHECK(ch->controller()->state() == ChassisState::active);
}

TEST_CASE("chassis module: velocity port lapses and outputs follow the controller") {
    ModelOptions o;
    o.root = RootJoint::free_flyer;
    ModelPtr m = load("diff_chassis.urdf", o);
    ModuleSet set = make_set(m);
    ChassisModuleConfig cc;
    cc.chassis.kinematics = ChassisKinematics::differential;
    cc.period = 0.02;
    auto ch = std::make_shared<ChassisModule>("chassis", cc);
    REQUIRE(set.add(ch));
    REQUIRE(set.configure());

    REQUIRE(ch->ports().size() == 2);
    CHECK(ch->ports()[1].name == "pose_goal");
    const PortInfo& port = ch->ports()[0];
    CHECK(port.name == "velocity");
    CHECK(port.size() == 3);
    CHECK(port.timeout == ChassisConfig{}.command_timeout_s);
    CHECK(port.lower[1] == 0.0);
    CHECK(port.upper[1] == 0.0);
    CHECK(port.upper[0] == ChassisConfig{}.max_linear);
    REQUIRE(ch->outputs().size() == 21);
    CHECK(ch->outputs()[6].name == "active");
    CHECK(ch->outputs()[13].name == "state");
    CHECK(ch->outputs()[13].type == OutputType::text);

    PortValue bad;
    bad.vector = Eigen::Vector2d(0.1, 0.0);
    CHECK(ch->write_port(0, bad).error().code == ErrorCode::invalid_argument);

    Loop loop{m, set, m->neutral()};
    loop.dt = 0.02;
    PortValue cmd;
    cmd.vector = Eigen::Vector3d(0.2, 0.0, 0.1);
    for (int i = 0; i < 100; ++i) {
        REQUIRE(ch->write_port(0, cmd));
        loop.step();
    }
    PortValue read;
    REQUIRE(ch->read_port(0, read));
    CHECK(read.vector.isApprox(cmd.vector));
    OutputValue v;
    REQUIRE(ch->read_output(0, v));
    CHECK(v.number == doctest::Approx(0.2).epsilon(1e-6));
    REQUIRE(ch->read_output(3, v));
    CHECK(v.number == doctest::Approx(ch->controller()->pose().x));
    CHECK(v.number > 0.1);
    REQUIRE(ch->read_output(6, v));
    CHECK(v.flag);
    REQUIRE(ch->read_output(7, v));
    CHECK_FALSE(v.flag);
    REQUIRE(ch->read_output(8, v));
    CHECK(v.number == doctest::Approx(0.2));
    REQUIRE(ch->read_output(12, v));
    CHECK_FALSE(v.points.allFinite());
    REQUIRE(ch->read_output(13, v));
    CHECK(v.text == "active");
    REQUIRE(ch->read_output(15, v));
    CHECK(v.text == "port");
    REQUIRE(ch->read_output(16, v));
    CHECK(v.text == "velocity");
    REQUIRE(ch->read_output(18, v));
    CHECK(v.flag);

    // pose_limits starts from the chassis max_* fields and tunes at runtime.
    ConfigNode t = ch->tunables();
    CHECK(t.find("pose_limits")->find("linear_velocity")->as_number() == ChassisConfig{}.max_linear);
    CHECK(t.find("pose_limits")->find("planner_mode")->as_string() == "straight");
    ConfigNode lim = ConfigNode::map();
    lim.set("linear_velocity", 0.3);
    lim.set("planner_mode", std::string("axis_time"));
    ConfigNode patch = ConfigNode::map();
    patch.set("pose_limits", lim);
    CHECK_FALSE(ch->tune(patch));  // omni-only mode on a differential base
    lim.set("planner_mode", std::string("straight"));
    patch.set("pose_limits", lim);
    REQUIRE(ch->tune(patch));
    CHECK(ch->controller()->pose_limits().linear_velocity == 0.3);
    CHECK(ch->controller()->pose_limits().planner_mode == ChassisPlannerMode::straight);
    lim.set("linear_velocity", -1.0);
    patch.set("pose_limits", lim);
    CHECK_FALSE(ch->tune(patch));
    CHECK(ch->controller()->pose_limits().linear_velocity == 0.3);
    CHECK(ch->tune_schema().find("properties")->find("pose_limits") != nullptr);

    // Without rewrites the command lapses and the base ramps to rest.
    loop.step(100);
    REQUIRE(ch->read_port(0, read));
    CHECK(read.vector.isZero());
    REQUIRE(ch->read_output(0, v));
    CHECK(std::abs(v.number) < 1e-9);
    REQUIRE(ch->read_output(18, v));
    CHECK_FALSE(v.flag);
}

TEST_CASE("chassis module: pose_goal port drives the base to the goal and lapses") {
    ModelOptions o;
    o.root = RootJoint::free_flyer;
    ModelPtr m = load("diff_chassis.urdf", o);
    ModuleSet set = make_set(m);
    ChassisModuleConfig cc;
    cc.period = 0.02;
    auto ch = std::make_shared<ChassisModule>("chassis", cc);
    REQUIRE(set.add(ch));
    REQUIRE(set.configure());

    PortValue read;
    REQUIRE(ch->read_port(1, read));
    CHECK(read.vector.isZero());
    PortValue bad;
    bad.vector = Eigen::Vector3d(0.1, std::nan(""), 0.0);
    CHECK(ch->write_port(1, bad).error().code == ErrorCode::invalid_argument);

    Loop loop{m, set, m->neutral()};
    loop.dt = 0.02;
    PortValue goal;
    goal.vector = Eigen::Vector3d(0.3, -0.2, 0.5);
    for (int i = 0; i < 400; ++i) {
        REQUIRE(ch->write_port(1, goal));
        loop.step();
    }
    REQUIRE(ch->read_port(1, read));
    CHECK(read.vector.isApprox(goal.vector));
    REQUIRE(ch->read_port(0, read));
    CHECK(read.vector.isZero());
    const Pose2D p = ch->controller()->pose();
    CHECK(p.x == doctest::Approx(0.3).epsilon(1e-3));
    CHECK(p.y == doctest::Approx(-0.2).epsilon(1e-3));
    CHECK(p.yaw == doctest::Approx(0.5).epsilon(1e-3));

    // A velocity write replaces the goal; once it lapses pose_goal reads back the current pose.
    PortValue cmd;
    cmd.vector = Eigen::Vector3d(0.1, 0.0, 0.0);
    REQUIRE(ch->write_port(0, cmd));
    loop.step(100);
    REQUIRE(ch->read_port(1, read));
    const Pose2D now = ch->controller()->pose();
    CHECK(read.vector.isApprox(Eigen::Vector3d(now.x, now.y, now.yaw)));
}

namespace {
bool active(const SolveResult& out, const std::string& task) {
    for (const auto& r : out.residuals)
        if (r.name == task) return true;
    return false;
}

Eigen::VectorXd arm_pose(const ModelPtr& m, double d) {
    Eigen::VectorXd q = m->neutral();
    for (const char* j : {"j1", "j2", "j3"}) {
        const JointId id = m->joint_id(j).value();
        REQUIRE(m->set_joint_position(q, id, d));
    }
    return q;
}

template <class M>
int run_until(Loop& loop, const M& module, HomeStatus status, int limit = 3000) {
    int n = 0;
    while (module.home_status() != status && n < limit) {
        loop.step();
        ++n;
    }
    return n;
}
}  // namespace

TEST_CASE("frame_task module: home preempts a plain claim, succeeds and rejects new targets meanwhile") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.trajectory = pose_limits();
    fc.home_motion.velocity_scale = 0.5;
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    JointTargetModuleConfig jc;
    jc.joints = {"j1"};
    auto jt = std::make_shared<JointTargetModule>("j1", jc);
    REQUIRE(set.add(ft));
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());
    CHECK(ft->home_status() == HomeStatus::idle);

    Loop loop{m, set, m->neutral()};
    loop.step();
    const Eigen::Isometry3d home = pose_at(m, loop.q, "tool");
    REQUIRE(ft->set_target(pose_at(m, arm_pose(m, 0.2), "tool")));
    loop.step(600);
    REQUIRE((pose_at(m, loop.q, "tool").translation() - home.translation()).norm() > 0.05);

    // Rejected while a preemptive home holds its joints; waits while a plain claim does.
    REQUIRE(jt->request_home());
    loop.step();
    REQUIRE(jt->home_status() == HomeStatus::executing);
    CHECK(std::string(jt->home_state()) == "executing");
    CHECK_FALSE(ft->request_home());
    run_until(loop, *jt, HomeStatus::succeeded);
    loop.step(2);
    REQUIRE(jt->claiming());
    REQUIRE(ft->request_home());
    loop.step(2);
    CHECK(ft->home_status() == HomeStatus::executing);
    CHECK(std::string(static_cast<const Module&>(*ft).home_state()) == "executing");
    CHECK(ft->held_by() == nullptr);
    CHECK_FALSE(ft->set_target(pose_at(m, arm_pose(m, -0.1), "tool")));
    const int steps = run_until(loop, *ft, HomeStatus::succeeded);
    INFO("steps " << steps);
    CHECK(ft->home_status() == HomeStatus::succeeded);
    const Eigen::Isometry3d reached = pose_at(m, loop.q, "tool");
    CHECK((reached.translation() - home.translation()).norm() < fc.position_tolerance);
    CHECK(ft->target().isApprox(home, 1e-9));
    loop.step(50);
    CHECK((pose_at(m, loop.q, "tool").translation() - home.translation()).norm() < fc.position_tolerance);

    // Once finished, new targets are accepted again.
    jt->set_enabled(false);
    loop.step(2);
    const Eigen::Isometry3d goal = pose_at(m, arm_pose(m, -0.1), "tool");
    REQUIRE(ft->set_target(goal));
    loop.step(800);
    CHECK((pose_at(m, loop.q, "tool").translation() - goal.translation()).norm() < 1e-4);
}

TEST_CASE("frame_task module: default home without a reference follows a moved root") {
    ModelOptions o;
    o.root = RootJoint::free_flyer;
    ModelPtr m = load("floating_base.urdf", o);
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.joints = {"joint1", "joint2", "joint3", "joint4"};
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    REQUIRE(set.add(ft));
    REQUIRE(set.configure());
    const Eigen::Isometry3d in_root = pose_at(m, m->neutral(), "tool");

    // The base moves (as a chassis would) while the arm bends away from home.
    Eigen::VectorXd q0 = m->neutral();
    Eigen::Isometry3d root = Eigen::Isometry3d::Identity();
    root.translation() << 0.3, -0.2, 0.198;
    root.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    REQUIRE(m->set_root_pose(q0, root));
    for (const char* j : {"joint1", "joint2", "joint3"})
        REQUIRE(m->set_joint_position(q0, m->joint_id(j).value(), 0.2));
    Loop loop{m, set, q0};
    loop.step();
    REQUIRE(ft->request_home());
    const int steps = run_until(loop, *ft, HomeStatus::succeeded);
    INFO("steps " << steps);
    REQUIRE(ft->home_status() == HomeStatus::succeeded);
    const Eigen::Isometry3d expected = root * in_root;
    CHECK((pose_at(m, loop.q, "tool").translation() - expected.translation()).norm() < fc.position_tolerance);
    CHECK(ft->target().isApprox(expected, 1e-9));
}

TEST_CASE(
    "frame_task module: held since start, the target follows the observed pose, not the configure-time one") {
    ModelOptions o;
    o.root = RootJoint::free_flyer;
    ModelPtr m = load("floating_base.urdf", o);
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.joints = {"joint1", "joint2", "joint3", "joint4"};
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    JointTargetModuleConfig jc;
    jc.joints = fc.joints;
    jc.claim_idle = true;
    auto jt = std::make_shared<JointTargetModule>("joints", jc);
    REQUIRE(set.add(ft));
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());

    // The root sits off the configure-time posture, as a chassis center offset puts it.
    Eigen::VectorXd q0 = m->neutral();
    Eigen::Isometry3d root = Eigen::Isometry3d::Identity();
    root.translation() << 0.0, 0.0, 0.198;
    REQUIRE(m->set_root_pose(q0, root));
    Loop loop{m, set, q0};
    loop.step(3);
    CHECK(ft->held_by() != nullptr);
    CHECK(ft->target().isApprox(pose_at(m, loop.q, "tool"), 1e-9));
}

TEST_CASE("frame_task module: running idle, the target holds where it was while the root moves") {
    ModelOptions o;
    o.root = RootJoint::free_flyer;
    ModelPtr m = load("floating_base.urdf", o);
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.joints = {"joint1", "joint2", "joint3", "joint4"};
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    REQUIRE(set.add(ft));
    REQUIRE(set.configure());

    Loop loop{m, set, m->neutral()};
    loop.step(3);
    REQUIRE(ft->running());
    const Eigen::Isometry3d held = ft->target();
    Eigen::Isometry3d root = Eigen::Isometry3d::Identity();
    root.translation() << 0.0, 0.0, 0.1;
    REQUIRE(m->set_root_pose(loop.q, root));
    loop.step(3);
    CHECK(ft->target().isApprox(held, 1e-12));
}

TEST_CASE("frame_task / joint_target: held, outputs and ports follow the robot, not a frozen ramp") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.joints = {"j1", "j2", "j3", "j4"};
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    JointTargetModuleConfig idle;
    idle.joints = {"j2"};
    idle.claim_idle = true;
    auto low = std::make_shared<JointTargetModule>("low", idle);
    JointTargetModuleConfig jc;
    jc.joints = {"j2"};
    jc.priority = 50;
    auto high = std::make_shared<JointTargetModule>("high", jc);
    REQUIRE(set.add(ft));
    REQUIRE(set.add(low));
    REQUIRE(set.add(high));
    REQUIRE(set.configure());

    Loop loop{m, set, m->neutral()};
    loop.step();
    Eigen::Isometry3d goal = pose_at(m, loop.q, "tool");
    goal.translation().x() += 0.05;
    REQUIRE(ft->set_target(goal));
    loop.step(5);
    REQUIRE(ft->running());

    REQUIRE(high->set_target(Eigen::VectorXd::Constant(1, 0.4)));
    loop.step(59);
    const Eigen::VectorXd measured = loop.q;  // ports are refreshed in prepare, from the cycle's input
    loop.step();
    REQUIRE(ft->held_by());
    REQUIRE(low->held_by());
    Workspace ws(m);
    REQUIRE(ws.update(loop.q));
    REQUIRE(ft->update_outputs(ws));
    OutputValue planned, speed;
    REQUIRE(ft->read_output(0, planned));
    REQUIRE(ft->read_output(2, speed));
    CHECK(planned.points.col(0).isApprox(pose_at(m, loop.q, "tool").translation(), 1e-9));
    CHECK(speed.number == 0.0);
    CHECK(low->target()[0] == doctest::Approx(joint_q(m, measured, "j2")));
    CHECK(std::abs(joint_q(m, loop.q, "j2") - joint_q(m, measured, "j2")) > 1e-4);  // still moving
}

TEST_CASE("frame_task module: home starts its ramp from the observed pose, not a stale one") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    REQUIRE(set.add(ft));
    REQUIRE(set.configure());

    Eigen::VectorXd q0 = m->neutral();
    q0[m->joints()[m->joint_id("j2").value()].idx_q] = 0.3;
    Loop loop{m, set, q0};
    loop.step();
    Eigen::Isometry3d far = pose_at(m, loop.q, "tool");
    far.translation() += Eigen::Vector3d(5.0, 0.0, 0.0);
    REQUIRE(ft->set_target(far));
    loop.step(400);
    const Eigen::Vector3d observed = pose_at(m, loop.q, "tool").translation();
    REQUIRE((ft->task()->target().translation() - observed).norm() >
            0.02);  // the ramp leads the unreachable target

    REQUIRE(ft->request_home());
    loop.step();
    REQUIRE(ft->home_status() == HomeStatus::executing);
    CHECK((ft->task()->target().translation() - observed).norm() < 0.005);
}

TEST_CASE("frame_task module: preemptive home holds others, rejects targets and times out") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.trajectory = pose_limits();
    fc.home_motion.timeout_s = 5.0;
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    JointTargetModuleConfig jc;
    jc.joints = {"j1"};
    jc.priority = 10;
    auto jt = std::make_shared<JointTargetModule>("j1", jc);
    REQUIRE(set.add(ft));
    REQUIRE(set.add(jt));
    REQUIRE(set.set_initial(arm_pose(m, 0.2)));
    REQUIRE(set.configure());
    CHECK_FALSE(set.set_initial(arm_pose(m, 0.2)));
    CHECK_FALSE(FrameTaskModule("x", fc).request_home());

    Loop loop{m, set, arm_pose(m, 0.3)};
    loop.step();
    const Eigen::Isometry3d home = pose_at(m, arm_pose(m, 0.2), "tool");
    REQUIRE(jt->set_target(Eigen::VectorXd::Constant(1, -0.3)));
    loop.step(20);
    REQUIRE(jt->claiming());
    REQUIRE(ft->request_home());
    loop.step();
    CHECK(ft->home_status() == HomeStatus::executing);
    REQUIRE(jt->held_by());
    CHECK(*jt->held_by() == "tool");
    CHECK_FALSE(ft->set_target(home));
    // Joint input and a joint home on the homing joints are rejected, not queued.
    REQUIRE(jt->locked_by());
    CHECK(*jt->locked_by() == "tool");
    CHECK_FALSE(jt->set_target(Eigen::VectorXd::Constant(1, 0.1)));
    CHECK_FALSE(jt->request_home());
    CHECK(ft->locked_by() == nullptr);
    const int steps = run_until(loop, *ft, HomeStatus::succeeded);
    INFO("steps " << steps);
    CHECK(ft->home_status() == HomeStatus::succeeded);
    CHECK((pose_at(m, loop.q, "tool").translation() - home.translation()).norm() < fc.position_tolerance);

    // The home took the joint claim over; the frame keeps control at home until joint input takes it back.
    loop.step(200);
    CHECK(jt->locked_by() == nullptr);
    CHECK(ft->running());
    CHECK_FALSE(jt->claiming());
    CHECK((pose_at(m, loop.q, "tool").translation() - home.translation()).norm() < 1e-3);
    REQUIRE(jt->set_target(Eigen::VectorXd::Constant(1, -0.3)));
    loop.step();
    REQUIRE(ft->held_by());
    CHECK(*ft->held_by() == "j1");
    while (!jt->settled()) loop.step();
    CHECK(joint_q(m, loop.q, "j1") == doctest::Approx(-0.3).epsilon(1e-3));

    // Timeout: blocked, so the observed pose makes no progress.
    REQUIRE(ft->home_status() == HomeStatus::succeeded);
    fc.home_motion.timeout_s = 0.1;
    ModuleSet slow_set = make_set(m);
    auto slow = std::make_shared<FrameTaskModule>("tool", fc);
    REQUIRE(slow_set.add(slow));
    REQUIRE(slow_set.configure());
    Loop slow_loop{m, slow_set, m->neutral()};
    slow_loop.step();
    REQUIRE(slow->set_target(pose_at(m, arm_pose(m, 0.3), "tool")));
    slow_loop.step(600);
    REQUIRE(slow->request_home());
    const Eigen::VectorXd blocked = slow_loop.q;
    for (int i = 0; i < 5; ++i) {
        slow_loop.step();
        slow_loop.q = blocked;
    }
    CHECK(slow->home_status() == HomeStatus::executing);
    for (int i = 0; i < 10; ++i) {
        slow_loop.step();
        slow_loop.q = blocked;
    }
    CHECK(slow->home_status() == HomeStatus::timed_out);
    CHECK(slow->claim().joints == nullptr);
    // Stops where the ramp was, and accepts targets again.
    const Eigen::Isometry3d stop = slow->target();
    slow_loop.step(100);
    CHECK((pose_at(m, slow_loop.q, "tool").translation() - stop.translation()).norm() < 1e-3);
    CHECK(slow->set_target(stop));

    // Disabling aborts.
    REQUIRE(ft->request_home());
    loop.step();
    REQUIRE(ft->home_status() == HomeStatus::executing);
    ft->set_enabled(false);
    loop.step();
    ft->set_enabled(true);
    loop.step();
    CHECK(ft->home_status() == HomeStatus::aborted);
}

TEST_CASE("joint_target module: home returns to the initial posture") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    JointTargetModuleConfig jc;
    jc.joints = {"j1", "j2", "j3"};
    jc.home_motion.velocity_scale = 0.5;
    jc.release_after_s = 0.0;  // immediate handover; the watchdog has its own test
    auto jt = std::make_shared<JointTargetModule>("joints", jc);
    REQUIRE(set.add(ft));
    REQUIRE(set.add(jt));
    CHECK_FALSE(set.set_initial(Eigen::VectorXd::Zero(m->nq() + 1)));
    Eigen::VectorXd nan = m->neutral();
    nan[0] = std::nan("");
    CHECK_FALSE(set.set_initial(nan));
    REQUIRE(set.set_initial(arm_pose(m, 0.1)));
    REQUIRE(set.configure());

    Loop loop{m, set, arm_pose(m, -0.2)};
    loop.step();
    Eigen::VectorXd target(3);
    target << 0.6, -0.4, 2.0;
    REQUIRE(jt->set_target(target));
    while (!jt->settled()) loop.step();
    REQUIRE(joint_q(m, loop.q, "j1") == doctest::Approx(0.6).epsilon(1e-3));

    REQUIRE(jt->request_home());
    loop.step();
    CHECK(jt->home_status() == HomeStatus::executing);
    REQUIRE(ft->held_by());
    CHECK(*ft->held_by() == "joints");
    CHECK_FALSE(jt->set_target(target));
    // Pose input and a pose home on the homing joints are rejected.
    REQUIRE(ft->locked_by());
    CHECK(*ft->locked_by() == "joints");
    CHECK_FALSE(ft->set_target(pose_at(m, loop.q, "tool")));
    CHECK_FALSE(ft->request_home());
    loop.step();
    REQUIRE(ft->held_by());
    CHECK(*ft->held_by() == "joints");
    CHECK(ft->home_status() == HomeStatus::idle);
    const int steps = run_until(loop, *jt, HomeStatus::succeeded);
    INFO("steps " << steps);
    CHECK(jt->home_status() == HomeStatus::succeeded);
    for (const char* j : {"j1", "j2", "j3"})
        CHECK(std::abs(std::remainder(joint_q(m, loop.q, j) - 0.1, 2 * M_PI)) < jc.tolerance);
    // The finished home keeps joint control; a pose target takes it over.
    CHECK(jt->claiming());
    loop.step(2);
    REQUIRE(ft->held_by());
    CHECK(*ft->held_by() == "joints");
    REQUIRE(ft->set_target(pose_at(m, loop.q, "tool")));
    loop.step(2);
    CHECK(ft->running());
    CHECK_FALSE(jt->claiming());

    // Disabling aborts a running home at once; a disabled module refuses to home.
    REQUIRE(jt->set_target(target));
    loop.step(5);
    REQUIRE(jt->request_home());
    loop.step();
    REQUIRE(jt->home_status() == HomeStatus::executing);
    jt->set_enabled(false);
    loop.step();
    CHECK(jt->home_status() == HomeStatus::aborted);
    CHECK_FALSE(jt->request_home());
    jt->set_enabled(true);
    loop.step();
    REQUIRE(ft->request_home());
    loop.step();
    REQUIRE(ft->home_status() == HomeStatus::executing);
    ft->set_enabled(false);
    loop.step();
    CHECK(ft->home_status() == HomeStatus::aborted);
    CHECK_FALSE(ft->request_home());
    ft->set_enabled(true);

    // Without set_initial, home is the model neutral.
    ModuleSet plain = make_set(m);
    auto neutral_jt = std::make_shared<JointTargetModule>("joints", jc);
    REQUIRE(plain.add(neutral_jt));
    REQUIRE(plain.configure());
    Loop plain_loop{m, plain, arm_pose(m, 0.3)};
    plain_loop.step();
    REQUIRE(neutral_jt->request_home());
    run_until(plain_loop, *neutral_jt, HomeStatus::succeeded);
    CHECK(neutral_jt->home_status() == HomeStatus::succeeded);
    for (const char* j : {"j1", "j2", "j3"})
        CHECK(std::abs(std::remainder(joint_q(m, plain_loop.q, j) - joint_q(m, m->neutral(), j), 2 * M_PI)) <
              jc.tolerance);

    // Explicit home, bad size rejected.
    jc.home = Eigen::VectorXd::Zero(2);
    ModuleSet bad = make_set(m);
    REQUIRE(bad.add(std::make_shared<JointTargetModule>("joints", jc)));
    CHECK_FALSE(bad.configure());
}

TEST_CASE(
    "joint_target module: a finished home aligns q to measured once, resynchronize aborts a running home") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    JointTargetModuleConfig jc;
    jc.joints = {"j1", "j2", "j3"};
    auto jt = std::make_shared<JointTargetModule>("joints", jc);
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());

    Loop loop{m, set, arm_pose(m, 0.1)};
    loop.step();
    Eigen::VectorXd target(3);
    target << 0.6, -0.4, 1.0;
    REQUIRE(jt->set_target(target));
    while (!jt->settled()) loop.step();

    const Eigen::Index j1 = m->joints()[*m->joint_id("j1")].idx_q;
    const Eigen::Index v1 = m->joints()[*m->joint_id("j1")].idx_v;
    const Eigen::Index v4 = m->joints()[*m->joint_id("j4")].idx_v;
    const auto step = [&] {
        Eigen::VectorXd measured = loop.q;
        measured[j1] += 0.05;
        loop.now += loop.dt;
        const Eigen::VectorXd v = loop.out.v;
        REQUIRE(set.step(loop.q, v, &measured, loop.now, loop.dt, loop.out));
        loop.q = loop.out.q_next;
        return measured;
    };
    REQUIRE(jt->request_home());
    step();
    REQUIRE(jt->home_status() == HomeStatus::executing);
    CHECK_FALSE(set.aligned().any());
    Eigen::VectorXd measured;
    for (int k = 0; k < 2000 && jt->home_status() == HomeStatus::executing; ++k) measured = step();
    REQUIRE(jt->home_status() == HomeStatus::succeeded);
    CHECK(set.aligned().test(static_cast<std::size_t>(v1)));
    CHECK_FALSE(set.aligned().test(static_cast<std::size_t>(v4)));
    // The solve starts from measured, then the joint task pulls j1 back toward the target.
    CHECK(std::abs(loop.q[j1] - (measured[j1] + loop.out.v[v1] * loop.dt)) < 1e-9);
    step();
    CHECK_FALSE(set.aligned().any());

    REQUIRE(jt->set_target(target));
    while (!jt->settled()) loop.step();
    REQUIRE(jt->request_home());
    loop.step();
    REQUIRE(jt->home_status() == HomeStatus::executing);
    set.resynchronize();
    loop.step();
    CHECK(jt->home_status() == HomeStatus::aborted);
}

TEST_CASE("gaze module: preempted while tracking, the focus is cleared rather than frozen") {
    ModelOptions o;
    o.root = RootJoint::free_flyer;
    ModelPtr m = load("omni_dual_arm_gripper.urdf", o);
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "left_tool_frame";
    for (int i = 1; i <= 7; ++i) fc.joints.push_back("left_arm_joint_" + std::to_string(i));
    fc.trajectory = pose_limits();
    auto arm = std::make_shared<FrameTaskModule>("left", fc);
    GazeModuleConfig gc;
    gc.frame = "head_yaw_link";
    gc.joints = {"head_pitch_joint", "head_yaw_joint"};
    gc.frames = {"left_tool_frame"};
    gc.target_idle_s = 3.0;
    auto gaze = std::make_shared<GazeModule>("gaze", gc);
    JointTargetModuleConfig jc;
    jc.joints = {"head_yaw_joint"};
    jc.priority = 50;
    jc.release_after_s = 1.0;
    auto head = std::make_shared<JointTargetModule>("head", jc);
    REQUIRE(set.add(arm));
    REQUIRE(set.add(gaze));
    REQUIRE(set.add(head));
    REQUIRE(set.configure());

    Loop loop{m, set, m->neutral()};
    Eigen::Isometry3d goal = pose_at(m, loop.q, "left_tool_frame");
    goal.translation().z() += 0.02;
    REQUIRE(arm->set_target(goal));
    loop.step(5);
    REQUIRE(gaze->active_frames() == 1);
    OutputValue shown;
    REQUIRE(gaze->read_output(0, shown));
    REQUIRE_FALSE(std::isnan(shown.points(0, 0)));

    REQUIRE(head->set_target(Eigen::VectorXd::Constant(1, 0.3)));
    loop.step(2);
    REQUIRE(gaze->held_by());
    CHECK(gaze->active_frames() == 0);
    REQUIRE(gaze->read_output(0, shown));
    CHECK(std::isnan(shown.points(0, 0)));
}

TEST_CASE("gaze module: aims at the mean of active frames and returns home when idle") {
    ModelOptions o;
    o.root = RootJoint::free_flyer;
    ModelPtr m = load("omni_dual_arm_gripper.urdf", o);
    ModuleSet set = make_set(m);
    std::vector<std::shared_ptr<FrameTaskModule>> arms;
    for (const char* side : {"left", "right"}) {
        FrameTaskModuleConfig fc;
        fc.frame = std::string(side) + "_tool_frame";
        for (int i = 1; i <= 7; ++i)
            fc.joints.push_back(std::string(side) + "_arm_joint_" + std::to_string(i));
        fc.trajectory = pose_limits();
        arms.push_back(std::make_shared<FrameTaskModule>(side, fc));
    }
    GazeModuleConfig gc;
    gc.frame = "head_yaw_link";
    gc.joints = {"head_pitch_joint", "head_yaw_joint"};
    gc.frames = {"left_tool_frame", "right_tool_frame"};
    gc.max_velocity = 1.0;
    gc.target_idle_s = 3.0;
    auto gaze = std::make_shared<GazeModule>("gaze", gc);
    JointTargetModuleConfig jc;
    jc.joints = {"head_yaw_joint"};
    jc.priority = 50;
    jc.release_after_s = 0.5;
    auto head = std::make_shared<JointTargetModule>("head", jc);
    REQUIRE(set.add(gaze));  // added first; dependencies still configure the arms before it
    for (auto& a : arms) REQUIRE(set.add(a));
    REQUIRE(set.add(head));
    Eigen::VectorXd initial = m->neutral();
    REQUIRE(m->set_joint_position(initial, m->joint_id("head_yaw_joint").value(), 0.2));
    REQUIRE(set.set_initial(initial));
    REQUIRE(set.configure());
    CHECK(gaze->dependencies().empty());
    CHECK(set.solver().constraint("gaze/velocity") != nullptr);

    Loop loop{m, set, m->neutral()};
    loop.step(5);
    // Start counts as a move: the head looks at both hands for target_idle_s.
    CHECK(gaze->active_frames() == 2);
    CHECK(active(loop.out, "gaze/gaze"));
    CHECK_FALSE(active(loop.out, "gaze/home"));
    OutputValue shown;
    REQUIRE(gaze->read_output(0, shown));
    CHECK_FALSE(std::isnan(shown.points(0, 0)));
    loop.step(300);
    // No targets after the idle time: nothing to look at, home task holds the head.
    CHECK(gaze->active_frames() == 0);
    CHECK(active(loop.out, "gaze/home"));
    CHECK_FALSE(active(loop.out, "gaze/gaze"));
    REQUIRE(gaze->read_output(0, shown));
    CHECK(std::isnan(shown.points(0, 0)));
    // Home is the configured initial posture, not where the loop started.
    loop.step(100);
    const double pitch0 = joint_q(m, initial, "head_pitch_joint");
    const double yaw0 = joint_q(m, initial, "head_yaw_joint");
    CHECK(joint_q(m, loop.q, "head_yaw_joint") == doctest::Approx(yaw0).epsilon(1e-3));

    // Joint input takes the head and gaze is held, also once the target is reached.
    REQUIRE(head->set_target(Eigen::VectorXd::Constant(1, yaw0)));
    loop.step();
    REQUIRE(gaze->held_by());
    CHECK(*gaze->held_by() == "head");
    loop.step(50);
    CHECK(head->claiming());
    REQUIRE(gaze->held_by());
    // A moving hand gets the head only once joint input has stopped for release_after_s (0.5 s here).
    Eigen::Isometry3d probe = pose_at(m, loop.q, "left_tool_frame");
    probe.translation().z() += 0.1;
    REQUIRE(arms[0]->set_target(probe));
    loop.step(15);
    CHECK_FALSE(gaze->held_by());
    REQUIRE(head->set_target(Eigen::VectorXd::Constant(1, yaw0)));
    loop.step(10);
    REQUIRE(gaze->held_by());
    CHECK(*gaze->held_by() == "head");
    REQUIRE(
        arms[0]->set_target(pose_at(m, loop.q, "left_tool_frame") * Eigen::Translation3d(0.0, 0.0, -0.1)));
    loop.step(20);
    CHECK(head->claiming());
    loop.step(30);
    CHECK_FALSE(head->claiming());
    CHECK_FALSE(gaze->held_by());
    // The hands go idle and the head returns home.
    loop.step(800);
    CHECK(joint_q(m, loop.q, "head_yaw_joint") == doctest::Approx(yaw0).epsilon(1e-3));

    const auto axis_error = [&] {
        const Eigen::Vector3d focus = pose_at(m, loop.q, "left_tool_frame").translation();
        const Eigen::Isometry3d cam = pose_at(m, loop.q, "head_yaw_link");
        const Eigen::Vector3d axis = cam.linear() * Eigen::Vector3d::UnitX();
        return std::acos(std::clamp(axis.dot((focus - cam.translation()).normalized()), -1.0, 1.0));
    };
    Eigen::Isometry3d left = pose_at(m, loop.q, "left_tool_frame");
    Eigen::Isometry3d right = pose_at(m, loop.q, "right_tool_frame");
    // One hand, brought forward: beside the head the pitch-yaw chain cannot look down (pitch limit 1.05).
    left.translation() += Eigen::Vector3d(0.3, -0.4, 0.0);
    REQUIRE(arms[0]->set_target(left));
    loop.step(10);
    // Once the hand actually moves (jerk-limited start, a few cycles) the head is taken back.
    CHECK_FALSE(gaze->held_by());
    CHECK_FALSE(head->claiming());
    CHECK(gaze->active_frames() == 1);
    CHECK(active(loop.out, "gaze/gaze"));
    CHECK_FALSE(active(loop.out, "gaze/home"));
    REQUIRE(gaze->read_output(0, shown));
    CHECK(shown.points.col(0).isApprox(pose_at(m, loop.q, "left_tool_frame").translation(), 0.05));
    REQUIRE(gaze->read_output(1, shown));
    CHECK(shown.number == 1.0);
    const double before = axis_error();
    loop.step(40);
    CHECK(gaze->active_frames() == 1);
    // max_velocity 1 rad/s bounds the head over these 50 steps.
    CHECK(std::abs(joint_q(m, loop.q, "head_yaw_joint") - yaw0) <= 0.50 + 1e-6);
    loop.step(200);
    const double after = axis_error();
    INFO("axis error " << before << " -> " << after);
    CHECK(after < 0.02);
    CHECK(after < before);
    const double head_step = std::abs(joint_q(m, loop.q, "head_pitch_joint") - pitch0) +
                             std::abs(joint_q(m, loop.q, "head_yaw_joint") - yaw0);
    CHECK(head_step > 1e-3);

    // Idle targets drop out of the focus; with none left the head returns.
    loop.step(600);
    CHECK(gaze->active_frames() == 0);
    CHECK(active(loop.out, "gaze/home"));
    CHECK(joint_q(m, loop.q, "head_pitch_joint") == doctest::Approx(pitch0).epsilon(1e-3));
    CHECK(joint_q(m, loop.q, "head_yaw_joint") == doctest::Approx(yaw0).epsilon(1e-3));

    // Both hands moving count as two frames.
    left.translation().z() += 0.02;
    right.translation().z() += 0.02;
    REQUIRE(arms[0]->set_target(left));
    REQUIRE(arms[1]->set_target(right));
    loop.step(10);
    CHECK(gaze->active_frames() == 2);

    // max_velocity is tunable; 0 lifts the limit, negative is rejected.
    ConfigNode slow = ConfigNode::map();
    slow.set("max_velocity", 0.1);
    REQUIRE(gaze->tune(slow));
    left.translation() += Eigen::Vector3d(0.0, 0.3, 0.1);
    REQUIRE(arms[0]->set_target(left));
    const double yaw1 = joint_q(m, loop.q, "head_yaw_joint");
    loop.step(10);
    CHECK(std::abs(joint_q(m, loop.q, "head_yaw_joint") - yaw1) <= 0.01 + 1e-6);
    CHECK(std::abs(joint_q(m, loop.q, "head_yaw_joint") - yaw1) > 1e-4);
    ConfigNode free = ConfigNode::map();
    free.set("max_velocity", 0.0);
    REQUIRE(gaze->tune(free));
    ConfigNode negative = ConfigNode::map();
    negative.set("max_velocity", -1.0);
    CHECK_FALSE(gaze->tune(negative));

    GazeModuleConfig bad = gc;
    bad.frames.push_back("no_such_frame");
    ModuleSet bad_set = make_set(m);
    REQUIRE(bad_set.add(std::make_shared<GazeModule>("gaze", bad)));
    CHECK_FALSE(bad_set.configure());
}

TEST_CASE("gaze module: follows frames moved by joint control, ignores base motion") {
    ModelOptions o;
    o.root = RootJoint::free_flyer;
    ModelPtr m = load("omni_dual_arm_gripper.urdf", o);
    ModuleSet set = make_set(m);
    JointTargetModuleConfig ac;
    for (int i = 1; i <= 7; ++i) ac.joints.push_back("left_arm_joint_" + std::to_string(i));
    auto arm = std::make_shared<JointTargetModule>("left", ac);
    GazeModuleConfig gc;
    gc.frame = "head_yaw_link";
    gc.joints = {"head_pitch_joint", "head_yaw_joint"};
    gc.frames = {"left_tool_frame"};
    gc.target_idle_s = 1.0;
    auto gaze = std::make_shared<GazeModule>("gaze", gc);
    REQUIRE(set.add(arm));
    REQUIRE(set.add(gaze));
    REQUIRE(set.configure());

    Loop loop{m, set, m->neutral()};
    loop.step(150);
    REQUIRE(gaze->active_frames() == 0);

    // Base motion alone: the frame is fixed relative to the root, so it stays idle.
    for (int i = 0; i < 50; ++i) {
        loop.q(0) += 0.01;
        loop.step();
    }
    CHECK(gaze->active_frames() == 0);

    Eigen::VectorXd target = Eigen::VectorXd::Zero(7);
    target(1) = 0.6;
    target(3) = -0.8;
    REQUIRE(arm->set_target(target));
    loop.step(20);
    CHECK(gaze->active_frames() == 1);
    CHECK(active(loop.out, "gaze/gaze"));
    loop.step(300);
    CHECK(gaze->active_frames() == 0);
    CHECK(active(loop.out, "gaze/home"));
}

TEST_CASE("center_of_mass module: only the recovery tasks' joints may pull the CoM back") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.joints = {"j1", "j2"};
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    CenterOfMassModuleConfig cc;
    cc.frame = "base_link";
    cc.polygon = {Eigen::Vector2d(-1, -1), Eigen::Vector2d(1, -1), Eigen::Vector2d(1, 1),
                  Eigen::Vector2d(-1, 1)};
    cc.recovery_tasks = {ft};
    auto com = std::make_shared<CenterOfMassModule>("com", cc);
    REQUIRE(set.add(com));
    REQUIRE(set.add(ft));
    REQUIRE(set.configure());
    CHECK(com->dependencies() == std::vector<std::string>{"tool"});
    REQUIRE(com->constraint());
    CHECK(set.solver().constraint("com/support") == com->constraint());

    Loop loop{m, set, arm_pose(m, 0.3)};
    loop.step(5);
    CHECK(com->running());
    Workspace ws(m);
    REQUIRE(ws.update(loop.q));
    const Index rows = com->constraint()->rows();
    const Index n = com->constraint()->polygon().edges();
    REQUIRE(rows == 2 * n);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(rows, m->nv());
    Eigen::VectorXd lo = Eigen::VectorXd::Constant(rows, -INFINITY),
                    hi = Eigen::VectorXd::Constant(rows, INFINITY);
    REQUIRE(com->constraint()->evaluate(Tick{ws, 0.01}, A, lo, hi));
    for (const JointInfo& j : m->joints()) {
        if (j.nv != 1) continue;
        const bool selected = j.name == "j1" || j.name == "j2";
        INFO(j.name);
        if (selected) {
            CHECK(A.topRows(n).col(j.idx_v).norm() > 0.0);
            CHECK(A.bottomRows(n).col(j.idx_v).norm() == 0.0);
        } else {
            CHECK(A.topRows(n).col(j.idx_v).norm() == 0.0);
        }
    }

    cc.recovery_tasks = {nullptr};
    ModuleSet bad = make_set(m);
    REQUIRE(bad.add(std::make_shared<CenterOfMassModule>("com", cc)));
    CHECK_FALSE(bad.configure());
}

TEST_CASE("center_of_mass module: display outputs") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    CenterOfMassModuleConfig cc;
    cc.frame = "base_link";
    cc.margin = 0.1;
    cc.polygon = {Eigen::Vector2d(-1, -1), Eigen::Vector2d(1, -1), Eigen::Vector2d(1, 1),
                  Eigen::Vector2d(-1, 1)};
    auto com = std::make_shared<CenterOfMassModule>("com", cc);
    REQUIRE(set.add(com));
    REQUIRE(set.configure());
    REQUIRE(com->outputs().size() == 6);
    CHECK(com->outputs()[2].type == OutputType::polygon);
    CHECK(com->outputs()[2].size == 4);
    CHECK(com->visuals().empty());

    Workspace ws(m);
    REQUIRE(ws.update(arm_pose(m, 0.3)));
    REQUIRE(com->update_outputs(ws));
    OutputValue com_point, ground, polygon, margin, slack, inside;
    REQUIRE(com->read_output(0, com_point));
    REQUIRE(com->read_output(1, ground));
    REQUIRE(com->read_output(2, polygon));
    REQUIRE(com->read_output(3, margin));
    REQUIRE(com->read_output(4, slack));
    REQUIRE(com->read_output(5, inside));
    Eigen::Isometry3d base;
    REQUIRE(ws.frame_pose(*m->frame_id("base_link"), base));
    const Eigen::Vector3d local = base.inverse() * ground.points.col(0);
    CHECK(local.z() == doctest::Approx(0.0).epsilon(1e-9));
    CHECK((base.inverse() * com_point.points.col(0)).head<2>().isApprox(local.head<2>()));
    REQUIRE(polygon.points.cols() == 4);
    REQUIRE(margin.points.cols() == 4);
    for (Eigen::Index i = 0; i < 4; ++i) {
        const Eigen::Vector3d outer = base.inverse() * polygon.points.col(i);
        const Eigen::Vector3d inner = base.inverse() * margin.points.col(i);
        CHECK(outer.head<2>().cwiseAbs().isApprox(Eigen::Vector2d(1, 1)));
        CHECK(inner.head<2>().cwiseAbs().isApprox(Eigen::Vector2d(0.9, 0.9)));
    }
    CHECK(slack.number == doctest::Approx(1.0 - local.head<2>().cwiseAbs().maxCoeff()));
    CHECK(inside.flag == (slack.number >= 0.1));
}

TEST_CASE("null_space_preference module: one preference task active, switchable at runtime") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.trajectory = pose_limits();
    fc.task.level = 0;
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    NullSpacePreferenceModuleConfig nc;
    nc.primary = ft;
    auto ns = std::make_shared<NullSpacePreferenceModule>("ns", nc);
    REQUIRE(set.add(ns));
    REQUIRE(set.add(ft));
    const Status configured = set.configure();
    REQUIRE_MESSAGE(configured, (configured ? "" : configured.error().message));
    CHECK(ns->dependencies() == std::vector<std::string>{"tool"});

    Loop loop{m, set, arm_pose(m, 0.2)};
    loop.step();
    CHECK(active(loop.out, "ns/posture"));
    CHECK_FALSE(active(loop.out, "ns/manipulability"));
    CHECK_FALSE(active(loop.out, "ns/joint_limits"));

    ns->set_preference(NullSpacePreference::manipulability);
    loop.step();
    CHECK(ns->preference() == NullSpacePreference::manipulability);
    CHECK_FALSE(active(loop.out, "ns/posture"));
    CHECK(active(loop.out, "ns/manipulability"));

    ns->set_preference(NullSpacePreference::joint_limits);
    loop.step();
    CHECK(active(loop.out, "ns/joint_limits"));
    CHECK_FALSE(active(loop.out, "ns/manipulability"));

    // The primary stays tracked under any preference.
    const Eigen::Isometry3d hold = pose_at(m, loop.q, "tool");
    loop.step(100);
    CHECK((pose_at(m, loop.q, "tool").translation() - hold.translation()).norm() < 1e-3);

    nc.reference = PostureReference::custom;
    nc.posture = Eigen::VectorXd::Zero(2);
    ModuleSet bad = make_set(m);
    auto ft2 = std::make_shared<FrameTaskModule>("tool", fc);
    REQUIRE(bad.add(ft2));
    nc.primary = ft2;
    REQUIRE(bad.add(std::make_shared<NullSpacePreferenceModule>("ns", nc)));
    CHECK_FALSE(bad.configure());
}

TEST_CASE("null_space_preference module: held with its primary, off while the primary is disabled") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.trajectory = pose_limits();
    fc.task.level = 0;
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    NullSpacePreferenceModuleConfig nc;
    nc.primary = ft;
    auto ns = std::make_shared<NullSpacePreferenceModule>("ns", nc);
    JointTargetModuleConfig jc;
    jc.joints = {"j1", "j2", "j3"};
    jc.priority = 1;
    auto jt = std::make_shared<JointTargetModule>("joints", jc);
    REQUIRE(set.add(ns));
    REQUIRE(set.add(ft));
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());

    Loop loop{m, set, arm_pose(m, 0.2)};
    loop.step();
    CHECK(ns->running());
    CHECK(active(loop.out, "ns/posture"));

    REQUIRE(jt->set_target(Eigen::Vector3d(0.5, -0.3, 0.4)));
    loop.step();
    CHECK_FALSE(ft->running());
    CHECK_FALSE(ns->running());
    REQUIRE(ns->held_by());
    CHECK(*ns->held_by() == "joints");
    CHECK_FALSE(active(loop.out, "ns/posture"));

    while (!jt->settled()) loop.step();
    loop.step(10);
    CHECK_FALSE(ns->running());
    // A pose target releases the frame task and its null space together.
    REQUIRE(ft->set_target(pose_at(m, loop.q, "tool")));
    loop.step();
    CHECK(ft->running());
    CHECK(ns->running());
    CHECK(active(loop.out, "ns/posture"));
    ft->set_enabled(false);
    loop.step();
    CHECK(ns->running());
    CHECK_FALSE(active(loop.out, "ns/posture"));
}
