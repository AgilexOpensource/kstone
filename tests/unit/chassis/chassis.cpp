#include "kstone/chassis/chassis.hpp"

#include "kstone/control/solver.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"

#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
constexpr double kPi = 3.14159265358979323846;

ChassisController started(ChassisConfig config, double period, double now = 0.0) {
    auto c = ChassisController::create(std::move(config));
    REQUIRE_MESSAGE(c, (c ? "" : c.error().message));
    REQUIRE(c->set_scheduler_period(period));
    REQUIRE(c->start(now));
    return std::move(*c);
}

bool accepted(const Result<bool>& r) {
    REQUIRE(r);
    return *r;
}

ChassisConfig differential() {
    ChassisConfig c;
    c.kinematics = ChassisKinematics::differential;
    return c;
}
}  // namespace

TEST_CASE("chassis: velocity mode limits and lease timeout") {
    const ChassisConfig config = differential();
    auto ctrl = started(config, 0.01);
    CHECK(accepted(ctrl.submit_velocity({1.0, 1.0, 2.0}, 0.0)));
    auto out = ctrl.step(0.01);
    CHECK(std::abs(out.velocity.vx - 0.0075) < 1e-12);
    CHECK(out.velocity.vy == 0.0);
    double now = 0.01;
    for (int i = 0; i < 100; ++i) {
        now += 0.01;
        REQUIRE(ctrl.submit_velocity({1.0, 1.0, 2.0}, now));
        out = ctrl.step(now);
        REQUIRE(std::hypot(out.velocity.vx, out.velocity.vy) <= config.max_linear + 1e-12);
        REQUIRE(std::abs(out.velocity.wz) <= config.max_angular + 1e-12);
    }
    CHECK(std::abs(out.velocity.vx - config.max_linear) < 1e-9);
    ctrl.step(now + 0.01);
    now += 0.01;
    out = ctrl.service(now + 0.3, true);
    CHECK(out.reason == ChassisReason::command_timeout);
    CHECK(out.velocity.vx == 0.0);
}

TEST_CASE("chassis: command arbitration by priority and freshness") {
    auto ctrl = started({}, 0.01);
    CHECK(accepted(ctrl.submit_velocity({0.1, 0, 0}, 0.0, "teleop", 10)));
    CHECK_FALSE(accepted(ctrl.submit_velocity({0.2, 0, 0}, 0.01, "ui", 0)));
    CHECK(accepted(ctrl.submit_velocity({0.2, 0, 0}, 0.30, "ui", 0)));
    CHECK(accepted(ctrl.submit_pose({1, 0, 0}, 0.31, "planner", 0)));
    CHECK(ctrl.command_snapshot().pose_mode);
    CHECK(ctrl.command_snapshot().source == "planner");
    CHECK(accepted(ctrl.submit_velocity({0.1, 0, 0}, 0.32, "ui", 0)));
    CHECK_FALSE(ctrl.command_snapshot().pose_mode);
}

TEST_CASE("chassis: scheduler discontinuity faults and resynchronizes") {
    ChassisConfig config;
    config.max_dt_s = 0.05;
    auto ctrl = started(config, 0.1);
    ctrl.step(0.11);
    CHECK(ctrl.state() == ChassisState::active);
    ctrl.step(0.32);
    CHECK(ctrl.state() == ChassisState::fault);
    CHECK(ctrl.reason() == ChassisReason::scheduler_discontinuity);
    REQUIRE(ctrl.resynchronize(1.0));
    CHECK(ctrl.state() == ChassisState::active);
    auto slow = started({}, 1.0);
    slow.step(1.0);
    CHECK(slow.state() == ChassisState::active);
}

TEST_CASE("chassis: odometry replaces the integrated pose while fresh") {
    auto ctrl = started({}, 0.01);
    CHECK(accepted(ctrl.submit_odometry({5, 0, 0}, 0.0, 0.0)));
    CHECK(ctrl.pose().x == 5.0);
    CHECK_FALSE(accepted(ctrl.submit_odometry({6, 0, 0}, 0.0, 0.0)));
    CHECK_FALSE(accepted(ctrl.submit_odometry({6, 0, 0}, -1.0, 0.0)));
    CHECK(ctrl.snapshot(0.0).source == PoseSource::odometry);
    CHECK(ctrl.snapshot(1.0).source == PoseSource::internal);
}

TEST_CASE("chassis: omni pose mode is bounded and reaches the goal") {
    for (double dt : {0.01, 0.02, 0.05, 0.08}) {
        INFO("dt=" << dt);
        auto ctrl = started({}, dt);
        const ChassisPoseLimits limits = ctrl.pose_limits();
        const Pose2D goal{0.6, -0.3, 0.8};
        double now = 0.0;
        Twist2D previous;
        const int cycles = static_cast<int>(12.0 / dt);
        for (int i = 0; i < cycles; ++i) {
            REQUIRE(ctrl.submit_pose(goal, now));
            now += dt;
            const auto out = ctrl.step(now);
            INFO("cycle=" << i << " error=" << ctrl.last_error());
            REQUIRE(out.reason == ChassisReason::tracking);
            const double dvx = out.velocity.vx - previous.vx;
            const double dvy = out.velocity.vy - previous.vy;
            REQUIRE(std::hypot(out.velocity.vx, out.velocity.vy) <= limits.linear_velocity + 1e-9);
            REQUIRE(std::hypot(dvx, dvy) <= limits.linear_acceleration * dt * 1.2 + 1e-9);
            REQUIRE(ctrl.pose().x <= goal.x + 1e-9);
            previous = out.velocity;
        }
        const Pose2D pose = ctrl.pose();
        CHECK(std::abs(pose.x - goal.x) < 1e-6);
        CHECK(std::abs(pose.y - goal.y) < 1e-6);
        CHECK(std::abs(wrap_angle(pose.yaw - goal.yaw)) < 1e-6);
        CHECK(std::abs(previous.vx) < 1e-9);
        CHECK(std::abs(previous.wz) < 1e-9);
    }
}

TEST_CASE("chassis: switching from velocity to pose mode does not keep accelerating sideways") {
    auto ctrl = started({}, 0.02);
    double now = 0.0;
    for (int i = 0; i < 5; ++i) {
        REQUIRE(ctrl.submit_velocity({0.0, 5.0, 0.0}, now));
        now += 0.02;
        ctrl.step(now);
    }
    const double vy = ctrl.snapshot(now).velocity.vy;
    REQUIRE(vy > 0.0);
    double max_vy = vy;
    for (int i = 0; i < 100; ++i) {
        REQUIRE(ctrl.submit_pose({2.0, 0.0, 0.0}, now));
        now += 0.02;
        max_vy = std::max(max_vy, ctrl.step(now).velocity.vy);
    }
    CHECK(max_vy <= vy + 1e-9);
}

TEST_CASE("chassis: omni pose mode keeps yaw wrapped across +-pi") {
    auto ctrl = started({}, 0.02);
    double now = 0.0;
    double max_wz = 0.0;
    for (const Pose2D goal : {Pose2D{0, 0, 3.0}, Pose2D{0, 0, -3.0}}) {
        for (int i = 0; i < 1000; ++i) {
            REQUIRE(ctrl.submit_pose(goal, now));
            now += 0.02;
            const auto out = ctrl.step(now);
            REQUIRE(out.reason == ChassisReason::tracking);
            REQUIRE(std::abs(ctrl.pose().yaw) <= kPi + 1e-12);
            if (goal.yaw < 0) max_wz = std::max(max_wz, out.velocity.wz);
        }
        CHECK(std::abs(wrap_angle(ctrl.pose().yaw - goal.yaw)) < 1e-6);
    }
    CHECK(max_wz > 0.0);
}

TEST_CASE("chassis: far pose goals still move at the speed limit") {
    for (const ChassisKinematics kinematics : {ChassisKinematics::omni, ChassisKinematics::differential}) {
        ChassisConfig config;
        config.kinematics = kinematics;
        auto ctrl = started(config, 0.02);
        const double max_linear = ctrl.pose_limits().linear_velocity;
        for (const Pose2D goal : {Pose2D{1e4, 0, 0}, Pose2D{1e7, 1e7, 1.0}}) {
            INFO("kinematics=" << static_cast<int>(kinematics) << " goal.x=" << goal.x);
            double now = 0.0;
            const Pose2D start = ctrl.pose();
            for (int i = 0; i < 500; ++i) {
                REQUIRE(ctrl.submit_pose(goal, now));
                now += 0.02;
                const auto out = ctrl.step(now);
                INFO("cycle=" << i << " error=" << ctrl.last_error());
                REQUIRE(out.reason == ChassisReason::tracking);
            }
            const double travelled = std::hypot(ctrl.pose().x - start.x, ctrl.pose().y - start.y);
            CHECK(travelled > 0.5 * max_linear * 10.0 - 3.0);
            CHECK(std::hypot(ctrl.output().vx, ctrl.output().vy) > 0.5 * max_linear);
            ctrl = started(config, 0.02);
        }
    }
}

TEST_CASE("chassis: zero pose limits bypass shaping") {
    auto ctrl = started({}, 0.01);
    REQUIRE(ctrl.tune_pose_limits(ChassisPoseLimits{}));
    REQUIRE(ctrl.submit_pose({1, 0, 1}, 0.0));
    const auto out = ctrl.step(0.01);
    CHECK(std::abs(out.velocity.vx - 100) < 1e-9);
    CHECK(std::abs(out.velocity.wz - 100) < 1e-9);
}

TEST_CASE("chassis: differential pose mode turns, drives and aligns") {
    const ChassisConfig config = differential();
    auto ctrl = started(config, 0.02);
    const Pose2D goal{0.5, 0.4, -0.5};
    double now = 0.0;
    bool saw_drive = false;
    bool saw_final = false;
    for (int i = 0; i < 1500; ++i) {
        REQUIRE(ctrl.submit_pose(goal, now));
        now += 0.02;
        const auto out = ctrl.step(now);
        INFO("cycle=" << i << " error=" << ctrl.last_error());
        REQUIRE(out.reason == ChassisReason::tracking);
        REQUIRE(out.velocity.vy == 0.0);
        REQUIRE(out.velocity.vx >= -1e-12);
        saw_drive |= ctrl.diff_phase() == DiffPhase::drive;
        saw_final |= ctrl.diff_phase() == DiffPhase::final_align;
    }
    const Pose2D pose = ctrl.pose();
    CHECK(saw_drive);
    CHECK(saw_final);
    CHECK(std::hypot(pose.x - goal.x, pose.y - goal.y) <= 1.5 * config.arrival_position_tolerance);
    CHECK(std::abs(wrap_angle(pose.yaw - goal.yaw)) <= 1.5 * config.arrival_rotation_tolerance);
    CHECK(ctrl.output().vx == 0.0);
    CHECK(ctrl.output().wz == 0.0);
}

TEST_CASE("chassis: unlimited axis does not replay another axis's brake profile") {
    // Angular limits are finite and get lowered below the current rate (Ruckig brakes); linear limits are
    // unbounded, so its update skips the brake computation and must not inherit the angular one.
    auto ctrl = started(differential(), 0.02);
    ChassisPoseLimits limits;
    limits.angular_velocity = 1.0;
    limits.angular_acceleration = 2.0;
    CHECK_FALSE(ctrl.tune_pose_limits(limits));  // default axis_phase is omni-only
    limits.planner_mode = ChassisPlannerMode::straight;
    REQUIRE(ctrl.tune_pose_limits(limits));
    const Pose2D goal{0.0, 2.0, 0.0};  // bearing +90 deg: a long align turn before driving
    double now = 0.0;
    bool lowered = false;
    int checked = 0;
    for (int i = 0; i < 200 && ctrl.diff_phase() != DiffPhase::drive; ++i) {
        REQUIRE(ctrl.submit_pose(goal, now));
        now += 0.02;
        const auto out = ctrl.step(now);
        INFO("cycle=" << i << " error=" << ctrl.last_error());
        REQUIRE(out.reason == ChassisReason::tracking);
        if (ctrl.diff_phase() == DiffPhase::align) {
            REQUIRE(out.velocity.vx == 0.0);
            ++checked;
        }
        if (!lowered && out.velocity.wz > 0.9) {
            limits.angular_velocity = 0.3;
            REQUIRE(ctrl.tune_pose_limits(limits));
            lowered = true;
        }
    }
    CHECK(lowered);
    CHECK(checked > 10);
}

TEST_CASE("chassis: SE(2) integration and angle wrapping") {
    const Pose2D pose = integrate_se2({0, 0, 0}, {1.0, 0.0, 1.0}, kPi / 2);
    CHECK(std::abs(pose.x - 1.0) < 1e-12);
    CHECK(std::abs(pose.y - 1.0) < 1e-12);
    CHECK(std::abs(wrap_angle(kPi) + kPi) < 1e-12);
}

TEST_CASE("chassis: invalid input is reported, not thrown") {
    ChassisConfig bad;
    bad.max_linear = 0.0;
    auto c = ChassisController::create(bad);
    REQUIRE_FALSE(c);
    CHECK(c.error().code == ErrorCode::invalid_argument);

    auto ctrl = ChassisController::create({});
    REQUIRE(ctrl);
    CHECK_FALSE(ctrl->start(0.0));  // no scheduler period
    CHECK(ctrl->state() == ChassisState::configured);
    CHECK(ctrl->step(0.01).reason == ChassisReason::not_started);
    CHECK_FALSE(ctrl->set_scheduler_period(-1.0));
    REQUIRE(ctrl->set_scheduler_period(0.01));
    REQUIRE(ctrl->start(0.0));
    CHECK_FALSE(ctrl->start(0.0));

    const double nan = std::numeric_limits<double>::quiet_NaN();
    auto r = ctrl->submit_velocity({nan, 0, 0}, 0.0);
    REQUIRE_FALSE(r);
    CHECK(r.error().code == ErrorCode::invalid_argument);
    CHECK_FALSE(ctrl->submit_pose({0, 0, 0}, 0.0, ""));
    CHECK_FALSE(ctrl->submit_pose({0, 0, 0}, 0.0, "x", -1));
    CHECK_FALSE(ctrl->submit_odometry({0, nan, 0}, 0.0, 0.0));

    ChassisPoseLimits limits;
    limits.linear_velocity = -1.0;
    CHECK_FALSE(ctrl->tune_pose_limits(limits));

    ctrl->stop(0.1);
    CHECK(ctrl->state() == ChassisState::stopped);
    CHECK_FALSE(accepted(ctrl->submit_velocity({0.1, 0, 0}, 0.1)));
}

TEST_CASE("chassis: root pose from a chassis center") {
    Eigen::Isometry3d center_in_root = Eigen::Isometry3d::Identity();
    center_in_root.translation() << 0.1, 0.0, 0.0;
    const Eigen::Isometry3d root = chassis_root_pose({1.0, 2.0, kPi / 2}, center_in_root);
    const Eigen::Isometry3d center = root * center_in_root;
    CHECK((center.translation() - Eigen::Vector3d(1.0, 2.0, 0.0)).norm() < 1e-12);
    CHECK((root.translation() - Eigen::Vector3d(1.0, 1.9, 0.0)).norm() < 1e-12);
    CHECK(std::abs(std::atan2(root.linear()(1, 0), root.linear()(0, 0)) - kPi / 2) < 1e-12);
}

// The chassis lives outside the QP: it drives the root pose, the solver moves only the arm.
TEST_CASE("chassis: drives the base of a mobile model with the root locked") {
    const fs::path fixtures = KSTONE_FIXTURES;
    ModelOptions mo;
    mo.root = RootJoint::free_flyer;
    auto loaded =
        Model::load(ModelSource::file((fixtures / "diff_chassis.urdf").string()), mo, Context::defaults());
    REQUIRE_MESSAGE(loaded, (loaded ? "" : loaded.error().message));
    const ModelPtr m = *loaded;

    // Target: the tool pose with the base parked at the goal and the arm displaced.
    const Pose2D goal{0.8, 0.3, 0.4};
    Eigen::VectorXd q_goal = m->neutral();
    Eigen::VectorXd dv = Eigen::VectorXd::Zero(m->nv());
    for (Index i = 6; i < m->nv(); ++i) dv[i] = 0.1;
    Eigen::VectorXd tmp(m->nq());
    REQUIRE(m->integrate(q_goal, dv, tmp));
    q_goal = tmp;
    REQUIRE(m->set_root_pose(q_goal, chassis_root_pose(goal)));
    Workspace ws(m);
    const auto tool = m->frame_id("tool");
    REQUIRE(tool);
    REQUIRE(ws.update(q_goal));
    Eigen::Isometry3d target;
    REQUIRE(ws.frame_pose(*tool, target));

    // Lock the root velocity coordinates (the free-flyer root owns the first six).
    SolverOptions so;
    so.locked = Mask(static_cast<std::size_t>(m->nv()), false);
    for (std::size_t i = 0; i < 6; ++i) so.locked.set(i);
    auto solver = Solver::create(m, so);
    REQUIRE_MESSAGE(solver, (solver ? "" : solver.error().message));
    const auto tool_task = std::make_shared<FrameTask>("tool", target);
    REQUIRE(solver->add_task("tool", tool_task));

    const double dt = 0.02;
    auto ctrl = started(differential(), dt);
    Eigen::VectorXd q = m->neutral();
    SolveResult r;
    double now = 0.0;
    for (int i = 0; i < 1500; ++i) {
        REQUIRE(ctrl.submit_pose(goal, now));
        now += dt;
        REQUIRE(ctrl.step(now).reason == ChassisReason::tracking);
        REQUIRE(m->set_root_pose(q, chassis_root_pose(ctrl.pose())));
        REQUIRE(solver->solve(q, dt, r));
        // The QP pins the root to 0; integrate/difference on the root leaves ~1e-12 round-off.
        REQUIRE(r.v.head<6>().norm() < 1e-9);
        q = r.q_next;
    }
    REQUIRE(m->set_root_pose(q, chassis_root_pose(ctrl.pose())));
    REQUIRE(ws.update(q));
    Eigen::Isometry3d reached;
    REQUIRE(ws.frame_pose(*tool, reached));
    INFO("position error " << (reached.translation() - target.translation()).norm() << " rotation error "
                           << (reached.linear() - target.linear()).norm() << " base " << ctrl.pose().x << ","
                           << ctrl.pose().y << "," << ctrl.pose().yaw << " residual "
                           << r.residuals[0].residual);
    auto base = m->root_pose(q);
    REQUIRE(base);
    const double base_error = std::hypot(base->translation().x() - goal.x, base->translation().y() - goal.y);
    const double tolerance = 1.5 * ChassisConfig{}.arrival_position_tolerance;
    CHECK(base_error < tolerance);
    // The differential base settles within its arrival tolerance; the 4-DOF arm cannot cancel that
    // offset for a full 6D target, so the tool error is bounded by the base tolerance.
    CHECK((reached.translation() - target.translation()).norm() < tolerance);
    CHECK((reached.linear() - target.linear()).norm() < 1.5 * ChassisConfig{}.arrival_rotation_tolerance);

    // Attribute that residual to the base offset alone: re-target the same arm configuration with the base
    // where it actually stopped; the locked-root solve must then converge to the target exactly.
    Eigen::VectorXd q_shifted = q_goal;
    REQUIRE(m->set_root_pose(q_shifted, chassis_root_pose(ctrl.pose())));
    REQUIRE(ws.update(q_shifted));
    Eigen::Isometry3d shifted;
    REQUIRE(ws.frame_pose(*tool, shifted));
    REQUIRE(tool_task->set_target(shifted));
    REQUIRE(ws.update(q));
    REQUIRE(ws.frame_pose(*tool, reached));
    const double before = (reached.translation() - shifted.translation()).norm();
    INFO("shifted error before re-solve " << before);
    CHECK(before > 1e-4);  // the arm had compensated for the offset, so it must move back
    for (int i = 0; i < 500; ++i) {
        REQUIRE(solver->solve(q, dt, r));
        q = r.q_next;
    }
    REQUIRE(ws.update(q));
    REQUIRE(ws.frame_pose(*tool, reached));
    INFO("shifted position error " << (reached.translation() - shifted.translation()).norm());
    CHECK((reached.translation() - shifted.translation()).norm() < 1e-6);
    CHECK((reached.linear() - shifted.linear()).norm() < 1e-6);
}
