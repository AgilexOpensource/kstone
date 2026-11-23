#include "kstone/chassis/chassis.hpp"

#include "kstone/control/solver.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

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
    // Clamped to the velocity limits only; the base ramps the step itself.
    CHECK(out.velocity.vx == doctest::Approx(config.max_linear_velocity));
    CHECK(out.velocity.vy == 0.0);
    CHECK(out.velocity.wz == doctest::Approx(config.max_angular_velocity));
    REQUIRE(ctrl.submit_velocity({0.0, 0.0, 0.0}, 0.01));
    CHECK(ctrl.step(0.02).velocity.vx == 0.0);
    double now = 0.01;
    for (int i = 0; i < 100; ++i) {
        now += 0.01;
        REQUIRE(ctrl.submit_velocity({1.0, 1.0, 2.0}, now));
        out = ctrl.step(now);
        REQUIRE(std::hypot(out.velocity.vx, out.velocity.vy) <= config.max_linear_velocity + 1e-12);
        REQUIRE(std::abs(out.velocity.wz) <= config.max_angular_velocity + 1e-12);
    }
    CHECK(std::abs(out.velocity.vx - config.max_linear_velocity) < 1e-9);
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

TEST_CASE("chassis: a velocity command drops the pose plan and its reported ramp pose") {
    auto ctrl = started({}, 0.01);
    const Pose2D goal{1.0, 0.0, 0.0};
    double now = 0.0;
    for (int i = 0; i < 20; ++i) {
        now += 0.01;
        REQUIRE(ctrl.submit_pose(goal, now));
        ctrl.step(now);
    }
    const Pose2D planned = ctrl.ramp_pose();
    CHECK(std::hypot(planned.x - ctrl.pose().x, planned.y - ctrl.pose().y) >
          1e-3);  // a plan, ahead of the base

    REQUIRE(ctrl.submit_velocity({0.2, 0.0, 0.0}, now));
    CHECK_FALSE(ctrl.command_snapshot().pose_mode);
    CHECK(ctrl.ramp_pose().x == doctest::Approx(ctrl.pose().x));
    CHECK(ctrl.ramp_pose().y == doctest::Approx(ctrl.pose().y));
    for (int i = 0; i < 20; ++i) {
        now += 0.01;
        REQUIRE(ctrl.submit_velocity({0.2, 0.0, 0.0}, now));
        ctrl.step(now);
        // Velocity mode has no trajectory: planned follows the base instead of freezing where the goal was.
        CHECK(ctrl.ramp_pose().x == doctest::Approx(ctrl.pose().x));
        CHECK(ctrl.ramp_pose().yaw == doctest::Approx(ctrl.pose().yaw));
    }
    CHECK(ctrl.pose().x > planned.x - 1.0);  // moved on, not held by the old plan
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

TEST_CASE("chassis: zero odometry timeout never goes stale; start and resynchronize seed the pose") {
    ChassisConfig config;
    config.odom_timeout_s = 0.0;
    auto ctrl = started(config, 0.01);
    CHECK(accepted(ctrl.submit_odometry({1, 0, 0}, 0.0, 100.0)));
    CHECK(ctrl.snapshot(1000.0).source == PoseSource::odometry);
    config.odom_timeout_s = -1.0;
    CHECK_FALSE(ChassisController::create(config));

    auto seeded = ChassisController::create({});
    REQUIRE(seeded);
    REQUIRE(seeded->set_scheduler_period(0.01));
    CHECK_FALSE(seeded->start(0.0, Pose2D{std::nan(""), 0, 0}));
    REQUIRE(seeded->start(0.0, Pose2D{1, 2, 0.5}));
    seeded->step(0.01);
    CHECK(seeded->pose().x == doctest::Approx(1.0));
    CHECK(seeded->pose().y == doctest::Approx(2.0));
    CHECK(seeded->pose().yaw == doctest::Approx(0.5));
    REQUIRE(seeded->pause(0.02));
    CHECK_FALSE(seeded->resynchronize(0.03, Pose2D{0, std::numeric_limits<double>::infinity(), 0}));
    REQUIRE(seeded->resynchronize(0.03, Pose2D{-1, 0, 0}));
    CHECK(seeded->pose().x == -1.0);
    CHECK(seeded->state() == ChassisState::active);
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
            REQUIRE(std::hypot(out.velocity.vx, out.velocity.vy) <= limits.max_linear_velocity + 1e-9);
            REQUIRE(std::hypot(dvx, dvy) <= limits.max_linear_acceleration * dt * 1.2 + 1e-9);
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

namespace {
// Base that applies each command after `delay` and ramps at `accel`, with wheel slip; odometry is stamped
// on receipt. Returns how often the output started moving, the odometry's back-and-forth over the last 4 s
// and its final distance to the goal.
std::tuple<int, double, double> settle(ChassisKinematics kinematics, double delay, double accel = 0.6,
                                       double slip = 0.0) {
    ChassisConfig config;
    config.kinematics = kinematics;
    config.max_linear_velocity = 0.5;
    config.max_linear_acceleration = config.max_linear_deceleration = 0.6;
    config.max_angular_acceleration = config.max_angular_deceleration = 0.65;
    config.position_tolerance = 0.01;
    config.orientation_tolerance = 0.017;
    const double dt = 0.02;
    auto ctrl = started(config, dt);
    int starts = 0;
    bool moving = false;
    std::deque<Twist2D> pipe(static_cast<std::size_t>(std::lround(delay / dt)), Twist2D{});
    const Pose2D goal{0.4, 0.0, 0.3};
    Pose2D base;
    Twist2D v;
    double now = 0.0, lo = 1e9, hi = -1e9, ylo = 1e9, yhi = -1e9;
    for (int i = 0; i < 600; ++i) {
        REQUIRE(accepted(ctrl.submit_odometry(base, now, now)));
        REQUIRE(ctrl.submit_pose(goal, now));
        now += dt;
        REQUIRE(ctrl.step(now).reason == ChassisReason::tracking);
        const Twist2D& o = ctrl.output();
        const bool now_moving = o.vx != 0.0 || o.vy != 0.0 || o.wz != 0.0;
        starts += now_moving && !moving;
        moving = now_moving;
        pipe.push_back(o);
        const Twist2D u = pipe.front();
        pipe.pop_front();
        v.vx += std::clamp(u.vx - v.vx, -accel * dt, accel * dt);
        v.vy += std::clamp(u.vy - v.vy, -accel * dt, accel * dt);
        v.wz += std::clamp(u.wz - v.wz, -0.65 * dt, 0.65 * dt);
        base = integrate_se2(base, {v.vx * (1 - slip), v.vy * (1 - slip), v.wz}, dt);
        if (i >= 400)
            lo = std::min(lo, base.x), hi = std::max(hi, base.x), ylo = std::min(ylo, base.yaw),
            yhi = std::max(yhi, base.yaw);
    }
    // differential: the part of the goal abeam of the heading is not reached
    const double c = std::cos(base.yaw), s = std::sin(base.yaw);
    const double along = (goal.x - base.x) * c + (goal.y - base.y) * s;
    const double position = kinematics == ChassisKinematics::omni
                                ? std::hypot(goal.x - base.x, goal.y - base.y)
                                : std::abs(along);
    return {starts, std::max(hi - lo, yhi - ylo),
            std::max(position, std::abs(wrap_angle(goal.yaw - base.yaw)))};
}
}  // namespace

TEST_CASE("chassis: pose mode stops once on the odometry whatever the base delay, without a delay model") {
    for (const ChassisKinematics kinematics : {ChassisKinematics::omni, ChassisKinematics::differential})
        for (double delay : {0.04, 0.1, 0.2, 0.3})
            for (double slip : {0.0, 0.02, 0.05}) {
                const auto [starts, swing, error] = settle(kinematics, delay, 0.6, slip);
                INFO("kinematics=" << static_cast<int>(kinematics) << " delay=" << delay << " slip=" << slip
                                   << " starts=" << starts << " swing=" << swing << " error=" << error);
                // one move; a slip beyond the tolerance adds a correction from rest
                CHECK(starts <= (slip > 0.03 ? 3 : 1));
                CHECK(swing < 1e-6);    // at rest: nothing moves
                CHECK(error < 0.0101);  // the odometry ends inside the tolerance
            }
}

TEST_CASE("chassis: pose mode plans on the integrated pose while the base moves") {
    auto ctrl = started({}, 0.02);
    double now = 0.0;
    for (int i = 0; i < 20; ++i) {
        REQUIRE(accepted(ctrl.submit_odometry({0, 0, 0}, now, now)));  // a base that has not moved yet
        REQUIRE(ctrl.submit_pose({1.0, 0, 0}, now));
        now += 0.02;
        ctrl.step(now);
    }
    CHECK(ctrl.pose().x > 0.01);
    CHECK(ctrl.snapshot(now).source == PoseSource::internal);
}

TEST_CASE("chassis: pose mode latches at the goal until a new goal comes") {
    ChassisConfig config;
    config.kinematics = ChassisKinematics::differential;
    config.position_tolerance = 0.01;
    config.orientation_tolerance = 0.017;
    auto ctrl = started(config, 0.02);
    double now = 0.0;
    const Pose2D goal{0.4, 0.0, 0.0};
    for (int i = 0; i < 300; ++i) {
        REQUIRE(ctrl.submit_pose(goal, now));
        now += 0.02;
        ctrl.step(now);
    }
    REQUIRE(ctrl.output().vx == 0.0);
    // the base stopped 8 mm short: inside the tolerance, so nothing moves once the odometry is folded in
    for (int i = 0; i < 100; ++i) {
        REQUIRE(accepted(ctrl.submit_odometry({0.392, 0.0, 0.005}, now, now)));
        REQUIRE(ctrl.submit_pose(goal, now));
        now += 0.02;
        const auto out = ctrl.step(now);
        CHECK(out.velocity.vx == 0.0);
        CHECK(out.velocity.wz == 0.0);
    }
    CHECK(ctrl.pose().x == doctest::Approx(0.392));
    // a base moved by someone else (remote control) is not driven back to the same goal
    for (int i = 0; i < 20; ++i) {
        REQUIRE(accepted(ctrl.submit_odometry({0.1, 0.2, 0.5}, now, now)));
        REQUIRE(ctrl.submit_pose(goal, now));
        now += 0.02;
        CHECK(ctrl.step(now).velocity.vx == 0.0);
    }
    CHECK(ctrl.pose().x == doctest::Approx(0.1));
    // a goal outside the tolerance moves it again
    bool moved = false;
    for (int i = 0; i < 10; ++i) {
        REQUIRE(ctrl.submit_pose({0.5, 0.0, 0.0}, now));
        now += 0.02;
        moved = moved || ctrl.step(now).velocity.vx > 0.0;
    }
    CHECK(moved);
}

TEST_CASE("chassis: pose mode takes a goal as done when another source keeps moving the base") {
    ChassisConfig config;
    config.kinematics = ChassisKinematics::differential;
    auto ctrl = started(config, 0.02);
    double now = 0.0;
    const Pose2D goal{0.3, 0.0, 0.0};
    Pose2D base;
    // the remote control takes over at once: kstone's output is ignored and the base backs off steadily
    for (int i = 0; i < 300; ++i) {
        REQUIRE(accepted(ctrl.submit_odometry(base, now, now)));
        REQUIRE(ctrl.submit_pose(goal, now));
        now += 0.02;
        ctrl.step(now);
        if (i < 200) base.x -= 0.002;
    }
    // released: the base stays where the remote control left it
    for (int i = 0; i < 200; ++i) {
        REQUIRE(accepted(ctrl.submit_odometry(base, now, now)));
        REQUIRE(ctrl.submit_pose(goal, now));
        now += 0.02;
        CHECK(ctrl.step(now).velocity.vx == 0.0);
    }
    CHECK(ctrl.pose().x == doctest::Approx(base.x));
}

namespace {
// A base that follows kstone's output after 0.1 s at 0.6 m/s^2, except while `remote(t)` returns a velocity
// (someone else drives it). Returns the outputs and the odometry.
struct Run {
    std::vector<Twist2D> out;
    std::vector<Pose2D> base;
    ChassisReason reason{ChassisReason::not_started};
};
template <class Remote>
Run drive(const Pose2D& goal, double seconds, Remote remote) {
    ChassisConfig config;
    config.kinematics = ChassisKinematics::differential;
    config.max_linear_velocity = 0.5;
    config.max_linear_acceleration = 0.6;
    config.max_linear_deceleration = 1.0;
    const double dt = 0.02;
    auto ctrl = started(config, dt);
    std::deque<Twist2D> pipe(5, Twist2D{});
    Pose2D base;
    Twist2D v;
    Run run;
    double now = 0.0;
    for (int i = 0; i < static_cast<int>(seconds / dt); ++i) {
        REQUIRE(accepted(ctrl.submit_odometry(base, now, now)));
        REQUIRE(ctrl.submit_pose(goal, now));
        now += dt;
        run.reason = ctrl.step(now).reason;
        pipe.push_back(ctrl.output());
        Twist2D u = pipe.front();
        pipe.pop_front();
        if (const auto r = remote(now)) u = *r;
        v.vx += std::clamp(u.vx - v.vx, -0.6 * dt, 0.6 * dt);
        v.wz += std::clamp(u.wz - v.wz, -0.65 * dt, 0.65 * dt);
        base = integrate_se2(base, v, dt);
        run.out.push_back(ctrl.output());
        run.base.push_back(base);
    }
    return run;
}
}  // namespace

TEST_CASE("chassis: pose mode yields the goal when the odometry leaves the path tolerance") {
    SUBCASE("a short push after arriving is not driven back") {
        const auto run = drive({0.3, 0, 0}, 8.0, [](double t) -> std::optional<Twist2D> {
            if (t > 3.0 && t < 3.6) return Twist2D{-0.3, 0, 0};
            if (t >= 3.6 && t < 4.2) return Twist2D{};
            return std::nullopt;
        });
        // the base came to rest where the push left it and stays there
        CHECK(run.base.back().x < 0.3 - 0.05);
        CHECK(run.reason == ChassisReason::yielded);
        for (std::size_t i = 300; i < run.out.size(); ++i) CHECK(run.out[i].vx == 0.0);
    }
    SUBCASE("a takeover in the middle of a move stops the output within a few cycles") {
        const auto run = drive({1.0, 0, 0}, 6.0, [](double t) -> std::optional<Twist2D> {
            if (t > 1.0 && t < 2.0) return Twist2D{-0.4, 0, 0};
            if (t >= 2.0 && t < 2.6) return Twist2D{};
            return std::nullopt;
        });
        // the base leaves the path tube once it is past the reference of the last kTrackLagS (about 1.5 s
        // in); from then on the output is zero and the base is not driven back to the goal
        std::size_t first_zero = run.out.size();
        for (std::size_t i = 50; i < run.out.size(); ++i)
            if (run.out[i].vx == 0.0) {
                first_zero = i;
                break;
            }
        CHECK(first_zero < 130);  // 2.6 s
        for (std::size_t i = first_zero; i < run.out.size(); ++i) CHECK(run.out[i].vx == 0.0);
        CHECK(run.base.back().x < 0.5);
    }
    SUBCASE("a base that only follows kstone is never taken for another source") {
        const auto run =
            drive({1.0, 0, 0.5}, 8.0, [](double) -> std::optional<Twist2D> { return std::nullopt; });
        CHECK(run.reason == ChassisReason::tracking);
        CHECK(std::abs(run.base.back().x * std::cos(run.base.back().yaw) +
                       run.base.back().y * std::sin(run.base.back().yaw)) > 0.5);
    }
}

TEST_CASE("chassis: lateral limits bound body y separately in velocity mode") {
    ChassisConfig config;
    config.max_linear_velocity = 1.0;
    config.max_linear_acceleration = 0.8;
    config.max_lateral_velocity = 0.5;
    config.max_lateral_acceleration = 0.4;
    auto ctrl = started(config, 0.01);
    double now = 0.0;
    REQUIRE(accepted(ctrl.submit_velocity({0, 2.0, 0}, now)));
    now += 0.01;
    CHECK(ctrl.step(now).velocity.vy == doctest::Approx(0.5));
    REQUIRE(ctrl.submit_velocity({2.0, 2.0, 0}, now));
    now += 0.01;
    const Twist2D out = ctrl.step(now).velocity;
    CHECK(std::hypot(out.vx / 1.0, out.vy / 0.5) == doctest::Approx(1.0));
    CHECK(out.vx == doctest::Approx(out.vy));  // direction kept
    config.max_lateral_acceleration = -1.0;
    CHECK_FALSE(ChassisController::create(config));
}

TEST_CASE("chassis: omni pose mode keeps body x and y within their own limits at any heading") {
    ChassisConfig config;
    config.max_linear_velocity = 1.0;
    config.max_linear_acceleration = config.max_linear_deceleration = 0.8;
    config.max_lateral_velocity = 0.5;
    config.max_lateral_acceleration = config.max_lateral_deceleration = 0.45;
    config.max_angular_velocity = 1.0;
    for (double yaw : {0.0, 0.5, 1.2, kPi / 2, 2.5, -2.0}) {
        INFO("yaw=" << yaw);
        const double dt = 0.02;
        auto created = ChassisController::create(config);
        REQUIRE(created);
        auto& ctrl = *created;
        REQUIRE(ctrl.set_scheduler_period(dt));
        REQUIRE(ctrl.start(0.0, Pose2D{0, 0, yaw}));
        const Pose2D goal{1.2, -0.7, yaw + 0.6};
        double now = 0.0;
        Twist2D previous;
        double peak_vy = 0.0;
        for (int i = 0; i < 600; ++i) {
            REQUIRE(ctrl.submit_pose(goal, now));
            now += dt;
            const auto out = ctrl.step(now);
            INFO("cycle=" << i << " error=" << ctrl.last_error());
            REQUIRE(out.reason == ChassisReason::tracking);
            const Twist2D& v = out.velocity;
            // Bounds are set at the cycle's start heading; the yaw turned within one cycle adds a little.
            REQUIRE(std::hypot(v.vx / 1.0, v.vy / 0.5) <= 1.0 + std::abs(v.wz) * dt + 1e-6);
            // Rotation also turns the body-frame velocity; allow its contribution on top of the planned rate.
            const double turn = std::abs(v.wz) * std::hypot(v.vx, v.vy) * dt;
            REQUIRE(std::abs(v.vx - previous.vx) <= 0.8 * dt * 1.05 + turn + 1e-9);
            REQUIRE(std::abs(v.vy - previous.vy) <= 0.45 * dt * 1.05 + turn + 1e-9);
            peak_vy = std::max(peak_vy, std::abs(v.vy));
            previous = v;
        }
        const Pose2D pose = ctrl.pose();
        CHECK(std::abs(pose.x - goal.x) < 1e-6);
        CHECK(std::abs(pose.y - goal.y) < 1e-6);
        CHECK(std::abs(wrap_angle(pose.yaw - goal.yaw)) < 1e-6);
        CHECK(peak_vy > 0.1);
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
        const double max_linear_velocity = ctrl.pose_limits().max_linear_velocity;
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
            CHECK(travelled > 0.5 * max_linear_velocity * 10.0 - 3.0);
            CHECK(std::hypot(ctrl.output().vx, ctrl.output().vy) > 0.5 * max_linear_velocity);
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

TEST_CASE("chassis: differential pose mode drives along its heading while it turns") {
    const ChassisConfig config = differential();
    auto ctrl = started(config, 0.02);
    const Pose2D goal{0.6, 0.0, 0.5};
    double now = 0.0;
    bool together = false;
    for (int i = 0; i < 1500; ++i) {
        REQUIRE(ctrl.submit_pose(goal, now));
        now += 0.02;
        const auto out = ctrl.step(now);
        INFO("cycle=" << i << " error=" << ctrl.last_error());
        REQUIRE(out.reason == ChassisReason::tracking);
        REQUIRE(out.velocity.vy == 0.0);
        together |= out.velocity.vx > 0.05 && out.velocity.wz > 0.05;  // no turn-then-drive phases
    }
    const Pose2D pose = ctrl.pose();
    CHECK(together);
    CHECK(std::abs(wrap_angle(pose.yaw - goal.yaw)) <= 1.5 * config.orientation_tolerance);
    // At rest the goal lies abeam: nothing is left along the heading.
    CHECK(std::abs((goal.x - pose.x) * std::cos(pose.yaw) + (goal.y - pose.y) * std::sin(pose.yaw)) < 1e-3);
    CHECK(std::abs(ctrl.output().vx) < 1e-6);
    CHECK(std::abs(ctrl.output().wz) < 1e-6);
}

TEST_CASE("chassis: differential pose mode never moves sideways") {
    auto ctrl = started(differential(), 0.02);
    double now = 0.0;
    for (int i = 0; i < 300; ++i) {
        REQUIRE(ctrl.submit_pose({0.0, 0.5, 0.0}, now));
        now += 0.02;
        const auto out = ctrl.step(now);
        REQUIRE(out.reason == ChassisReason::tracking);
        REQUIRE(out.velocity.vx == 0.0);
        REQUIRE(out.velocity.vy == 0.0);
        REQUIRE(out.velocity.wz == 0.0);
    }
    CHECK(std::hypot(ctrl.pose().x, ctrl.pose().y) < 1e-12);
}

TEST_CASE("chassis: unlimited axis does not replay another axis's brake profile") {
    // Angular limits are finite and get lowered below the current rate (Ruckig brakes); linear limits are
    // unbounded, so its update skips the brake computation and must not inherit the angular one.
    auto ctrl = started(differential(), 0.02);
    ChassisPoseLimits limits;
    limits.max_angular_velocity = 1.0;
    limits.max_angular_acceleration = 2.0;
    REQUIRE(ctrl.tune_pose_limits(limits));
    const Pose2D goal{0.0, 0.0, 2.5};  // a pure turn: nothing to drive
    double now = 0.0;
    bool lowered = false;
    for (int i = 0; i < 200; ++i) {
        REQUIRE(ctrl.submit_pose(goal, now));
        now += 0.02;
        const auto out = ctrl.step(now);
        INFO("cycle=" << i << " error=" << ctrl.last_error());
        REQUIRE(out.reason == ChassisReason::tracking);
        REQUIRE(std::abs(out.velocity.vx) < 1e-9);
        if (!lowered && out.velocity.wz > 0.9) {
            limits.max_angular_velocity = 0.3;
            REQUIRE(ctrl.tune_pose_limits(limits));
            lowered = true;
        }
    }
    CHECK(lowered);
}

TEST_CASE("chassis: SE(2) integration and angle wrapping") {
    const Pose2D pose = integrate_se2({0, 0, 0}, {1.0, 0.0, 1.0}, kPi / 2);
    CHECK(std::abs(pose.x - 1.0) < 1e-12);
    CHECK(std::abs(pose.y - 1.0) < 1e-12);
    CHECK(std::abs(wrap_angle(kPi) + kPi) < 1e-12);
}

TEST_CASE("chassis: invalid input is reported, not thrown") {
    ChassisConfig bad;
    bad.max_linear_velocity = 0.0;
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
    limits.max_linear_velocity = -1.0;
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
    const Pose2D goal{0.8, 0.0, 0.0};  // ahead: a differential base reaches it without moving sideways
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
    const double tolerance = 1.5 * ChassisConfig{}.position_tolerance;
    CHECK(base_error < tolerance);
    // The differential base settles within its arrival tolerance; the 4-DOF arm cannot cancel that
    // offset for a full 6D target, so the tool error is bounded by the base tolerance.
    CHECK((reached.translation() - target.translation()).norm() < tolerance);
    CHECK((reached.linear() - target.linear()).norm() < 1.5 * ChassisConfig{}.orientation_tolerance);

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
    CHECK(before > 1e-5);  // the arm had compensated for the offset, so it must move back
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
