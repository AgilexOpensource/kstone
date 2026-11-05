#include "kstone/motion/pose_trajectory.hpp"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

using namespace kstone;

namespace {

PoseTrajectory make(const PoseTrajectoryConfig& config) {
    auto created = PoseTrajectory::create(config);
    REQUIRE(created);
    return std::move(*created);
}

Eigen::Isometry3d advance(PoseTrajectory& planner, const Eigen::Isometry3d& observed,
                          const Eigen::Isometry3d& target, double dt) {
    REQUIRE(planner.step(observed, target, dt));
    return planner.ramp_pose();
}

Eigen::Isometry3d at_x(double x) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().x() = x;
    return pose;
}

}  // namespace

// Per-axis speed bound, straight-line synchronisation, convergence; a sync change zeroes the motion state.
TEST_CASE("pose trajectory: axis modes") {
    for (PoseSync sync : {PoseSync::axis_phase, PoseSync::axis_time}) {
        CAPTURE(std::string(to_string(sync)));
        PoseTrajectoryConfig config;
        config.sync = sync;
        config.translation.velocity = 0.2;
        config.translation.acceleration = 0.5;
        config.translation.jerk = 4.0;
        config.rotation.velocity = 1.0;
        config.rotation.acceleration = 2.0;
        PoseTrajectory planner = make(config);
        const Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
        goal.translation() << 0.3, -0.1, 0.05;
        goal.linear() = Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const double dt = 0.01;
        CHECK(advance(planner, start, goal, 0.0).isApprox(start));
        Eigen::Vector3d previous = Eigen::Vector3d::Zero();
        bool bounded = true, straight = true;
        for (int cycle = 0; cycle < 400; ++cycle) {
            const Eigen::Vector3d position = advance(planner, start, goal, dt).translation();
            bounded = bounded &&
                      !((position - previous).array().abs() > config.translation.velocity * dt + 1e-7).any();
            if (sync == PoseSync::axis_phase && position.norm() > 1e-6) {
                const Eigen::Vector3d direction = goal.translation().normalized();
                straight = straight && (position - position.dot(direction) * direction).norm() <= 1e-6;
            }
            previous = position;
        }
        CHECK(bounded);
        CHECK(straight);
        CHECK((previous - goal.translation()).norm() <= 1e-6);
        CHECK(planner.stopped());
        const Eigen::AngleAxisd residual(
            Eigen::Matrix3d(planner.ramp_pose().linear().transpose() * goal.linear()));
        CHECK(std::abs(residual.angle()) <= 1e-5);

        // Mid-flight sync change: motion state zeroed, ramp pose kept.
        Eigen::Isometry3d away = goal;
        away.translation().x() = -0.3;
        for (int cycle = 0; cycle < 20; ++cycle) advance(planner, start, away, dt);
        REQUIRE_FALSE(planner.stopped());
        const Eigen::Isometry3d held = planner.ramp_pose();
        config.sync = sync == PoseSync::axis_phase ? PoseSync::axis_time : PoseSync::axis_phase;
        REQUIRE(planner.tune(config));
        CHECK(planner.stopped());
        CHECK(planner.ramp_pose().isApprox(held));
    }
}

// Zero limits: all zero passes through, acceleration 0 is constant speed, the rest fall back to acceleration.
TEST_CASE("pose trajectory: zero limits") {
    for (PoseSync sync : {PoseSync::axis_phase, PoseSync::axis_time}) {
        CAPTURE(std::string(to_string(sync)));
        const Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d goal = at_x(0.3);
        goal.linear() = Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const double dt = 0.01;
        PoseTrajectoryConfig config;
        config.sync = sync;
        PoseTrajectory passthrough = make(config);
        CHECK(advance(passthrough, start, goal, dt).isApprox(goal, 1e-12));
        CHECK(passthrough.stopped());

        config.translation.velocity = 0.2;
        PoseTrajectory constant = make(config);
        for (int cycle = 1; cycle <= 10; ++cycle) {
            const Eigen::Isometry3d pose = advance(constant, start, goal, dt);
            CHECK(pose.translation().x() == doctest::Approx(0.2 * dt * cycle).epsilon(1e-9));
            CHECK(pose.linear().isApprox(goal.linear(), 1e-12));
        }

        config.translation.acceleration = 0.5;
        PoseTrajectory trapezoid = make(config);
        double previous = 0.0, previous_speed = 0.0, peak = 0.0;
        for (int cycle = 0; cycle < 400; ++cycle) {
            const double position = advance(trapezoid, start, goal, dt).translation().x();
            const double speed = (position - previous) / dt;
            peak = std::max(peak, std::abs(speed - previous_speed) / dt);
            previous = position;
            previous_speed = speed;
        }
        CHECK(std::abs(previous - 0.3) <= 1e-6);
        CHECK(trapezoid.stopped());
        CHECK(peak <= 0.5 * 1.05 + 1e-6);
        CHECK(peak >= 0.5 * 0.5);
    }
}

// The leash holds the ramp near a blocked frame and lets it turn back from there.
TEST_CASE("pose trajectory: lead leash") {
    PoseTrajectoryConfig config;
    config.translation.velocity = 0.5;
    config.translation.acceleration = 2.0;
    config.lead_speed_scale = 2.0;
    PoseTrajectory planner = make(config);
    const double dt = 0.02;
    const Eigen::Isometry3d blocked = Eigen::Isometry3d::Identity(), far = at_x(1.0);
    Eigen::Isometry3d ramp = blocked;
    for (int cycle = 0; cycle < 200; ++cycle) ramp = advance(planner, blocked, far, dt);
    CHECK(std::abs(ramp.translation().x() - 1.0 * dt) <= 1e-9);
    bool steady = true;
    for (int cycle = 0; cycle < 100; ++cycle) {
        ramp = advance(planner, blocked, far, cycle % 2 ? 0.019 : 0.021);
        steady = steady &&
                 std::abs(ramp.translation().x() - config.lead_speed_scale * config.translation.velocity *
                                                       PoseTrajectoryConfig::lead_period) <= 1e-9;
    }
    CHECK(steady);
    ramp = advance(planner, blocked, blocked, dt);
    CHECK(ramp.translation().x() <= 1.0 * dt + 1e-12);
    for (int cycle = 0; cycle < 20; ++cycle) ramp = advance(planner, blocked, blocked, dt);
    CHECK(std::abs(ramp.translation().x()) <= 1e-6);
    config.translation.velocity = 0.25;
    REQUIRE(planner.tune(config));
    for (int cycle = 0; cycle < 100; ++cycle) ramp = advance(planner, blocked, far, dt);
    CHECK(std::abs(ramp.translation().x() - 0.01) <= 1e-9);
    config.translation.velocity = 0.0;
    REQUIRE(planner.tune(config));
    CHECK(advance(planner, blocked, far, dt).isApprox(far, 1e-12));
    PoseTrajectoryConfig bad = config;
    bad.lead_speed_scale = -1.0;
    CHECK_FALSE(PoseTrajectory::create(bad));
    CHECK_FALSE(planner.tune(bad));
}

TEST_CASE("pose trajectory: lead bound independent of loop rate") {
    PoseTrajectoryConfig config;
    config.translation.velocity = 0.5;
    config.translation.acceleration = 2.0;
    config.lead_speed_scale = 2.0;
    const double expected =
        config.lead_speed_scale * config.translation.velocity * PoseTrajectoryConfig::lead_period;
    const Eigen::Isometry3d blocked = Eigen::Isometry3d::Identity(), far = at_x(1.0);
    PoseTrajectory planner = make(config);
    for (const double dt : {0.02, 0.005, 0.001}) {
        CAPTURE(dt);
        REQUIRE(planner.reset(blocked));
        Eigen::Isometry3d ramp = blocked;
        for (int cycle = 0; cycle < 2000; ++cycle) ramp = advance(planner, blocked, far, dt);
        CHECK(std::abs(ramp.translation().x() - expected) <= 1e-9);
    }
}

// A frame pushed ahead of the ramp and past the target leaves the ramp on its own profile.
TEST_CASE("pose trajectory: leash ignores a frame ahead") {
    PoseTrajectoryConfig config;
    config.translation.velocity = 0.25;
    config.translation.acceleration = 0.15;
    config.lead_speed_scale = 3.0;
    PoseTrajectory planner = make(config);
    const double dt = 0.02;
    const Eigen::Isometry3d target = at_x(0.5);
    double previous = 0.0;
    bool below = true, bounded = true;
    for (int cycle = 0; cycle < 1000; ++cycle) {
        const Eigen::Isometry3d frame = at_x(std::min(0.8 * cycle * dt, 0.6));
        const double ramp = advance(planner, frame, target, dt).translation().x();
        below = below && ramp <= 0.5 + 1e-9;
        bounded = bounded && std::abs(ramp - previous) <= config.translation.velocity * dt * 1.001;
        previous = ramp;
    }
    CHECK(below);
    CHECK(bounded);
    CHECK(std::abs(previous - 0.5) <= 1e-9);
}

// A move a hair past a Ruckig profile-shape boundary (2 a^3 / j^2 = 0.03 m) must still arrive.
TEST_CASE("pose trajectory: profile boundary") {
    PoseTrajectoryConfig config;
    config.translation.velocity = 0.5;
    config.translation.acceleration = 1.5;
    config.translation.jerk = 15.0;
    PoseTrajectory planner = make(config);
    Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
    start.translation().z() = 0.7555084442166413;
    Eigen::Isometry3d target = start;
    target.translation().z() += 0.03;
    const double dt = 0.02;
    double previous = start.translation().z();
    bool bounded = true;
    for (int cycle = 0; cycle < 50; ++cycle) {
        const double z = advance(planner, start, target, dt).translation().z();
        bounded = bounded && z - previous <= config.translation.velocity * dt + 1e-9;
        previous = z;
    }
    CHECK(bounded);
    CHECK(std::abs(previous - target.translation().z()) <= 1e-9);
    CHECK(planner.stopped());
}

TEST_CASE("pose trajectory: rejects bad input") {
    PoseTrajectoryConfig config;
    config.translation.velocity = 0.2;
    PoseTrajectory planner = make(config);
    Eigen::Isometry3d skewed = Eigen::Isometry3d::Identity();
    skewed.linear()(0, 1) = 0.5;
    CHECK(planner.step(skewed, skewed, 0.01).error().code == ErrorCode::invalid_argument);
    CHECK_FALSE(planner.initialized());
    CHECK_FALSE(planner.step(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), -0.01));
    CHECK_FALSE(planner.reset(skewed));
    config.rotation.jerk = -1.0;
    CHECK_FALSE(PoseTrajectory::create(config));
}
