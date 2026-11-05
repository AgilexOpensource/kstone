#include "kstone/motion/joint_trajectory.hpp"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

using namespace kstone;

namespace {

JointTrajectory make(Eigen::Index size, const JointTrajectoryConfig& config, std::vector<bool> wrap = {}) {
    auto created = JointTrajectory::create(size, config, std::move(wrap));
    REQUIRE(created);
    return std::move(*created);
}

Eigen::VectorXd vec(std::initializer_list<double> values) {
    Eigen::VectorXd v(static_cast<Eigen::Index>(values.size()));
    Eigen::Index i = 0;
    for (double value : values) v[i++] = value;
    return v;
}

// Closed loop where the joint follows the reference exactly.
void follow(JointTrajectory& trajectory, Eigen::VectorXd& position, const Eigen::VectorXd& target,
            const Eigen::VectorXd& limit, double dt, int cycles) {
    for (int cycle = 0; cycle < cycles; ++cycle) {
        REQUIRE(trajectory.step(position, target, trajectory.velocity(), limit, dt));
        position = trajectory.reference();
    }
}

}  // namespace

TEST_CASE("joint trajectory: direct passes the target through") {
    JointTrajectoryConfig config;
    config.trigger = JointTrigger::direct;
    JointTrajectory trajectory = make(2, config);
    const Eigen::VectorXd target = vec({1.0, -2.0});
    REQUIRE(trajectory.step(Eigen::VectorXd::Zero(2), target, Eigen::VectorXd::Zero(2),
                            Eigen::VectorXd::Ones(2), 0.01));
    CHECK(trajectory.reference() == target);
    CHECK_FALSE(trajectory.active());
}

TEST_CASE("joint trajectory: threshold entry and exit") {
    JointTrajectoryConfig config;
    config.trigger = JointTrigger::threshold;
    config.threshold = 0.1;
    const double dt = 0.02;
    const Eigen::VectorXd limit = vec({1.0}), zero = vec({0.0}), target = vec({0.0});
    JointTrajectory trajectory = make(1, config);

    REQUIRE(trajectory.step(vec({0.1}), target, zero, limit, dt));  // tau == threshold: not engaged
    CHECK_FALSE(trajectory.active());
    CHECK(trajectory.reference()[0] == 0.0);
    REQUIRE(trajectory.step(vec({0.1001}), target, zero, limit, dt));
    CHECK(trajectory.active());
    CHECK(trajectory.reference()[0] > 0.09);  // starts from the measured state

    REQUIRE(trajectory.step(vec({0.01}), target, zero, limit, dt));  // tau == 0.5 dt: still engaged
    CHECK(trajectory.active());
    REQUIRE(trajectory.step(vec({0.0099}), target, zero, limit, dt));
    CHECK_FALSE(trajectory.active());
    CHECK(trajectory.reference()[0] == 0.0);
    CHECK(trajectory.velocity()[0] == 0.0);
}

TEST_CASE("joint trajectory: continuous converges within limits") {
    JointTrajectoryConfig config;
    config.trigger = JointTrigger::continuous;
    config.velocity_scale = 0.25;
    const double dt = 0.01;
    const Eigen::VectorXd limit = vec({1.0}), target = vec({1.0});
    JointTrajectory trajectory = make(1, config);
    Eigen::VectorXd position = vec({0.0});
    double peak = 0.0;
    for (int cycle = 0; cycle < 800; ++cycle) {
        REQUIRE(trajectory.step(position, target, trajectory.velocity(), limit, dt));
        position = trajectory.reference();
        peak = std::max(peak, std::abs(trajectory.velocity()[0]));
    }
    CHECK(peak <= 0.25 + 1e-9);
    CHECK(peak >= 0.2);
    CHECK(position[0] == doctest::Approx(1.0).epsilon(1e-6));
}

TEST_CASE("joint trajectory: velocity cap lowers the cruise") {
    JointTrajectoryConfig config;
    config.trigger = JointTrigger::continuous;
    config.velocity_cap = vec({0.1, std::numeric_limits<double>::infinity()});
    config.sync = JointSync::none;
    JointTrajectory trajectory = make(2, config);
    Eigen::VectorXd position = Eigen::VectorXd::Zero(2);
    const Eigen::VectorXd limit = Eigen::VectorXd::Ones(2), target = vec({1.0, 1.0});
    double peak0 = 0.0, peak1 = 0.0;
    for (int cycle = 0; cycle < 300; ++cycle) {
        REQUIRE(trajectory.step(position, target, trajectory.velocity(), limit, 0.01));
        position = trajectory.reference();
        peak0 = std::max(peak0, trajectory.velocity()[0]);
        peak1 = std::max(peak1, trajectory.velocity()[1]);
    }
    CHECK(peak0 <= 0.1 + 1e-9);
    CHECK(peak1 > 0.5);
}

TEST_CASE("joint trajectory: phase sync keeps the ratio, none does not") {
    const Eigen::VectorXd limit = Eigen::VectorXd::Ones(2), target = vec({1.0, 0.5});
    for (JointSync sync : {JointSync::phase, JointSync::none}) {
        CAPTURE(std::string(to_string(sync)));
        JointTrajectoryConfig config;
        config.trigger = JointTrigger::continuous;
        config.sync = sync;
        JointTrajectory trajectory = make(2, config);
        Eigen::VectorXd position = Eigen::VectorXd::Zero(2);
        double worst = 0.0;
        for (int cycle = 0; cycle < 50; ++cycle) {
            REQUIRE(trajectory.step(position, target, trajectory.velocity(), limit, 0.01));
            position = trajectory.reference();
            if (position[0] > 1e-6) worst = std::max(worst, std::abs(position[1] / position[0] - 0.5));
        }
        if (sync == JointSync::phase)
            CHECK(worst <= 1e-6);
        else
            CHECK(worst > 1e-3);
    }
}

TEST_CASE("joint trajectory: continuous joint takes the shortest arc") {
    JointTrajectoryConfig config;
    config.trigger = JointTrigger::continuous;
    JointTrajectory trajectory = make(1, config, {true});
    const double pi = 3.141592653589793;
    Eigen::VectorXd position = vec({pi - 0.2});
    const Eigen::VectorXd target = vec({-pi + 0.2}), limit = vec({1.0});
    REQUIRE(trajectory.step(position, target, vec({0.0}), limit, 0.01));
    CHECK(trajectory.reference()[0] > position[0]);  // forward across +pi, not back through 0
    follow(trajectory, position, target, limit, 0.01, 300);
    CHECK(std::remainder(position[0] - target[0], 2 * pi) == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(position[0] == doctest::Approx(pi + 0.2).epsilon(1e-6));
}

TEST_CASE("joint trajectory: configure restarts except for threshold") {
    JointTrajectoryConfig config;
    config.trigger = JointTrigger::continuous;
    JointTrajectory trajectory = make(1, config);
    Eigen::VectorXd position = vec({0.0});
    follow(trajectory, position, vec({1.0}), vec({1.0}), 0.01, 10);
    REQUIRE(trajectory.active());
    config.threshold = 0.3;
    REQUIRE(trajectory.configure(config));
    CHECK(trajectory.active());
    config.velocity_scale = 0.5;
    REQUIRE(trajectory.configure(config));
    CHECK_FALSE(trajectory.active());
}

TEST_CASE("joint trajectory: rejects bad input") {
    JointTrajectoryConfig config;
    CHECK_FALSE(JointTrajectory::create(0, config));
    CHECK_FALSE(JointTrajectory::create(2, config, {true}));
    config.threshold = -1.0;
    CHECK_FALSE(JointTrajectory::create(1, config));
    config.threshold = 0.1;
    config.velocity_scale = 0.0;
    CHECK_FALSE(JointTrajectory::create(1, config));
    config.velocity_scale = 1.5;
    CHECK_FALSE(JointTrajectory::create(1, config));
    config.velocity_scale = 1.0;
    config.velocity_cap = vec({0.0});
    CHECK_FALSE(JointTrajectory::create(1, config));
    config.velocity_cap = vec({1.0, 1.0});
    CHECK_FALSE(JointTrajectory::create(1, config));
    config.velocity_cap.resize(0);
    config.trigger = JointTrigger::continuous;
    JointTrajectory trajectory = make(1, config);
    JointTrajectoryConfig negative;
    negative.threshold = -1.0;
    CHECK_FALSE(trajectory.configure(negative));
    const Eigen::VectorXd zero = vec({0.0}), one = vec({1.0});
    CHECK(trajectory.step(zero, one, zero, zero, 0.01).error().code == ErrorCode::invalid_argument);
    CHECK_FALSE(trajectory.step(zero, one, zero, vec({std::numeric_limits<double>::infinity()}), 0.01));
    CHECK_FALSE(trajectory.step(zero, one, zero, one, 0.0));
    CHECK_FALSE(trajectory.step(zero, Eigen::VectorXd::Ones(2), zero, one, 0.01));
    CHECK_FALSE(trajectory.step(zero, vec({std::nan("")}), zero, one, 0.01));
}
