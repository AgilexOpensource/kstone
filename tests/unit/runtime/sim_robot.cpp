#include "support/sim_robot.hpp"

#include "kstone/core/clock.hpp"
#include "kstone/model/model.hpp"
#include "kstone/module/builtins.hpp"
#include "kstone/module/yaml.hpp"
#include "kstone/runtime/config.hpp"
#include "kstone/runtime/robot.hpp"

#include <doctest/doctest.h>

#include <filesystem>
#include <string>

using namespace kstone;
using kstone::test::SimRobot;
using kstone::test::SimRobotOptions;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

template <typename T>
std::string message(const Result<T>& r) {
    return r ? std::string{} : r.error().message;
}

std::shared_ptr<const Model> arm() {
    auto m = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE_MESSAGE(m, message(m));
    return *m;
}

RobotCommand command_to(const Model& m, double j1) {
    RobotCommand c;
    c.q = m.neutral();
    REQUIRE(m.set_joint_position(c.q, *m.joint_id("j1"), j1));
    c.v = Eigen::VectorXd::Zero(m.nv());
    return c;
}

double j1_of(const Model& m, const RobotState& s) { return *m.joint_position(s.q, *m.joint_id("j1")); }

}  // namespace

TEST_CASE("sim robot: zero options track the command exactly") {
    auto m = arm();
    auto r = SimRobot::create(m, m->neutral());
    REQUIRE_MESSAGE(r, message(r));
    REQUIRE((*r)->write(command_to(*m, 0.5)));
    RobotState s;
    REQUIRE((*r)->read(s));
    CHECK(j1_of(*m, s) == doctest::Approx(0.5));
    const JointInfo& j1 = m->joints()[static_cast<std::size_t>(*m->joint_id("j1"))];
    CHECK(s.v[j1.idx_v] == doctest::Approx(50.0));
}

TEST_CASE("sim robot: first-order lag and latency") {
    auto m = arm();
    SimRobotOptions o;
    o.rate_hz = 100;
    o.time_constant_s = 0.05;
    o.latency_s = 0.03;
    auto r = SimRobot::create(m, m->neutral(), o);
    REQUIRE_MESSAGE(r, message(r));
    RobotState s;
    for (int i = 0; i < 3; ++i) {
        REQUIRE((*r)->write(command_to(*m, 1.0)));
        REQUIRE((*r)->read(s));
        CHECK(j1_of(*m, s) == doctest::Approx(0.0));
    }
    REQUIRE((*r)->write(command_to(*m, 1.0)));
    REQUIRE((*r)->read(s));
    CHECK(j1_of(*m, s) == doctest::Approx(1.0 - std::exp(-0.2)));
    for (int i = 0; i < 100; ++i) REQUIRE((*r)->write(command_to(*m, 1.0)));
    REQUIRE((*r)->read(s));
    CHECK(j1_of(*m, s) == doctest::Approx(1.0).epsilon(1e-6));
}

TEST_CASE("sim robot: noise is reproducible from the seed") {
    auto m = arm();
    SimRobotOptions o;
    o.position_noise = 1e-3;
    o.velocity_noise = 1e-2;
    o.seed = 7;
    auto a = SimRobot::create(m, m->neutral(), o);
    auto b = SimRobot::create(m, m->neutral(), o);
    o.seed = 8;
    auto c = SimRobot::create(m, m->neutral(), o);
    REQUIRE(a);
    REQUIRE(b);
    REQUIRE(c);
    RobotState sa, sb, sc;
    REQUIRE((*a)->read(sa));
    REQUIRE((*b)->read(sb));
    REQUIRE((*c)->read(sc));
    CHECK(sa.q.isApprox(sb.q));
    CHECK(sa.v.isApprox(sb.v));
    CHECK_FALSE(sa.q.isApprox(sc.q));
    CHECK_FALSE(sa.q.isApprox(m->neutral()));
}

TEST_CASE("sim robot: invalid options and commands") {
    auto m = arm();
    SimRobotOptions o;
    o.time_constant_s = -1;
    auto bad = SimRobot::create(m, m->neutral(), o);
    REQUIRE_FALSE(bad);
    CHECK(bad.error().message.find("time_constant_s must be finite and >= 0") != std::string::npos);
    CHECK_FALSE(SimRobot::create(m, Eigen::VectorXd::Zero(1)));
    auto r = SimRobot::create(m, m->neutral());
    REQUIRE(r);
    RobotCommand c = command_to(*m, 0.1);
    c.q[0] = std::nan("");
    CHECK_FALSE((*r)->write(c));
    c.q.resize(1);
    CHECK_FALSE((*r)->write(c));
}

TEST_CASE("sim robot: injected into a fixture engine, joint targets still converge") {
    auto m = arm();
    SimRobotOptions o;
    o.time_constant_s = 0.02;
    o.latency_s = 0.01;
    o.position_noise = 1e-5;
    auto robot = SimRobot::create(m, m->neutral(), o);
    REQUIRE_MESSAGE(robot, message(robot));
    EngineLoadOptions load;
    load.robot = *robot;
    auto e = load_engine(kFixtures / "arm_loopback.yaml", load);
    REQUIRE_MESSAGE(e, message(e));
    auto arm_module = std::dynamic_pointer_cast<JointTargetModule>(e->modules().module("arm"));
    REQUIRE(arm_module);
    Eigen::VectorXd target(3);
    target << 0.5, -0.3, 1.0;
    REQUIRE(arm_module->set_target(target));
    REQUIRE(e->run(20.0));
    const Model& em = *e->modules().model();
    CHECK(*em.joint_position(e->state().q, *em.joint_id("j1")) == doctest::Approx(0.5).epsilon(1e-3));
    CHECK(*em.joint_position(e->state().q, *em.joint_id("j2")) == doctest::Approx(-0.3).epsilon(1e-3));
}
