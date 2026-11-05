#include "kstone/runtime/robot_plugin.hpp"

#include "kstone/module/builtins.hpp"
#include "kstone/module/yaml.hpp"
#include "kstone/runtime/config.hpp"

#include <doctest/doctest.h>

#include <filesystem>
#include <string>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

template <typename T>
std::string message(const Result<T>& r) {
    return r ? std::string{} : r.error().message;
}

ConfigNode plugin_config(const std::string& library, ConfigNode params = {}) {
    auto doc = load_yaml(kFixtures / "arm_loopback.yaml");
    REQUIRE(doc);
    ConfigNode robot = ConfigNode::map({{"type", "plugin"}, {"library", library}});
    if (!params.is_null()) robot.set("params", std::move(params));
    ConfigNode cfg = *doc;
    cfg.set("robot", std::move(robot));
    return cfg;
}

}  // namespace

TEST_CASE("robot plugin: example driver runs the fixture to convergence") {
    auto e = build_engine(plugin_config(KSTONE_EXAMPLE_ROBOT), kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    auto arm = std::dynamic_pointer_cast<JointTargetModule>(e->modules().module("arm"));
    REQUIRE(arm);
    Eigen::VectorXd target(3);
    target << 0.5, -0.3, 1.0;
    REQUIRE(arm->set_target(target));
    REQUIRE(e->run(20.0));
    const Model& m = *e->modules().model();
    CHECK(*m.joint_position(e->state().q, *m.joint_id("j1")) == doctest::Approx(0.5).epsilon(1e-3));
    CHECK(*m.joint_position(e->state().q, *m.joint_id("j2")) == doctest::Approx(-0.3).epsilon(1e-3));
}

TEST_CASE("robot plugin: params, rate and joints reach the driver") {
    auto e = build_engine(plugin_config(KSTONE_FLAKY_ROBOT, ConfigNode::map({{"port", "can0"}})), kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    REQUIRE(e->run(0.02));
    CHECK(e->stats().cycles == 2);
}

TEST_CASE("robot plugin: driver errors keep their code and message") {
    auto e =
        build_engine(plugin_config(KSTONE_FLAKY_ROBOT, ConfigNode::map({{"fail_create", true}})), kFixtures);
    REQUIRE_FALSE(e);
    CHECK(e.error().code == ErrorCode::io_error);
    CHECK_MESSAGE(
        e.error().message.find("robot.library: robot plugin 'flaky': create failed: no robot on can0 "
                               "[KST-E303]") != std::string::npos,
        e.error().message);

    e = build_engine(plugin_config(KSTONE_FLAKY_ROBOT, ConfigNode::map({{"fail_read", true}})), kFixtures);
    REQUIRE_FALSE(e);
    CHECK(e.error().code == ErrorCode::io_error);
    CHECK_MESSAGE(e.error().message == "robot plugin 'flaky': read failed: encoder timeout",
                  e.error().message);

    e = build_engine(plugin_config(KSTONE_FLAKY_ROBOT, ConfigNode::map({{"fail_write", true}})), kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    Status s = e->step(0.01);
    REQUIRE_FALSE(s);
    CHECK(s.error().code == ErrorCode::internal);
    CHECK(s.error().message == "robot plugin 'flaky': write failed: bus off");
}

TEST_CASE("robot plugin: write sees the fresh flag and stale commands are still written") {
    // flaky leaves stamps unknown, so its feedback is stale unless the timeout is 0.
    auto e =
        build_engine(plugin_config(KSTONE_FLAKY_ROBOT, ConfigNode::map({{"reject_stale", true}})), kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    Status s = e->step(0.01);
    REQUIRE_FALSE(s);
    CHECK_FALSE(e->fresh());
    CHECK(s.error().message == "robot plugin 'flaky': write failed: stale command");

    ConfigNode cfg = plugin_config(KSTONE_FLAKY_ROBOT, ConfigNode::map({{"reject_stale", true}}));
    cfg.set("feedback_timeout_s", 0.0);
    e = build_engine(cfg, kFixtures);
    REQUIRE_MESSAGE(e, message(e));
    s = e->step(0.01);
    CHECK_MESSAGE(s, message(s));
    CHECK(e->fresh());
}

TEST_CASE("robot plugin: loading errors") {
    auto e = build_engine(plugin_config(KSTONE_FUTURE_ROBOT), kFixtures);
    REQUIRE_FALSE(e);
    CHECK(e.error().code == ErrorCode::unsupported);
    CHECK_MESSAGE(e.error().message.find("ABI version 99, expected 1") != std::string::npos,
                  e.error().message);

    e = build_engine(plugin_config("./missing_robot.so"), kFixtures);
    REQUIRE_FALSE(e);
    CHECK(e.error().code == ErrorCode::not_found);
    CHECK_MESSAGE(e.error().message.find((kFixtures / "missing_robot.so").string()) != std::string::npos,
                  e.error().message);

    // An empty base_dir (config loaded by a relative name) must not turn ./libm.so.6 into a search-path name.
    auto rel = parse_yaml("model: {file: '" + (kFixtures / "arm.urdf").string() +
                              "'}\nrobot: {type: plugin, library: ./libm.so.6}\n",
                          "t.yaml");
    REQUIRE(rel);
    e = build_engine(*rel, "");
    REQUIRE_FALSE(e);
    CHECK(e.error().code == ErrorCode::not_found);
    CHECK_MESSAGE(
        e.error().message.find((std::filesystem::current_path() / "libm.so.6").string()) != std::string::npos,
        e.error().message);

    auto doc = parse_yaml("model: {file: arm.urdf}\nrobot: {type: plugin}\n", "t.yaml");
    REQUIRE(doc);
    e = build_engine(*doc, kFixtures);
    REQUIRE_FALSE(e);
    CHECK_MESSAGE(
        e.error().message.find("t.yaml:2:15: robot.library: required for type plugin [KST-E103]") == 0,
        e.error().message);

    doc =
        parse_yaml("model: {file: arm.urdf}\nrobot: {type: plugin, library: x.so, params: [1]}\n", "t.yaml");
    REQUIRE(doc);
    e = build_engine(*doc, kFixtures);
    REQUIRE_FALSE(e);
    CHECK_MESSAGE(
        e.error().message.find("robot.params: expected a map, got list [KST-E101]") != std::string::npos,
        e.error().message);
}

TEST_CASE("robot plugin: direct load checks the initial size") {
    auto model = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE(model);
    RobotPluginOptions options;
    options.library = KSTONE_EXAMPLE_ROBOT;
    options.initial = Eigen::VectorXd::Zero(2);
    auto r = load_robot_plugin(*model, options);
    REQUIRE_FALSE(r);
    CHECK(r.error().code == ErrorCode::invalid_argument);

    options.initial = (*model)->neutral();
    options.initial[0] = 0.25;
    r = load_robot_plugin(*model, options);
    REQUIRE_MESSAGE(r, message(r));
    RobotState state;
    REQUIRE((*r)->read(state));
    CHECK(state.q[0] == 0.25);
    CHECK(state.v.size() == (*model)->nv());
    RobotCommand bad;
    CHECK_FALSE((*r)->write(bad));
}
