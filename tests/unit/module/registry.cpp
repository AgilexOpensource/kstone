#include "kstone/module/registry.hpp"

#include "kstone/module/builtins.hpp"

#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

ModelPtr load(const char* file) {
    auto m = Model::load(ModelSource::file((kFixtures / file).string()), {}, Context::defaults());
    REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
    return *m;
}

ConfigNode limits(double velocity) {
    return ConfigNode::map({{"velocity", velocity}, {"acceleration", 1.0}, {"jerk", 10.0}});
}

ConfigNode frame_task_params() {
    ConfigNode p;
    p.set("frame", "tool");
    p.set("gain", 0.5);
    ConfigNode t;
    t.set("translation", limits(0.3));
    t.set("rotation", limits(1.0));
    p.set("trajectory", t);
    return p;
}

std::string message(const Status& s) { return s ? std::string{} : s.error().message; }

template <typename T>
std::string message(const Result<T>& r) {
    return r ? std::string{} : r.error().message;
}

struct PoseHolder {
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    std::optional<double> scale;
};
}  // namespace

TEST_CASE("registry: builtin types and schemas") {
    const ModuleRegistry r = ModuleRegistry::with_builtins();
    const std::vector<std::string> expected{"center_of_mass", "chassis",        "frame_task",
                                            "gaze",           "joint_target",   "null_space_preference",
                                            "self_collision", "world_collision"};
    CHECK(r.types() == expected);
    for (const auto& type : expected) {
        auto s = r.schema(type);
        REQUIRE(s);
        CHECK(s->find("properties") != nullptr);
        REQUIRE(s->find("additionalProperties") != nullptr);
        CHECK_FALSE(s->find("additionalProperties")->as_bool());
    }
    CHECK(r.schema("nope").error().code == ErrorCode::not_found);

    const std::string json = to_json(*r.schema("frame_task"));
    CHECK(json.find("\"required\"") != std::string::npos);
    CHECK(json.find("\"frame\"") != std::string::npos);
    CHECK(json.find("\"maximum\": 1") != std::string::npos);
    CHECK(json.find("\"axis_phase\"") != std::string::npos);
    const std::string gaze = to_json(*r.schema("gaze"));
    CHECK(gaze.find("\"frames\"") != std::string::npos);
    CHECK(gaze.find("\"x-module-type\"") == std::string::npos);
}

TEST_CASE("registry: duplicate and empty types are rejected") {
    ModuleRegistry r = ModuleRegistry::with_builtins();
    CHECK_FALSE(r.add<FrameTaskModule>("frame_task", ParamSchema<FrameTaskModuleConfig>{}));
    CHECK_FALSE(r.add<FrameTaskModule>("", ParamSchema<FrameTaskModuleConfig>{}));
}

TEST_CASE("registry: builds modules that configure and step") {
    ModelPtr m = load("arm.urdf");
    const ModuleRegistry r = ModuleRegistry::with_builtins();
    auto set = ModuleSet::create(m);
    REQUIRE(set);
    BuildContext build{m, Context::defaults(), {}, {}, {}};
    build.refs.find = [&](const std::string& id) { return set->module(id); };

    auto tool = r.create("frame_task", "tool", frame_task_params(), build, "modules.tool");
    REQUIRE_MESSAGE(tool, message(tool));
    CHECK(std::string((*tool)->type()) == "frame_task");
    REQUIRE(set->add(*tool));

    ConfigNode ns;
    ns.set("primary", "tool");
    ns.set("preference", "manipulability");
    CHECK(r.references("null_space_preference", ns)->at(0).id == "tool");
    auto pref = r.create("null_space_preference", "ns", ns, build);
    REQUIRE_MESSAGE(pref, message(pref));
    REQUIRE(set->add(*pref));

    ConfigNode fingers;
    fingers.set("joints", ConfigNode::list({"finger_left_joint", "j1"}));
    auto jt = r.create("joint_target", "fingers", fingers, build);
    REQUIRE_MESSAGE(jt, message(jt));
    REQUIRE(set->add(*jt));

    REQUIRE(set->configure());
    SolveResult out;
    Eigen::VectorXd q = m->neutral();
    const Eigen::VectorXd v = Eigen::VectorXd::Zero(m->nv());
    Status s = set->step(q, v, 0.01, 0.01, out);
    CHECK_MESSAGE(s, message(s));
}

TEST_CASE("registry: errors carry the path") {
    ModelPtr m = load("arm.urdf");
    const ModuleRegistry r = ModuleRegistry::with_builtins();
    BuildContext build{m, Context::defaults(), {}, {}, {}};

    auto check = [&](const std::string& type, const ConfigNode& p, ErrorCode code, const std::string& part) {
        auto res = r.create(type, "x", p, build, "modules.x");
        REQUIRE_FALSE(res);
        CHECK(res.error().code == code);
        CHECK_MESSAGE(res.error().message.find(part) != std::string::npos, res.error().message);
    };

    ConfigNode p = frame_task_params();
    p.set("gain", 1.5);
    check("frame_task", p, ErrorCode::invalid_argument, "modules.x.gain: must be <= 1");

    p = frame_task_params();
    p.set("weight", 1e300);
    check("frame_task", p, ErrorCode::invalid_argument, "modules.x.weight: must be <= 1e+06");

    p = frame_task_params();
    p.set("position_weight", ConfigNode::list({1.0, 1e154, 1.0}));
    check("frame_task", p, ErrorCode::invalid_argument, "modules.x.position_weight[1]: must be <= 1e+06");

    p = frame_task_params();
    p.set("gain", "fast");
    check("frame_task", p, ErrorCode::invalid_argument, "modules.x.gain: expected a number, got 'fast'");
    p.set("gain", "0.5");
    check("frame_task", p, ErrorCode::invalid_argument,
          "got '0.5' (quoted scalars stay strings; remove the quotes)");

    p = frame_task_params();
    p.set("gian", 0.5);
    check("frame_task", p, ErrorCode::invalid_argument, "modules.x.gian: unknown key");

    p = ConfigNode::map({{"frame", "tool"}});
    check("frame_task", p, ErrorCode::invalid_argument, "modules.x.trajectory: required key is missing");

    p = frame_task_params();
    ConfigNode t;
    t.set("translation", limits(-1.0));
    t.set("rotation", limits(1.0));
    p.set("trajectory", t);
    check("frame_task", p, ErrorCode::invalid_argument, "modules.x.trajectory.translation.velocity");

    p = frame_task_params();
    p.set("trajectory", ConfigNode::map({{"sync", "sideways"}}));
    check("frame_task", p, ErrorCode::invalid_argument, "modules.x.trajectory.sync");

    ConfigNode g;
    g.set("frame", "base_link");
    g.set("polygon", ConfigNode::list({ConfigNode::list({1.0, 1.0}), ConfigNode::list({-1.0, 1.0}),
                                       ConfigNode::list({0.0, -1.0})}));
    g.set("recovery_tasks", ConfigNode::list({"missing"}));
    check("center_of_mass", g, ErrorCode::not_found, "modules.x.recovery_tasks[0]: unknown module 'missing'");

    check("nope", ConfigNode::map(), ErrorCode::not_found, "nope");
    CHECK_FALSE(r.create("frame_task", "", frame_task_params(), build));
}

TEST_CASE("registry: references must have the required type") {
    ModelPtr m = load("arm.urdf");
    const ModuleRegistry r = ModuleRegistry::with_builtins();
    auto jt = std::make_shared<JointTargetModule>("fingers", JointTargetModuleConfig{});
    BuildContext build{m, Context::defaults(), {}, {}, {}};
    build.refs.find = [&](const std::string& id) -> std::shared_ptr<Module> {
        return id == "fingers" ? jt : nullptr;
    };
    auto res = r.create("null_space_preference", "ns", ConfigNode::map({{"primary", "fingers"}}), build);
    REQUIRE_FALSE(res);
    CHECK(res.error().message.find("primary") != std::string::npos);
    CHECK(res.error().message.find("frame_task") != std::string::npos);

    auto refs = r.references("center_of_mass",
                             ConfigNode::map({{"recovery_tasks", ConfigNode::list({"a", "b"})}}), "m");
    REQUIRE(refs);
    REQUIRE(refs->size() == 2);
    CHECK(refs->at(1).id == "b");
    CHECK(refs->at(1).type == "frame_task");
    CHECK(refs->at(1).path == "m.recovery_tasks[1]");
}

TEST_CASE("registry: self_collision builds from the URDF") {
    ModelPtr m = load("slider_collision.urdf");
    const ModuleRegistry r = ModuleRegistry::with_builtins();
    BuildContext build{m, Context::defaults(), {}, {}, {}};
    ConfigNode p;
    p.set("urdf", (kFixtures / "slider_collision.urdf").string());
    // Only adjacent links: no self pairs survive filtering.
    p.set("self_collision", false);
    p.set("constraint", ConfigNode::map({{"d_min", 0.02}, {"max_rows", 4}}));
    auto res = r.create("self_collision", "collision", p, build);
    REQUIRE_MESSAGE(res, message(res));
    CHECK(std::string((*res)->type()) == "self_collision");

    p.set("urdf", (kFixtures / "missing.urdf").string());
    CHECK_FALSE(r.create("self_collision", "collision", p, build));
    p.set("urdf", (kFixtures / "slider_collision.urdf").string());
    p.set("constraint", ConfigNode::map({{"max_rows", 2.5}}));
    auto bad = r.create("self_collision", "collision", p, build);
    REQUIRE_FALSE(bad);
    CHECK(bad.error().message.find("constraint.max_rows") != std::string::npos);
}

TEST_CASE("registry: world_collision builds obstacles from the config") {
    ModelPtr m = load("slider_collision.urdf");
    const ModuleRegistry r = ModuleRegistry::with_builtins();
    BuildContext build{m, Context::defaults(), {}, {}, {}};
    ConfigNode ball = ConfigNode::map(
        {{"name", "ball"},
         {"shape", ConfigNode::map({{"type", "sphere"}, {"size", ConfigNode::list({0.1, 0.0, 0.0})}})},
         {"pose", ConfigNode::map({{"translation", ConfigNode::list({0.5, 0.0, 0.0})}})},
         {"enabled", false}});
    ConfigNode p;
    p.set("urdf", (kFixtures / "slider_collision.urdf").string());
    p.set("obstacles", ConfigNode::list({ball}));
    auto res = r.create("world_collision", "world", p, build);
    REQUIRE_MESSAGE(res, message(res));
    CHECK(std::string((*res)->type()) == "world_collision");
    auto set = ModuleSet::create(m);
    REQUIRE(set);
    REQUIRE(set->add(*res));
    REQUIRE(set->configure());
    auto& world = static_cast<WorldCollisionModule&>(**res);
    REQUIRE(world.world()->size() == 1);
    CHECK(world.world()->pose(0).translation().x() == doctest::Approx(0.5));
    CHECK_FALSE(world.world()->enabled(0));
    CHECK(world.ports()[1].name == "ball/enabled");

    p.set(
        "obstacles",
        ConfigNode::list({ConfigNode::map(
            {{"name", "ball"},
             {"shape", ConfigNode::map({{"type", "mesh"}, {"size", ConfigNode::list({0.1, 0.0, 0.0})}})}})}));
    auto bad = r.create("world_collision", "world", p, build);
    REQUIRE_FALSE(bad);
    CHECK(bad.error().message.find("obstacles[0].shape.type") != std::string::npos);
}

TEST_CASE("schema: isometry from quaternion or rpy") {
    ParamSchema<PoseHolder> s;
    s.field("pose", &PoseHolder::pose);
    s.field("scale", &PoseHolder::scale).above(0);

    PoseHolder h;
    ConfigNode rpy = ConfigNode::map({{"translation", ConfigNode::list({1.0, 2.0, 3.0})},
                                      {"rpy", ConfigNode::list({0.0, 0.0, M_PI / 2})}});
    REQUIRE(s.read(ConfigNode::map({{"pose", rpy}}), h));
    CHECK(h.pose.translation().y() == doctest::Approx(2.0));
    CHECK((h.pose.linear() * Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitY()).norm() < 1e-12);
    CHECK_FALSE(h.scale);

    const double c = std::sqrt(0.5);
    ConfigNode quat = ConfigNode::map({{"quaternion", ConfigNode::list({c, 0.0, 0.0, c})}});
    PoseHolder h2;
    REQUIRE(s.read(ConfigNode::map({{"pose", quat}, {"scale", 2.0}}), h2));
    CHECK(h2.pose.isApprox(Eigen::Isometry3d(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()))));
    CHECK(*h2.scale == 2.0);

    ConfigNode both = quat;
    both.set("rpy", ConfigNode::list({0.0, 0.0, 0.0}));
    CHECK_FALSE(s.read(ConfigNode::map({{"pose", both}}), h));
    ConfigNode skew = ConfigNode::map({{"quaternion", ConfigNode::list({1.0, 1.0, 0.0, 0.0})}});
    CHECK_FALSE(s.read(ConfigNode::map({{"pose", skew}}), h));
    CHECK_FALSE(s.read(ConfigNode::map({{"scale", -1.0}}), h));
    CHECK_FALSE(s.read(ConfigNode::map({{"scale", std::nan("")}}), h));
    CHECK_FALSE(s.read(ConfigNode::list(), h));
}
