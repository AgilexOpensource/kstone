#include "kstone/collision/model.hpp"
#include "kstone/collision/world.hpp"
#include "kstone/module/builtins.hpp"
#include "kstone/module/registry.hpp"
#include "kstone/module/yaml.hpp"

#include <doctest/doctest.h>

#include <filesystem>
#include <memory>
#include <string>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

ModelPtr load(const char* file) {
    auto m = Model::load(ModelSource::file((kFixtures / file).string()), {}, Context::defaults());
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

std::string message(const Status& s) { return s ? std::string{} : s.error().message; }

double number(const ConfigNode& n, std::initializer_list<const char*> path) {
    const ConfigNode* at = &n;
    for (const char* key : path) {
        at = at->find(key);
        REQUIRE_MESSAGE(at != nullptr, key);
    }
    return at->as_number();
}

void step(ModuleSet& set, Eigen::VectorXd& q, double& now, int n = 1) {
    SolveResult out;
    for (int i = 0; i < n; ++i) {
        now += 0.01;
        const Eigen::VectorXd v = out.v.size() ? out.v : Eigen::VectorXd::Zero(set.model()->nv());
        Status s = set.step(q, v, now, 0.01, out);
        REQUIRE_MESSAGE(s, message(s));
        q = out.q_next;
    }
}
}  // namespace

TEST_CASE("tune: schema patch is all-or-nothing and only touches tunable keys") {
    const auto r = ModuleRegistry::with_builtins();
    const std::string json = to_json(*r.schema("frame_task"));
    CHECK(json.find("\"tunable\": true") != std::string::npos);

    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig c;
    c.frame = "tool";
    c.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", c);
    CHECK(ft->homeable());
    CHECK_FALSE(ft->tune(ConfigNode::map({{"gain", 0.5}})));  // not configured
    REQUIRE(set.add(ft));
    REQUIRE(set.configure());

    Status ok = ft->tune(ConfigNode::map(
        {{"gain", 0.5},
         {"trajectory", ConfigNode::map({{"translation", ConfigNode::map({{"velocity", 0.2}})}})}}));
    REQUIRE_MESSAGE(ok, message(ok));
    CHECK(ft->task()->gain() == doctest::Approx(0.5));
    CHECK(number(ft->tunables(), {"trajectory", "translation", "velocity"}) == doctest::Approx(0.2));
    CHECK(number(ft->tunables(), {"trajectory", "translation", "acceleration"}) == doctest::Approx(1.0));
    const ConfigNode values = ft->tunables();
    CHECK(number(values, {"gain"}) == doctest::Approx(0.5));
    CHECK(number(values, {"trajectory", "translation", "velocity"}) == doctest::Approx(0.2));
    CHECK(values.find("frame") == nullptr);

    // Defaults in tune_schema come from the construction config, not the tuned one.
    const std::string schema = to_json(ft->tune_schema());
    CHECK(schema.find("\"frame\"") == std::string::npos);
    CHECK(schema.find("\"gain\"") != std::string::npos);
    CHECK(schema.find("\"required\"") == std::string::npos);

    const auto rejected = [&](const ConfigNode& p, ConfigDiag diag) {
        Status s = ft->tune(p);
        REQUIRE_FALSE(s);
        CHECK(s.error().message.find(to_string(diag)) != std::string::npos);
        CHECK(ft->task()->gain() == doctest::Approx(0.5));
        CHECK(number(ft->tunables(), {"trajectory", "translation", "velocity"}) == doctest::Approx(0.2));
    };
    rejected(ConfigNode::map({{"gain", 0.9}, {"frame", "base_link"}}), ConfigDiag::not_tunable);
    rejected(ConfigNode::map({{"gain", 0.9}, {"nope", 1.0}}), ConfigDiag::unknown_key);
    rejected(ConfigNode::map({{"gain", 0.9}, {"weight", -1.0}}), ConfigDiag::invalid_value);
    CHECK_FALSE(ft->tune(ConfigNode::list()));
    CHECK_FALSE(ft->tune(ConfigNode::map(
        {{"trajectory", ConfigNode::map({{"translation", ConfigNode::map({{"velocity", -1.0}})}})}})));
    CHECK(number(ft->tunables(), {"trajectory", "translation", "velocity"}) == doctest::Approx(0.2));
}

TEST_CASE("tune: frame_task and joint_target reject motion changes while homing") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.trajectory = pose_limits();
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    JointTargetModuleConfig jc;
    jc.joints = {"j1"};
    auto jt = std::make_shared<JointTargetModule>("j1", jc);
    REQUIRE(set.add(ft));
    REQUIRE(set.add(jt));
    REQUIRE(set.configure());

    Eigen::VectorXd q = m->neutral();
    double now = 0.0;
    step(set, q, now);
    REQUIRE(jt->set_target(Eigen::VectorXd::Constant(1, 0.3)));
    step(set, q, now, 300);
    REQUIRE(jt->request_home());
    step(set, q, now);
    REQUIRE(jt->home_status() == HomeStatus::executing);

    const ConfigNode slower = ConfigNode::map({{"trajectory", ConfigNode::map({{"velocity_scale", 0.5}})}});
    CHECK_FALSE(jt->tune(slower));
    CHECK_FALSE(jt->tune(ConfigNode::map({{"home_motion", ConfigNode::map({{"timeout_s", 1.0}})}})));
    Status gain = jt->tune(ConfigNode::map({{"gain", 0.7}}));
    CHECK_MESSAGE(gain, message(gain));
    CHECK(jt->task()->gain() == doctest::Approx(0.7));

    for (int i = 0; i < 2000 && jt->home_status() == HomeStatus::executing; ++i) step(set, q, now);
    REQUIRE(jt->home_status() == HomeStatus::succeeded);
    Status ok = jt->tune(slower);
    REQUIRE_MESSAGE(ok, message(ok));
    CHECK(number(jt->tunables(), {"trajectory", "velocity_scale"}) == doctest::Approx(0.5));
    const ConfigNode jt_values = jt->tunables();
    const ConfigNode* home = jt_values.find("home");
    REQUIRE(home);
    REQUIRE(home->items().size() == 1);
    CHECK(home->items()[0].as_number() == doctest::Approx(m->neutral()[0]));
    CHECK_FALSE(
        jt->tune(ConfigNode::map({{"home", ConfigNode::list({home->items()[0].as_number() + 0.1})}})));
    CHECK(jt->tune(jt_values));
    CHECK(jt->tune(ConfigNode::map({{"home_motion", ConfigNode::map({{"timeout_s", 3.0}})}})));
    CHECK_FALSE(jt->tune(ConfigNode::map({{"home_motion", ConfigNode::map({{"timeout_s", 0.0}})}})));
    CHECK_FALSE(jt->tune(ConfigNode::map({{"joints", ConfigNode::list({"j2"})}})));
    CHECK_FALSE(jt->tune(ConfigNode::map({{"tolerance", 0.0}})));
    step(set, q, now, 5);
}

TEST_CASE("tune: center_of_mass, null_space_preference and world_collision apply to their constraints") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.joints = {"j1", "j2"};
    fc.trajectory = pose_limits();
    fc.task.level = 0;
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    CenterOfMassModuleConfig cc;
    cc.frame = "base_link";
    cc.polygon = {Eigen::Vector2d(-1, -1), Eigen::Vector2d(1, -1), Eigen::Vector2d(1, 1),
                  Eigen::Vector2d(-1, 1)};
    cc.recovery_tasks = {ft};
    auto com = std::make_shared<CenterOfMassModule>("com", cc);
    NullSpacePreferenceModuleConfig nc;
    nc.primary = ft;
    auto ns = std::make_shared<NullSpacePreferenceModule>("ns", nc);
    CHECK_FALSE(com->homeable());
    REQUIRE(set.add(ft));
    REQUIRE(set.add(com));
    REQUIRE(set.add(ns));
    REQUIRE(set.configure());

    Status ok = com->tune(ConfigNode::map({{"margin", 0.2}, {"gain", 0.8}}));
    REQUIRE_MESSAGE(ok, message(ok));
    CHECK(number(com->tunables(), {"margin"}) == doctest::Approx(0.2));
    CHECK(number(com->tunables(), {"gain"}) == doctest::Approx(0.8));
    CHECK_FALSE(com->tune(ConfigNode::map({{"margin", 1.5}})));  // beyond the inscribed radius
    CHECK_FALSE(com->tune(ConfigNode::map({{"polygon", ConfigNode::list()}})));
    CHECK(number(com->tunables(), {"margin"}) == doctest::Approx(0.2));

    ok = ns->tune(ConfigNode::map({{"preference", "manipulability"}, {"posture_weight", 0.3}}));
    REQUIRE_MESSAGE(ok, message(ok));
    CHECK(ns->preference() == NullSpacePreference::manipulability);
    CHECK(ns->posture_task()->weights()[0] == doctest::Approx(0.3));
    CHECK(ns->tunables().find("preference")->as_string() == "manipulability");
    ns->set_preference(NullSpacePreference::posture);
    CHECK(ns->tunables().find("preference")->as_string() == "posture");
    CHECK_FALSE(ns->tune(ConfigNode::map({{"manipulability_rate", 1.0}})));

    Eigen::VectorXd q = m->neutral();
    double now = 0.0;
    step(set, q, now, 5);

    ModelPtr slider = load("slider_collision.urdf");
    CollisionModelOptions co;
    co.self_collision = false;
    auto geometry = CollisionModel::from_urdf(
        slider, ModelSource::file((kFixtures / "slider_collision.urdf").string()), co, Context::defaults());
    REQUIRE(geometry);
    ModuleSet cset = make_set(slider);
    WorldCollisionModuleConfig sc;
    sc.model = *geometry;
    WorldObstacle ball;
    ball.name = "ball";
    ball.shape = Shape::sphere(0.1);
    sc.obstacles = {ball};
    auto col = std::make_shared<WorldCollisionModule>("collision", sc);
    REQUIRE(cset.add(col));
    REQUIRE(cset.configure());
    ok = col->tune(ConfigNode::map({{"constraint", ConfigNode::map({{"d_min", 0.05}})}}));
    REQUIRE_MESSAGE(ok, message(ok));
    CHECK(col->constraint()->options().d_min == doctest::Approx(0.05));
    Status rows = col->tune(ConfigNode::map({{"constraint", ConfigNode::map({{"max_rows", 4}})}}));
    REQUIRE_FALSE(rows);
    CHECK(rows.error().message.find(to_string(ConfigDiag::not_tunable)) != std::string::npos);
    CollisionConstraintOptions o = col->constraint()->options();
    o.max_rows = 4;
    CHECK_FALSE(col->constraint()->set_options(o));
    CHECK(col->constraint()->options().max_rows == 16);
}

TEST_CASE("tune: null_space_preference posture reference, values and reseed") {
    ModelPtr m = load("arm.urdf");
    ModuleSet set = make_set(m);
    FrameTaskModuleConfig fc;
    fc.frame = "tool";
    fc.joints = {"j1", "j2"};
    fc.trajectory = pose_limits();
    fc.task.level = 0;
    auto ft = std::make_shared<FrameTaskModule>("tool", fc);
    NullSpacePreferenceModuleConfig nc;
    nc.primary = ft;
    nc.posture_values = {{"j2", 0.4}};
    auto ns = std::make_shared<NullSpacePreferenceModule>("ns", nc);
    REQUIRE(set.add(ft));
    REQUIRE(set.add(ns));
    Status ok = set.configure();
    REQUIRE_MESSAGE(ok, message(ok));
    const Eigen::Index i1 = m->joints()[*m->joint_id("j1")].idx_q, i2 = m->joints()[*m->joint_id("j2")].idx_q;
    const Eigen::VectorXd base = m->neutral();
    CHECK(ns->posture_task()->target()[i1] == doctest::Approx(base[i1]));
    CHECK(ns->posture_task()->target()[i2] == doctest::Approx(0.4));
    CHECK(number(ns->tunables(), {"posture_values", "j2"}) == doctest::Approx(0.4));

    ok = ns->tune(ConfigNode::map({{"posture_values", ConfigNode::map({{"j1", -0.5}})}}));
    REQUIRE_MESSAGE(ok, message(ok));
    CHECK(ns->posture_task()->target()[i1] == doctest::Approx(-0.5));
    CHECK(ns->posture_task()->target()[i2] == doctest::Approx(base[i2]));

    // Each rejection leaves the target and tunables unchanged.
    const Eigen::VectorXd kept = ns->posture_task()->target();
    CHECK_FALSE(ns->tune(ConfigNode::map({{"posture_values", ConfigNode::map({{"j3", 0.1}})}})));
    CHECK_FALSE(ns->tune(ConfigNode::map({{"posture_values", ConfigNode::map({{"j2", 2.0}})}})));
    CHECK_FALSE(ns->tune(ConfigNode::map({{"posture_values", ConfigNode::list({0.1})}})));
    CHECK_FALSE(
        ns->tune(ConfigNode::map({{"reference", "neutral"}, {"posture", ConfigNode::list({0.1, 0.2})}})));
    CHECK_FALSE(ns->tune(
        ConfigNode::map({{"reference", "current"}, {"posture_values", ConfigNode::map({{"j1", 0.1}})}})));
    CHECK_FALSE(ns->tune(ConfigNode::map({{"reference", "custom"}, {"posture", ConfigNode::list({0.1})}})));
    CHECK_FALSE(
        ns->tune(ConfigNode::map({{"reference", "custom"}, {"posture", ConfigNode::list({0.1, 1.6})}})));
    CHECK_FALSE(ns->tune(ConfigNode::map({{"reseed_current", true}, {"reference", "neutral"}})));
    CHECK(ns->posture_task()->target().isApprox(kept));
    CHECK(number(ns->tunables(), {"posture_values", "j1"}) == doctest::Approx(-0.5));

    // A reference alone drops the old values.
    ok = ns->tune(ConfigNode::map({{"reference", "custom"}, {"posture", ConfigNode::list({0.3, -0.2})}}));
    REQUIRE_MESSAGE(ok, message(ok));
    CHECK(ns->posture_task()->target()[i1] == doctest::Approx(0.3));
    CHECK(ns->posture_task()->target()[i2] == doctest::Approx(-0.2));
    CHECK(ns->tunables().find("posture_values")->items().empty());
    ok = ns->tune(ConfigNode::map({{"reference", "neutral"}}));
    REQUIRE_MESSAGE(ok, message(ok));
    CHECK(ns->posture_task()->target().isApprox(base));
    CHECK(ns->tunables().find("posture")->items().empty());

    Eigen::VectorXd q = base;
    q[i1] = 0.6;
    q[i2] = -0.7;
    double now = 0.0;
    step(set, q, now);
    const Eigen::VectorXd seen = q;
    step(set, q, now);
    ns->set_preference(NullSpacePreference::manipulability);
    ok = ns->tune(ConfigNode::map({{"reseed_current", true}}));
    REQUIRE_MESSAGE(ok, message(ok));
    CHECK(ns->preference() == NullSpacePreference::posture);
    const ConfigNode values = ns->tunables();
    CHECK(values.find("reference")->as_string() == "custom");
    CHECK_FALSE(values.find("reseed_current")->as_bool());
    REQUIRE(values.find("posture")->items().size() == 2);
    CHECK(values.find("posture")->items()[0].as_number() == doctest::Approx(seen[i1]));
    CHECK(values.find("posture")->items()[1].as_number() == doctest::Approx(seen[i2]));
    CHECK(ns->posture_task()->target()[i1] == doctest::Approx(seen[i1]));
    CHECK(ns->tune(values));
    step(set, q, now, 5);

    nc.reseed_current = true;
    ModuleSet bad = make_set(m);
    auto ft2 = std::make_shared<FrameTaskModule>("tool", fc);
    nc.primary = ft2;
    REQUIRE(bad.add(ft2));
    REQUIRE(bad.add(std::make_shared<NullSpacePreferenceModule>("ns", nc)));
    CHECK_FALSE(bad.configure());
}

TEST_CASE("tune: null_space_preference neutral clamp and custom switch seeding") {
    ModelPtr base = load("arm.urdf");
    JointLimitOverride narrow;
    narrow.joint = "j2";
    narrow.lower = 0.2;
    narrow.upper = 1.0;
    auto narrowed = base->with_limits({narrow});
    REQUIRE(narrowed);
    ModelPtr m = *narrowed;
    const Eigen::Index i1 = m->joints()[*m->joint_id("j1")].idx_q, i2 = m->joints()[*m->joint_id("j2")].idx_q;

    auto build = [&](ModuleSet& set, NullSpacePreferenceModuleConfig nc) {
        FrameTaskModuleConfig fc;
        fc.frame = "tool";
        fc.joints = {"j1", "j2"};
        fc.trajectory = pose_limits();
        fc.task.level = 0;
        auto ft = std::make_shared<FrameTaskModule>("tool", fc);
        nc.primary = ft;
        auto ns = std::make_shared<NullSpacePreferenceModule>("ns", nc);
        REQUIRE(set.add(ft));
        REQUIRE(set.add(ns));
        Eigen::VectorXd init = m->neutral();
        init[i2] = 0.5;
        REQUIRE(set.set_initial(init));
        Status ok = set.configure();
        REQUIRE_MESSAGE(ok, message(ok));
        return ns;
    };

    // Neutral follows the model limits (narrowed by yaml) and runtime joint_limits tuning.
    ModuleSet set = make_set(m);
    NullSpacePreferenceModuleConfig nc;
    nc.reference = PostureReference::neutral;
    auto ns = build(set, nc);
    CHECK(ns->posture_task()->target()[i1] == doctest::Approx(0.0));
    CHECK(ns->posture_task()->target()[i2] == doctest::Approx(0.2));
    Eigen::VectorXd q = set.initial();
    double now = 0.0;
    step(set, q, now);
    ConfigNode lower = ConfigNode::map({{"j1", 0.3}});
    REQUIRE(set.tune_joint_limits(ConfigNode::map({{"position", ConfigNode::map({{"lower", lower}})}})));
    step(set, q, now);
    CHECK(ns->posture_task()->target()[i1] == doctest::Approx(0.3));
    REQUIRE(set.tune_joint_limits(ConfigNode::map({{"reset", true}})));
    step(set, q, now);
    CHECK(ns->posture_task()->target()[i1] == doctest::Approx(0.0));

    // custom alone, nothing configured: the pre-switch q, written back as concrete values.
    q[i1] = 0.6;
    q[i2] = 0.7;
    step(set, q, now);
    const Eigen::VectorXd seen = q;
    step(set, q, now);
    Status ok = ns->tune(ConfigNode::map({{"reference", "custom"}}));
    REQUIRE_MESSAGE(ok, message(ok));
    ConfigNode values = ns->tunables();
    REQUIRE(values.find("posture")->items().size() == 2);
    CHECK(values.find("posture")->items()[0].as_number() == doctest::Approx(seen[i1]));
    CHECK(values.find("posture")->items()[1].as_number() == doctest::Approx(seen[i2]));
    CHECK(values.find("posture_values")->items().empty());

    // custom alone with configured values: those joints from yaml, the rest from the pre-switch q.
    ModuleSet set2 = make_set(m);
    nc.posture_values = {{"j2", 0.9}};
    auto ns2 = build(set2, nc);
    q = set2.initial();
    q[i1] = -0.4;
    step(set2, q, now);
    const Eigen::VectorXd seen2 = q;
    step(set2, q, now);
    ok = ns2->tune(ConfigNode::map({{"reference", "custom"}}));
    REQUIRE_MESSAGE(ok, message(ok));
    values = ns2->tunables();
    CHECK(values.find("posture")->items()[0].as_number() == doctest::Approx(seen2[i1]));
    CHECK(values.find("posture")->items()[1].as_number() == doctest::Approx(0.9));
    CHECK(ns2->posture_task()->target()[i2] == doctest::Approx(0.9));
    CHECK(ns2->tune(values));

    // A configured custom posture wins over q on every switch back to custom.
    ModuleSet set3 = make_set(m);
    nc.reference = PostureReference::custom;
    nc.posture = Eigen::Vector2d(0.1, 0.3);
    nc.posture_values.clear();
    auto ns3 = build(set3, nc);
    q = set3.initial();
    q[i1] = 1.0;
    step(set3, q, now, 2);
    REQUIRE(ns3->tune(ConfigNode::map({{"reference", "initial"}})));
    REQUIRE(ns3->tune(ConfigNode::map({{"reference", "custom"}})));
    CHECK(ns3->posture_task()->target()[i1] == doctest::Approx(0.1));
    CHECK(ns3->posture_task()->target()[i2] == doctest::Approx(0.3));
}

TEST_CASE("tune: solver options patch partially and all-or-nothing") {
    ModelPtr m = load("arm.urdf");
    SolverOptions start;
    start.damping = 2e-3;
    auto created = ModuleSet::create(m, start);
    REQUIRE(created);
    ModuleSet& set = *created;

    const ConfigNode schema = set.solver_tune_schema();
    CHECK(number(schema, {"properties", "damping", "default"}) == doctest::Approx(2e-3));
    CHECK(schema.find("properties")->find("mode") != nullptr);

    ConfigNode p = ConfigNode::map();
    p.set("limit_gain", 0.25);
    p.set("mode", "lexicographic");
    REQUIRE_MESSAGE(set.tune_solver(p), message(set.tune_solver(p)));
    CHECK(set.solver().options().limit_gain == doctest::Approx(0.25));
    CHECK(set.solver().options().mode == SolveMode::lexicographic);
    CHECK(set.solver().options().damping == doctest::Approx(2e-3));
    CHECK(number(set.solver_params(), {"limit_gain"}) == doctest::Approx(0.25));
    CHECK(set.solver_params().find("mode")->as_string() == "lexicographic");

    ConfigNode bad = ConfigNode::map();
    bad.set("damping", 0.5);
    bad.set("limit_gain", 2.0);
    CHECK_FALSE(set.tune_solver(bad));
    CHECK(set.solver().options().damping == doctest::Approx(2e-3));
    CHECK(set.solver().options().limit_gain == doctest::Approx(0.25));

    ConfigNode unknown = ConfigNode::map();
    unknown.set("velocity_limits", 1.0);
    CHECK_FALSE(set.tune_solver(unknown));
    CHECK(set.solver_params().find("velocity_limits") == nullptr);
}

TEST_CASE("tune: joint limits tighten only and all-or-nothing") {
    ModelPtr m = load("arm.urdf");
    auto created = ModuleSet::create(m, SolverOptions{});
    REQUIRE(created);
    ModuleSet& set = *created;
    const Eigen::Index v1 = m->joints()[static_cast<std::size_t>(*m->joint_id("j1"))].idx_v;
    const Eigen::Index q2 = m->joints()[static_cast<std::size_t>(*m->joint_id("j2"))].idx_q;
    auto parse = [](const char* yaml) { return *parse_yaml(yaml); };

    CHECK(set.joint_limits().find("velocity")->entries().empty());
    Status ok = set.tune_joint_limits(
        parse("{velocity: {j1: 0.5, j3: 1.0}, position: {lower: {j2: -1.0}, upper: {j2: 0.5}}}"));
    REQUIRE_MESSAGE(ok, message(ok));
    CHECK(set.solver().options().velocity_limit[v1] == doctest::Approx(0.5));
    CHECK(set.solver().options().lower_position_limit[q2] == doctest::Approx(-1.0));
    CHECK(number(set.joint_limits(), {"position", "upper", "j2"}) == doctest::Approx(0.5));
    CHECK(number(set.joint_limits(), {"velocity", "j3"}) == doctest::Approx(1.0));

    CHECK_FALSE(set.tune_joint_limits(parse("{velocity: {j1: 0.4, j2: 5.0}}")));
    CHECK(set.solver().options().velocity_limit[v1] == doctest::Approx(0.5));
    CHECK_FALSE(set.tune_joint_limits(parse("{position: {lower: {j1: -3.0}}}")));
    CHECK_FALSE(set.tune_joint_limits(parse("{position: {lower: {j2: 0.6}}}")));
    CHECK_FALSE(set.tune_joint_limits(parse("{position: {lower: {j3: 0.1}}}")));
    CHECK_FALSE(set.tune_joint_limits(parse("{velocity: {nope: 1.0}}")));
    CHECK_FALSE(set.tune_joint_limits(parse("{speed: {j1: 1.0}}")));
    Status bad_key = set.tune_joint_limits(parse("{speed: 1}"));
    REQUIRE_FALSE(bad_key);
    CHECK(bad_key.error().message.find("unknown key speed") != std::string::npos);

    REQUIRE(set.tune_joint_limits(parse("{velocity: {j1: 0}, position: {upper: {j2: 1.5}}}")));
    CHECK(set.joint_limits().find("velocity")->find("j1") == nullptr);
    CHECK(set.joint_limits().find("position")->find("upper")->find("j2") == nullptr);
    CHECK(number(set.joint_limits(), {"position", "lower", "j2"}) == doctest::Approx(-1.0));

    REQUIRE(set.tune_joint_limits(parse("{reset: true}")));
    CHECK(set.joint_limits().find("velocity")->entries().empty());
    CHECK(set.joint_limits().find("position")->find("lower")->entries().empty());
}
