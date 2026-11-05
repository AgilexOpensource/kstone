// URDF / SRDF collision loading: ids, placements, mesh hulls and self-pair filtering.
#include "kstone/collision/model.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"

#include <doctest/doctest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

using namespace kstone;
namespace fs = std::filesystem;

namespace {

void write(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream(path) << text;
}

// A triangular prism: a hull that is no primitive.
const char* kWedge =
    "v 0 0 0\nv 0.1 0 0\nv 0 0.1 0\nv 0 0 0.1\nv 0.1 0 0.1\nv 0 0.1 0.1\n"
    "f 1 3 2\nf 4 5 6\nf 1 2 5\nf 1 5 4\nf 1 4 6\nf 1 6 3\nf 2 3 6\nf 2 6 5\n";

#ifdef KSTONE_URDF_HAS_CAPSULE
#define KSTONE_TEST_ROBOT_TAG R"(<robot name="t" version="1.1">)"
#define KSTONE_TEST_MOUNT_SHAPE R"(<capsule radius="0.02" length="0.1"/>)"
constexpr ShapeType kMountType = ShapeType::capsule;
#else
// urdfdom-headers 1.1 has neither capsule nor URDF 1.1.
#define KSTONE_TEST_ROBOT_TAG R"(<robot name="t">)"
#define KSTONE_TEST_MOUNT_SHAPE R"(<cylinder radius="0.02" length="0.1"/>)"
constexpr ShapeType kMountType = ShapeType::cylinder;
#endif

const std::string kRobot = KSTONE_TEST_ROBOT_TAG R"(
  <link name="base">
    <collision name="hull"><origin xyz="0 0.5 0"/><geometry><mesh filename="pkg://pkg/meshes/wedge.obj"/></geometry></collision>
    <collision><geometry><box size="0.1 0.2 0.3"/></geometry></collision>
  </link>
  <link name="mount">
    <collision name="dup"><origin xyz="0 0 0.1"/><geometry>)" KSTONE_TEST_MOUNT_SHAPE
                                                 R"(</geometry></collision>
  </link>
  <link name="arm"><collision name="dup"><geometry><sphere radius="0.05"/></geometry></collision></link>
  <link name="tip"><collision><geometry><cylinder radius="0.01" length="0.2"/></geometry></collision></link>
  <link name="finger"/>
  <joint name="fix" type="fixed"><parent link="base"/><child link="mount"/><origin xyz="0 0 1" rpy="0 0 1.5707963267948966"/></joint>
  <joint name="shoulder" type="revolute"><parent link="mount"/><child link="arm"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
  <joint name="wrist" type="continuous"><parent link="arm"/><child link="tip"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/></joint>
  <joint name="grip" type="prismatic"><parent link="tip"/><child link="finger"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="0.1" velocity="1" effort="1"/><mimic joint="shoulder" multiplier="0.5" offset="0.01"/></joint>
</robot>)";

struct Fixture {
    fs::path root = fs::temp_directory_path() / ("kstone-collision-urdf-" + std::to_string(getpid()));
    Context context = Context::defaults();
    ModelPtr model;
    std::string urdf;

    Fixture() {
        write(root / "pkg/meshes/wedge.obj", kWedge);
        write(root / "pkg/urdf/robot.urdf", kRobot);
        write(root / "pkg/urdf/robot.srdf",
              R"(<robot name="t"><disable_collisions link1="tip" link2="base" reason="x"/>
          <disable_collisions link1="base" link2="tip"/><disable_collisions link1="arm" link2="arm"/></robot>)");
        context.resources.add_scheme("pkg", make_directory_scheme({root}));
        urdf = (root / "pkg/urdf/robot.urdf").string();
        auto m = Model::load(ModelSource::file(urdf), ModelOptions{}, context);
        REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
        model = *m;
    }
    ~Fixture() {
        std::error_code ec;
        fs::remove_all(root, ec);
    }

    Result<CollisionModelPtr> build(const CollisionModelOptions& o) const {
        return CollisionModel::from_urdf(model, ModelSource::file(urdf), o, context);
    }
};

std::vector<std::string> pair_names(const CollisionModel& c) {
    std::vector<std::string> out;
    for (const auto& [a, b] : c.pairs())
        out.push_back(c.geometries()[static_cast<std::size_t>(a)].name + "-" +
                      c.geometries()[static_cast<std::size_t>(b)].name);
    return out;
}

}  // namespace

TEST_CASE("collision urdf: geometries, ids, placements and mesh hull") {
    Fixture f;
    auto built = f.build(CollisionModelOptions{});
    REQUIRE_MESSAGE(built, (built ? "" : built.error().message));
    const CollisionModel& c = **built;
    const auto& g = c.geometries();
    REQUIRE(g.size() == 5);
    CHECK(g[0].name == "hull");
    CHECK(g[0].shape.type == ShapeType::convex);
    CHECK(g[0].shape.vertices.cols() == 6);
    CHECK(g[0].placement.translation().isApprox(Eigen::Vector3d(0, 0.5, 0)));
    CHECK(g[1].name == "base_1");
    CHECK(g[1].shape.type == ShapeType::box);
    CHECK(g[1].shape.size.isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
    CHECK(g[2].name == "mount_0");  // duplicate collision names fall back to <link>_<i>
    CHECK(g[3].name == "arm_0");
    CHECK(g[4].name == "tip_0");
    CHECK(g[2].shape.type == kMountType);
    CHECK(g[4].shape.type == ShapeType::cylinder);
    CHECK(g[2].frame == "mount");

    // Bodies: universe {hull, base_1, mount_0}, shoulder {arm_0}, wrist {tip_0}.
    const JointId shoulder = f.model->joint_id("shoulder").value();
    const JointId wrist = f.model->joint_id("wrist").value();
    CHECK(c.joints() == std::vector<JointId>{0, 0, 0, shoulder, wrist});
    // The mount shape rides the fixed mount: placement in the universe joint is z 1.1, yaw 90 deg.
    Workspace ws(f.model);
    REQUIRE(ws.update(f.model->neutral()));
    Eigen::Isometry3d mount;
    REQUIRE(ws.frame_pose(f.model->frame_id("mount").value(), mount));
    const Eigen::Isometry3d placement = mount * g[2].placement;
    CHECK(placement.translation().isApprox(Eigen::Vector3d(0, 0, 1.1), 1e-12));
    CHECK(placement.linear()(1, 0) == doctest::Approx(1.0));

    // Only universe x wrist is not adjacent.
    CHECK(pair_names(c) == std::vector<std::string>{"hull-tip_0", "base_1-tip_0", "mount_0-tip_0"});
}

TEST_CASE("collision urdf: SRDF, groups and exclusions") {
    Fixture f;
    auto pairs = srdf_disabled_pairs(ModelSource::file((f.root / "pkg/urdf/robot.srdf").string()), f.context);
    REQUIRE(pairs);
    CHECK(*pairs == std::vector<NamePair>{{"base", "tip"}});

    CollisionModelOptions o;
    o.srdf = ModelSource::file("robot.srdf");  // relative to the URDF directory
    auto srdf = f.build(o);
    REQUIRE_MESSAGE(srdf, (srdf ? "" : srdf.error().message));
    CHECK(pair_names(**srdf) == std::vector<std::string>{"mount_0-tip_0"});

    o.srdf = ModelSource::file("missing.srdf");
    auto missing = f.build(o);
    REQUIRE_FALSE(missing);
    CHECK(missing.error().code == ErrorCode::not_found);

    o.srdf.reset();
    o.exclude_pairs = {{"nowhere", "tip"}};
    auto unknown = f.build(o);
    REQUIRE_FALSE(unknown);
    CHECK(unknown.error().code == ErrorCode::not_found);
    CHECK(unknown.error().message.find("nowhere") != std::string::npos);

    // A frame excludes its whole body: mount sits on the universe, so every pair goes.
    o.exclude_pairs = {{"mount", "tip_0"}};
    auto excluded = f.build(o);
    REQUIRE_FALSE(excluded);
    CHECK(excluded.error().code == ErrorCode::invalid_argument);

    o.exclude_pairs.clear();
    o.exclude_subtrees = {{"tip", "nowhere"}};
    auto unknown_subtree = f.build(o);
    REQUIRE_FALSE(unknown_subtree);
    CHECK(unknown_subtree.error().message.find("exclude_subtrees") != std::string::npos);
    // The wrist subtree against itself drops nothing; against the universe subtree it drops every pair.
    o.exclude_subtrees = {{"tip", "tip"}};
    auto own = f.build(o);
    REQUIRE(own);
    CHECK((*own)->pairs().size() == 3);
    o.exclude_subtrees = {{"arm", "base"}};
    CHECK_FALSE(f.build(o));

    o.exclude_subtrees.clear();
    o.groups = {BodyGroup{"arm", {}, {"wrist", "universe"}, {}}};
    CHECK_FALSE(f.build(o));
    o.groups = {BodyGroup{"arm", {"wr", "uni"}, {}, {"wrist"}}};  // wrist is excluded from the group
    auto grouped = f.build(o);
    REQUIRE(grouped);
    CHECK((*grouped)->pairs().size() == 3);
    o.groups = {BodyGroup{"typo", {"wirst"}, {}, {}}};
    auto unused = f.build(o);
    REQUIRE_FALSE(unused);
    CHECK(unused.error().message.find("'typo' matches no joint") != std::string::npos);

    o.groups.clear();
    o.self_collision = false;
    auto off = f.build(o);
    REQUIRE(off);
    CHECK((*off)->pairs().empty());
}

TEST_CASE("collision urdf: errors") {
    Fixture f;
    std::string text = kRobot;
    const auto broken = [&](const std::string& from, const std::string& to) {
        std::string out = text;
        out.replace(out.find(from), from.size(), to);
        return ModelSource::text(out, "urdf", f.root / "pkg/urdf");
    };
    CollisionModelOptions o;
    auto missing = CollisionModel::from_urdf(f.model, broken("wedge.obj", "none.obj"), o, f.context);
    REQUIRE_FALSE(missing);
    CHECK(missing.error().code == ErrorCode::not_found);

    auto bad_size =
        CollisionModel::from_urdf(f.model, broken("radius=\"0.05\"", "radius=\"-1\""), o, f.context);
    CHECK_FALSE(bad_size);

    auto not_xml =
        CollisionModel::from_urdf(f.model, ModelSource::text("<robot><link", "urdf"), o, f.context);
    REQUIRE_FALSE(not_xml);
    CHECK(not_xml.error().code == ErrorCode::parse_error);

    auto bad_srdf = srdf_disabled_pairs(
        ModelSource::text("<robot><disable_collisions link1=\"a\"/></robot>", "srdf"), f.context);
    REQUIRE_FALSE(bad_srdf);
    CHECK(bad_srdf.error().code == ErrorCode::parse_error);

    std::vector<CollisionGeometry> unknown_frame(1);
    unknown_frame[0].name = "x";
    unknown_frame[0].frame = "nope";
    unknown_frame[0].shape = Shape::sphere(0.1);
    auto frame = CollisionModel::create(f.model, unknown_frame, o, f.context);
    REQUIRE_FALSE(frame);
    CHECK(frame.error().code == ErrorCode::not_found);
}

TEST_CASE("collision urdf: pairs penetrating at the initial posture are dropped with a warning") {
    // c folds back onto a at q = 0 (jm keeps them non-adjacent); base and c stay apart.
    const char* chain = R"(<robot name="fold">
  <link name="base"><collision><origin xyz="-1 0 0"/><geometry><sphere radius="0.05"/></geometry></collision></link>
  <link name="a"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <link name="m"/>
  <link name="b"/>
  <link name="c"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="a"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" velocity="1" effort="1"/></joint>
  <joint name="jm" type="revolute"><parent link="a"/><child link="m"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
  <joint name="j2" type="revolute"><parent link="m"/><child link="b"/><origin xyz="0.3 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.2" upper="3.2" velocity="1" effort="1"/></joint>
  <joint name="j3" type="fixed"><parent link="b"/><child link="c"/><origin xyz="-0.15 0 0"/></joint>
</robot>)";
    Context context = Context::defaults();
    std::vector<std::string> logged;
    context.log = Logger(make_callback_sink([&](const LogRecord& r) { logged.emplace_back(r.message); }),
                         LogLevel::warn);
    auto model = Model::load(ModelSource::text(chain, "urdf"), ModelOptions{}, context);
    REQUIRE_MESSAGE(model, (model ? "" : model.error().message));
    const auto build = [&](const CollisionModelOptions& o) {
        return CollisionModel::from_urdf(*model, ModelSource::text(chain, "urdf"), o, context);
    };

    CollisionModelOptions o;
    auto dropped = build(o);
    REQUIRE_MESSAGE(dropped, (dropped ? "" : dropped.error().message));
    CHECK(pair_names(**dropped) == std::vector<std::string>{"base_0-c_0"});
    REQUIRE(logged.size() == 1);
    CHECK(logged[0].find("'a_0' / 'c_0'") != std::string::npos);
    CHECK(logged[0].find("-0.05") != std::string::npos);

    logged.clear();
    o.initial = Eigen::VectorXd::Zero(3);
    o.initial[2] = 3.14;  // c swings away from a
    auto moved = build(o);
    REQUIRE(moved);
    CHECK((*moved)->pairs().size() == 2);
    CHECK(logged.empty());

    o.initial.resize(1);
    auto bad = build(o);
    REQUIRE_FALSE(bad);
    CHECK(bad.error().code == ErrorCode::invalid_argument);

    o.initial.resize(0);
    o.exclude_initial_collisions = false;
    auto kept = build(o);
    REQUIRE(kept);
    CHECK((*kept)->pairs().size() == 2);
    CHECK(logged.empty());
}
