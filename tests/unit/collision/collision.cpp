// Collision distances, Jacobians and the collision constraint on the slider fixture
// (slider sphere r 0.1 on prismatic "slide" along x, base sphere r 0.1 at x 0.5).
#include "kstone/collision/constraint.hpp"
#include "kstone/control/solver.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"

#include <Eigen/Geometry>
#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;
constexpr double kInf = std::numeric_limits<double>::infinity();

ModelPtr load(const char* file) {
    auto m = Model::load(ModelSource::file((kFixtures / file).string()), ModelOptions{}, Context::defaults());
    REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
    return *m;
}

CollisionModelPtr robot_geometry(const ModelPtr& m, const char* file, bool self = false) {
    CollisionModelOptions o;
    o.self_collision = self;
    auto c =
        CollisionModel::from_urdf(m, ModelSource::file((kFixtures / file).string()), o, Context::defaults());
    REQUIRE_MESSAGE(c, (c ? "" : c.error().message));
    return *c;
}

Eigen::Isometry3d at(double x, double y = 0.0, double z = 0.0) {
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation() = Eigen::Vector3d(x, y, z);
    return p;
}

Eigen::VectorXd slider_q(double x) { return Eigen::VectorXd::Constant(1, x); }

struct Rows {
    Eigen::MatrixXd A;
    Eigen::VectorXd lo, hi;
};

Rows evaluate(Constraint& c, Workspace& ws, double dt) {
    Rows r{Eigen::MatrixXd::Zero(c.rows(), ws.model()->nv()), Eigen::VectorXd::Constant(c.rows(), -kInf),
           Eigen::VectorXd::Constant(c.rows(), kInf)};
    Tick tick{ws, dt};
    Status s = c.evaluate(tick, r.A, r.lo, r.hi);
    REQUIRE_MESSAGE(s, (s ? "" : s.error().message));
    return r;
}
}  // namespace

TEST_CASE("collision geometry: shapes validate and meshes become hulls") {
    CHECK(validate(Shape::sphere(0.1)));
    CHECK_FALSE(validate(Shape::sphere(0.0)));
    CHECK_FALSE(validate(Shape::box(0.1, -1.0, 0.1)));
    CHECK(validate(Shape::capsule(0.1, 0.0)));
    CHECK_FALSE(validate(Shape::cylinder(0.1, std::nan(""))));
    Eigen::Matrix3Xd flat(3, 3);
    flat.setRandom();
    CHECK_FALSE(validate(Shape::convex(flat)));

    // A cube with an interior point: the hull keeps the 8 corners.
    Eigen::Matrix3Xd cube(3, 9);
    for (int i = 0; i < 8; ++i) cube.col(i) = Eigen::Vector3d(i & 1, (i >> 1) & 1, (i >> 2) & 1) * 0.1;
    cube.col(8) = Eigen::Vector3d::Constant(0.05);
    CollisionWorld world;
    auto id = world.add("cube", Shape::convex(cube), at(1.0));
    REQUIRE(id);
    CHECK(world.size() == 1);
    CHECK(world.find("cube").value() == *id);
    CHECK_FALSE(world.add("cube", Shape::sphere(0.1), at(0.0)));
    CHECK_FALSE(world.add("bad", Shape::convex(flat), at(0.0)));
    CHECK(world.find("missing").error().code == ErrorCode::not_found);
}

TEST_CASE("collision world: revisions and ids") {
    CollisionWorld world;
    const auto r0 = world.revision();
    auto a = world.add("a", Shape::sphere(0.1), at(0.0));
    auto b = world.add("b", Shape::box(0.1, 0.1, 0.1), at(1.0));
    REQUIRE(a);
    REQUIRE(b);
    CHECK(*a != *b);
    CHECK(world.revision() == r0 + 2);
    CHECK(world.set_pose(*a, at(2.0)));
    CHECK(world.revision() == r0 + 2);
    Eigen::Isometry3d nan_pose = at(std::nan(""));
    CHECK(world.set_pose(*a, nan_pose).error().code == ErrorCode::invalid_argument);
    CHECK(world.remove(*a));
    CHECK(world.remove(*a).error().code == ErrorCode::not_found);
    auto c = world.add("a", Shape::sphere(0.1), at(0.0));
    REQUIRE(c);
    CHECK(*c != *a);  // ids are never reused
    world.clear();
    CHECK(world.size() == 0);
}

TEST_CASE("collision model: adjacent self pairs are filtered") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelOptions o;
    auto self = CollisionModel::from_urdf(
        m, ModelSource::file((kFixtures / "slider_collision.urdf").string()), o, Context::defaults());
    // base and slider are parent / child: nothing is left to check.
    REQUIRE_FALSE(self);
    CHECK(self.error().code == ErrorCode::invalid_argument);

    CollisionModelPtr c = robot_geometry(m, "slider_collision.urdf");
    REQUIRE(c->geometries().size() == 2);
    CHECK(c->geometries()[0].frame == "base");
    CHECK(c->geometries()[1].frame == "slider");
    CHECK(c->joints()[0] == 0);
    CHECK(c->joints()[1] == m->joint_id("slide").value());
    CHECK(c->pairs().empty());
    CHECK(c->geometry_index(c->geometries()[1].name).value() == 1);
}

TEST_CASE("collision query: slider against an obstacle sphere") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelPtr c = robot_geometry(m, "slider_collision.urdf");
    auto world = std::make_shared<CollisionWorld>();
    REQUIRE(world->add("ball", Shape::sphere(0.1), at(0.5)));
    CollisionQuery query(c, world);
    // The base geometry is welded to the world: only slider x ball is checked.
    REQUIRE(query.pairs().size() == 1);
    CHECK(query.pairs()[0].first == 1);
    CHECK(query.pairs()[0].obstacle);
    CHECK(query.pair_name(0) == c->geometries()[1].name + "/ball");

    Workspace ws(m);
    REQUIRE(ws.update(slider_q(0.26)));
    REQUIRE(query.update(ws, 1.0));
    REQUIRE(query.proximities().size() == 1);
    const Proximity& p = query.proximities()[0];
    CHECK(p.distance == doctest::Approx(0.04).epsilon(1e-9));
    CHECK(p.normal.isApprox(Eigen::Vector3d::UnitX(), 1e-9));
    CHECK(p.point1.isApprox(Eigen::Vector3d(0.36, 0, 0), 1e-9));
    CHECK(p.point2.isApprox(Eigen::Vector3d(0.40, 0, 0), 1e-9));
    Eigen::RowVectorXd J(1);
    REQUIRE(query.jacobian(ws, p, J));
    CHECK(J[0] == doctest::Approx(-1.0));  // moving +x closes the gap

    // Outside the threshold: no proximity, but the nearest is still exact.
    REQUIRE(query.update(ws, 0.01));
    CHECK(query.proximities().empty());
    CHECK(query.nearest().distance == doctest::Approx(0.04).epsilon(1e-9));

    // Penetration gives a negative signed distance.
    REQUIRE(ws.update(slider_q(0.45)));
    REQUIRE(query.update(ws, 1.0));
    CHECK(query.nearest().distance == doctest::Approx(-0.15).epsilon(1e-6));

    // Changing the world rebuilds the pairs.
    const auto layout = query.layout();
    REQUIRE(world->add("ball2", Shape::sphere(0.1), at(-0.5)));
    REQUIRE(query.update(ws, 1.0));
    CHECK(query.layout() != layout);
    CHECK(query.pairs().size() == 2);
}

TEST_CASE("collision query: repeated queries follow obstacle and threshold changes") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelPtr c = robot_geometry(m, "slider_collision.urdf");
    auto world = std::make_shared<CollisionWorld>();
    auto ball = world->add("ball", Shape::sphere(0.1), at(0.5));
    REQUIRE(ball);
    CollisionQuery query(c, world);
    Workspace ws(m);
    REQUIRE(ws.update(slider_q(0.26)));

    // A narrow inexact query must not stand in for a wider or exact one at the same poses.
    REQUIRE(query.update(ws, 0.01, false));
    CHECK(query.proximities().empty());
    CHECK(query.nearest().pair == -1);
    REQUIRE(query.update(ws, 1.0, false));
    REQUIRE(query.proximities().size() == 1);
    CHECK(query.nearest().distance == doctest::Approx(0.04).epsilon(1e-9));
    REQUIRE(query.update(ws, 0.01, false));
    CHECK(query.proximities().empty());
    CHECK(query.nearest().pair == -1);
    REQUIRE(query.update(ws, 0.01));
    CHECK(query.nearest().distance == doctest::Approx(0.04).epsilon(1e-9));

    // set_pose and set_enabled do not bump the revision but still change the result.
    REQUIRE(world->set_pose(*ball, at(0.6)));
    REQUIRE(query.update(ws, 1.0));
    CHECK(query.nearest().distance == doctest::Approx(0.14).epsilon(1e-9));
    REQUIRE(world->set_enabled(*ball, false));
    REQUIRE(query.update(ws, 1.0));
    CHECK(query.proximities().empty());
    REQUIRE(world->set_enabled(*ball, true));
    REQUIRE(ws.update(slider_q(0.3)));
    REQUIRE(query.update(ws, 1.0));
    CHECK(query.nearest().distance == doctest::Approx(0.10).epsilon(1e-9));
}

TEST_CASE("collision query: round-off pose jitter reuses the cache, real motion does not") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelPtr c = robot_geometry(m, "slider_collision.urdf");
    auto world = std::make_shared<CollisionWorld>();
    REQUIRE(world->add("ball", Shape::sphere(0.1), at(0.5)));
    CollisionQuery query(c, world);
    Workspace ws(m);
    REQUIRE(ws.update(slider_q(0.26)));
    REQUIRE(query.update(ws, 1.0));
    const double d0 = query.nearest().distance;

    // Jitter stays within the tolerance of the cached pose, so the cached distance comes back bit-for-bit.
    for (double dx : {1e-13, -1e-13, 5e-13}) {
        REQUIRE(ws.update(slider_q(0.26 + dx)));
        REQUIRE(query.update(ws, 1.0));
        CHECK(query.nearest().distance == d0);
    }
    REQUIRE(ws.update(slider_q(0.26 + 1e-9)));
    REQUIRE(query.update(ws, 1.0));
    CHECK(std::abs(query.nearest().distance - d0) > 1e-10);
}

TEST_CASE("collision query: jacobian matches finite differences on the arm") {
    ModelPtr m = load("arm.urdf");
    CollisionModelOptions o;
    o.self_collision = false;
    std::vector<CollisionGeometry> geometries;
    for (const char* frame : {"link2", "link3", "link4", "tool"}) {
        CollisionGeometry g;
        g.name = std::string(frame) + "_box";
        g.frame = frame;
        g.placement = at(0.02, -0.01, 0.03);
        g.placement.linear() =
            Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
        g.shape = Shape::box(0.05, 0.04, 0.08);
        geometries.push_back(g);
    }
    auto c = CollisionModel::create(m, geometries, o, Context::defaults());
    REQUIRE_MESSAGE(c, (c ? "" : c.error().message));
    auto world = std::make_shared<CollisionWorld>();
    REQUIRE(world->add("post", Shape::capsule(0.05, 0.5), at(0.3, 0.2, 0.4)));
    CollisionQuery query(*c, world);

    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    Workspace ws(m), ws2(m);
    const double h = 1e-5;
    int checked = 0;
    for (int trial = 0; trial < 20; ++trial) {
        Eigen::VectorXd dq(m->nv());
        for (Index i = 0; i < m->nv(); ++i) dq[i] = 0.8 * u(rng);
        Eigen::VectorXd q(m->nq());
        REQUIRE(m->integrate(m->neutral(), dq, q));
        Eigen::VectorXd v(m->nv());
        for (Index i = 0; i < m->nv(); ++i) v[i] = u(rng);
        REQUIRE(ws.update(q));
        REQUIRE(query.update(ws, 10.0));
        std::vector<Proximity> base = query.proximities();
        std::vector<double> rates(query.pairs().size());
        for (const Proximity& p : base) {
            Eigen::RowVectorXd J(m->nv());
            REQUIRE(query.jacobian(ws, p, J));
            rates[static_cast<std::size_t>(p.pair)] = J.dot(v);
        }
        Eigen::VectorXd q2(m->nq());
        REQUIRE(m->integrate(q, v * h, q2));
        REQUIRE(ws2.update(q2));
        // Cold queries on both sides: a warm-started GJK converges to a slightly different point
        // (within its ~1e-7 tolerance), which a finite difference would amplify.
        CollisionQuery cold1(*c, world), cold2(*c, world);
        REQUIRE(cold1.update(ws, 10.0));
        REQUIRE(cold2.update(ws2, 10.0));
        for (const Proximity& p2 : cold2.proximities()) {
            for (const Proximity& p : cold1.proximities()) {
                if (p.pair != p2.pair || p.distance < 1e-3) continue;  // skip near-contact (non-smooth)
                CHECK((p2.distance - p.distance) / h ==
                      doctest::Approx(rates[static_cast<std::size_t>(p.pair)]).epsilon(1e-3).scale(1.0));
                ++checked;
            }
        }
    }
    CHECK(checked > 40);
}

TEST_CASE("collision constraint: rows, hysteresis and dropped candidates") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelPtr c = robot_geometry(m, "slider_collision.urdf");
    auto world = std::make_shared<CollisionWorld>();
    REQUIRE(world->add("ball", Shape::sphere(0.1), at(0.5)));

    CollisionConstraintOptions bad;
    bad.max_rows = 0;
    CHECK_FALSE(CollisionConstraint::create(c, world, bad));
    bad = {};
    bad.gain = -1.0;
    CHECK_FALSE(CollisionConstraint::create(c, world, bad));
    bad = {};
    bad.max_cuts = -1;
    CHECK_FALSE(CollisionConstraint::create(c, world, bad));

    CollisionConstraintOptions o;  // d_min .03, hysteresis .005
    o.gain = 1.0;
    o.activation_margin = 0.01;
    o.approach_deceleration = 0.0;
    o.max_rows = 2;
    o.max_cuts = 0;
    auto created = CollisionConstraint::create(c, world, o);
    REQUIRE(created);
    std::shared_ptr<CollisionConstraint> con = *created;
    REQUIRE(con->bind(m));
    CHECK(con->rows() == 2);
    Workspace ws(m);
    const double dt = 0.01;

    // d = 0.043: outside the activation band and not yet active.
    REQUIRE(ws.update(slider_q(0.257)));
    Rows r = evaluate(*con, ws, dt);
    CHECK(con->report().candidates == 0);
    CHECK(r.A.isZero());
    CHECK(std::isinf(r.lo[0]));
    CHECK(con->report().nearest.distance == doctest::Approx(0.043).epsilon(1e-9));

    // d = 0.04 -> active: J v >= -gain (d - d_min) = -0.01.
    REQUIRE(ws.update(slider_q(0.26)));
    r = evaluate(*con, ws, dt);
    CHECK(con->report().candidates == 1);
    CHECK(con->report().rows == 1);
    CHECK(r.A(0, 0) == doctest::Approx(-1.0));
    CHECK(r.lo[0] == doctest::Approx(-0.01).epsilon(1e-9));
    CHECK(std::isinf(r.hi[0]));
    CHECK(r.A.row(1).isZero());
    CHECK(std::isinf(r.lo[1]));

    // d = 0.043 again: kept by hysteresis.
    REQUIRE(ws.update(slider_q(0.257)));
    r = evaluate(*con, ws, dt);
    CHECK(con->report().rows == 1);
    CHECK(r.lo[0] == doctest::Approx(-0.013).epsilon(1e-9));
    // d = 0.047: released.
    REQUIRE(ws.update(slider_q(0.253)));
    r = evaluate(*con, ws, dt);
    CHECK(con->report().rows == 0);

    // Braking: rate = max(gain m, min(sqrt(2 a m), 0.5 m / dt)).
    o.approach_deceleration = 2.0;
    auto braking = CollisionConstraint::create(c, world, o);
    REQUIRE(braking);
    REQUIRE((*braking)->bind(m));
    REQUIRE(ws.update(slider_q(0.26)));
    r = evaluate(**braking, ws, dt);
    CHECK(r.lo[0] == doctest::Approx(-std::min(std::sqrt(2.0 * 2.0 * 0.01), 0.5 * 0.01 / dt)));

    // Four obstacles in the band, two rows: the two nearest win and two are dropped.
    REQUIRE(world->add("b2", Shape::sphere(0.1), at(0.022, 0, 0)));      // d = 0.038, behind
    REQUIRE(world->add("b3", Shape::sphere(0.1), at(0.26, 0.235, 0)));   // d = 0.035
    REQUIRE(world->add("b4", Shape::sphere(0.1), at(0.26, -0.239, 0)));  // d = 0.039
    REQUIRE(ws.update(slider_q(0.26)));
    r = evaluate(*con, ws, dt);
    CHECK(con->report().candidates == 4);
    CHECK(con->report().rows == 2);
    CHECK(con->report().dropped == 2);
    CHECK(con->report().nearest.distance == doctest::Approx(0.035).epsilon(1e-9));
    CHECK(r.lo[0] == doctest::Approx(-0.005).epsilon(1e-9));
    CHECK(r.lo[1] == doctest::Approx(-0.008).epsilon(1e-9));
    CHECK(r.A(0, 0) == doctest::Approx(0.0).scale(1.0));  // b3 is beside the slider: no x component
    CHECK(r.A(1, 0) == doctest::Approx(1.0));             // b2 is behind: moving +x opens the gap
}

TEST_CASE("collision constraint: closed loop keeps d_min from a moving obstacle") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelPtr c = robot_geometry(m, "slider_collision.urdf");
    auto world = std::make_shared<CollisionWorld>();
    auto ball = world->add("ball", Shape::sphere(0.1), at(0.9));
    REQUIRE(ball);
    CollisionConstraintOptions o;
    o.gain = 50.0;
    o.approach_deceleration = 5.0;
    auto con = CollisionConstraint::create(c, world, o);
    REQUIRE(con);

    auto solver = Solver::create(m);
    REQUIRE(solver);
    REQUIRE(solver->add_task("joint",
                             std::make_shared<JointTask>(std::vector<std::string>{"slide"}, slider_q(0.6))));
    REQUIRE(solver->add_constraint("collision", *con));

    const double dt = 0.01;
    Eigen::VectorXd q = slider_q(-0.2);
    SolveResult result;
    double min_distance = kInf;
    for (int step = 0; step < 400; ++step) {
        // The ball sweeps in from x 0.9 to x 0.3 and back.
        const double t = step * dt;
        const double x = 0.6 + 0.3 * std::cos(t * 1.5);
        REQUIRE(world->set_pose(*ball, at(x)));
        Status s = solver->solve(q, dt, result);
        REQUIRE_MESSAGE(s, (s ? "" : s.error().message) << " at step " << step);
        q = result.q_next;
        const double d = (x - q[0]) - 0.2;
        min_distance = std::min(min_distance, d);
    }
    // The constraint is linearized at the start of the step while the ball also moves within it,
    // so allow the ball's own displacement per step (<= 0.45 * dt).
    CHECK(min_distance >= o.d_min - 0.0045 - 1e-9);
    CHECK(min_distance < o.d_min + o.activation_margin);  // the constraint was actually engaged
    MESSAGE("closed loop min distance " << min_distance);
}

TEST_CASE("collision constraint: obstacle velocity feed-forward keeps a fast obstacle out") {
    // The ball closes at 1 m/s; gain alone lets it sink by about speed / gain.
    auto run = [](bool feed) {
        ModelPtr m = load("slider_collision.urdf");
        CollisionModelPtr c = robot_geometry(m, "slider_collision.urdf");
        auto world = std::make_shared<CollisionWorld>();
        auto ball = world->add("ball", Shape::sphere(0.1), at(0.9));
        REQUIRE(ball);
        CollisionConstraintOptions o;
        o.gain = 20.0;
        auto con = CollisionConstraint::create(c, world, o);
        REQUIRE(con);
        auto solver = Solver::create(m);
        REQUIRE(solver);
        REQUIRE(solver->add_task(
            "joint", std::make_shared<JointTask>(std::vector<std::string>{"slide"}, slider_q(0.0))));
        REQUIRE(solver->add_constraint("collision", *con));
        const double dt = 0.01, speed = 1.0;
        Eigen::VectorXd q = slider_q(0.0);
        SolveResult result;
        double min_margin = kInf;
        for (int step = 0; step < 50; ++step) {
            const double x = 0.6 - speed * step * dt;
            REQUIRE(world->set_pose(*ball, at(x)));
            if (feed)
                REQUIRE(world->set_velocity(*ball, Eigen::Vector3d(-speed, 0, 0), Eigen::Vector3d::Zero()));
            REQUIRE(solver->solve(q, dt, result));
            q = result.q_next;
            min_margin = std::min(min_margin, (x - speed * dt - q[0]) - 0.2 - o.d_min);
        }
        return min_margin;
    };
    const double without = run(false), with = run(true);
    MESSAGE("min margin without " << without << ", with " << with);
    CHECK(without < -0.02);
    CHECK(with > -1e-3);
}

TEST_CASE("collision world: obstacle velocity") {
    CollisionWorld world;
    auto id = world.add("box", Shape::box(0.1, 0.1, 0.1), at(1.0));
    REQUIRE(id);
    CHECK(world.linear_velocity(0).isZero());
    REQUIRE(world.set_velocity(*id, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 2)));
    CHECK(world.point_velocity(0, Eigen::Vector3d(1.0, 0.5, 0)).isApprox(Eigen::Vector3d(0, 0, 0)));
    CHECK_FALSE(world.set_velocity(*id, Eigen::Vector3d(kInf, 0, 0), Eigen::Vector3d::Zero()));
    CHECK_FALSE(world.set_velocity(*id + 100, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
    REQUIRE(world.set_pose(*id, at(2.0)));
    CHECK(world.linear_velocity(0).isApprox(Eigen::Vector3d(1, 0, 0)));
}

TEST_CASE("collision constraint: a push-out beyond the velocity limit still recovers") {
    ModelPtr m = load("slider_collision.urdf");
    CollisionModelPtr c = robot_geometry(m, "slider_collision.urdf");
    auto world = std::make_shared<CollisionWorld>();
    REQUIRE(world->add("ball", Shape::sphere(0.1), at(0.3)));
    CollisionConstraintOptions o;
    o.gain = 100.0;  // starting 1 cm inside demands 4 m/s, twice the slide velocity limit
    o.exempt_penetrating = false;
    auto con = CollisionConstraint::create(c, world, o);
    REQUIRE(con);

    auto solver = Solver::create(m);
    REQUIRE(solver);
    // The task pulls into the ball.
    REQUIRE(solver->add_task("joint",
                             std::make_shared<JointTask>(std::vector<std::string>{"slide"}, slider_q(0.3))));
    REQUIRE(solver->add_constraint("collision", *con));

    const double dt = 0.01;
    Eigen::VectorXd q = slider_q(0.11);
    SolveResult result;
    REQUIRE(solver->solve(q, dt, result));
    CHECK(result.held);                 // only the scaled recovery was feasible
    CHECK(result.v[0] < -1.0);          // and it pushes out instead of holding still
    CHECK((*con)->report().cuts == 0);  // a relaxed step cannot meet a cut either: no re-solves
    for (int step = 0; step < 50; ++step) {
        q = result.q_next;
        Status s = solver->solve(q, dt, result);
        REQUIRE_MESSAGE(s, (s ? "" : s.error().message) << " at step " << step);
    }
    const double d = (0.3 - result.q_next[0]) - 0.2;
    CHECK(d == doctest::Approx(o.d_min).epsilon(1e-3));
    CHECK_FALSE(result.held);
}

namespace {
// Slider driven toward `goal` by a joint task with the ball at x 0.3; returns the final slide position.
struct Slide {
    std::shared_ptr<CollisionWorld> world = std::make_shared<CollisionWorld>();
    std::shared_ptr<CollisionConstraint> con;
    std::shared_ptr<JointTask> task;
    std::unique_ptr<Solver> solver;
    ObstacleId ball{};
    Eigen::VectorXd q;
    Slide(double x) {
        ModelPtr m = load("slider_collision.urdf");
        ball = *world->add("ball", Shape::sphere(0.1), at(0.3));
        con = *CollisionConstraint::create(robot_geometry(m, "slider_collision.urdf"), world, {});
        task = std::make_shared<JointTask>(std::vector<std::string>{"slide"}, slider_q(x));
        solver = std::make_unique<Solver>(std::move(*Solver::create(m)));
        REQUIRE(solver->add_task("joint", task));
        REQUIRE(solver->add_constraint("collision", con));
        q = slider_q(x);
    }
    double run(double goal, int steps = 200) {
        REQUIRE(task->set_target(slider_q(goal)));
        SolveResult r;
        for (int i = 0; i < steps; ++i) {
            REQUIRE(solver->solve(q, 0.01, r));
            q = r.q_next;
        }
        return q[0];
    }
};
}  // namespace

TEST_CASE("collision constraint: a pair penetrating on the first step is free until clear") {
    Slide s(0.15);  // 5 cm inside the ball
    const double out = s.run(0.0);
    CHECK(s.con->report().exempt == 0);
    CHECK(out == doctest::Approx(0.0).epsilon(1e-3));
    // Clear again, so moving back in stops at d_min.
    const double back = s.run(0.3);
    CHECK((0.3 - back) - 0.2 == doctest::Approx(s.con->options().d_min).epsilon(1e-2));
}

TEST_CASE("collision constraint: an exempt pair is not pushed out") {
    Slide s(0.15);
    const double held = s.run(0.15, 20);
    CHECK(s.con->report().exempt == 1);
    CHECK(s.con->report().rows == 0);
    CHECK(held == doctest::Approx(0.15));
}

TEST_CASE("collision constraint: enabling an obstacle around the body exempts it") {
    Slide s(0.0);
    REQUIRE(s.world->set_enabled(s.ball, false));
    s.run(0.15);
    REQUIRE(s.world->set_enabled(s.ball, true));
    CHECK(s.run(0.15, 5) == doctest::Approx(0.15));
    CHECK(s.con->report().exempt == 1);
    // Tracked from outside: approaching stops at d_min, nothing is exempt.
    s.run(0.0);
    s.run(0.3);
    CHECK(s.con->report().exempt == 0);
    CHECK((0.3 - s.q[0]) - 0.2 == doctest::Approx(s.con->options().d_min).epsilon(1e-2));
    // reset_tracking with the pair inside d_min but not penetrating: still constrained.
    s.con->reset_tracking();
    s.run(0.3, 5);
    CHECK(s.con->report().exempt == 0);
}

TEST_CASE("collision constraint: exempt_penetrating false pushes a first-step penetration out") {
    Slide s(0.15);
    CollisionConstraintOptions o = s.con->options();
    o.exempt_penetrating = false;
    REQUIRE(s.con->set_options(o));
    s.run(0.15);
    CHECK(s.con->report().exempt == 0);
    CHECK((0.3 - s.q[0]) - 0.2 == doctest::Approx(o.d_min).epsilon(1e-2));
}

namespace {
// Arm with a box on link4 and one on the tool; `world` holds a box obstacle beside the reach.
CollisionModelPtr arm_boxes(const ModelPtr& m) {
    CollisionModelOptions o;
    o.self_collision = false;
    std::vector<CollisionGeometry> geometries;
    for (const char* frame : {"link4", "tool"}) {
        CollisionGeometry g;
        g.name = std::string(frame) + "_box";
        g.frame = frame;
        g.shape = Shape::box(0.06, 0.08, 0.04);
        geometries.push_back(g);
    }
    auto c = CollisionModel::create(m, geometries, o, Context::defaults());
    REQUIRE_MESSAGE(c, (c ? "" : c.error().message));
    return *c;
}
}  // namespace

TEST_CASE("collision query: pruned updates match an unpruned one") {
    ModelPtr m = load("arm.urdf");
    CollisionModelPtr c = arm_boxes(m);
    auto world = std::make_shared<CollisionWorld>();
    REQUIRE(world->add("post", Shape::capsule(0.05, 0.5), at(0.3, 0.2, 0.4)));
    REQUIRE(world->add("block", Shape::box(0.1, 0.1, 0.1), at(0.55, -0.15, 0.3)));
    REQUIRE(world->add("far", Shape::sphere(0.05), at(-2.0, 0.0, 0.0)));

    std::mt19937 rng(11);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    Workspace ws(m);
    for (int trial = 0; trial < 30; ++trial) {
        Eigen::VectorXd dq(m->nv());
        for (Index i = 0; i < m->nv(); ++i) dq[i] = 0.8 * u(rng);
        Eigen::VectorXd q(m->nq());
        REQUIRE(m->integrate(m->neutral(), dq, q));
        REQUIRE(ws.update(q));
        CollisionQuery all(c, world);
        REQUIRE(all.update(ws, 1e3));
        REQUIRE(all.proximities().size() == all.pairs().size());
        for (const double threshold : {0.0, 0.02, 0.1, 0.3}) {
            CollisionQuery pruned(c, world);
            REQUIRE(pruned.update(ws, threshold));
            CHECK(pruned.nearest().pair == all.nearest().pair);
            CHECK(pruned.nearest().distance ==
                  doctest::Approx(all.nearest().distance).epsilon(1e-9).scale(1e-9));
            std::vector<Index> expected, got;
            for (const Proximity& p : all.proximities())
                if (p.distance <= threshold) expected.push_back(p.pair);
            for (const Proximity& p : pruned.proximities()) got.push_back(p.pair);
            std::sort(expected.begin(), expected.end());
            std::sort(got.begin(), got.end());
            CHECK(got == expected);
        }
    }
}

TEST_CASE("collision query: a reused query along a path matches fresh ones") {
    ModelPtr m = load("arm.urdf");
    CollisionModelPtr c = arm_boxes(m);
    auto world = std::make_shared<CollisionWorld>();
    REQUIRE(world->add("post", Shape::capsule(0.05, 0.5), at(0.3, 0.2, 0.4)));
    auto block = world->add("block", Shape::box(0.1, 0.1, 0.1), at(0.55, -0.15, 0.3));
    REQUIRE(block);

    std::mt19937 rng(5);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    Workspace ws(m);
    CollisionQuery reused(c, world);
    const auto same = [](double a, double b) { return a == b || std::abs(a - b) < 1e-9; };
    int finite = 0, close = 0;
    Eigen::VectorXd offset = Eigen::VectorXd::Zero(m->nv()), q(m->nq());
    for (int step = 0; step < 400; ++step) {
        // Mostly small steps (where the motion bound prunes), with occasional jumps.
        const double scale = step % 37 == 0 ? 0.8 : 0.03;
        for (Index i = 0; i < m->nv(); ++i) offset[i] = std::clamp(offset[i] + scale * u(rng), -0.8, 0.8);
        REQUIRE(m->integrate(m->neutral(), offset, q));
        REQUIRE(ws.update(q));
        Eigen::Isometry3d pose = at(0.55 + 0.1 * std::sin(0.05 * step), -0.15, 0.3);
        pose.rotate(Eigen::AngleAxisd(0.07 * step, Eigen::Vector3d::UnitZ()));
        REQUIRE(world->set_pose(*block, pose));
        REQUIRE(world->set_enabled(*block, step % 50 < 40));

        const double threshold = std::array<double, 3>{0.0, 0.2, 0.5}[static_cast<std::size_t>(step % 3)];
        const bool exact = step % 2 == 0;
        CollisionQuery fresh(c, world);
        REQUIRE(fresh.update(ws, threshold, exact));
        REQUIRE(reused.update(ws, threshold, exact));
        INFO("step " << step << " exact " << exact << " threshold " << threshold);
        CHECK(reused.nearest().pair == fresh.nearest().pair);
        CHECK(same(reused.nearest().distance, fresh.nearest().distance));
        if (std::isfinite(fresh.nearest().distance)) ++finite;
        if (!fresh.proximities().empty()) ++close;
        REQUIRE(reused.proximities().size() == fresh.proximities().size());
        for (std::size_t k = 0; k < fresh.proximities().size(); ++k) {
            CHECK(reused.proximities()[k].pair == fresh.proximities()[k].pair);
            CHECK(same(reused.proximities()[k].distance, fresh.proximities()[k].distance));
        }
    }
    CHECK(finite > 100);
    CHECK(close > 20);
}

TEST_CASE("collision constraint: cutting planes keep a rotating box at d_min") {
    ModelPtr m = load("arm.urdf");
    CollisionModelPtr c = arm_boxes(m);
    const auto run = [&](Index max_cuts, Index& cut_steps, Eigen::Vector3d bp, Eigen::Vector4d t1,
                         Eigen::Vector4d t2) {
        auto world = std::make_shared<CollisionWorld>();
        REQUIRE(world->add("block", Shape::box(0.2, 0.2, 0.2), at(bp.x(), bp.y(), bp.z())));
        CollisionConstraintOptions o;
        o.d_min = 0.002;
        o.gain = 1.5;
        o.activation_margin = 0.01;
        o.approach_deceleration = 20.0;
        o.max_cuts = max_cuts;
        if (max_cuts == 0) o.clip_slack = 10.0;  // isolate the first-order rows: no endpoint clip
        auto con = CollisionConstraint::create(c, world, o);
        REQUIRE(con);
        auto solver = Solver::create(m);
        REQUIRE(solver);
        auto task = std::make_shared<JointTask>(std::vector<std::string>{"j1", "j2", "j3", "j4"}, t1);
        REQUIRE(solver->add_task("joint", task));
        REQUIRE(solver->add_constraint("collision", *con));
        Eigen::VectorXd q = m->neutral();
        SolveResult result;
        double lowest = kInf;
        cut_steps = 0;
        for (int step = 0; step < 300; ++step) {
            if (step == 150) REQUIRE(task->set_target(t2));
            Status s = solver->solve(q, 0.01, result);
            REQUIRE_MESSAGE(s, (s ? "" : s.error().message) << " at step " << step);
            q = result.q_next;
            if ((*con)->report().cuts > 0) ++cut_steps;
            Workspace ws(m);
            REQUIRE(ws.update(q));
            CollisionQuery query(c, world);
            REQUIRE(query.update(ws, 0.0));
            if (step > 20) lowest = std::min(lowest, query.nearest().distance);
        }
        return lowest;
    };
    // Swing the tool box onto the block's face, then roll the wrist: the face-to-face contact is where the
    // first-order distance model is optimistic.
    const Eigen::Vector3d block(0.6, 0.25, 0.3);
    const Eigen::Vector4d swing(1.0, 0.0, 0.0, 0.0), roll(1.0, 0.0, 1.5, 0.0);
    Index plain_cuts = 0, cut_steps = 0;
    const double plain = run(0, plain_cuts, block, swing, roll);
    const double cut = run(8, cut_steps, block, swing, roll);
    MESSAGE("lowest without cuts " << plain << ", with " << cut << " (" << cut_steps << " steps cut)");
    CHECK(plain_cuts == 0);
    CHECK(plain < 0.0);
    CHECK(cut_steps > 0);
    CHECK(cut >= 0.002 - 1e-5);
}

namespace {
struct ClipRun {
    int violations = 0, clipped = 0, frozen = 0, degraded_clips = 0;
    double worst = 0.0;
};

// Random joint targets around two random blocks with a one-cut budget; counts ends below each pair's floor.
ClipRun clip_run(const ModelPtr& m, const CollisionModelPtr& c, double spread, double clip_slack,
                 double floor_slack) {
    CollisionConstraintOptions o;
    o.d_min = 0.002;
    o.gain = 1.5;
    o.activation_margin = 0.01;
    o.approach_deceleration = 20.0;
    o.max_cuts = 1;
    o.clip_slack = clip_slack;
    o.exempt_penetrating = false;
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    ClipRun r;
    for (int trial = 0; trial < 60; ++trial) {
        auto world = std::make_shared<CollisionWorld>();
        for (const char* name : {"a", "b"}) {
            // Draw order fixed explicitly: argument evaluation order is unspecified and differs across targets.
            const double z = 0.3 + 0.2 * u(rng);
            const double y = spread * 0.3 * u(rng);
            const double x = spread * (0.45 + 0.15 * u(rng));
            REQUIRE(world->add(name, Shape::box(0.2, 0.2, 0.2), at(x, y, z)));
        }
        auto con = CollisionConstraint::create(c, world, o);
        REQUIRE(con);
        auto solver = Solver::create(m);
        REQUIRE(solver);
        Eigen::VectorXd t = Eigen::VectorXd::Zero(4);
        auto task = std::make_shared<JointTask>(std::vector<std::string>{"j1", "j2", "j3", "j4"}, t);
        REQUIRE(task->set_gain(0.5));
        REQUIRE(solver->add_task("joint", task));
        REQUIRE(solver->add_constraint("collision", *con));
        Eigen::VectorXd q = m->neutral();
        Workspace ws(m);
        CollisionQuery query(c, world);
        REQUIRE(ws.update(q));
        REQUIRE(query.update(ws, 0.0));
        double prev = query.nearest().distance;
        int zero_run = 0;
        SolveResult result;
        for (int step = 0; step < 300; ++step) {
            if (step % 75 == 0)
                for (int i = 0; i < 4; ++i) t[i] = 1.5 * u(rng);
            REQUIRE(task->set_target(t));
            Status s = solver->solve(q, 0.01, result);
            REQUIRE_MESSAGE(s, (s ? "" : s.error().message));
            if (result.step_scale < 1.0) {
                ++r.clipped;
                r.degraded_clips += solve_quality(result) == SolveQuality::degraded;
            }
            zero_run = result.step_scale == 0.0 ? zero_run + 1 : 0;
            r.frozen = std::max(r.frozen, zero_run);
            q = result.q_next;
            REQUIRE(ws.update(q));
            REQUIRE(query.update(ws, 0.0));
            const double d = query.nearest().distance;
            const double floor = std::min(prev, o.d_min) - floor_slack;
            if (d < floor - 1e-6) {
                ++r.violations;
                r.worst = std::max(r.worst, floor - d);
            }
            prev = d;
        }
    }
    return r;
}
}  // namespace

TEST_CASE("collision constraint: endpoint clip catches unverified ends") {
    ModelPtr m = load("arm.urdf");
    CollisionModelPtr c = arm_boxes(m);
    // A slack far above any reach disables the clip.
    const ClipRun off = clip_run(m, c, 1.0, 10.0, 0.001);
    // Trials that start deep inside a block may freeze: never going deeper wins over moving.
    const ClipRun on = clip_run(m, c, 1.0, 0.0, 0.0);
    MESSAGE("without clip " << off.violations << " ends below floor (worst " << off.worst * 1e3
                            << " mm); with clip " << on.violations << ", " << on.clipped
                            << " steps clipped, longest freeze " << on.frozen);
    CHECK(off.clipped == 0);
    CHECK(off.violations > 0);
    CHECK(on.violations == 0);
    CHECK(on.clipped > 0);
    CHECK(on.degraded_clips == on.clipped);
}

TEST_CASE("collision constraint: a pair entering from beyond the query range still clips to a partial step") {
    ModelPtr m = load("arm.urdf");
    CollisionModelPtr c = arm_boxes(m);
    const double turn = 0.6;
    Eigen::VectorXd q_goal = m->neutral();
    q_goal[0] = turn;
    Workspace ws(m);
    REQUIRE(ws.update(q_goal));
    Eigen::Isometry3d tool;
    REQUIRE(ws.frame_pose(*m->frame_id("tool"), tool));
    auto world = std::make_shared<CollisionWorld>();
    REQUIRE(world->add("block", Shape::box(0.1, 0.1, 0.1),
                       at(tool.translation().x(), tool.translation().y(), tool.translation().z())));
    CollisionConstraintOptions o;
    o.d_min = 0.002;
    o.activation_margin = 0.01;
    o.max_cuts = 0;
    auto con = CollisionConstraint::create(c, world, o);
    REQUIRE(con);
    auto solver = Solver::create(m);
    REQUIRE(solver);
    auto task = std::make_shared<JointTask>(std::vector<std::string>{"j1", "j2", "j3", "j4"},
                                            Eigen::VectorXd(Eigen::Vector4d(turn, 0, 0, 0)));
    REQUIRE(task->set_gain(1.0));
    REQUIRE(solver->add_task("joint", task));
    REQUIRE(solver->add_constraint("collision", *con));
    const Eigen::VectorXd q = m->neutral();
    REQUIRE(ws.update(q));
    CollisionQuery query(c, world);
    REQUIRE(query.update(ws, 0.0));
    REQUIRE(query.nearest().distance > 0.1);
    SolveResult result;
    REQUIRE(solver->solve(q, 0.5, result));
    // The pair had no recorded start distance; the estimate must still admit a partial step rather than hold.
    CHECK(result.step_scale > 0.0);
    CHECK(result.step_scale < 1.0);
    REQUIRE(ws.update(result.q_next));
    REQUIRE(query.update(ws, 0.0));
    CHECK(query.nearest().distance >= o.d_min - 1e-9);
}

TEST_CASE("collision constraint: far steps skip the endpoint clip") {
    ModelPtr m = load("arm.urdf");
    CollisionModelPtr c = arm_boxes(m);
    const ClipRun r = clip_run(m, c, 8.0, 0.0, 0.0);
    CHECK(r.clipped == 0);
    CHECK(r.violations == 0);
}
