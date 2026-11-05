// Collision cost benchmark: primitives vs convex hulls.
// Part 1: one robot x obstacle pair per shape combination. Part 2: dual-arm self collision with the
// URDF primitives replaced by spheres, capsules or convex hulls of n vertices.
// Usage: kstone_collision_bench [steps]   (build with -DKSTONE_BUILD_BENCH=ON, Release)
#include "kstone/collision/query.hpp"
#include "kstone/collision/world.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

using namespace kstone;

namespace {

using Clock = std::chrono::steady_clock;
const std::string kFixtures = KSTONE_FIXTURES;

struct Stats {
    double median, p99, max;
};

Stats stats(std::vector<double> us) {
    std::sort(us.begin(), us.end());
    const auto at = [&](double p) {
        return us[std::min(us.size() - 1, static_cast<std::size_t>(p * static_cast<double>(us.size())))];
    };
    return {at(0.5), at(0.99), us.back()};
}

// n points on the ellipsoid with semi-axes a (Fibonacci lattice): all of them are hull vertices.
Eigen::Matrix3Xd ellipsoid(const Eigen::Vector3d& a, int n) {
    Eigen::Matrix3Xd v(3, n);
    const double golden = M_PI * (3.0 - std::sqrt(5.0));
    for (int i = 0; i < n; ++i) {
        const double z = 1.0 - 2.0 * (i + 0.5) / n, r = std::sqrt(1.0 - z * z), t = golden * i;
        v.col(i) = Eigen::Vector3d(r * std::cos(t), r * std::sin(t), z).cwiseProduct(a);
    }
    return v;
}

// Half extents of a primitive in its frame.
Eigen::Vector3d half_extents(const Shape& s) {
    switch (s.type) {
        case ShapeType::sphere:
            return Eigen::Vector3d::Constant(s.size.x());
        case ShapeType::box:
            return s.size / 2.0;
        case ShapeType::cylinder:
            return Eigen::Vector3d(s.size.x(), s.size.x(), s.size.y() / 2.0);
        case ShapeType::capsule:
            return Eigen::Vector3d(s.size.x(), s.size.x(), s.size.y() / 2.0 + s.size.x());
        case ShapeType::convex:
            return s.vertices.cwiseAbs().rowwise().maxCoeff();
    }
    return Eigen::Vector3d::Zero();
}

Shape make_shape(const std::string& kind, const Eigen::Vector3d& h) {
    if (kind == "sphere") return Shape::sphere(h.norm());
    if (kind == "box") return Shape::box(2 * h.x(), 2 * h.y(), 2 * h.z());
    if (kind == "cylinder") return Shape::cylinder(std::max(h.x(), h.y()), 2 * h.z());
    if (kind == "capsule")
        return Shape::capsule(std::max(h.x(), h.y()), std::max(0.0, 2 * h.z() - std::max(h.x(), h.y())));
    return Shape::convex(ellipsoid(h, std::atoi(kind.c_str() + 6)));  // "convexN"
}

// Replace a geometry by `kind` with the same extents; capsules run along the longest axis.
CollisionGeometry replace(CollisionGeometry g, const std::string& kind) {
    if (kind == "urdf") return g;
    Eigen::Vector3d h = half_extents(g.shape);
    if (kind == "capsule") {
        Eigen::Index axis = 0;
        h.maxCoeff(&axis);
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        if (axis != 2) {
            R.col(2) = Eigen::Vector3d::Unit(axis);
            R.col(0) = Eigen::Vector3d::Unit(2);
            R.col(1) = R.col(2).cross(R.col(0));
            const double hz = h[axis];
            h[axis] = h.z();
            h.z() = hz;
        }
        g.placement.linear() = g.placement.linear() * R;
        const double r = Eigen::Vector2d(h.x(), h.y()).norm();
        g.shape = Shape::capsule(r, std::max(0.0, 2 * h.z() - r));
        return g;
    }
    g.shape = make_shape(kind, h);
    return g;
}

ModelPtr load(const std::string& file) {
    auto m = Model::load(ModelSource::file(kFixtures + "/" + file), ModelOptions{}, Context::defaults());
    if (!m) {
        std::fprintf(stderr, "%s: %s\n", file.c_str(), m.error().message.c_str());
        std::exit(1);
    }
    return *m;
}

// One sliding robot shape against a tumbling obstacle; only the distance query is timed.
int pair_case(const ModelPtr& m, const std::string& robot, const std::string& obstacle, int steps) {
    const Eigen::Vector3d h(0.05, 0.05, 0.12);
    CollisionGeometry g{"g", "slider", Eigen::Isometry3d::Identity(), make_shape(robot, h)};
    CollisionModelOptions o;
    o.self_collision = false;
    auto c = CollisionModel::create(m, {g}, o, Context::defaults());
    if (!c) {
        std::fprintf(stderr, "%s: %s\n", robot.c_str(), c.error().message.c_str());
        return 1;
    }
    auto world = std::make_shared<CollisionWorld>();
    auto id = world->add("o", make_shape(obstacle, h), Eigen::Isometry3d::Identity());
    if (!id) return 1;
    CollisionQuery query(*c, world);
    Workspace ws(m);
    std::vector<double> us;
    us.reserve(static_cast<std::size_t>(steps));
    double nearest = 1e9;
    for (int i = 0; i < 100 + steps; ++i) {
        const double t = 0.002 * i;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.linear() = (Eigen::AngleAxisd(1.3 * t, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(0.7 * t, Eigen::Vector3d::UnitY()))
                            .toRotationMatrix();
        pose.translation() = Eigen::Vector3d(0.36, 0.0, 0.0);
        (void)world->set_pose(*id, pose);
        if (!ws.update(Eigen::VectorXd::Constant(1, 0.1 + 0.08 * std::sin(3.0 * t)))) return 1;
        const auto t0 = Clock::now();
        const Status s = query.update(ws, 0.0, true);
        const auto t1 = Clock::now();
        if (!s) return 1;
        nearest = std::min(nearest, query.nearest().distance);
        if (i >= 100) us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    const Stats st = stats(us);
    std::printf("  %-11s x %-11s median %6.2f us  p99 %6.2f us  max %7.2f us  min distance %+.3f\n",
                robot.c_str(), obstacle.c_str(), st.median, st.p99, st.max, nearest);
    return 0;
}

// Geometry pairs penetrating at the neutral pose, as an SRDF would disable them.
void overlapping(const ModelPtr& m, const std::vector<CollisionGeometry>& base, const std::string& kind,
                 std::vector<NamePair>& out) {
    std::vector<CollisionGeometry> geometries;
    for (const CollisionGeometry& g : base) geometries.push_back(replace(g, kind));
    auto c = CollisionModel::create(m, geometries, CollisionModelOptions{}, Context::defaults());
    Workspace ws(m);
    if (!c || !ws.update(m->neutral())) return;
    CollisionQuery query(*c);
    if (!query.update(ws, 0.0, true)) return;
    for (const Proximity& p : query.proximities()) {
        if (p.distance >= 0.0) continue;
        const CollisionPair& pair = query.pairs()[static_cast<std::size_t>(p.pair)];
        out.emplace_back(base[static_cast<std::size_t>(pair.first)].name,
                         base[static_cast<std::size_t>(pair.second)].name);
    }
}

// Dual-arm self collision along a joint-space sweep: update (threshold 0.1, both nearest modes) plus
// the Jacobian of every reported proximity, as the collision constraint does per step.
int robot_case(const ModelPtr& m, const std::vector<CollisionGeometry>& base, const std::string& kind,
               const std::vector<NamePair>& excluded, int steps) {
    std::vector<CollisionGeometry> geometries;
    for (const CollisionGeometry& g : base) geometries.push_back(replace(g, kind));
    CollisionModelOptions o;
    o.exclude_pairs = excluded;
    auto c = CollisionModel::create(m, geometries, o, Context::defaults());
    if (!c) {
        std::fprintf(stderr, "%s: %s\n", kind.c_str(), c.error().message.c_str());
        return 1;
    }
    std::size_t vertices = 0;
    for (const CollisionGeometry& g : (*c)->geometries())
        vertices += static_cast<std::size_t>(g.shape.vertices.cols());
    Workspace ws(m);
    CollisionQuery query(*c);
    const Eigen::VectorXd q0 = m->neutral();
    Eigen::VectorXd q(m->nq()), v(m->nv()), row(m->nv());
    std::vector<double> exact, culled, jac;
    int near = 0, hits = 0;
    for (int i = 0; i < 100 + steps; ++i) {
        const double t = 0.002 * i;
        for (Eigen::Index k = 0; k < v.size(); ++k)
            v[k] = 0.8 * std::sin((1.0 + 0.37 * static_cast<double>(k)) * t + static_cast<double>(k));
        if (!m->integrate(q0, v, q) || !ws.update(q)) return 1;
        const auto t0 = Clock::now();
        if (!query.update(ws, 0.1, true)) return 1;
        const auto t1 = Clock::now();
        for (const Proximity& p : query.proximities()) (void)query.jacobian(ws, p, row);
        const auto t2 = Clock::now();
        if (!query.update(ws, 0.1, false)) return 1;
        const auto t3 = Clock::now();
        if (i < 100) continue;
        exact.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        jac.push_back(std::chrono::duration<double, std::micro>(t2 - t1).count());
        culled.push_back(std::chrono::duration<double, std::micro>(t3 - t2).count());
        near += static_cast<int>(query.proximities().size());
        hits += query.nearest().distance < 0.0 ? 1 : 0;
    }
    const Stats e = stats(exact), cu = stats(culled), j = stats(jac);
    std::printf(
        "  %-10s vertices %5zu  exact %7.1f / %7.1f us  culled %7.1f / %7.1f us  jacobians %5.1f us  "
        "near pairs %4.1f  penetrating %3.0f%%\n",
        kind.c_str(), vertices, e.median, e.p99, cu.median, cu.p99, j.median,
        static_cast<double>(near) / steps, 100.0 * hits / steps);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const int steps = argc > 1 ? std::max(1, std::atoi(argv[1])) : 5000;
    int rc = 0;

    std::printf("pair distance (one pair, median / p99 / max):\n");
    const ModelPtr slider = load("slider_collision.urdf");
    const std::vector<std::string> shapes = {"sphere",   "capsule",  "box",       "cylinder",
                                             "convex16", "convex64", "convex256", "convex1024"};
    for (const std::string& s : shapes) rc |= pair_case(slider, s, "box", steps);
    for (const char* s : {"capsule", "convex64", "convex256"}) rc |= pair_case(slider, s, s, steps);

    const std::string file = "omni_dual_arm_gripper.urdf";
    const ModelPtr robot = load(file);
    CollisionModelOptions o;
    auto urdf =
        CollisionModel::from_urdf(robot, ModelSource::file(kFixtures + "/" + file), o, Context::defaults());
    if (!urdf) {
        std::fprintf(stderr, "%s: %s\n", file.c_str(), urdf.error().message.c_str());
        return 1;
    }
    const std::vector<std::string> kinds = {"urdf",     "sphere",    "capsule",
                                            "convex32", "convex128", "convex512"};
    std::vector<NamePair> excluded;
    for (const std::string& k : kinds) overlapping(robot, (*urdf)->geometries(), k, excluded);
    o.exclude_pairs = excluded;
    auto filtered = CollisionModel::create(robot, (*urdf)->geometries(), o, Context::defaults());
    if (!filtered) return 1;
    std::printf(
        "\n%s self collision: %zu geometries, %zu of %zu pairs after excluding overlaps at neutral "
        "(median / p99):\n",
        file.c_str(), (*urdf)->geometries().size(), (*filtered)->pairs().size(), (*urdf)->pairs().size());
    for (const std::string& k : kinds) rc |= robot_case(robot, (*urdf)->geometries(), k, excluded, steps);
    return rc;
}
