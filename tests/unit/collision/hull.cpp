// Built-in quickhull: closed, outward-oriented and containing every input point.
#include "hull.hpp"

#include <Eigen/Geometry>
#include <doctest/doctest.h>

#include <cmath>
#include <map>
#include <random>
#include <utility>

using kstone::collision_detail::convex_hull;
using kstone::collision_detail::Hull;

namespace {

void check_hull(const Hull& hull, const Eigen::Matrix3Xd& input) {
    const auto v = hull.vertices.cols();
    const auto f = static_cast<Eigen::Index>(hull.triangles.size());
    std::map<std::pair<int, int>, int> edges;
    for (const auto& t : hull.triangles)
        for (std::size_t k = 0; k < 3; ++k) ++edges[{t[k], t[(k + 1) % 3]}];
    for (const auto& [e, count] : edges) {
        CHECK(count == 1);
        CHECK(edges.count({e.second, e.first}) == 1);
    }
    CHECK(v - static_cast<Eigen::Index>(edges.size() / 2) + f == 2);

    const Eigen::Vector3d centre = hull.vertices.rowwise().mean();
    const double tol = 1e-7 * (input.rowwise().maxCoeff() - input.rowwise().minCoeff()).norm();
    for (const auto& t : hull.triangles) {
        const Eigen::Vector3d a = hull.vertices.col(t[0]);
        const Eigen::Vector3d n =
            (hull.vertices.col(t[1]) - a).cross(hull.vertices.col(t[2]) - a).normalized();
        CHECK(n.dot(centre - a) < 0.0);
        CHECK(((n.transpose() * (input.colwise() - a)).array() <= tol).all());
    }
}

}  // namespace

TEST_CASE("convex hull: random ball points") {
    std::mt19937 rng(7);
    std::normal_distribution<double> g;
    Eigen::Matrix3Xd p(3, 1000);
    for (Eigen::Index i = 0; i < p.cols(); ++i) {
        Eigen::Vector3d x(g(rng), g(rng), g(rng));
        p.col(i) = x.normalized() * (i % 3 == 0 ? 0.5 : 1.0) + Eigen::Vector3d(3.0, -2.0, 1.0);
    }
    auto hull = convex_hull(p);
    REQUIRE(hull);
    CHECK(hull->vertices.cols() > 100);
    check_hull(*hull, p);
}

TEST_CASE("convex hull: coplanar and duplicate points collapse to the corners") {
    Eigen::Matrix3Xd p(3, 11 * 11 * 11 + 8);
    Eigen::Index c = 0;
    for (int i = 0; i <= 10; ++i)
        for (int j = 0; j <= 10; ++j)
            for (int k = 0; k <= 10; ++k) p.col(c++) = Eigen::Vector3d(i, j, k) * 0.02;
    for (int i = 0; i < 8; ++i) p.col(c++) = Eigen::Vector3d(i & 1, (i >> 1) & 1, (i >> 2) & 1) * 0.2;
    auto hull = convex_hull(p);
    REQUIRE(hull);
    CHECK(hull->vertices.cols() == 8);
    CHECK(hull->triangles.size() == 12);
    check_hull(*hull, p);
}

TEST_CASE("convex hull: degenerate input is rejected") {
    Eigen::Matrix3Xd flat = Eigen::Matrix3Xd::Random(3, 50);
    flat.row(2).setConstant(0.3);
    CHECK_FALSE(convex_hull(flat));
    CHECK_FALSE(convex_hull(Eigen::Matrix3Xd::Random(3, 3)));
    CHECK_FALSE(convex_hull(Eigen::Matrix3Xd::Zero(3, 10)));
    Eigen::Matrix3Xd bad = Eigen::Matrix3Xd::Random(3, 10);
    bad(0, 4) = std::nan("");
    CHECK_FALSE(convex_hull(bad));
}
