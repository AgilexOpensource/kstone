#include "hull.hpp"
#include "internal.hpp"
#include <cmath>
#include <exception>

namespace kstone {

Shape Shape::sphere(double radius) {
    Shape s;
    s.type = ShapeType::sphere;
    s.size = Eigen::Vector3d(radius, 0.0, 0.0);
    return s;
}

Shape Shape::box(double x, double y, double z) {
    Shape s;
    s.type = ShapeType::box;
    s.size = Eigen::Vector3d(x, y, z);
    return s;
}

Shape Shape::cylinder(double radius, double length) {
    Shape s;
    s.type = ShapeType::cylinder;
    s.size = Eigen::Vector3d(radius, length, 0.0);
    return s;
}

Shape Shape::capsule(double radius, double length) {
    Shape s;
    s.type = ShapeType::capsule;
    s.size = Eigen::Vector3d(radius, length, 0.0);
    return s;
}

Shape Shape::convex(Eigen::Matrix3Xd vertices) {
    Shape s;
    s.type = ShapeType::convex;
    s.vertices = std::move(vertices);
    return s;
}

Status validate(const Shape& shape) {
    const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
    switch (shape.type) {
        case ShapeType::sphere:
            if (!positive(shape.size[0]))
                return make_error(ErrorCode::invalid_argument, "sphere radius must be > 0");
            return {};
        case ShapeType::box:
            if (!positive(shape.size[0]) || !positive(shape.size[1]) || !positive(shape.size[2]))
                return make_error(ErrorCode::invalid_argument, "box extents must be > 0");
            return {};
        case ShapeType::cylinder:
            if (!positive(shape.size[0]) || !positive(shape.size[1]))
                return make_error(ErrorCode::invalid_argument, "cylinder radius and length must be > 0");
            return {};
        case ShapeType::capsule:
            if (!positive(shape.size[0]) || !std::isfinite(shape.size[1]) || shape.size[1] < 0.0)
                return make_error(ErrorCode::invalid_argument, "capsule radius must be > 0 and length >= 0");
            return {};
        case ShapeType::convex:
            if (shape.vertices.cols() < 4 || !shape.vertices.allFinite())
                return make_error(ErrorCode::invalid_argument,
                                  "convex shapes need at least 4 finite vertices");
            return {};
    }
    return make_error(ErrorCode::invalid_argument, "unknown shape type");
}

namespace collision_detail {
namespace {

// Null when Coal was built without qhull or rejects the points; the built-in hull then decides.
std::shared_ptr<coal::ConvexBase> coal_hull(const Eigen::Matrix3Xd& vertices) {
    const auto n = static_cast<unsigned int>(vertices.cols());
    auto points = std::make_shared<std::vector<coal::Vec3s>>(n);
    for (unsigned int i = 0; i < n; ++i) (*points)[i] = vertices.col(static_cast<Index>(i));
    try {
        std::shared_ptr<coal::ConvexBase> hull(coal::ConvexBase::convexHull(points, n, true, nullptr));
        if (hull && hull->num_points >= 4) return hull;
    } catch (const std::exception&) {
    }
    return nullptr;
}

Result<std::shared_ptr<coal::ConvexBase>> builtin_hull(const Eigen::Matrix3Xd& vertices) {
    auto hull = convex_hull(vertices);
    if (!hull) return std::move(hull).error();
    const auto np = static_cast<unsigned int>(hull->vertices.cols());
    const auto nt = static_cast<unsigned int>(hull->triangles.size());
    auto points = std::make_shared<std::vector<coal::Vec3s>>(np);
    for (unsigned int i = 0; i < np; ++i) (*points)[i] = hull->vertices.col(static_cast<Index>(i));
    auto triangles = std::make_shared<std::vector<coal::Triangle>>(nt);
    for (unsigned int i = 0; i < nt; ++i) {
        const auto& t = hull->triangles[i];
        (*triangles)[i].set(static_cast<coal::Triangle::index_type>(t[0]),
                            static_cast<coal::Triangle::index_type>(t[1]),
                            static_cast<coal::Triangle::index_type>(t[2]));
    }
    return std::shared_ptr<coal::ConvexBase>(
        std::make_shared<coal::Convex<coal::Triangle>>(std::move(points), np, std::move(triangles), nt));
}

}  // namespace

Result<std::shared_ptr<const coal::CollisionGeometry>> make_geometry(Shape& shape) {
    if (Status s = validate(shape); !s) return s.error();
    std::shared_ptr<coal::CollisionGeometry> out;
    try {
        switch (shape.type) {
            case ShapeType::sphere:
                out = std::make_shared<coal::Sphere>(shape.size[0]);
                break;
            case ShapeType::box:
                out = std::make_shared<coal::Box>(shape.size[0], shape.size[1], shape.size[2]);
                break;
            case ShapeType::cylinder:
                out = std::make_shared<coal::Cylinder>(shape.size[0], shape.size[1]);
                break;
            case ShapeType::capsule:
                out = std::make_shared<coal::Capsule>(shape.size[0], shape.size[1]);
                break;
            case ShapeType::convex: {
                auto hull = coal_hull(shape.vertices);
                if (!hull) {
                    auto own = builtin_hull(shape.vertices);
                    if (!own) return std::move(own).error();
                    hull = std::move(*own);
                }
                Eigen::Matrix3Xd vertices(3, hull->num_points);
                for (unsigned int i = 0; i < hull->num_points; ++i)
                    vertices.col(static_cast<Index>(i)) = (*hull->points)[i];
                shape.vertices = std::move(vertices);
                out = std::move(hull);
                break;
            }
        }
        out->computeLocalAABB();
    } catch (const std::exception& e) {
        return make_error(ErrorCode::invalid_argument, std::string("collision geometry: ") + e.what());
    }
    if (!out) return make_error(ErrorCode::internal, "collision geometry was not created");
    return std::shared_ptr<const coal::CollisionGeometry>(std::move(out));
}

}  // namespace collision_detail
}  // namespace kstone
