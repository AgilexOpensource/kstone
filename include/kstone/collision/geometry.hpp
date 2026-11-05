#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

namespace kstone {

enum class ShapeType { sphere, box, cylinder, capsule, convex };

/// A collision primitive in its own frame. Cylinders and capsules run along z; `length` is the
/// cylindrical part (a capsule adds a hemisphere of `radius` at each end). Convex shapes are the
/// convex hull of `vertices` (meshes are always reduced to their hull).
struct KSTONE_EXPORT Shape {
    ShapeType type{ShapeType::sphere};
    /// sphere: (radius, 0, 0); box: full extents (x, y, z); cylinder / capsule: (radius, length, 0).
    Eigen::Vector3d size{Eigen::Vector3d::Zero()};
    Eigen::Matrix3Xd vertices;  ///< convex only, 3 x n, n >= 4 and not coplanar.

    static Shape sphere(double radius);
    static Shape box(double x, double y, double z);
    static Shape cylinder(double radius, double length);
    static Shape capsule(double radius, double length);
    static Shape convex(Eigen::Matrix3Xd vertices);
};

/// Finite, positive dimensions (>= 4 finite vertices for convex).
KSTONE_EXPORT Status validate(const Shape& shape);

}  // namespace kstone
