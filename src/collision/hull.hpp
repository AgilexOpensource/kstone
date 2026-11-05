#pragma once

// Fallback 3D convex hull for Coal builds without qhull.

#include "kstone/core/error.hpp"

#include <Eigen/Core>

#include <array>
#include <vector>

namespace kstone::collision_detail {

struct Hull {
    Eigen::Matrix3Xd vertices;
    std::vector<std::array<int, 3>> triangles;  ///< Counter-clockwise seen from outside.
};

/// Quickhull over the columns of `points`; fails when they span less than a volume.
Result<Hull> convex_hull(const Eigen::Matrix3Xd& points);

}  // namespace kstone::collision_detail
