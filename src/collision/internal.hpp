#pragma once

// Coal / Pinocchio side of the collision layer; never included by public headers.

#include "kstone/collision/model.hpp"
#include "kstone/collision/world.hpp"

#include <coal/collision_object.h>
#include <coal/distance.h>
#include <coal/shape/convex.h>
#include <coal/shape/geometric_shapes.h>
#include <pinocchio/spatial/se3.hpp>

#include <memory>
#include <string>
#include <vector>

namespace kstone {

namespace collision_detail {

/// Coal geometry with its local AABB computed. Convex shapes are replaced by their hull vertices.
Result<std::shared_ptr<const coal::CollisionGeometry>> make_geometry(Shape& shape);

inline pinocchio::SE3 to_se3(const Eigen::Isometry3d& pose) {
    return pinocchio::SE3(pose.linear(), pose.translation());
}

inline bool finite_pose(const Eigen::Isometry3d& pose) { return pose.matrix().allFinite(); }

}  // namespace collision_detail

struct CollisionModel::Impl {
    std::vector<std::shared_ptr<const coal::CollisionGeometry>> geometries;
    std::vector<pinocchio::SE3> placements;  ///< Geometry in its joint frame.
};

struct CollisionWorld::Obstacle {
    ObstacleId id{0};
    std::string name;
    Shape shape;
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
    bool enabled{true};
    std::shared_ptr<const coal::CollisionGeometry> geometry;
};

struct CollisionWorld::Impl {
    std::vector<Obstacle> obstacles;
};

}  // namespace kstone
