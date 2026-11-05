#pragma once

#include "kstone/collision/geometry.hpp"
#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"

#include <Eigen/Geometry>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace kstone {

using ObstacleId = std::uint64_t;

/// Static or externally moved obstacles in the world frame, checked against every robot geometry.
/// Not thread-safe: mutate it between solves, from the thread that runs the solver.
/// add / remove allocate and bump revision(); set_pose, set_velocity and set_enabled do not allocate.
class KSTONE_EXPORT CollisionWorld {
  public:
    struct Impl;
    struct Obstacle;

    CollisionWorld();
    ~CollisionWorld();
    CollisionWorld(const CollisionWorld&) = delete;
    CollisionWorld& operator=(const CollisionWorld&) = delete;

    Result<ObstacleId> add(std::string name, Shape shape, const Eigen::Isometry3d& pose);
    Status remove(ObstacleId id);
    Status set_pose(ObstacleId id, const Eigen::Isometry3d& pose);
    /// World-frame velocity of the obstacle origin (linear, angular); collision rows retreat from an
    /// approaching obstacle by its closing speed. Starts at zero; set_pose does not change it.
    Status set_velocity(ObstacleId id, const Eigen::Vector3d& linear, const Eigen::Vector3d& angular);
    /// Disabled obstacles are skipped by every query; added obstacles start enabled.
    Status set_enabled(ObstacleId id, bool enabled);
    void clear();

    std::size_t size() const noexcept;
    Result<ObstacleId> find(std::string_view name) const;
    /// Obstacle at index i < size(), in the order CollisionQuery pairs them; invalidated by add / remove.
    const std::string& name(std::size_t i) const;
    const Shape& shape(std::size_t i) const;
    const Eigen::Isometry3d& pose(std::size_t i) const;
    bool enabled(std::size_t i) const;
    const Eigen::Vector3d& linear_velocity(std::size_t i) const;
    const Eigen::Vector3d& angular_velocity(std::size_t i) const;
    /// World velocity of the obstacle-fixed point currently at `point`.
    Eigen::Vector3d point_velocity(std::size_t i, const Eigen::Vector3d& point) const;
    /// Changes on add / remove / clear only.
    std::uint64_t revision() const noexcept { return revision_; }

    const Impl& impl() const noexcept { return *impl_; }

  private:
    std::unique_ptr<Impl> impl_;
    std::uint64_t revision_{0};
    ObstacleId next_id_{1};
};

}  // namespace kstone
