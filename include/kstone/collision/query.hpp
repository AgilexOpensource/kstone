#pragma once

#include "kstone/collision/model.hpp"
#include "kstone/collision/world.hpp"
#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/model/workspace.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace kstone {

/// One checked pair: robot geometry `first` against robot geometry `second`, or against the
/// world obstacle at index `second` when `obstacle` is true.
struct CollisionPair {
    Index first{0};
    Index second{0};
    bool obstacle{false};
};

/// Signed distance of a pair (negative when penetrating) with world witness points and the unit
/// normal from point1 towards point2 (the direction in which the distance grows).
struct Proximity {
    Index pair{-1};
    double distance{std::numeric_limits<double>::infinity()};
    Eigen::Vector3d point1{Eigen::Vector3d::Zero()};
    Eigen::Vector3d point2{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
};

/// Distances between the robot geometries (self pairs) and between robot geometries and world
/// obstacles. Per-thread like Workspace. update() does not allocate unless the world's revision
/// changed since the previous call (the pair list is then rebuilt).
class KSTONE_EXPORT CollisionQuery {
  public:
    struct Impl;

    explicit CollisionQuery(CollisionModelPtr model, std::shared_ptr<const CollisionWorld> world = nullptr);
    ~CollisionQuery();
    CollisionQuery(CollisionQuery&&) noexcept;
    CollisionQuery& operator=(CollisionQuery&&) noexcept;
    CollisionQuery(const CollisionQuery&) = delete;
    CollisionQuery& operator=(const CollisionQuery&) = delete;

    /// Pairs within `threshold` (>= 0) into proximities(), nearest first, and the exact minimum over
    /// all pairs. `workspace` must be updated and belong to the collision model's Model. With
    /// `exact_nearest` false only pairs that can be within `threshold` are evaluated, so nearest() is
    /// the nearest of proximities() (pair -1 and +inf distance when it is empty).
    Status update(const Workspace& workspace, double threshold, bool exact_nearest = true);

    const std::vector<Proximity>& proximities() const noexcept;
    /// Nearest pair of the last update (pair -1 and +inf distance when there are no pairs).
    const Proximity& nearest() const noexcept;

    /// d distance / d v of a proximity at the workspace configuration; out has nv entries.
    /// Accepts a row of a column-major matrix.
    Status jacobian(const Workspace& workspace, const Proximity& proximity,
                    Eigen::Ref<Eigen::RowVectorXd, 0, Eigen::InnerStride<>> out);

    /// Self pairs first (CollisionModel::pairs() order), then robot x obstacle pairs (geometries
    /// on joint 0 are fixed in the world and are not paired with obstacles).
    const std::vector<CollisionPair>& pairs() const noexcept;
    std::string pair_name(Index pair) const;
    /// Incremented whenever pairs() is rebuilt.
    std::uint64_t layout() const noexcept;

    const CollisionModelPtr& model() const noexcept;
    /// Null without obstacles.
    const std::shared_ptr<const CollisionWorld>& world() const noexcept;

  private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace kstone
