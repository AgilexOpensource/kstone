#pragma once

#include "kstone/collision/geometry.hpp"
#include "kstone/core/context.hpp"
#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/source.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace kstone {

/// A robot collision geometry attached to a frame (placement in that frame).
struct CollisionGeometry {
    std::string name;
    std::string frame;
    Eigen::Isometry3d placement{Eigen::Isometry3d::Identity()};
    Shape shape;
};

/// Bodies whose joint matches are never checked against each other. A joint matches if it is not
/// in exclude_joints and is in joint_names or starts with one of joint_prefixes.
struct BodyGroup {
    std::string name;
    std::vector<std::string> joint_prefixes;
    std::vector<std::string> joint_names;
    std::vector<std::string> exclude_joints;
};

using NamePair = std::pair<std::string, std::string>;

struct CollisionModelOptions {
    /// Build self-collision pairs; when true, filtering must leave at least one pair.
    bool self_collision{true};
    /// SRDF whose <disable_collisions link1 link2> entries are skipped (matched on frame names).
    std::optional<ModelSource> srdf;
    std::vector<BodyGroup> groups;
    /// Frame or geometry names; each excludes every pair between the two bodies they sit on.
    std::vector<NamePair> exclude_pairs;
    /// Frame or geometry names; excludes every pair between the two bodies' kinematic subtrees (e.g. two hands).
    std::vector<NamePair> exclude_subtrees;
    /// Drop self pairs already penetrating at `initial`, logging each at warn.
    bool exclude_initial_collisions{true};
    /// Reference posture (size nq) for exclude_initial_collisions; empty = model neutral.
    Eigen::VectorXd initial;
};

/// Robot collision geometries, the rigid body (joint) carrying each, and the filtered self pairs.
/// Immutable after creation; share it between threads freely.
class KSTONE_EXPORT CollisionModel {
  public:
    struct Impl;

    /// Geometries of the URDF <collision> elements (mesh -> convex hull). Meshes resolve through
    /// context.resources relative to the URDF directory (or source.base_dir for text).
    static Result<std::shared_ptr<const CollisionModel>> from_urdf(ModelPtr model, const ModelSource& urdf,
                                                                   const CollisionModelOptions& options,
                                                                   const Context& context);
    /// Explicit geometries. `context` is only used to resolve options.srdf.
    static Result<std::shared_ptr<const CollisionModel>> create(ModelPtr model,
                                                                std::vector<CollisionGeometry> geometries,
                                                                const CollisionModelOptions& options,
                                                                const Context& context);

    ~CollisionModel();
    CollisionModel(const CollisionModel&) = delete;
    CollisionModel& operator=(const CollisionModel&) = delete;

    const ModelPtr& model() const noexcept { return model_; }
    /// Convex geometries report their hull vertices.
    const std::vector<CollisionGeometry>& geometries() const noexcept { return geometries_; }
    /// Joint carrying each geometry (0 = world).
    const std::vector<JointId>& joints() const noexcept { return joints_; }
    /// Self pairs (first < second), in lexicographic order.
    const std::vector<std::pair<Index, Index>>& pairs() const noexcept { return pairs_; }
    Result<Index> geometry_index(std::string_view name) const;

    const Impl& impl() const noexcept { return *impl_; }

  private:
    CollisionModel();

    ModelPtr model_;
    std::vector<CollisionGeometry> geometries_;
    std::vector<JointId> joints_;
    std::vector<std::pair<Index, Index>> pairs_;
    std::unique_ptr<Impl> impl_;
};

using CollisionModelPtr = std::shared_ptr<const CollisionModel>;

/// <disable_collisions> pairs of an SRDF: ordered, deduplicated, sorted, self pairs dropped.
KSTONE_EXPORT Result<std::vector<NamePair>> srdf_disabled_pairs(const ModelSource& srdf,
                                                                const Context& context);

}  // namespace kstone
