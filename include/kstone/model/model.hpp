#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/core/mask.hpp"
#include "kstone/model/source.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace kstone {

struct Context;

using Index = Eigen::Index;
using JointId = std::size_t;  ///< 0 is the universe (world) joint.
using FrameId = std::size_t;

enum class JointType {
    revolute,     ///< 1-DoF rotation with bounded angle (nq = nv = 1).
    continuous,   ///< 1-DoF unbounded rotation, q stored as (cos, sin) (nq = 2, nv = 1).
    prismatic,    ///< 1-DoF translation.
    planar,       ///< nq = 4, nv = 3.
    free_flyer,   ///< nq = 7, nv = 6.
    spherical,    ///< nq = 4, nv = 3.
    translation,  ///< nq = nv = 3.
    mimic,        ///< Coupled follower, nq = nv = 0.
    composite,    ///< Composite / helical / universal / other multi-DoF joints.
    universe,     ///< Joint 0.
};

KSTONE_EXPORT const char* to_string(JointType type) noexcept;

struct JointInfo {
    std::string name;
    JointType type;
    JointId id;
    JointId parent;
    Index idx_q;  ///< First configuration coordinate.
    Index nq;
    Index idx_v;  ///< First velocity coordinate.
    Index nv;
};

enum class FrameKind { operational, joint, fixed_joint, body, sensor };

struct FrameInfo {
    std::string name;
    FrameKind kind;
    FrameId id;
    JointId parent_joint;
};

/// Leader/follower relation declared by the source: q_follower = multiplier * q_leader + offset.
struct MimicRelation {
    std::string follower;
    std::string leader;
    double multiplier{1.0};
    double offset{0.0};
};

/// Replacement limits for one joint; unset members keep the source's value.
struct JointLimitOverride {
    std::string joint;
    std::optional<double> lower;  ///< Position, revolute/prismatic joints only.
    std::optional<double> upper;
    std::optional<double> velocity;  ///< > 0, every velocity coordinate of the joint.
};

/// Immutable kinematic/inertial model. Share across threads as std::shared_ptr<const Model>;
/// per-thread mutable state lives in Workspace.
class KSTONE_EXPORT Model {
  public:
    struct Impl;

    static Result<std::shared_ptr<const Model>> load(const ModelSource& source, const ModelOptions& options,
                                                     const Context& context);

    /// Copy with replaced limits (may widen the source); invalid_argument on unknown joints or an empty range.
    Result<std::shared_ptr<const Model>> with_limits(const std::vector<JointLimitOverride>& overrides) const;

    ~Model();
    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;

    const std::string& name() const noexcept;
    Index nq() const noexcept;
    Index nv() const noexcept;
    RootJoint root() const noexcept;
    /// What load() read: file locations resolved to absolute paths, format filled in. Empty after make_model().
    const ModelSource& source() const noexcept;

    // ---- topology / naming -------------------------------------------------------------
    const std::vector<JointInfo>& joints() const noexcept;  ///< Includes universe at index 0.
    const std::vector<FrameInfo>& frames() const noexcept;
    const std::vector<MimicRelation>& mimics() const noexcept;
    Result<JointId> joint_id(std::string_view name) const;
    Result<FrameId> frame_id(std::string_view name) const;
    /// Owning joint of each velocity coordinate (size nv).
    const std::vector<JointId>& velocity_owner() const noexcept;

    bool is_ancestor(JointId ancestor, JointId joint) const noexcept;  ///< Reflexive.
    /// Joints from `from` (exclusive) down to `to` (inclusive). `from` must be an ancestor of `to`.
    Result<std::vector<JointId>> joint_chain(JointId from, JointId to) const;
    bool frame_in_subtree(FrameId frame, JointId root) const noexcept;

    Mask velocity_mask(const std::vector<JointId>& joints) const;  ///< Size nv.
    Mask subtree_velocity_mask(JointId root) const;

    // ---- limits (sizes nq / nv / nv) --------------------------------------------------------
    const Eigen::VectorXd& lower_position_limit() const noexcept;
    const Eigen::VectorXd& upper_position_limit() const noexcept;
    const Eigen::VectorXd& velocity_limit() const noexcept;
    const Eigen::VectorXd& effort_limit() const noexcept;
    /// True for scalar joints whose source declared a non-degenerate position range.
    bool has_position_limit(JointId joint) const noexcept;

    // ---- inertia ----------------------------------------------------------------------------
    double total_mass() const noexcept;  ///< Includes inertia attached to the fixed base.
    double subtree_mass(JointId root) const noexcept;

    // ---- configuration space ----------------------------------------------------------------
    Eigen::VectorXd neutral() const;
    /// Size, finiteness and normalization of non-Euclidean joints.
    Status validate(const Eigen::Ref<const Eigen::VectorXd>& q) const;
    /// Rescales quaternions and (cos, sin) pairs to unit norm; fails if any norm is off by more than tolerance.
    Status normalize(Eigen::Ref<Eigen::VectorXd> q, double tolerance = 1e-3) const;
    Status check_position_limits(const Eigen::Ref<const Eigen::VectorXd>& q, double tolerance = 1e-6) const;
    /// Allocation-free: out must already have size nq.
    Status integrate(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v,
                     Eigen::Ref<Eigen::VectorXd> out) const;
    /// Allocation-free: out must already have size nv. Returns v such that integrate(q0, v) == q1.
    Status difference(const Eigen::Ref<const Eigen::VectorXd>& q0,
                      const Eigen::Ref<const Eigen::VectorXd>& q1, Eigen::Ref<Eigen::VectorXd> out) const;

    /// Scalar joints only (nv == 1). Continuous joints go via (cos, sin) and read back in (-pi, pi], no unwrap.
    Result<double> joint_position(const Eigen::Ref<const Eigen::VectorXd>& q, JointId joint) const;
    Status set_joint_position(Eigen::Ref<Eigen::VectorXd> q, JointId joint, double value) const;

    /// Root joint pose in the world. Fixed roots return identity.
    Result<Eigen::Isometry3d> root_pose(const Eigen::Ref<const Eigen::VectorXd>& q) const;
    /// Free-flyer roots take the full pose.
    Status set_root_pose(Eigen::Ref<Eigen::VectorXd> q, const Eigen::Isometry3d& pose) const;

    const Impl& impl() const noexcept { return *impl_; }

  private:
    explicit Model(std::unique_ptr<Impl> impl);
    friend struct ModelFactory;
    std::unique_ptr<Impl> impl_;
};

using ModelPtr = std::shared_ptr<const Model>;

/// Precomputed mapping between an external ordered list of scalar-joint names and q / v.
/// Lets hosts exchange joint vectors in their own ordering without per-call lookups.
class KSTONE_EXPORT NameMap {
  public:
    static Result<NameMap> create(const ModelPtr& model, const std::vector<std::string>& names);

    std::size_t size() const noexcept { return joints_.size(); }
    const std::vector<std::string>& names() const noexcept { return names_; }

    /// external.size() must equal size(); q / v keep entries for joints not in the map; continuous in (-pi, pi].
    Status scatter_positions(const Eigen::Ref<const Eigen::VectorXd>& external,
                             Eigen::Ref<Eigen::VectorXd> q) const;
    Status gather_positions(const Eigen::Ref<const Eigen::VectorXd>& q,
                            Eigen::Ref<Eigen::VectorXd> external) const;
    Status scatter_velocities(const Eigen::Ref<const Eigen::VectorXd>& external,
                              Eigen::Ref<Eigen::VectorXd> v) const;
    Status gather_velocities(const Eigen::Ref<const Eigen::VectorXd>& v,
                             Eigen::Ref<Eigen::VectorXd> external) const;

  private:
    ModelPtr model_;
    std::vector<std::string> names_;
    std::vector<JointId> joints_;
    std::vector<Index> idx_v_;
};

}  // namespace kstone
