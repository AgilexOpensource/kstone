#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/model/model.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>

namespace kstone {

enum class ReferenceFrame {
    local,                ///< Expressed in the frame itself.
    local_world_aligned,  ///< Origin at the frame, axes aligned with the world.
    world,                ///< Origin and axes of the world.
};

/// Per-thread mutable kinematic state for one Model. Buffers are preallocated at construction;
/// update() and the queries below do not allocate. Not thread-safe; create one per thread.
class KSTONE_EXPORT Workspace {
  public:
    struct Impl;

    explicit Workspace(ModelPtr model);
    ~Workspace();
    Workspace(Workspace&&) noexcept;
    Workspace& operator=(Workspace&&) noexcept;
    Workspace(const Workspace&) = delete;
    Workspace& operator=(const Workspace&) = delete;

    const ModelPtr& model() const noexcept { return model_; }

    /// Forward kinematics, joint Jacobians and frame placements for q.
    Status update(const Eigen::Ref<const Eigen::VectorXd>& q);
    bool updated() const noexcept { return updated_; }
    const Eigen::VectorXd& q() const noexcept;

    Status frame_pose(FrameId frame, Eigen::Isometry3d& out) const;
    Status joint_pose(JointId joint, Eigen::Isometry3d& out) const;
    /// out must be 6 x nv, rows (linear; angular).
    Status frame_jacobian(FrameId frame, ReferenceFrame ref, Eigen::Ref<Eigen::MatrixXd> out);

    /// Whole-model centre of mass in the world, including inertia welded to the base.
    Status center_of_mass(Eigen::Vector3d& out);
    /// out must be 3 x nv.
    Status center_of_mass_jacobian(Eigen::Ref<Eigen::MatrixXd> out);
    /// Centre of mass of the subtree rooted at `root`; root 0 is the whole model (same as center_of_mass).
    Status subtree_center_of_mass(JointId root, Eigen::Vector3d& out);
    Status subtree_center_of_mass_jacobian(JointId root, Eigen::Ref<Eigen::MatrixXd> out);

    Impl& impl() noexcept { return *impl_; }
    const Impl& impl() const noexcept { return *impl_; }

  private:
    Status require_updated() const;
    Status ensure_com();

    ModelPtr model_;
    std::unique_ptr<Impl> impl_;
    bool updated_{false};
    bool com_ready_{false};
};

}  // namespace kstone
