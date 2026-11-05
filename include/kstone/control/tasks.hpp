#pragma once

#include "kstone/control/task.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

namespace kstone {

/// Drive a frame pose to a target (6 rows: linear, angular). The error is log6(current^-1 target),
/// expressed in the frame (default) or rotated into world axes. With a reference frame the pose of
/// the frame relative to the reference is controlled instead.
class KSTONE_EXPORT FrameTask final : public Task {
  public:
    FrameTask(std::string frame, const Eigen::Isometry3d& target);

    /// The rotation part must be orthonormal with determinant 1.
    Status set_target(const Eigen::Isometry3d& target);
    const Eigen::Isometry3d& target() const noexcept { return target_; }
    void set_world_axes(bool world) noexcept { world_ = world; }
    /// Empty name controls the pose in the world.
    void set_reference(std::string frame) {
        reference_name_ = std::move(frame);
        reference_dirty_ = true;
    }
    const std::string& frame() const noexcept { return frame_name_; }

  private:
    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) override;

    std::string frame_name_, reference_name_;
    FrameId frame_{0}, reference_{0};
    bool has_reference_{false}, reference_dirty_{false}, world_{false};
    Eigen::Isometry3d target_;
    Eigen::MatrixXd tip_jac_, ref_jac_;
};

/// Drive scalar joints (nv == 1) to target positions; continuous joints wrap the error to (-pi, pi].
class KSTONE_EXPORT JointTask final : public Task {
  public:
    JointTask(std::vector<std::string> joints, Eigen::VectorXd target);

    Status set_target(const Eigen::Ref<const Eigen::VectorXd>& target);
    const Eigen::VectorXd& target() const noexcept { return target_; }

  private:
    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) override;

    std::vector<std::string> names_;
    std::vector<JointId> joints_;
    Eigen::VectorXd target_;
};

/// Drive the selected scalar joints toward a full configuration (size nq): e = (q - target)[joints].
/// An empty joint list selects every scalar joint.
class KSTONE_EXPORT PostureTask final : public Task {
  public:
    explicit PostureTask(Eigen::VectorXd target, std::vector<std::string> joints = {});

    Status set_target(const Eigen::Ref<const Eigen::VectorXd>& target);
    const Eigen::VectorXd& target() const noexcept { return target_; }
    /// Velocity indices of the selected joints; valid after bind().
    const std::vector<Index>& indices() const noexcept { return indices_; }

  private:
    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) override;

    std::vector<std::string> names_;
    std::vector<Index> indices_;
    Eigen::VectorXd target_, diff_;
};

/// Point a frame axis at a world point (3 rows). The ray starts at the frame origin plus a
/// frame-fixed offset; closer than min_distance the task is inert.
class KSTONE_EXPORT GazeTask final : public Task {
  public:
    GazeTask(std::string frame, const Eigen::Vector3d& focus);

    Status set_focus(const Eigen::Vector3d& focus);
    const Eigen::Vector3d& focus() const noexcept { return focus_; }
    Status set_axis(const Eigen::Vector3d& axis);
    Status set_offset(const Eigen::Vector3d& offset);
    Status set_min_distance(double distance);

  private:
    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) override;

    std::string frame_name_;
    FrameId frame_{0};
    Eigen::Vector3d focus_, axis_{Eigen::Vector3d::UnitX()}, offset_{Eigen::Vector3d::Zero()};
    double min_distance_{1e-4};
    Eigen::MatrixXd jac_, linear_;
};

/// Increase the manipulability f = 0.5 log det(S S^T + r^2 I) of a frame (1 row), where S holds
/// the selected local Jacobian axes and joint columns. The gradient is a central finite difference.
class KSTONE_EXPORT ManipulabilityTask final : public Task {
  public:
    /// axes: 6 entries in {0, 1} selecting (linear; angular) rows. Empty joints selects all scalar joints.
    ManipulabilityTask(std::string frame, std::vector<std::string> joints = {},
                       Eigen::Matrix<double, 6, 1> axes = Eigen::Matrix<double, 6, 1>::Ones());
    ~ManipulabilityTask() override;

    /// Requested increase of f per second, > 0.
    Status set_rate(double rate);
    double rate() const noexcept { return rate_; }
    Status set_regularization(double r);

    /// f and its gradient (size nv, zero outside the selected joints) at q. Requires bind().
    Status measure(const Eigen::Ref<const Eigen::VectorXd>& q, double& value, Eigen::VectorXd* gradient);
    const std::vector<Index>& indices() const noexcept { return indices_; }

  private:
    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) override;
    Status value_at(const Eigen::Ref<const Eigen::VectorXd>& q, double& value);

    std::string frame_name_;
    std::vector<std::string> names_;
    FrameId frame_{0};
    std::vector<Index> indices_;
    Eigen::Matrix<double, 6, 1> axes_;
    double rate_{0.1}, regularization_{0.01};
    std::unique_ptr<Workspace> probe_;
    Eigen::MatrixXd jac_, selected_, gram_;
    Eigen::LLT<Eigen::MatrixXd> llt_;
    Eigen::VectorXd step_, shifted_, gradient_, q0_;
};

/// Common settings of tasks projected into the null space of a primary task.
struct NullSpaceSettings {
    double damping{1e-6};
    double threshold{0.0};  ///< > 0 enables adaptive damping.
};

/// Pull scalar joints toward a configuration without disturbing the primary task.
class KSTONE_EXPORT NullSpacePostureTask final : public Task {
  public:
    NullSpacePostureTask(std::shared_ptr<Task> primary, Eigen::VectorXd target,
                         std::vector<std::string> joints = {});

    Status set_target(const Eigen::Ref<const Eigen::VectorXd>& target);
    const Eigen::VectorXd& target() const noexcept { return target_; }
    Status set_settings(NullSpaceSettings settings);
    /// The projected pull is scaled so no joint exceeds scale * velocity limit. scale in (0, 1].
    Status set_velocity_scale(double scale);
    /// Per selected joint; non-finite entries fall back to the model limit.
    Status set_velocity_limit(const Eigen::Ref<const Eigen::VectorXd>& limit);

  private:
    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) override;

    std::shared_ptr<Task> primary_;
    std::vector<std::string> names_;
    std::vector<Index> indices_;
    Eigen::VectorXd target_, diff_, limit_, selected_, projected_;
    NullSpaceSettings settings_;
    double velocity_scale_{0.8};
    Eigen::MatrixXd projector_;
    NullSpaceScratch scratch_;
};

/// Push scalar joints away from their position limits inside the primary null space. Rows are
/// active within `band` of a limit; the weight scales with the strongest activation.
class KSTONE_EXPORT NullSpaceJointLimitTask final : public Task {
  public:
    NullSpaceJointLimitTask(std::shared_ptr<Task> primary, std::vector<std::string> joints = {});

    Status set_settings(NullSpaceSettings settings);
    Status set_band(double band);
    /// Push speed in rad/s (or m/s) at full activation, > 0.
    Status set_rate(double rate);
    /// Override limits per selected joint; non-finite entries keep the model limit.
    Status set_limits(const Eigen::Ref<const Eigen::VectorXd>& lower,
                      const Eigen::Ref<const Eigen::VectorXd>& upper);
    /// Largest activation in [0, 1] at the last evaluation.
    double activation() const noexcept { return activation_; }

  private:
    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) override;

    std::shared_ptr<Task> primary_;
    std::vector<std::string> names_;
    std::vector<JointId> joints_;
    std::vector<Index> indices_;
    Eigen::VectorXd lower_, upper_, push_;
    NullSpaceSettings settings_;
    double band_{0.2}, rate_{1.0}, activation_{0.0};
    Eigen::MatrixXd projector_;
    NullSpaceScratch scratch_;
};

/// Climb the manipulability of `preference` inside the primary null space.
class KSTONE_EXPORT NullSpaceManipulabilityTask final : public Task {
  public:
    NullSpaceManipulabilityTask(std::shared_ptr<Task> primary,
                                std::shared_ptr<ManipulabilityTask> preference);

    Status set_settings(NullSpaceSettings settings);

  private:
    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) override;

    std::shared_ptr<Task> primary_;
    std::shared_ptr<ManipulabilityTask> preference_;
    NullSpaceSettings settings_;
    Eigen::MatrixXd projector_;
    Eigen::VectorXd gradient_, selected_, ascent_;
    NullSpaceScratch scratch_;
};

/// Rows supplied by the caller each step (J is rows x nv, e has rows entries). Empty means inert.
class KSTONE_EXPORT ExternalTask final : public Task {
  public:
    ExternalTask();

    Status set(const Eigen::Ref<const Eigen::MatrixXd>& J, const Eigen::Ref<const Eigen::VectorXd>& e);
    void clear();

  private:
    Status on_bind(const Model& model) override;
    Status compute(const Tick& tick, Eigen::Ref<Eigen::MatrixXd> J, Eigen::Ref<Eigen::VectorXd> e) override;

    Eigen::MatrixXd J_;
    Eigen::VectorXd e_;
};

}  // namespace kstone
