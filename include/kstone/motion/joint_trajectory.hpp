#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"

#include <Eigen/Core>

#include <memory>
#include <vector>

namespace kstone {

/// direct: no shaping, the reference is the target. threshold: shape only moves whose time at the velocity limit
/// exceeds `threshold`, let go within half a cycle of the target. continuous: always shape.
enum class JointTrigger { direct, threshold, continuous };
/// Ruckig synchronisation; phase falls back to time when not collinear, both to none when unsynchronisable.
enum class JointSync { time, phase, none };

KSTONE_EXPORT const char* to_string(JointTrigger value) noexcept;
KSTONE_EXPORT const char* to_string(JointSync value) noexcept;

struct JointTrajectoryConfig {
    JointTrigger trigger{JointTrigger::continuous};
    JointSync sync{JointSync::phase};
    double threshold{0.1};            ///< s at the velocity limit; threshold trigger only, >= 0.
    double acceleration_factor{2.0};  ///< max acceleration = factor * velocity limit, > 0.
    double jerk_factor{10.0};         ///< max jerk = factor * max acceleration, > 0.
    double velocity_scale{1.0};       ///< (0, 1]: lowers the cruise speed only; braking keeps full limits.
    Eigen::VectorXd velocity_cap;  ///< Empty, or one entry > 0 per joint (inf = none); caps the cruise speed.

    KSTONE_EXPORT Status validate(Eigen::Index size) const;
};

/// Jerk-limited reference for a set of scalar joints (Ruckig), independent of any task: feed reference() to
/// JointTask::set_target. Each step() replans from the previous reference state, so target and limit changes
/// take effect on the next cycle. No allocation after create() on the success path.
class KSTONE_EXPORT JointTrajectory {
  public:
    /// `wrap[i]` marks an unbounded angle (continuous joint): it moves along the shortest arc to the target.
    /// Empty wrap = none. invalid_argument on a bad size or config.
    static Result<JointTrajectory> create(Eigen::Index size, JointTrajectoryConfig config = {},
                                          std::vector<bool> wrap = {});

    ~JointTrajectory();
    JointTrajectory(JointTrajectory&&) noexcept;
    JointTrajectory& operator=(JointTrajectory&&) noexcept;
    JointTrajectory(const JointTrajectory&) = delete;
    JointTrajectory& operator=(const JointTrajectory&) = delete;

    /// Changing anything but the threshold restarts the profile from the measured state on the next step.
    Status configure(const JointTrajectoryConfig& config);
    const JointTrajectoryConfig& config() const noexcept;
    /// Drop the profile; the next shaped step starts from the measured state.
    void reset() noexcept;

    /// Advance one cycle. `measured` and `measured_velocity` seed a new profile; `velocity_limit` must be finite
    /// and > 0 per joint (a change restarts the profile). internal on a Ruckig failure.
    Status step(const Eigen::Ref<const Eigen::VectorXd>& measured,
                const Eigen::Ref<const Eigen::VectorXd>& target,
                const Eigen::Ref<const Eigen::VectorXd>& measured_velocity,
                const Eigen::Ref<const Eigen::VectorXd>& velocity_limit, double dt);

    Eigen::Index size() const noexcept;
    /// True while a profile is shaping the reference.
    bool active() const noexcept;
    /// Largest |error| / velocity limit seen by the last step, in seconds.
    double tau() const noexcept;
    /// Shaped position while active, otherwise the last target.
    const Eigen::VectorXd& reference() const noexcept;
    /// Profile velocity and acceleration (zero when inactive).
    const Eigen::VectorXd& velocity() const noexcept;
    const Eigen::VectorXd& acceleration() const noexcept;

  private:
    struct Impl;
    explicit JointTrajectory(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

}  // namespace kstone
