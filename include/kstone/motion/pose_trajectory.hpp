#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"

#include <Eigen/Geometry>

#include <memory>

namespace kstone {

/// Zero means "not set": velocity 0 passes the target through unshaped; acceleration 0 gives a constant-speed
/// profile; deceleration 0 uses the acceleration, reversal_deceleration 0 the deceleration; jerk 0 a trapezoid.
struct MotionLimits {
    double velocity{0.0};
    double acceleration{0.0};
    double deceleration{0.0};
    double reversal_deceleration{0.0};
    double jerk{0.0};

    KSTONE_EXPORT bool enabled() const noexcept;
    KSTONE_EXPORT Status validate() const;
};

/// axis_phase: phase-synchronised axes (time fallback); axis_time: time-synchronised axes.
enum class PoseSync { axis_phase, axis_time };
KSTONE_EXPORT const char* to_string(PoseSync value) noexcept;

struct PoseTrajectoryConfig {
    PoseSync sync{PoseSync::axis_phase};
    MotionLimits translation;  ///< World axes, per-axis velocity.
    MotionLimits rotation;     ///< Error-aligned axes.
    /// Dimensionless: the ramp is kept at most lead_speed_scale * translation.velocity * lead_period ahead of the
    /// observed frame on each world axis where it leads in the direction still to go, so it does not run on while
    /// the frame is blocked. 0 = free.
    double lead_speed_scale{0.0};
    static constexpr double lead_period =
        0.02;  ///< s; fixed so the lead distance does not depend on the loop rate.

    KSTONE_EXPORT Status validate() const;
};

/// Jerk-limited Cartesian reference for one frame (Ruckig), independent of any task: feed ramp_pose() to
/// FrameTask::set_target. A plan is cached and resampled until the target moves. No allocation after create().
class KSTONE_EXPORT PoseTrajectory {
  public:
    static Result<PoseTrajectory> create(PoseTrajectoryConfig config = {});

    ~PoseTrajectory();
    PoseTrajectory(PoseTrajectory&&) noexcept;
    PoseTrajectory& operator=(PoseTrajectory&&) noexcept;
    PoseTrajectory(const PoseTrajectory&) = delete;
    PoseTrajectory& operator=(const PoseTrajectory&) = delete;

    const PoseTrajectoryConfig& config() const noexcept;
    /// Change limits or sync at runtime; plans are rebuilt from the current motion state. A sync change zeroes
    /// the motion state but keeps the ramp pose.
    Status tune(const PoseTrajectoryConfig& config);
    /// Forget everything; the next step starts at rest from the observed pose.
    void reset() noexcept;
    /// Start at rest from `pose`.
    Status reset(const Eigen::Isometry3d& pose);

    /// Advance one cycle toward `target`; dt == 0 holds the ramp. invalid_argument on a non-rigid pose or dt < 0.
    /// A Ruckig rejection is handled internally by a braking step, never an error.
    Status step(const Eigen::Isometry3d& observed, const Eigen::Isometry3d& target, double dt);

    bool initialized() const noexcept;
    const Eigen::Isometry3d& ramp_pose() const noexcept;
    const Eigen::Vector3d& linear_velocity() const noexcept;
    const Eigen::Vector3d& angular_velocity() const noexcept;
    /// All velocities and accelerations below 1e-9.
    bool stopped() const noexcept;

  private:
    struct Impl;
    explicit PoseTrajectory(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

}  // namespace kstone
