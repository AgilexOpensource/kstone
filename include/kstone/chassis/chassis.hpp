#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"

#include <Eigen/Geometry>

#include <memory>
#include <optional>
#include <string>

namespace kstone {

/// Planar pose of the control center in the odometry frame.
struct Pose2D {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

/// Body-frame planar velocity at the control center.
struct Twist2D {
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
};

enum class ChassisKinematics { omni, differential };
enum class ChassisState { configured, active, paused, fault, stopped };
/// Pose tracking: axis_phase = phase-synchronised (time fallback), axis_time = time-synchronised.
enum class ChassisPlannerMode { axis_phase, axis_time };
enum class PoseSource { internal, odometry };
enum class ChassisReason {
    not_started,
    ready,
    paused,
    resynchronized,
    stopped,
    solve_paused,
    command_timeout,
    scheduler_discontinuity,
    tracking,
    trajectory_error,  ///< Ruckig failed; see ChassisController::last_error().
    yielded,  ///< The odometry left the pose path (another source moved the base); goal taken as done.
};

KSTONE_EXPORT const char* to_string(ChassisKinematics value) noexcept;
KSTONE_EXPORT const char* to_string(ChassisState value) noexcept;
KSTONE_EXPORT const char* to_string(ChassisPlannerMode value) noexcept;
KSTONE_EXPORT const char* to_string(PoseSource value) noexcept;
KSTONE_EXPORT const char* to_string(ChassisReason value) noexcept;

struct ChassisConfig {
    ChassisKinematics kinematics{ChassisKinematics::omni};
    double odom_timeout_s{
        0.25};  ///< 0 = odometry never goes stale. The chassis module uses the engine feedback_timeout_s.
    double command_timeout_s{0.25};
    double max_dt_s{0.1};
    int allowed_missed_cycles{2};
    double max_linear_velocity{0.3};
    double max_angular_velocity{0.5};
    double max_linear_acceleration{0.75};
    double max_linear_deceleration{0.75};
    double max_angular_acceleration{1.5};
    double max_angular_deceleration{1.5};
    /// Omni body-y limits; 0 = use the matching max_linear_* value. Translation is then bounded by the
    /// ellipse (vx/x_limit)^2 + (vy/y_limit)^2 <= 1, which is the planar norm when both are equal.
    double max_lateral_velocity{0.0};
    double max_lateral_acceleration{0.0};
    double max_lateral_deceleration{0.0};
    double position_tolerance{0.01};
    double orientation_tolerance{0.02};
    std::string base_frame{"base_link"};

    KSTONE_EXPORT Status validate() const;
};

/// Runtime-tunable pose-tracking limits; 0 disables a bound. Velocity mode uses the config velocity maxima.
struct ChassisPoseLimits {
    double max_linear_velocity{0.0};
    double max_linear_acceleration{0.0};
    double max_linear_deceleration{0.0};
    double max_lateral_velocity{0.0};  ///< Body y; 0 = use the matching linear bound.
    double max_lateral_acceleration{0.0};
    double max_lateral_deceleration{0.0};
    double max_angular_velocity{0.0};
    double max_angular_acceleration{0.0};
    double max_angular_deceleration{0.0};
    ChassisPlannerMode planner_mode{ChassisPlannerMode::axis_phase};

    KSTONE_EXPORT Status validate() const;
    /// True when every bound is 0: pose mode then emits the nominal (unshaped) twist.
    KSTONE_EXPORT bool bypass() const noexcept;
};

struct ChassisOutput {
    Twist2D velocity;
    Pose2D pose;
    double timestamp{0.0};
    ChassisState state{ChassisState::configured};
    PoseSource source{PoseSource::internal};
    ChassisReason reason{ChassisReason::not_started};
};

struct ChassisCommandSnapshot {
    bool pose_mode{false};
    Twist2D velocity;
    Pose2D pose;
    double timestamp{0.0};
    double expires_at{0.0};
    std::string source;
    int priority{0};
};

/// Wraps to [-pi, pi).
KSTONE_EXPORT double wrap_angle(double angle) noexcept;
/// Exact SE(2) integration of a constant body twist over dt.
KSTONE_EXPORT Pose2D integrate_se2(const Pose2D& pose, const Twist2D& twist, double dt) noexcept;
/// World pose of the model root for Model::set_root_pose: center pose composed with the inverse of
/// the control center's fixed pose in the root frame.
KSTONE_EXPORT Eigen::Isometry3d chassis_root_pose(
    const Pose2D& center, const Eigen::Isometry3d& center_in_root = Eigen::Isometry3d::Identity());

/// Planar base controller outside the QP: velocity/pose command leases with priority arbitration,
/// Ruckig pose tracking (synchronised x/y/yaw; differential: the position error along the current heading only),
/// dead reckoning and odometry override. Every method takes the controller clock `now` explicitly.
/// Lock the root velocity coordinates in SolverOptions::locked and write pose() into q each cycle.
class KSTONE_EXPORT ChassisController {
  public:
    /// invalid_argument if the config is invalid. Pose limits start at the config maxima.
    static Result<ChassisController> create(ChassisConfig config = {});

    ~ChassisController();
    ChassisController(ChassisController&&) noexcept;
    ChassisController& operator=(ChassisController&&) noexcept;
    ChassisController(const ChassisController&) = delete;
    ChassisController& operator=(const ChassisController&) = delete;

    const ChassisConfig& config() const noexcept;
    const ChassisPoseLimits& pose_limits() const noexcept;
    Status tune_pose_limits(const ChassisPoseLimits& limits);

    /// While active, a changed period takes effect on the next step (which then emits no new output).
    Status set_scheduler_period(double period);
    std::optional<double> scheduler_period() const noexcept;
    /// configured -> active; requires a scheduler period. `pose` (finite) replaces the initial pose.
    Status start(double now, const std::optional<Pose2D>& pose = std::nullopt);
    /// active -> paused; clears motion.
    Status pause(double now);
    /// paused | fault -> active; `pose` (finite) replaces the current pose, e.g. the measured one.
    Status resynchronize(double now, const std::optional<Pose2D>& pose = std::nullopt);
    /// Any state -> stopped (terminal).
    void stop(double now);

    /// invalid_argument on non-finite input, empty source or negative priority. false when not active
    /// or when a fresh command with higher priority holds the lease.
    Result<bool> submit_velocity(const Twist2D& twist, double now, const std::string& source = "external",
                                 int priority = 10);
    Result<bool> submit_pose(const Pose2D& pose, double now, const std::string& source = "external_pose",
                             int priority = 10);
    /// Clears the command unless its priority exceeds max_priority.
    bool clear_velocity(std::optional<int> max_priority = std::nullopt);
    /// false when stopped, stale, in the future or not newer than the last accepted stamp.
    Result<bool> submit_odometry(const Pose2D& pose, double stamp, double now);

    /// Lease bookkeeping between steps: solve_enabled = false zeroes motion and resets the step clock.
    ChassisOutput service(double now, bool solve_enabled);
    /// One control cycle. A no-op snapshot before start(); faults on scheduler discontinuity.
    ChassisOutput step(double now);
    ChassisOutput snapshot(double now) const;
    ChassisCommandSnapshot command_snapshot() const;

    ChassisState state() const noexcept;
    ChassisReason reason() const noexcept;
    /// Detail of the last trajectory_error; empty otherwise.
    const std::string& last_error() const noexcept;
    const Pose2D& pose() const noexcept;
    const Pose2D& ramp_pose() const noexcept;
    const Twist2D& output() const noexcept;
    const Twist2D& nominal_target() const noexcept;
    double maximum_scheduler_dt() const noexcept;

  private:
    struct Impl;
    explicit ChassisController(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

}  // namespace kstone
