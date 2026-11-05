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
/// Omni pose tracking: axis_phase = phase-synchronised (time fallback), axis_time = time-synchronised.
/// straight (differential only): align -> straight drive -> final_align. Other kinematics/mode pairs are rejected.
enum class ChassisPlannerMode { axis_phase, axis_time, straight };
enum class DiffPhase { align, drive, final_align, path_error };
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
    path_error,        ///< Differential straight segment ended outside the position tolerance.
};

KSTONE_EXPORT const char* to_string(ChassisKinematics value) noexcept;
KSTONE_EXPORT const char* to_string(ChassisState value) noexcept;
KSTONE_EXPORT const char* to_string(ChassisPlannerMode value) noexcept;
KSTONE_EXPORT const char* to_string(DiffPhase value) noexcept;
KSTONE_EXPORT const char* to_string(PoseSource value) noexcept;
KSTONE_EXPORT const char* to_string(ChassisReason value) noexcept;

struct ChassisConfig {
    ChassisKinematics kinematics{ChassisKinematics::omni};
    double target_translation_deadband{0.0};  ///< Differential: goal moves below this keep the current path.
    double target_rotation_deadband{0.0};
    double odom_timeout_s{0.25};
    double command_timeout_s{0.25};
    double max_dt_s{0.1};
    int allowed_missed_cycles{2};
    double max_linear{0.3};
    double max_angular{0.5};
    double max_linear_acceleration{0.75};
    double max_linear_deceleration{0.75};
    double max_linear_reversal_deceleration{0.75};
    double max_angular_acceleration{1.5};
    double max_angular_deceleration{1.5};
    double max_angular_reversal_deceleration{1.5};
    double max_linear_jerk{3.0};
    double max_angular_jerk{6.0};
    double arrival_position_tolerance{0.01};
    double arrival_rotation_tolerance{0.02};
    std::string base_frame{"base_link"};

    KSTONE_EXPORT Status validate() const;
};

/// Runtime-tunable pose-tracking limits; 0 disables a bound. Velocity mode uses the config maxima.
struct ChassisPoseLimits {
    double linear_velocity{0.0};
    double linear_acceleration{0.0};
    double linear_deceleration{0.0};
    double linear_reversal_deceleration{0.0};
    double linear_jerk{0.0};
    double angular_velocity{0.0};
    double angular_acceleration{0.0};
    double angular_deceleration{0.0};
    double angular_reversal_deceleration{0.0};
    double angular_jerk{0.0};
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
/// Ruckig pose tracking (omni: synchronised x/y/yaw; differential: align -> drive -> final_align),
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
    /// configured -> active; requires a scheduler period.
    Status start(double now);
    /// active -> paused; clears motion.
    Status pause(double now);
    /// paused | fault -> active.
    Status resynchronize(double now);
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
    DiffPhase diff_phase() const noexcept;
    double maximum_scheduler_dt() const noexcept;

  private:
    struct Impl;
    explicit ChassisController(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

}  // namespace kstone
