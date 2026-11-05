#pragma once

#include "kstone/chassis/chassis.hpp"
#include "kstone/collision/constraint.hpp"
#include "kstone/control/constraints.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/module/homing.hpp"
#include "kstone/module/module.hpp"
#include "kstone/motion/joint_trajectory.hpp"
#include "kstone/motion/pose_trajectory.hpp"

#include <Eigen/Geometry>

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace kstone {

// Setters on these modules are control-thread only (call them between ModuleSet::step calls).

struct FrameTaskModuleConfig {
    std::string frame;
    /// Driven joints; empty = every joint from the root down to the frame. Non-empty also masks the task.
    std::vector<std::string> joints;
    std::string reference;  ///< Empty = world.
    bool world_axes{false};
    double weight{1.0};
    /// Per-axis weight factors (x, y, z) in the task axes (frame, or world with world_axes); 0 frees an axis.
    Eigen::Vector3d position_weight{Eigen::Vector3d::Ones()};
    Eigen::Vector3d orientation_weight{Eigen::Vector3d::Ones()};
    double gain{1.0};
    double lm_damping{0.0};  ///< Task::set_lm_damping.
    TaskOptions task{0};
    PoseTrajectoryConfig trajectory;  ///< Limits must be set (> 0).
    /// In the reference frame. Empty = pose at ModuleSetup::initial(), kept relative to the root when there is no
    /// reference, so it follows a moving base.
    std::optional<Eigen::Isometry3d> home;
    HomeConfig home_motion;
    double position_tolerance{1e-4};     ///< Home arrival (m).
    double orientation_tolerance{1e-3};  ///< Home arrival (rad).
    /// When held by another module: abort (default) drops the target and holds the pose observed on
    /// resume; wait keeps it and chases it again. An executing yielding home always aborts.
    OnHeld on_held{OnHeld::abort};
    /// Freeze the driven joints (requires explicit joints). Unlocking holds the current pose and drops any
    /// target written before or during the lock. Homing overrides it.
    bool locked{false};
};

/// FrameTask fed by a PoseTrajectory. Without a target it holds the pose observed at reset.
class KSTONE_EXPORT FrameTaskModule final : public Module {
  public:
    FrameTaskModule(std::string id, FrameTaskModuleConfig config);
    const char* type() const noexcept override { return "frame_task"; }

    /// The rotation part must be orthonormal with determinant 1.
    Status set_target(const Eigen::Isometry3d& target);
    const Eigen::Isometry3d& target() const noexcept { return target_; }
    bool has_target() const noexcept { return has_target_; }
    const std::shared_ptr<FrameTask>& task() const noexcept { return task_; }
    const PoseTrajectory* trajectory() const noexcept { return trajectory_ ? &*trajectory_ : nullptr; }
    /// Valid after configure().
    FrameId frame_id() const noexcept { return frame_; }
    const Mask& driven() const noexcept { return driven_; }
    const TaskOptions& task_options() const noexcept { return config_.task; }

    /// Port "target" (pose): set_target().
    Status read_port(std::size_t index, PortValue& out) const override;
    Status write_port(std::size_t index, const PortValue& value) override;
    Status request_home() override;
    bool homeable() const noexcept override { return true; }
    HomeStatus home_status() const noexcept { return home_.status(); }
    const char* home_state() const noexcept override { return to_string(home_.status()); }
    /// trajectory and home_motion are rejected while homing.
    Status tune(const ConfigNode& params) override;
    ConfigNode tunables() const override;
    ConfigNode tune_schema() const override;
    /// Outputs: planned (point, world ramp position), lag (m, target to ramp), speed (m/s), angular_speed (rad/s),
    /// error (m) and angular_error (rad) from the target to the frame at the evaluated q.
    Status update_outputs(Workspace& at) override;
    Status read_output(std::size_t index, OutputValue& out) const override;

    Status configure(ModuleSetup& setup) override;
    Status prepare(Cycle& cycle) override;
    Claim claim() const override;
    Status reset(const Cycle& cycle) override;
    Status contribute(const Cycle& cycle) override;
    Status on_preempted(const Cycle& cycle, const std::string& by) override;
    Status on_disabled(const Cycle& cycle) override;

  private:
    Status observe(const Workspace& workspace, Eigen::Isometry3d& pose) const;
    Status finish(HomeStatus status);

    FrameTaskModuleConfig config_, defaults_;
    Eigen::Vector3d planned_{Eigen::Vector3d::Zero()};
    double lag_{0.0}, speed_{0.0}, angular_speed_{0.0}, error_{0.0}, angular_error_{0.0};
    std::shared_ptr<FrameTask> task_;
    std::optional<PoseTrajectory> trajectory_;
    PoseTrajectoryConfig home_trajectory_;
    FrameId frame_{0}, reference_{0};
    bool has_reference_{false};
    Mask driven_;
    Eigen::Isometry3d target_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d home_pose_{Eigen::Isometry3d::Identity()};
    bool has_target_{false};
    bool home_in_root_{false};
    bool lock_applied_{false};
    TaskHandle handle_;
    Homing home_;
    const Model* model_{nullptr};
};

struct JointTargetModuleConfig {
    std::vector<std::string> joints;  ///< Scalar joints (nv == 1).
    double weight{1.0};
    double gain{1.0};
    TaskOptions task;  ///< Unset level = 0, so joint control outranks pose tasks.
    JointTrajectoryConfig trajectory;
    /// Per-joint profile velocity limit; empty = model limits (each must then be finite and > 0).
    Eigen::VectorXd velocity_limit;
    double tolerance{
        1e-3};  ///< Arrival (target and home): |target - q| below this per joint once the profile ends.
    int priority{0};
    /// When held by another module: wait keeps the target and resumes it; abort drops it (stops claiming,
    /// idle until the next set_target). An executing yielding home always aborts.
    OnHeld on_held{OnHeld::wait};
    /// Idle (neither claiming nor homing): true holds the last target; false drops the task so others move the joints.
    /// After yielding to new input elsewhere it never holds.
    bool hold_idle{true};
    /// Idle: also hold the last target and claim the joints, so conflicting modules stay held (direct joint
    /// control). Resuming idle (e.g. on enable) starts from the measured joints.
    bool claim_idle{false};
    /// Input watchdog: a request for these joints within this long (s) of the last set_target() or finished home
    /// is deferred, and the joints are released once that long passes without new input. 0 = yield at once.
    double release_after_s{0.25};
    Eigen::VectorXd home;    ///< Per joint; empty = positions at ModuleSetup::initial().
    HomeConfig home_motion;  ///< Home runs continuous at velocity_scale * velocity limit.
};

/// JointTask fed by a JointTrajectory. Claims its joints from set_target() or a finished home, also after arrival,
/// until a module driving them gets new input (a target or home request) and release_after_s has passed since its
/// own last input; its own input in the same cycle always wins.
class KSTONE_EXPORT JointTargetModule final : public Module {
  public:
    JointTargetModule(std::string id, JointTargetModuleConfig config);
    const char* type() const noexcept override { return "joint_target"; }

    /// Clamps each bounded joint to its position limits (the port's lower/upper).
    Status set_target(const Eigen::Ref<const Eigen::VectorXd>& target);
    const Eigen::VectorXd& target() const noexcept { return target_; }
    bool claiming() const noexcept { return claiming_; }
    /// The last target was reached (claim kept).
    bool settled() const noexcept { return settled_; }
    const std::shared_ptr<JointTask>& task() const noexcept { return task_; }

    /// Port "target" (vector, one element per joint): set_target().
    Status read_port(std::size_t index, PortValue& out) const override;
    Status write_port(std::size_t index, const PortValue& value) override;
    Status request_home() override;
    bool homeable() const noexcept override { return true; }
    HomeStatus home_status() const noexcept { return home_.status(); }
    const char* home_state() const noexcept override { return to_string(home_.status()); }
    /// trajectory and home_motion are rejected while homing.
    Status tune(const ConfigNode& params) override;
    /// Also reports the resolved per-joint "home" (read-only).
    ConfigNode tunables() const override;
    ConfigNode tune_schema() const override;

    Status configure(ModuleSetup& setup) override;
    Status prepare(Cycle& cycle) override;
    Claim claim() const override;
    Status reset(const Cycle& cycle) override;
    Status contribute(const Cycle& cycle) override;
    Status on_preempted(const Cycle& cycle, const std::string& by) override;
    Status on_disabled(const Cycle& cycle) override;
    bool yield_control() override;

  private:
    bool holds_claim() const noexcept;
    void measure(const Cycle& cycle);
    bool arrived(const Eigen::VectorXd& goal) const;
    Status finish(HomeStatus status);
    Status step_home(const Cycle& cycle);

    JointTargetModuleConfig config_, defaults_;
    std::shared_ptr<JointTask> task_;
    TaskHandle handle_;
    std::optional<JointTrajectory> trajectory_;
    JointTrajectoryConfig home_trajectory_;
    std::vector<JointInfo> joints_;
    std::vector<bool> wrap_;
    Mask mask_;
    Eigen::VectorXd target_, measured_, measured_v_, limit_, home_pose_;
    bool has_target_{false};
    bool claiming_{false};
    bool settled_{false};
    bool yielded_{false};  ///< Gave the joints away; idle without holding until the next set_target().
    bool fresh_{false}, fresh_now_{false};
    bool release_pending_{false};  ///< A request arrived inside the watchdog window.
    double now_{0.0}, input_at_{-std::numeric_limits<double>::infinity()};
    Homing home_;
    const Model* model_{nullptr};
};

struct GazeModuleConfig {
    std::string frame;                ///< Camera frame.
    std::vector<std::string> joints;  ///< Scalar joints that aim it.
    /// Focus on the mean world position of the active ones; whatever drives them (pose or joint control).
    std::vector<std::string> frames;
    Eigen::Vector3d offset{Eigen::Vector3d::Zero()};  ///< Optical centre in the camera frame.
    Eigen::Vector3d optical_axis{Eigen::Vector3d::UnitX()};
    double weight{1.0};
    double gain{1.0};
    TaskOptions task;
    double max_velocity{0.0};  ///< Joint speed limit while aiming; 0 = none.
    /// A frame counts only within this long after it last moved relative to the model root (startup counts as a
    /// move); <= 0 = always.
    double target_idle_s{0.0};
};

/// Points a camera at the active frames; with none active, returns its joints to the initial positions.
/// Motion is measured on the solved pose relative to the model root, one cycle late, so driving the base does not
/// count. Claims its joints (priority::yielding) whenever it has frames, so any joint target claim wins; each cycle
/// a frame moves it requests them back (request_control()), which a joint target grants per its release_after_s.
/// Outputs: focus (NaN while homing), active_frames.
class KSTONE_EXPORT GazeModule final : public Module {
  public:
    GazeModule(std::string id, GazeModuleConfig config);
    const char* type() const noexcept override { return "gaze"; }

    const std::shared_ptr<GazeTask>& task() const noexcept { return task_; }
    /// Number of frames in the focus at the last step.
    std::size_t active_frames() const noexcept { return active_count_; }
    /// After configure(). See the module's schema for the tunable keys.
    Status tune(const ConfigNode& params) override;
    ConfigNode tunables() const override;
    ConfigNode tune_schema() const override;

    Status configure(ModuleSetup& setup) override;
    Status prepare(Cycle& cycle) override;
    Claim claim() const override;
    Status contribute(const Cycle& cycle) override;
    Status read_output(std::size_t index, OutputValue& out) const override;

  private:
    GazeModuleConfig config_, defaults_;
    std::shared_ptr<GazeTask> task_;
    std::shared_ptr<JointTask> home_task_;
    std::shared_ptr<JointVelocityConstraint> velocity_;
    TaskHandle aim_handle_, home_handle_;
    std::vector<JointInfo> joints_;
    Mask mask_;
    Eigen::VectorXd home_;
    std::vector<FrameId> frame_ids_;
    std::vector<Eigen::Isometry3d> anchor_;  ///< Root-relative pose at the last move.
    std::vector<double> moved_at_;
    std::vector<char> seen_, active_;
    std::size_t active_count_{0};
    bool free_root_{false};
    Eigen::Vector3d focus_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
};

struct CenterOfMassModuleConfig {
    std::string frame;  ///< Support polygon frame.
    std::vector<Eigen::Vector2d> polygon;
    std::string subtree;  ///< Empty = whole model.
    double margin{0.0};
    double gain{0.5};
    double recovery_speed{0.1};
    std::vector<Payload> payloads;
    /// Outside the margin only these modules' driven joints pull the CoM back (others may not push it out); also dependencies. Empty = all joints.
    std::vector<std::shared_ptr<FrameTaskModule>> recovery_tasks;
    /// Also bound the measured state (fresh feedback), on top of the commanded one.
    bool closed_loop{false};
};

/// One CenterOfMassConstraint; drives and claims nothing.
/// Outputs: com, ground (com on the polygon plane), polygon, margin_polygon, min_slack (m), inside (slack >= margin).
class KSTONE_EXPORT CenterOfMassModule final : public Module {
  public:
    CenterOfMassModule(std::string id, CenterOfMassModuleConfig config);
    const char* type() const noexcept override { return "center_of_mass"; }

    std::vector<std::string> dependencies() const override;
    const std::shared_ptr<CenterOfMassConstraint>& constraint() const noexcept { return constraint_; }
    /// After configure(). See the module's schema for the tunable keys.
    Status tune(const ConfigNode& params) override;
    ConfigNode tunables() const override;
    ConfigNode tune_schema() const override;

    Status configure(ModuleSetup& setup) override;
    Status update_outputs(Workspace& at) override;
    Status read_output(std::size_t index, OutputValue& out) const override;

  private:
    CenterOfMassModuleConfig config_, defaults_;
    std::shared_ptr<CenterOfMassConstraint> constraint_;
    SupportPolygon::Evaluation display_;
    Eigen::Matrix2Xd local_;
    Eigen::Matrix3Xd points_[4];  ///< com, ground, polygon, margin_polygon.
    double min_slack_{0.0};
    bool inside_{false};
};

enum class NullSpacePreference { posture, manipulability, joint_limits };
enum class PostureReference { initial, current, neutral, custom };

struct NullSpacePreferenceModuleConfig {
    std::shared_ptr<FrameTaskModule> primary;  ///< Also a dependency.
    std::vector<std::string> joints;           ///< Scalar joints; empty = the primary's driven scalar joints.
    NullSpacePreference preference{NullSpacePreference::posture};
    NullSpaceSettings projector{1e-2, 0.1};
    TaskOptions task;  ///< Unset level = the primary's level.

    PostureReference reference{PostureReference::initial};
    Eigen::VectorXd posture;  ///< Per joint, custom reference only.
    std::vector<std::pair<std::string, double>>
        posture_values;          ///< Named joints over the reference's posture; not with current.
    bool reseed_current{false};  ///< Tune only: the current q becomes the custom posture.
    double posture_weight{1.0};
    double posture_velocity_scale{0.8};

    double manipulability_weight{1.0};
    double manipulability_rate{0.1};
    Eigen::Matrix<double, 6, 1> manipulability_axes{
        (Eigen::Matrix<double, 6, 1>() << 1, 1, 1, 0, 0, 0).finished()};

    double joint_limits_weight{10.0};
    double joint_limits_band{0.2};
    double joint_limits_rate{1.0};
};

/// One of three null-space preferences under a frame task; switchable at runtime without allocation.
class KSTONE_EXPORT NullSpacePreferenceModule final : public Module {
  public:
    NullSpacePreferenceModule(std::string id, NullSpacePreferenceModuleConfig config);
    const char* type() const noexcept override { return "null_space_preference"; }

    std::vector<std::string> dependencies() const override;
    /// False (unchanged) for joint_limits after configure() found no bounded joints.
    bool set_preference(NullSpacePreference preference) noexcept {
        if (preference == NullSpacePreference::joint_limits && posture_ && !joint_limits_) return false;
        preference_ = preference;
        return true;
    }
    NullSpacePreference preference() const noexcept { return preference_; }
    const std::shared_ptr<NullSpacePostureTask>& posture_task() const noexcept { return posture_; }
    const std::shared_ptr<NullSpaceManipulabilityTask>& manipulability_task() const noexcept {
        return manipulability_;
    }
    /// Null when none of the joints is bounded.
    const std::shared_ptr<NullSpaceJointLimitTask>& joint_limits_task() const noexcept {
        return joint_limits_;
    }
    /// After configure(). See the module's schema for the tunable keys.
    Status tune(const ConfigNode& params) override;
    ConfigNode tunables() const override;
    ConfigNode tune_schema() const override;

    Status configure(ModuleSetup& setup) override;
    Status contribute(const Cycle& cycle) override;

  private:
    Status posture_target(const NullSpacePreferenceModuleConfig& c, Eigen::VectorXd& out) const;
    void clamp_neutral(const NullSpacePreferenceModuleConfig& c, Eigen::VectorXd& out) const;
    Eigen::VectorXd seed_from_q() const;

    NullSpacePreferenceModuleConfig config_, defaults_;
    ModelPtr model_;
    std::vector<JointInfo> infos_;
    Eigen::VectorXd initial_, last_q_, neutral_, scratch_;
    const Eigen::VectorXd* lower_{nullptr};
    const Eigen::VectorXd* upper_{nullptr};
    std::uint64_t limits_seen_{~std::uint64_t{0}};
    bool have_q_{false};
    std::shared_ptr<NullSpacePostureTask> posture_;
    std::shared_ptr<NullSpaceManipulabilityTask> manipulability_;
    std::shared_ptr<NullSpaceJointLimitTask> joint_limits_;
    TaskHandle handles_[3];  ///< Indexed by NullSpacePreference.
    NullSpacePreference preference_;
};

/// Shared by self_collision and world_collision. Visuals: the robot geometries, then the obstacles.
/// Outputs: min_distance, closest (witness points), closest_pair (the nearest pair's visuals, empty
/// outside the band), band (visuals within d_min + activation_margin), pair (nearest pair "a/b"), pairs (checked),
/// active_pairs (within the band at the latest solve).
class KSTONE_EXPORT CollisionModuleBase : public Module {
  public:
    const std::shared_ptr<CollisionConstraint>& constraint() const noexcept { return constraint_; }
    Status update_outputs(Workspace& at) override;
    Status read_output(std::size_t index, OutputValue& out) const override;
    /// Pairs penetrating when re-enabled start exempt (CollisionConstraint::reset_tracking()).
    Status on_disabled(const Cycle&) override;

  protected:
    using Module::Module;
    /// From configure(): creates and registers the constraint, declares the visuals (robot geometries if
    /// robot_visuals, then obstacles) and outputs.
    Status configure_collision(ModuleSetup& setup, CollisionModelPtr model,
                               std::shared_ptr<const CollisionWorld> world,
                               const CollisionConstraintOptions& options, bool closed_loop,
                               bool robot_visuals);
    /// Number of robot geometry visuals (obstacle visuals follow); 0 without robot_visuals.
    std::size_t geometry_visuals() const noexcept { return geometries_; }

  private:
    int visual_of(const CollisionPair& pair, bool second) const noexcept;

    std::size_t geometries_{0};
    std::shared_ptr<CollisionConstraint> constraint_;
    std::optional<CollisionQuery> display_;
    Proximity nearest_;
    std::string pair_;
    std::vector<int> closest_pair_, band_;
    std::vector<char> in_band_;
};

struct SelfCollisionModuleConfig {
    CollisionModelPtr model;
    CollisionConstraintOptions constraint;
    /// Also bound the measured state (fresh feedback), on top of the commanded one.
    bool closed_loop{false};
};

/// One CollisionConstraint on the robot's self pairs; drives and claims nothing.
class KSTONE_EXPORT SelfCollisionModule final : public CollisionModuleBase {
  public:
    SelfCollisionModule(std::string id, SelfCollisionModuleConfig config);
    const char* type() const noexcept override { return "self_collision"; }

    /// After configure(). See the module's schema for the tunable keys.
    Status tune(const ConfigNode& params) override;
    ConfigNode tunables() const override;
    ConfigNode tune_schema() const override;

    Status configure(ModuleSetup& setup) override;

  private:
    SelfCollisionModuleConfig config_, defaults_;
};

/// An obstacle of a world_collision module, in the world frame.
struct WorldObstacle {
    std::string name;
    Shape shape;
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    bool enabled{true};
};

struct WorldCollisionModuleConfig {
    /// Robot geometry without self pairs (CollisionModelOptions::self_collision = false); rejected otherwise.
    CollisionModelPtr model;
    /// Fixed set, at least one; names are unique.
    std::vector<WorldObstacle> obstacles;
    CollisionConstraintOptions constraint;
    /// Also bound the measured state (fresh feedback), on top of the commanded one.
    bool closed_loop{false};
    /// Obstacle velocity is estimated from successive pose writes (port or set_obstacle_pose) and
    /// zeroed this long (s) after the last write; 0 keeps every obstacle static.
    double velocity_timeout{0.2};
};

/// One CollisionConstraint between the robot and fixed-count obstacles; drives and claims nothing.
/// Ports per obstacle: "<name>" (pose in the world) and "<name>/enabled" (flag); a disabled
/// obstacle is not checked and its visual is hidden.
class KSTONE_EXPORT WorldCollisionModule final : public CollisionModuleBase {
  public:
    WorldCollisionModule(std::string id, WorldCollisionModuleConfig config);
    const char* type() const noexcept override { return "world_collision"; }

    /// After configure(); control-thread only, no allocation.
    Status set_obstacle_pose(std::size_t obstacle, const Eigen::Isometry3d& pose);
    Status set_obstacle_enabled(std::size_t obstacle, bool enabled);
    /// After configure(), in config order.
    const std::shared_ptr<const CollisionWorld>& world() const noexcept { return view_; }

    /// After configure(). See the module's schema for the tunable keys.
    Status tune(const ConfigNode& params) override;
    ConfigNode tunables() const override;
    ConfigNode tune_schema() const override;

    Status configure(ModuleSetup& setup) override;
    Status prepare(Cycle& cycle) override;
    Status read_port(std::size_t index, PortValue& out) const override;
    Status write_port(std::size_t index, const PortValue& value) override;
    bool visual_placement(std::size_t i, Eigen::Isometry3d& placement) const override;

  private:
    struct Motion {
        Eigen::Isometry3d last{Eigen::Isometry3d::Identity()};  ///< Pose at the last velocity sample.
        double sampled{0.0};                                    ///< cycle.now of that sample.
        bool written{false};                                    ///< Pose written since the last prepare().
        bool seen{false};                                       ///< last / sampled are valid.
    };
    WorldCollisionModuleConfig config_, defaults_;
    std::shared_ptr<CollisionWorld> world_;
    std::shared_ptr<const CollisionWorld> view_;
    std::vector<ObstacleId> ids_;
    std::vector<Motion> motion_;
};

struct ChassisModuleConfig {
    ChassisConfig chassis;
    Eigen::Isometry3d center_in_root{Eigen::Isometry3d::Identity()};
    double period{0.01};  ///< Initial scheduler period (s); each cycle then sets it to the engine dt.
    ChassisPoseLimits
        pose_limits;  ///< Tune-only; configure() seeds it from the controller (chassis max_* fields).
};

/// ChassisController outside the QP: locks the free-flyer root and writes its pose into q in prepare(); z = 0 is the ground contact.
/// Starts on the first step; a skipped step (module disabled) pauses and resynchronises the controller.
/// Port `velocity` (vx, vy, wz) and `pose_goal` (x, y, yaw in world) both lapse after command_timeout_s; the
/// last write of either is submitted as source "port" on the next step.
/// Outputs: vx, vy, wz, x, y, yaw, active, fault, nominal_vx/vy/wz (unshaped target), planned (ramp pose), goal
/// (pose command, NaN otherwise), state, reason, command (source), command_mode, command_priority, command_fresh,
/// source (pose source), phase (differential pose phase).
class KSTONE_EXPORT ChassisModule final : public Module {
  public:
    ChassisModule(std::string id, ChassisModuleConfig config);
    const char* type() const noexcept override { return "chassis"; }

    /// Submit commands and odometry here between steps. Null before configure().
    ChassisController* controller() noexcept { return controller_ ? &*controller_ : nullptr; }
    /// Velocity command of the last step (to send to the base).
    const ChassisOutput& output() const noexcept { return output_; }
    /// After configure(): pose_limits.
    Status tune(const ConfigNode& params) override;
    ConfigNode tunables() const override;
    ConfigNode tune_schema() const override;

    Status configure(ModuleSetup& setup) override;
    Status prepare(Cycle& cycle) override;
    Status read_port(std::size_t index, PortValue& out) const override;
    Status write_port(std::size_t index, const PortValue& value) override;
    Status read_output(std::size_t index, OutputValue& out) const override;

  private:
    bool fresh() const noexcept { return !command_.source.empty() && now_ <= command_.expires_at; }

    ChassisModuleConfig config_, defaults_;
    std::optional<ChassisController> controller_;
    ChassisCommandSnapshot command_;
    const Model* model_{nullptr};
    ChassisOutput output_;
    std::uint64_t last_index_{0};
    bool stepped_{false};
    std::string source_{"port"};
    Twist2D port_command_;
    Pose2D port_goal_;
    std::size_t port_mode_{0};  ///< Port index of the last write: 0 velocity, 1 pose_goal.
    bool port_pending_{false};
    std::optional<double> port_stamp_;  ///< Cycle time the last port command was submitted.
    double now_{0.0};
};

}  // namespace kstone
