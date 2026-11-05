#include "builtin_schemas.hpp"

#include "kstone/collision/model.hpp"
#include "kstone/module/builtins.hpp"
#include "kstone/module/registry.hpp"

#include <optional>

namespace kstone {
namespace {

// Far above any useful weight, low enough that squared residual rows stay finite.
constexpr double kMaxWeight = 1e6;

ParamSchema<TaskOptions> task_options() {
    ParamSchema<TaskOptions> s("Solver registration of the module's tasks.");
    s.field("level", &TaskOptions::level)
        .at_least(0)
        .doc("Priority level; lower is more important. Unset: only the last layered stage.");
    s.field("enabled", &TaskOptions::enabled);
    s.field("locked", &TaskOptions::locked).doc("Contributes no rows and holds the task's joints still.");
    return s;
}

ParamSchema<MotionLimits> motion_limits() {
    ParamSchema<MotionLimits> s(
        "Per-axis motion limits; 0 disables a bound, velocity 0 disables shaping (the goal is fed "
        "directly).");
    s.field("velocity", &MotionLimits::velocity).at_least(0).tunable();
    s.field("acceleration", &MotionLimits::acceleration).at_least(0).tunable();
    s.field("deceleration", &MotionLimits::deceleration).at_least(0).tunable();
    s.field("reversal_deceleration", &MotionLimits::reversal_deceleration).at_least(0).tunable();
    s.field("jerk", &MotionLimits::jerk).at_least(0).tunable();
    return s;
}

ParamSchema<PoseTrajectoryConfig> pose_trajectory() {
    ParamSchema<PoseTrajectoryConfig> s("Jerk-limited Cartesian reference.");
    s.field("sync", &PoseTrajectoryConfig::sync,
            {{"axis_phase", PoseSync::axis_phase}, {"axis_time", PoseSync::axis_time}})
        .tunable();
    s.field("translation", &PoseTrajectoryConfig::translation, motion_limits()).doc("World axes (m, s).");
    s.field("rotation", &PoseTrajectoryConfig::rotation, motion_limits()).doc("Error-aligned axes (rad, s).");
    s.field("lead_speed_scale", &PoseTrajectoryConfig::lead_speed_scale)
        .at_least(0)
        .tunable()
        .doc("0 = free.");
    return s;
}

ParamSchema<JointTrajectoryConfig> joint_trajectory() {
    ParamSchema<JointTrajectoryConfig> s("Jerk-limited joint reference.");
    s.field("trigger", &JointTrajectoryConfig::trigger,
            {{"direct", JointTrigger::direct},
             {"threshold", JointTrigger::threshold},
             {"continuous", JointTrigger::continuous}})
        .tunable();
    s.field("sync", &JointTrajectoryConfig::sync,
            {{"time", JointSync::time}, {"phase", JointSync::phase}, {"none", JointSync::none}})
        .tunable();
    s.field("threshold", &JointTrajectoryConfig::threshold)
        .at_least(0)
        .tunable()
        .doc("s at the velocity limit.");
    s.field("acceleration_factor", &JointTrajectoryConfig::acceleration_factor).above(0).tunable();
    s.field("jerk_factor", &JointTrajectoryConfig::jerk_factor).above(0).tunable();
    s.field("velocity_scale", &JointTrajectoryConfig::velocity_scale).above(0).at_most(1).tunable();
    s.field("velocity_cap", &JointTrajectoryConfig::velocity_cap)
        .above(0)
        .allow_infinite()
        .tunable()
        .doc("Empty, or one cruise-speed cap per joint (.inf = none).");
    return s;
}

ParamSchema<HomeConfig> home_config() {
    ParamSchema<HomeConfig> s("Homing motion.");
    s.field("velocity_scale", &HomeConfig::velocity_scale).above(0).at_most(1).tunable();
    s.field("timeout_s", &HomeConfig::timeout_s)
        .above(0)
        .tunable()
        .doc("Times out after this long without progress toward home; no limit while it keeps closing in.");
    return s;
}

ParamSchema<FrameTaskModuleConfig> frame_task() {
    using C = FrameTaskModuleConfig;
    ParamSchema<C> s("Frame pose task fed by a jerk-limited trajectory.");
    s.field("frame", &C::frame).required().non_empty();
    s.field("joints", &C::joints).doc("Driven joints; empty = root to frame.");
    s.field("reference", &C::reference).doc("Empty = world.");
    s.field("world_axes", &C::world_axes);
    s.field("weight", &C::weight).at_least(0).at_most(kMaxWeight).tunable();
    s.field("position_weight", &C::position_weight)
        .at_least(0)
        .at_most(kMaxWeight)
        .tunable()
        .doc("Per axis, times weight; 0 frees it.");
    s.field("orientation_weight", &C::orientation_weight)
        .at_least(0)
        .at_most(kMaxWeight)
        .tunable()
        .doc("Per axis, times weight.");
    s.field("gain", &C::gain).range(0, 1).tunable();
    s.field("lm_damping", &C::lm_damping).at_least(0).tunable().doc("Levenberg-Marquardt damping.");
    s.field("task", &C::task, task_options());
    s.field("trajectory", &C::trajectory, pose_trajectory())
        .required()
        .doc("Required, but {} is valid: an axis with velocity 0 is unshaped.");
    s.field("home", &C::home)
        .doc(
            "In the reference frame; absent = pose at the initial posture (relative to the root if no "
            "reference).");
    s.field("home_motion", &C::home_motion, home_config());
    s.field("position_tolerance", &C::position_tolerance).above(0).tunable().doc("Home arrival (m).");
    s.field("orientation_tolerance", &C::orientation_tolerance).above(0).tunable().doc("Home arrival (rad).");
    s.field("on_held", &C::on_held, {{"wait", OnHeld::wait}, {"abort", OnHeld::abort}})
        .tunable()
        .doc("When held: wait resumes the target, abort drops it.");
    s.field("locked", &C::locked)
        .tunable()
        .doc(
            "Freeze the driven joints (requires explicit joints); unlocking holds the current pose and drops "
            "earlier targets.");
    return s;
}

ParamSchema<JointTargetModuleConfig> joint_target() {
    using C = JointTargetModuleConfig;
    ParamSchema<C> s(
        "Joint task fed by a jerk-limited trajectory; claims its joints until new input elsewhere takes "
        "them.");
    s.field("joints", &C::joints).required().non_empty().doc("Scalar joints; at most one per mimic group.");
    s.field("weight", &C::weight).at_least(0).at_most(kMaxWeight).tunable();
    s.field("gain", &C::gain).range(0, 1).tunable();
    s.field("task", &C::task, task_options()).doc("Unset level = 0: joint control outranks pose tasks.");
    s.field("trajectory", &C::trajectory, joint_trajectory());
    s.field("velocity_limit", &C::velocity_limit)
        .above(0)
        .doc("Per joint, capped by the solver (model) limit; empty = model limits.");
    s.field("tolerance", &C::tolerance).above(0).tunable().doc("Arrival per joint.");
    s.field("priority", &C::priority).tunable().doc("Claim priority.");
    s.field("on_held", &C::on_held, {{"wait", OnHeld::wait}, {"abort", OnHeld::abort}})
        .tunable()
        .doc("When held: wait resumes the target, abort drops it.");
    s.field("hold_idle", &C::hold_idle)
        .tunable()
        .doc("Idle: true holds the last target; false drops the task so others move the joints.");
    s.field("claim_idle", &C::claim_idle)
        .tunable()
        .doc("Idle: also hold the last target and claim the joints, so conflicting modules stay held.");
    s.field("release_after_s", &C::release_after_s)
        .at_least(0)
        .tunable()
        .doc(
            "Input watchdog: other requests wait until this long (s) without joint input; default 0.25, 0 = "
            "yield at once.");
    s.field("home", &C::home).doc("Per joint; empty = positions at the initial posture.");
    s.field("home_motion", &C::home_motion, home_config());
    return s;
}

ParamSchema<GazeModuleConfig> gaze() {
    using C = GazeModuleConfig;
    ParamSchema<C> s("Points a camera at the frames that moved recently.");
    s.field("frame", &C::frame).required().non_empty().doc("Camera frame.");
    s.field("joints", &C::joints).required().non_empty().doc("Scalar joints that aim the camera.");
    s.field("frames", &C::frames)
        .doc("Focus on the mean position of the active ones, under pose or joint control.");
    s.field("offset", &C::offset).doc("Optical centre in the camera frame.");
    s.field("optical_axis", &C::optical_axis);
    s.field("weight", &C::weight).at_least(0).at_most(kMaxWeight).tunable();
    s.field("gain", &C::gain).range(0, 1).tunable();
    s.field("task", &C::task, task_options());
    s.field("max_velocity", &C::max_velocity).at_least(0).tunable().doc("0 = none.");
    s.field("target_idle_s", &C::target_idle_s)
        .tunable()
        .doc("A frame counts only this long after it last moved relative to the model root; <= 0 = always.");
    return s;
}

ParamSchema<Payload> payload() {
    ParamSchema<Payload> s("Point mass attached to a frame.");
    s.field("frame", &Payload::frame).required().non_empty();
    s.field("mass", &Payload::mass).at_least(0);
    s.field("offset", &Payload::offset);
    return s;
}

ParamSchema<CenterOfMassModuleConfig> center_of_mass() {
    using C = CenterOfMassModuleConfig;
    ParamSchema<C> s("Keeps the projected centre of mass inside a support polygon.");
    s.field("frame", &C::frame).required().non_empty().doc("Support polygon frame.");
    s.field("polygon", &C::polygon).required().doc("Convex, at least 3 [x, y] vertices.");
    s.field("subtree", &C::subtree).doc("Empty = whole model.");
    s.field("margin", &C::margin)
        .tunable()
        .doc("Kept inside the polygon edges (m); negative allows that much outside.");
    s.field("gain", &C::gain).above(0).at_most(1).tunable();
    s.field("recovery_speed", &C::recovery_speed).above(0).tunable();
    s.field("payloads", &C::payloads, payload()).tunable();
    s.ref("recovery_tasks", &C::recovery_tasks, "frame_task")
        .doc(
            "Joints that compensate and pull the CoM back; the others stop at the boundary once these "
            "saturate, never pulled back. Empty = all joints.");
    s.field("closed_loop", &C::closed_loop)
        .tunable()
        .doc("Also bound the measured state (fresh feedback), on top of the commanded one.");
    return s;
}

ParamSchema<NullSpaceSettings> null_space_settings() {
    ParamSchema<NullSpaceSettings> s("Null-space projector damping.");
    s.field("damping", &NullSpaceSettings::damping).at_least(0).tunable();
    s.field("threshold", &NullSpaceSettings::threshold)
        .at_least(0)
        .tunable()
        .doc("> 0 enables adaptive damping.");
    return s;
}

ParamSchema<NullSpacePreferenceModuleConfig> null_space_preference() {
    using C = NullSpacePreferenceModuleConfig;
    ParamSchema<C> s("Null-space preference under a frame task.");
    s.ref("primary", &C::primary, "frame_task").required();
    s.field("joints", &C::joints).doc("Empty = the primary's driven scalar joints.");
    s.field("preference", &C::preference,
            {{"posture", NullSpacePreference::posture},
             {"manipulability", NullSpacePreference::manipulability},
             {"joint_limits", NullSpacePreference::joint_limits}})
        .tunable();
    s.field("projector", &C::projector, null_space_settings());
    s.field("task", &C::task, task_options());
    s.field("reference", &C::reference,
            {{"initial", PostureReference::initial},
             {"current", PostureReference::current},
             {"neutral", PostureReference::neutral},
             {"custom", PostureReference::custom}})
        .tunable()
        .doc(
            "Posture base; neutral is clamped into the effective joint limits. Tuning it alone to custom "
            "takes the "
            "configured posture, else the current q, then the configured posture_values.");
    s.field("posture", &C::posture).tunable().doc("Per joint; custom reference only. Empty = the initial q.");
    s.field("posture_values", &C::posture_values)
        .tunable()
        .doc("{joint: value} over the reference's posture; not with current.");
    s.field("reseed_current", &C::reseed_current)
        .tunable()
        .doc("Tune only: true makes the current q the custom posture and selects the posture preference.");
    s.field("posture_weight", &C::posture_weight).at_least(0).at_most(kMaxWeight).tunable();
    s.field("posture_velocity_scale", &C::posture_velocity_scale).above(0).at_most(1).tunable();
    s.field("manipulability_weight", &C::manipulability_weight).at_least(0).at_most(kMaxWeight).tunable();
    s.field("manipulability_rate", &C::manipulability_rate).above(0);
    s.field("manipulability_axes", &C::manipulability_axes).at_least(0);
    s.field("joint_limits_weight", &C::joint_limits_weight).at_least(0).at_most(kMaxWeight).tunable();
    s.field("joint_limits_band", &C::joint_limits_band).above(0).tunable();
    s.field("joint_limits_rate", &C::joint_limits_rate).above(0).tunable();
    return s;
}

struct SelfCollisionParams {
    std::string urdf;
    std::optional<std::string> srdf;
    bool self_collision{true};
    std::vector<BodyGroup> groups;
    std::vector<NamePair> exclude_pairs;
    std::vector<NamePair> exclude_subtrees;
    bool exclude_initial_collisions{true};
    CollisionConstraintOptions constraint;
    bool closed_loop{false};
};

ParamSchema<BodyGroup> body_group() {
    ParamSchema<BodyGroup> s("Bodies selected by joint name or prefix.");
    s.field("name", &BodyGroup::name).required().non_empty();
    s.field("joint_prefixes", &BodyGroup::joint_prefixes);
    s.field("joint_names", &BodyGroup::joint_names);
    s.field("exclude_joints", &BodyGroup::exclude_joints);
    return s;
}

ParamSchema<CollisionConstraintOptions> collision_constraint() {
    using C = CollisionConstraintOptions;
    ParamSchema<C> s("Collision avoidance rows.");
    s.field("d_min", &C::d_min).at_least(0).tunable().doc("Distance kept between pairs (m).");
    s.field("gain", &C::gain).at_least(0).tunable().doc("Approach speed per metre above d_min (1/s).");
    s.field("activation_margin", &C::activation_margin).at_least(0).tunable();
    s.field("hysteresis", &C::hysteresis).at_least(0).tunable();
    s.field("approach_deceleration", &C::approach_deceleration)
        .at_least(0)
        .tunable()
        .doc("0 disables braking.");
    s.field("max_rows", &C::max_rows).above(0);
    s.field("max_cuts", &C::max_cuts).at_least(0).doc("Cutting-plane rows; 0 disables them.");
    s.field("clip_slack", &C::clip_slack).at_least(0).doc("Endpoint clip floor below d_min (m).");
    s.field("exempt_penetrating", &C::exempt_penetrating)
        .tunable()
        .doc("Leave pairs already penetrating on entering the band free until clear.");
    return s;
}

ParamSchema<SelfCollisionParams> self_collision() {
    using C = SelfCollisionParams;
    ParamSchema<C> s("Self-collision constraint built from the URDF collision geometry.");
    s.field("urdf", &C::urdf)
        .required()
        .non_empty()
        .doc("Path or URI; relative paths try the config file folder first.");
    s.field("srdf", &C::srdf).non_empty().doc("Its disable_collisions pairs are skipped.");
    s.field("self_collision", &C::self_collision);
    s.field("groups", &C::groups, body_group());
    s.field("exclude_pairs", &C::exclude_pairs).doc("Frame or geometry name pairs.");
    s.field("exclude_subtrees", &C::exclude_subtrees)
        .doc("Frame or geometry name pairs; skips every pair between their subtrees.");
    s.field("exclude_initial_collisions", &C::exclude_initial_collisions)
        .doc("Drop pairs penetrating at the initial posture (robot.initial, else neutral), with a warning.");
    s.field("constraint", &C::constraint, collision_constraint());
    s.field("closed_loop", &C::closed_loop)
        .tunable()
        .doc("Also bound the measured state (fresh feedback), on top of the commanded one.");
    return s;
}

Result<std::shared_ptr<Module>> make_self_collision(const std::string& id, SelfCollisionParams p,
                                                    const BuildContext& build) {
    if (!build.model) return make_error(ErrorCode::invalid_argument, "self_collision '" + id + "': no model");
    CollisionModelOptions options;
    options.self_collision = p.self_collision;
    if (p.srdf) options.srdf = ModelSource::file(*p.srdf);
    options.groups = std::move(p.groups);
    options.exclude_pairs = std::move(p.exclude_pairs);
    options.exclude_subtrees = std::move(p.exclude_subtrees);
    options.exclude_initial_collisions = p.exclude_initial_collisions;
    options.initial = build.initial;
    ModelSource urdf = ModelSource::file(p.urdf);
    urdf.base_dir = build.base_dir;
    auto model = CollisionModel::from_urdf(build.model, urdf, options, build.context);
    if (!model)
        return make_error(model.error().code, "self_collision '" + id + "': " + model.error().message);
    SelfCollisionModuleConfig config;
    config.model = std::move(*model);
    config.constraint = p.constraint;
    config.closed_loop = p.closed_loop;
    return std::shared_ptr<Module>(std::make_shared<SelfCollisionModule>(id, std::move(config)));
}

struct WorldCollisionParams {
    std::string urdf;
    std::vector<WorldObstacle> obstacles;
    CollisionConstraintOptions constraint;
    bool closed_loop{false};
    double velocity_timeout{0.2};
};

ParamSchema<Shape> shape() {
    ParamSchema<Shape> s("Collision primitive; cylinders and capsules run along z.");
    s.field("type", &Shape::type,
            {{"sphere", ShapeType::sphere},
             {"box", ShapeType::box},
             {"cylinder", ShapeType::cylinder},
             {"capsule", ShapeType::capsule}})
        .required();
    s.field("size", &Shape::size)
        .required()
        .doc("sphere (r, 0, 0); box (x, y, z); cylinder / capsule (r, length, 0).");
    return s;
}

ParamSchema<WorldObstacle> world_obstacle() {
    ParamSchema<WorldObstacle> s("Obstacle in the world frame.");
    s.field("name", &WorldObstacle::name)
        .required()
        .non_empty()
        .doc("Also its pose port; <name>/enabled is its flag port. No '/'.");
    s.field("shape", &WorldObstacle::shape, shape()).required();
    s.field("pose", &WorldObstacle::pose);
    s.field("enabled", &WorldObstacle::enabled);
    return s;
}

ParamSchema<WorldCollisionParams> world_collision() {
    using C = WorldCollisionParams;
    ParamSchema<C> s(
        "Collision constraint between the URDF collision geometry and a fixed set of obstacles.");
    s.field("urdf", &C::urdf)
        .required()
        .non_empty()
        .doc("Path or URI; relative paths try the config file folder first.");
    s.field("obstacles", &C::obstacles, world_obstacle()).required();
    s.field("constraint", &C::constraint, collision_constraint());
    s.field("closed_loop", &C::closed_loop)
        .tunable()
        .doc("Also bound the measured state (fresh feedback), on top of the commanded one.");
    s.field("velocity_timeout", &C::velocity_timeout)
        .at_least(0)
        .tunable()
        .doc(
            "Obstacle velocity is estimated from successive pose writes and zeroed this long (s) after the "
            "last; 0 disables.");
    return s;
}

Result<std::shared_ptr<Module>> make_world_collision(const std::string& id, WorldCollisionParams p,
                                                     const BuildContext& build) {
    if (!build.model)
        return make_error(ErrorCode::invalid_argument, "world_collision '" + id + "': no model");
    CollisionModelOptions options;
    options.self_collision = false;
    ModelSource urdf = ModelSource::file(p.urdf);
    urdf.base_dir = build.base_dir;
    auto model = CollisionModel::from_urdf(build.model, urdf, options, build.context);
    if (!model)
        return make_error(model.error().code, "world_collision '" + id + "': " + model.error().message);
    WorldCollisionModuleConfig config;
    config.model = std::move(*model);
    config.obstacles = std::move(p.obstacles);
    config.constraint = p.constraint;
    config.closed_loop = p.closed_loop;
    config.velocity_timeout = p.velocity_timeout;
    return std::shared_ptr<Module>(std::make_shared<WorldCollisionModule>(id, std::move(config)));
}

ParamSchema<ChassisConfig> chassis_config() {
    using C = ChassisConfig;
    ParamSchema<C> s("Chassis controller.");
    s.field("kinematics", &C::kinematics,
            {{"omni", ChassisKinematics::omni}, {"differential", ChassisKinematics::differential}});
    s.field("target_translation_deadband", &C::target_translation_deadband).at_least(0);
    s.field("target_rotation_deadband", &C::target_rotation_deadband).at_least(0);
    s.field("odom_timeout_s", &C::odom_timeout_s).above(0);
    s.field("command_timeout_s", &C::command_timeout_s).above(0);
    s.field("max_dt_s", &C::max_dt_s).above(0);
    s.field("allowed_missed_cycles", &C::allowed_missed_cycles).at_least(0);
    s.field("max_linear", &C::max_linear).above(0);
    s.field("max_angular", &C::max_angular).above(0);
    s.field("max_linear_acceleration", &C::max_linear_acceleration).above(0);
    s.field("max_linear_deceleration", &C::max_linear_deceleration).above(0);
    s.field("max_linear_reversal_deceleration", &C::max_linear_reversal_deceleration).above(0);
    s.field("max_angular_acceleration", &C::max_angular_acceleration).above(0);
    s.field("max_angular_deceleration", &C::max_angular_deceleration).above(0);
    s.field("max_angular_reversal_deceleration", &C::max_angular_reversal_deceleration).above(0);
    s.field("max_linear_jerk", &C::max_linear_jerk).above(0);
    s.field("max_angular_jerk", &C::max_angular_jerk).above(0);
    s.field("arrival_position_tolerance", &C::arrival_position_tolerance).above(0);
    s.field("arrival_rotation_tolerance", &C::arrival_rotation_tolerance).above(0);
    s.field("base_frame", &C::base_frame).non_empty();
    return s;
}

ParamSchema<ChassisPoseLimits> chassis_pose_limits() {
    using C = ChassisPoseLimits;
    ParamSchema<C> s("Pose-goal shaping limits; 0 disables a bound, all 0 emits the unshaped twist.");
    s.field("linear_velocity", &C::linear_velocity).at_least(0).tunable();
    s.field("linear_acceleration", &C::linear_acceleration).at_least(0).tunable();
    s.field("linear_deceleration", &C::linear_deceleration).at_least(0).tunable();
    s.field("linear_reversal_deceleration", &C::linear_reversal_deceleration).at_least(0).tunable();
    s.field("linear_jerk", &C::linear_jerk).at_least(0).tunable();
    s.field("angular_velocity", &C::angular_velocity).at_least(0).tunable();
    s.field("angular_acceleration", &C::angular_acceleration).at_least(0).tunable();
    s.field("angular_deceleration", &C::angular_deceleration).at_least(0).tunable();
    s.field("angular_reversal_deceleration", &C::angular_reversal_deceleration).at_least(0).tunable();
    s.field("angular_jerk", &C::angular_jerk).at_least(0).tunable();
    s.field("planner_mode", &C::planner_mode,
            {{"axis_phase", ChassisPlannerMode::axis_phase},
             {"axis_time", ChassisPlannerMode::axis_time},
             {"straight", ChassisPlannerMode::straight}})
        .tunable();
    return s;
}

ParamSchema<ChassisModuleConfig> chassis_module() {
    using C = ChassisModuleConfig;
    ParamSchema<C> s("Chassis controller outside the QP; drives the model root.");
    s.field("chassis", &C::chassis, chassis_config());
    s.field("center_in_root", &C::center_in_root);
    s.field("period", &C::period)
        .above(0)
        .doc("Initial scheduler period (s); the engine dt replaces it each cycle.");
    return s;
}

// Tune-only: pose_limits starts from the controller's limits, which derive from the chassis max_* fields.
ParamSchema<ChassisModuleConfig> chassis_tune() {
    auto s = chassis_module();
    s.field("pose_limits", &ChassisModuleConfig::pose_limits, chassis_pose_limits());
    return s;
}

ParamSchema<SelfCollisionModuleConfig> self_collision_module() {
    ParamSchema<SelfCollisionModuleConfig> s("Self-collision constraint.");
    s.field("constraint", &SelfCollisionModuleConfig::constraint, collision_constraint());
    s.field("closed_loop", &SelfCollisionModuleConfig::closed_loop)
        .tunable()
        .doc("Also bound the measured state (fresh feedback), on top of the commanded one.");
    return s;
}

ParamSchema<WorldCollisionModuleConfig> world_collision_module() {
    ParamSchema<WorldCollisionModuleConfig> s("World collision constraint.");
    s.field("constraint", &WorldCollisionModuleConfig::constraint, collision_constraint());
    s.field("closed_loop", &WorldCollisionModuleConfig::closed_loop)
        .tunable()
        .doc("Also bound the measured state (fresh feedback), on top of the commanded one.");
    s.field("velocity_timeout", &WorldCollisionModuleConfig::velocity_timeout)
        .at_least(0)
        .tunable()
        .doc(
            "Obstacle velocity is estimated from successive pose writes and zeroed this long (s) after the "
            "last; 0 disables.");
    return s;
}

ParamSchema<SolverOptions> solver_options() {
    ParamSchema<SolverOptions> s("Differential IK solver.");
    s.field("mode", &SolverOptions::mode,
            {{"weighted", SolveMode::weighted},
             {"lexicographic", SolveMode::lexicographic},
             {"layered_independent", SolveMode::layered_independent},
             {"layered_coupled", SolveMode::layered_coupled}})
        .tunable();
    s.field("damping", &SolverOptions::damping).above(0).tunable();
    s.field("limit_gain", &SolverOptions::limit_gain).above(0).at_most(1).tunable();
    s.field("feasibility_tol", &SolverOptions::feasibility_tol).above(0).tunable();
    s.field("check_feasibility", &SolverOptions::check_feasibility).tunable();
    s.field("hold_retry", &SolverOptions::hold_retry).tunable();
    return s;
}

}  // namespace

namespace builtin_schemas {

const ParamSchema<FrameTaskModuleConfig>& frame_task() {
    static const auto s = kstone::frame_task();
    return s;
}
const ParamSchema<JointTargetModuleConfig>& joint_target() {
    static const auto s = kstone::joint_target();
    return s;
}
const ParamSchema<GazeModuleConfig>& gaze() {
    static const auto s = kstone::gaze();
    return s;
}
const ParamSchema<CenterOfMassModuleConfig>& center_of_mass() {
    static const auto s = kstone::center_of_mass();
    return s;
}
const ParamSchema<NullSpacePreferenceModuleConfig>& null_space_preference() {
    static const auto s = kstone::null_space_preference();
    return s;
}
const ParamSchema<SelfCollisionModuleConfig>& self_collision() {
    static const auto s = self_collision_module();
    return s;
}
const ParamSchema<WorldCollisionModuleConfig>& world_collision() {
    static const auto s = world_collision_module();
    return s;
}
const ParamSchema<ChassisModuleConfig>& chassis() {
    static const auto s = chassis_tune();
    return s;
}
const ParamSchema<SolverOptions>& solver() {
    static const auto s = solver_options();
    return s;
}

}  // namespace builtin_schemas

ModuleRegistry ModuleRegistry::with_builtins() {
    ModuleRegistry r;
    // Types are distinct and factories non-null, so these cannot fail.
    (void)r.add<FrameTaskModule>("frame_task", builtin_schemas::frame_task());
    (void)r.add<JointTargetModule>("joint_target", builtin_schemas::joint_target());
    (void)r.add<GazeModule>("gaze", builtin_schemas::gaze());
    (void)r.add<CenterOfMassModule>("center_of_mass", builtin_schemas::center_of_mass());
    (void)r.add<NullSpacePreferenceModule>("null_space_preference", builtin_schemas::null_space_preference());
    (void)r.add("self_collision", self_collision(), make_self_collision);
    (void)r.add("world_collision", world_collision(), make_world_collision);
    (void)r.add<ChassisModule>("chassis", chassis_module());
    return r;
}

}  // namespace kstone
