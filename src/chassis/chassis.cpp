#include "kstone/chassis/chassis.hpp"

#include <ruckig/ruckig.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

namespace kstone {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kPi = 3.14159265358979323846;
constexpr double kRootTwo = 1.41421356237309504880;

double or_infinity(double value) { return value > 0.0 ? value : kInfinity; }
double zero_small(double value) { return std::abs(value) < 1e-12 ? 0.0 : value; }
bool finite(const Pose2D& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw); }
bool finite(const Twist2D& t) { return std::isfinite(t.vx) && std::isfinite(t.vy) && std::isfinite(t.wz); }

Status require_positive(double value, const char* name) {
    if (!std::isfinite(value) || value <= 0.0)
        return make_error(ErrorCode::invalid_argument,
                          std::string("chassis: ") + name + " must be finite and > 0");
    return {};
}

Status require_nonnegative(double value, const char* name) {
    if (!std::isfinite(value) || value < 0.0)
        return make_error(ErrorCode::invalid_argument,
                          std::string("chassis: ") + name + " must be finite and >= 0");
    return {};
}

bool ruckig_ok(ruckig::Result result) {
    return result == ruckig::Result::Working || result == ruckig::Result::Finished;
}

// Acceleration cap for one axis: reversal while moving against the displacement, deceleration at the goal.
double acceleration_cap(double velocity, double displacement, double acceleration, double deceleration,
                        double reversal) {
    if (velocity * displacement < -1e-12) return reversal;
    if (std::abs(displacement) < 1e-9) return deceleration;
    return acceleration;
}

// Distance one axis can cover in about 1000 s from rest; farther goals are planned toward this horizon and
// re-planned every cycle, since Ruckig rejects trajectories longer than ~7.6e3 s.
double planning_horizon(double velocity, double acceleration, double jerk) {
    constexpr double t = 1000.0;
    double horizon = std::numeric_limits<double>::infinity();
    if (std::isfinite(velocity)) horizon = std::min(horizon, velocity * t / 2);
    if (std::isfinite(acceleration)) horizon = std::min(horizon, acceleration * t * t / 8);
    if (std::isfinite(jerk)) horizon = std::min(horizon, jerk * t * t * t / 64);
    return horizon;
}

double clamp_magnitude(double value, double limit) { return std::clamp(value, -limit, limit); }

// ULP guard: values a hair above a limit are snapped onto it so Ruckig accepts the state.
void snap(double& value, double limit) {
    const double magnitude = std::abs(value);
    if (magnitude > limit && magnitude <= limit * (1 + 1e-12) + 1e-15) value = std::copysign(limit, value);
}

// Runs one Ruckig update on fresh state. The output must be cleared too: with infinite jerk and acceleration
// Ruckig skips writing the brake profile, so a reused trajectory would replay the previous call's brake.
template <std::size_t N>
bool run_ruckig(ruckig::Ruckig<N>& generator, const ruckig::InputParameter<N>& input,
                ruckig::OutputParameter<N>& output, double dt, const char* what, std::string& error) {
    generator.delta_time = dt;
    generator.reset();
    output = ruckig::OutputParameter<N>();
    ruckig::Result result = ruckig::Result::Error;
    try {
        result = generator.update(input, output);
    } catch (const std::exception& e) {
        error = std::string(what) + e.what();
        return false;
    }
    if (!ruckig_ok(result)) {
        error = std::string(what) + std::to_string(static_cast<int>(result));
        return false;
    }
    return true;
}

struct Regulated {
    double velocity{0.0};
    double acceleration{0.0};
};

}  // namespace

const char* to_string(ChassisKinematics value) noexcept {
    switch (value) {
        case ChassisKinematics::omni:
            return "omni";
        case ChassisKinematics::differential:
            return "differential";
    }
    return "unknown";
}

const char* to_string(ChassisState value) noexcept {
    switch (value) {
        case ChassisState::configured:
            return "configured";
        case ChassisState::active:
            return "active";
        case ChassisState::paused:
            return "paused";
        case ChassisState::fault:
            return "fault";
        case ChassisState::stopped:
            return "stopped";
    }
    return "unknown";
}

const char* to_string(ChassisPlannerMode value) noexcept {
    switch (value) {
        case ChassisPlannerMode::axis_phase:
            return "axis_phase";
        case ChassisPlannerMode::axis_time:
            return "axis_time";
        case ChassisPlannerMode::straight:
            return "straight";
    }
    return "unknown";
}

const char* to_string(DiffPhase value) noexcept {
    switch (value) {
        case DiffPhase::align:
            return "align";
        case DiffPhase::drive:
            return "drive";
        case DiffPhase::final_align:
            return "final_align";
        case DiffPhase::path_error:
            return "path_error";
    }
    return "unknown";
}

const char* to_string(PoseSource value) noexcept {
    switch (value) {
        case PoseSource::internal:
            return "internal";
        case PoseSource::odometry:
            return "odometry";
    }
    return "unknown";
}

const char* to_string(ChassisReason value) noexcept {
    switch (value) {
        case ChassisReason::not_started:
            return "not_started";
        case ChassisReason::ready:
            return "ready";
        case ChassisReason::paused:
            return "paused";
        case ChassisReason::resynchronized:
            return "resynchronized";
        case ChassisReason::stopped:
            return "stopped";
        case ChassisReason::solve_paused:
            return "solve_paused";
        case ChassisReason::command_timeout:
            return "command_timeout";
        case ChassisReason::scheduler_discontinuity:
            return "scheduler_discontinuity";
        case ChassisReason::tracking:
            return "tracking";
        case ChassisReason::trajectory_error:
            return "trajectory_error";
        case ChassisReason::path_error:
            return "path_error";
    }
    return "unknown";
}

double wrap_angle(double angle) noexcept {
    double wrapped = std::fmod(angle + kPi, 2 * kPi);
    if (wrapped < 0) wrapped += 2 * kPi;
    return wrapped - kPi;
}

Pose2D integrate_se2(const Pose2D& pose, const Twist2D& twist, double dt) noexcept {
    const double theta = twist.wz * dt;
    double a = 0.0;
    double b = 0.0;
    if (std::abs(theta) < 1e-8) {
        a = dt * (1 - theta * theta / 6);
        b = dt * (theta / 2 - theta * theta * theta / 24);
    } else {
        a = std::sin(theta) / twist.wz;
        b = (1 - std::cos(theta)) / twist.wz;
    }
    const double xb = a * twist.vx - b * twist.vy;
    const double yb = b * twist.vx + a * twist.vy;
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    return {pose.x + c * xb - s * yb, pose.y + s * xb + c * yb, wrap_angle(pose.yaw + theta)};
}

Eigen::Isometry3d chassis_root_pose(const Pose2D& center, const Eigen::Isometry3d& center_in_root) {
    Eigen::Isometry3d world_center = Eigen::Isometry3d::Identity();
    world_center.translation() = Eigen::Vector3d(center.x, center.y, 0.0);
    world_center.linear() = Eigen::AngleAxisd(center.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return world_center * center_in_root.inverse();
}

Status ChassisConfig::validate() const {
    const std::pair<double, const char*> nonnegative[] = {
        {target_translation_deadband, "target_translation_deadband"},
        {target_rotation_deadband, "target_rotation_deadband"},
    };
    for (const auto& [value, name] : nonnegative)
        if (auto status = require_nonnegative(value, name); !status) return status;
    const std::pair<double, const char*> positive[] = {
        {odom_timeout_s, "odom_timeout_s"},
        {command_timeout_s, "command_timeout_s"},
        {max_dt_s, "max_dt_s"},
        {max_linear, "max_linear"},
        {max_angular, "max_angular"},
        {max_linear_acceleration, "max_linear_acceleration"},
        {max_linear_deceleration, "max_linear_deceleration"},
        {max_linear_reversal_deceleration, "max_linear_reversal_deceleration"},
        {max_angular_acceleration, "max_angular_acceleration"},
        {max_angular_deceleration, "max_angular_deceleration"},
        {max_angular_reversal_deceleration, "max_angular_reversal_deceleration"},
        {max_linear_jerk, "max_linear_jerk"},
        {max_angular_jerk, "max_angular_jerk"},
        {arrival_position_tolerance, "arrival_position_tolerance"},
        {arrival_rotation_tolerance, "arrival_rotation_tolerance"},
    };
    for (const auto& [value, name] : positive)
        if (auto status = require_positive(value, name); !status) return status;
    if (allowed_missed_cycles < 1)
        return make_error(ErrorCode::invalid_argument, "chassis: allowed_missed_cycles must be >= 1");
    if (base_frame.empty())
        return make_error(ErrorCode::invalid_argument, "chassis: base_frame must be non-empty");
    return {};
}

Status ChassisPoseLimits::validate() const {
    for (double value : {linear_velocity, linear_acceleration, linear_deceleration,
                         linear_reversal_deceleration, linear_jerk, angular_velocity, angular_acceleration,
                         angular_deceleration, angular_reversal_deceleration, angular_jerk})
        if (auto status = require_nonnegative(value, "pose limit"); !status) return status;
    switch (planner_mode) {
        case ChassisPlannerMode::axis_phase:
        case ChassisPlannerMode::axis_time:
        case ChassisPlannerMode::straight:
            return {};
    }
    return make_error(ErrorCode::invalid_argument, "chassis: unknown planner_mode");
}

bool ChassisPoseLimits::bypass() const noexcept {
    for (double value : {linear_velocity, linear_acceleration, linear_deceleration,
                         linear_reversal_deceleration, linear_jerk, angular_velocity, angular_acceleration,
                         angular_deceleration, angular_reversal_deceleration, angular_jerk})
        if (value != 0.0) return false;
    return true;
}

struct ChassisController::Impl {
    ChassisConfig config;
    ChassisPoseLimits limits;
    ChassisState state{ChassisState::configured};
    ChassisReason reason{ChassisReason::not_started};
    std::string error;
    std::optional<double> period, pending_period, last_step, integrated_until;
    std::optional<double> odom_stamp, command_stamp, output_valid_until;
    std::string command_source;
    int command_priority{0};
    Pose2D pose, ramp;
    std::optional<Pose2D> pose_target, planned_pose_override;
    Twist2D target, out, planned_velocity, out_acceleration;
    // Differential path state.
    std::optional<Pose2D> path_goal, settled;
    DiffPhase phase{DiffPhase::align};
    std::optional<double> straight_bearing;
    double straight_direction{1.0};
    Pose2D straight_origin;
    double straight_path_bearing{0.0};
    double straight_length{0.0};
    // Reused generators: fixed-DOF Ruckig keeps its state in std::array.
    ruckig::Ruckig<3> omni{0.01};
    ruckig::InputParameter<3> omni_input;
    ruckig::OutputParameter<3> omni_output;
    ruckig::Ruckig<1> axis{0.01};
    ruckig::InputParameter<1> axis_input;
    ruckig::OutputParameter<1> axis_output;

    double maximum_scheduler_dt() const {
        double cycle = period.value_or(0.0);
        if (pending_period) cycle = std::max(cycle, *pending_period);
        return std::max(config.max_dt_s, cycle * static_cast<double>(config.allowed_missed_cycles));
    }

    bool command_fresh(double now) const {
        return command_stamp && now - *command_stamp >= 0 && now - *command_stamp <= config.command_timeout_s;
    }

    bool odom_fresh(double now) const {
        return odom_stamp && now - *odom_stamp >= 0 && now - *odom_stamp <= config.odom_timeout_s;
    }

    void clear_motion() {
        target = out = planned_velocity = out_acceleration = Twist2D{};
        pose_target.reset();
        command_stamp.reset();
        command_source.clear();
        command_priority = 0;
        output_valid_until.reset();
        ramp = pose;
        planned_pose_override.reset();
        path_goal.reset();
        settled.reset();
        phase = DiffPhase::align;
        straight_bearing.reset();
        straight_direction = 1.0;
        straight_origin = Pose2D{};
        straight_path_bearing = 0.0;
        straight_length = 0.0;
    }

    void fault(ChassisReason why) {
        state = ChassisState::fault;
        reason = why;
        clear_motion();
    }

    ChassisOutput snapshot(double now) const {
        ChassisOutput output;
        output.velocity = out;
        output.pose = pose;
        output.timestamp = now;
        output.state = state;
        output.source = odom_fresh(now) ? PoseSource::odometry : PoseSource::internal;
        output.reason = reason;
        return output;
    }

    Status validate_command(bool is_finite, const std::string& source, int priority) const {
        if (!is_finite) return make_error(ErrorCode::invalid_argument, "chassis: command must be finite");
        if (source.empty())
            return make_error(ErrorCode::invalid_argument, "chassis: command source must be non-empty");
        if (priority < 0)
            return make_error(ErrorCode::invalid_argument, "chassis: command priority must be >= 0");
        return {};
    }

    bool accept_command(double now, const std::string& source, int priority) {
        if (state != ChassisState::active) return false;
        if (priority < command_priority && command_fresh(now)) return false;
        command_stamp = now;
        command_source = source;
        command_priority = priority;
        return true;
    }

    void advance_pose(double now) {
        const auto begin_stamp = integrated_until;
        integrated_until = now;
        if (!begin_stamp || state != ChassisState::active) return;
        double begin = *begin_stamp;
        if (now < begin || now - begin > maximum_scheduler_dt()) {
            fault(ChassisReason::scheduler_discontinuity);
            return;
        }
        const bool fresh = odom_fresh(now);
        if (odom_stamp) begin = std::max(begin, *odom_stamp + config.odom_timeout_s);
        const double end = output_valid_until ? std::min(now, *output_valid_until) : now;
        const double dt = output_valid_until ? std::max(0.0, end - begin) : 0.0;
        if (!fresh && planned_pose_override && pose_target && config.kinematics == ChassisKinematics::omni)
            pose = *planned_pose_override;
        else
            pose = integrate_se2(pose, out, dt);
        planned_pose_override.reset();
    }

    Twist2D pose_velocity(double dt) const {
        if (!pose_target) return target;
        const Pose2D& goal = *pose_target;
        const double dx = goal.x - pose.x;
        const double dy = goal.y - pose.y;
        const double distance = std::hypot(dx, dy);
        const double step_dt = std::max(dt, 1e-15);
        double wx = 0.0;
        double wy = 0.0;
        if (distance > config.arrival_position_tolerance) {
            double speed = distance / step_dt;
            if (limits.linear_velocity > 0) speed = std::min(speed, limits.linear_velocity);
            if (limits.linear_acceleration > 0)
                speed = std::min(speed, std::sqrt(2 * limits.linear_acceleration * distance));
            wx = speed * dx / distance;
            wy = speed * dy / distance;
        }
        const double c = std::cos(pose.yaw);
        const double s = std::sin(pose.yaw);
        Twist2D result{c * wx + s * wy, -s * wx + c * wy, 0.0};
        if (config.kinematics == ChassisKinematics::differential) result.vy = 0.0;
        const double error_yaw = wrap_angle(goal.yaw - pose.yaw);
        if (std::abs(error_yaw) > config.arrival_rotation_tolerance) {
            double rate = std::abs(error_yaw) / step_dt;
            if (limits.angular_velocity > 0) rate = std::min(rate, limits.angular_velocity);
            if (limits.angular_acceleration > 0)
                rate = std::min(rate, std::sqrt(2 * limits.angular_acceleration * std::abs(error_yaw)));
            result.wz = std::copysign(rate, error_yaw);
        }
        return result;
    }

    bool omni_pose_step(const Twist2D& nominal, double dt, Twist2D& body) {
        const Pose2D& goal = *pose_target;
        if (limits.bypass()) {
            out_acceleration = Twist2D{};
            ramp = goal;
            body = nominal;
            return true;
        }
        const double c = std::cos(pose.yaw);
        const double s = std::sin(pose.yaw);
        std::array<double, 3> velocity{c * out.vx - s * out.vy, s * out.vx + c * out.vy, out.wz};
        std::array<double, 3> acceleration{c * out_acceleration.vx - s * out_acceleration.vy,
                                           s * out_acceleration.vx + c * out_acceleration.vy,
                                           out_acceleration.wz};
        const double yaw_goal = pose.yaw + wrap_angle(goal.yaw - pose.yaw);
        const std::array<double, 3> position{pose.x, pose.y, pose.yaw};
        std::array<double, 3> goal_position{goal.x, goal.y, yaw_goal};
        for (std::size_t i = 0; i < 3; ++i)
            if (std::abs(goal_position[i] - position[i]) < 1e-9 && std::abs(velocity[i]) < 1e-8)
                velocity[i] = acceleration[i] = 0.0;
        const double dx = goal.x - pose.x;
        const double dy = goal.y - pose.y;
        const double lv = or_infinity(limits.linear_velocity);
        const double av = or_infinity(limits.angular_velocity);
        const double la = or_infinity(limits.linear_acceleration);
        const double ld = limits.linear_deceleration > 0 ? limits.linear_deceleration : la;
        const double lr = limits.linear_reversal_deceleration > 0 ? limits.linear_reversal_deceleration : ld;
        const double aa = or_infinity(limits.angular_acceleration);
        const double ad = limits.angular_deceleration > 0 ? limits.angular_deceleration : aa;
        const double ar =
            limits.angular_reversal_deceleration > 0 ? limits.angular_reversal_deceleration : ad;
        const std::array<double, 3> max_velocity{lv / kRootTwo, lv / kRootTwo, av};
        const std::array<double, 3> max_acceleration{
            acceleration_cap(velocity[0], dx, la / kRootTwo, ld / kRootTwo, lr / kRootTwo),
            acceleration_cap(velocity[1], dy, la / kRootTwo, ld / kRootTwo, lr / kRootTwo),
            acceleration_cap(velocity[2], yaw_goal - pose.yaw, aa, ad, ar)};
        const double lj = or_infinity(limits.linear_jerk) / kRootTwo;
        const double aj = or_infinity(limits.angular_jerk);
        for (std::size_t i = 0; i < 3; ++i) {
            snap(velocity[i], max_velocity[i]);
            snap(acceleration[i], max_acceleration[i]);
        }
        const double reach = planning_horizon(max_velocity[0], la / kRootTwo, lj);
        const double distance = std::hypot(dx, dy);
        if (distance > reach) {
            goal_position[0] = pose.x + dx * reach / distance;
            goal_position[1] = pose.y + dy * reach / distance;
        }
        goal_position[2] = pose.yaw + clamp_magnitude(yaw_goal - pose.yaw, planning_horizon(av, aa, aj));
        auto& input = omni_input;
        input.current_position = position;
        input.current_velocity = velocity;
        input.current_acceleration = acceleration;
        input.target_position = goal_position;
        input.target_velocity = {0.0, 0.0, 0.0};
        input.target_acceleration = {0.0, 0.0, 0.0};
        input.max_velocity = max_velocity;
        input.min_velocity = std::array<double, 3>{-max_velocity[0], -max_velocity[1], -max_velocity[2]};
        input.max_acceleration = max_acceleration;
        input.min_acceleration =
            std::array<double, 3>{-max_acceleration[0], -max_acceleration[1], -max_acceleration[2]};
        input.max_jerk = {lj, lj, aj};
        const bool phase_sync = limits.planner_mode != ChassisPlannerMode::axis_time;
        input.synchronization = phase_sync ? ruckig::Synchronization::Phase : ruckig::Synchronization::Time;
        if (!run_ruckig(omni, input, omni_output, dt, "omni chassis Ruckig update failed: ", error)) {
            if (!phase_sync) return false;
            input.synchronization = ruckig::Synchronization::Time;
            if (!run_ruckig(omni, input, omni_output, dt, "omni chassis Ruckig update failed: ", error))
                return false;
        }
        const auto& v = omni_output.new_velocity;
        const auto& a = omni_output.new_acceleration;
        const auto& p = omni_output.new_position;
        // Express the sampled world state in the body frame at the sampled yaw, so the next cycle's
        // body->world conversion reproduces Ruckig's state exactly while the base rotates.
        const double cn = std::cos(p[2]);
        const double sn = std::sin(p[2]);
        body = {zero_small(cn * v[0] + sn * v[1]), zero_small(-sn * v[0] + cn * v[1]), zero_small(v[2])};
        out_acceleration = {cn * a[0] + sn * a[1], -sn * a[0] + cn * a[1], a[2]};
        ramp = {p[0], p[1], wrap_angle(p[2])};
        planned_pose_override = ramp;
        return true;
    }

    // Single-axis regulator used by the differential phase machine.
    bool position_regulator(double position, double velocity, double acceleration, double goal,
                            double velocity_limit, double acceleration_limit, double deceleration,
                            double reversal, double jerk, double dt, Regulated& result) {
        const double displacement = goal - position;
        const double cap =
            or_infinity(acceleration_cap(velocity, displacement, acceleration_limit, deceleration, reversal));
        const double effective = std::max(cap, std::abs(acceleration));
        auto& input = axis_input;
        input.current_position = {position};
        input.current_velocity = {velocity};
        input.current_acceleration = {acceleration};
        input.target_position = {
            position + clamp_magnitude(displacement,
                                       planning_horizon(or_infinity(velocity_limit),
                                                        or_infinity(acceleration_limit), or_infinity(jerk)))};
        input.target_velocity = {0.0};
        input.target_acceleration = {0.0};
        input.max_velocity = {or_infinity(velocity_limit)};
        input.min_velocity = std::array<double, 1>{-or_infinity(velocity_limit)};
        input.max_acceleration = {effective};
        input.min_acceleration = std::array<double, 1>{-effective};
        input.max_jerk = {or_infinity(jerk)};
        if (!run_ruckig(axis, input, axis_output, dt,
                        "differential chassis Ruckig position update failed: ", error))
            return false;
        result = {zero_small(axis_output.new_velocity[0]), zero_small(axis_output.new_acceleration[0])};
        return true;
    }

    bool differential_pose_step(double dt, Twist2D& body) {
        const Pose2D request = *pose_target;
        const double capture_position = 1.5 * config.arrival_position_tolerance;
        const double capture_rotation = 1.5 * config.arrival_rotation_tolerance;
        const double position_change =
            path_goal ? std::hypot(request.x - path_goal->x, request.y - path_goal->y) : kInfinity;
        const double yaw_change = path_goal ? std::abs(wrap_angle(request.yaw - path_goal->yaw)) : kInfinity;
        if (position_change > config.target_translation_deadband ||
            yaw_change > config.target_rotation_deadband) {
            const double dx = request.x - pose.x;
            const double dy = request.y - pose.y;
            const double distance = std::hypot(dx, dy);
            const double bearing = std::atan2(dy, dx);
            const double forward_error = std::abs(wrap_angle(bearing - pose.yaw));
            const double reverse_heading = wrap_angle(bearing + kPi);
            const double reverse_error = std::abs(wrap_angle(reverse_heading - pose.yaw));
            const bool reverse = reverse_error < forward_error;
            straight_direction = reverse ? -1.0 : 1.0;
            straight_bearing = reverse ? reverse_heading : bearing;
            straight_origin = pose;
            straight_path_bearing = bearing;
            straight_length = distance;
            path_goal = request;
            phase = distance <= capture_position ? DiffPhase::final_align : DiffPhase::align;
            settled.reset();
        }
        const Pose2D goal = *path_goal;
        const double distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
        const double yaw_error = wrap_angle(goal.yaw - pose.yaw);
        if (settled) {
            if (distance <= 2 * capture_position && std::abs(yaw_error) <= 2 * capture_rotation) {
                out_acceleration = Twist2D{};
                ramp = goal;
                body = Twist2D{};
                return true;
            }
            settled.reset();
            phase = DiffPhase::path_error;  // drifted after settling: stop and report, never replan silently
        }
        const double heading = straight_bearing.value_or(pose.yaw);
        const double path_yaw_error = wrap_angle(heading - pose.yaw);
        double linear_position = 0.0;
        double linear_target = 0.0;
        double angular_displacement = path_yaw_error;
        if (phase == DiffPhase::align) {
            ramp = {pose.x, pose.y, heading};
            if (std::abs(path_yaw_error) <= capture_rotation && std::abs(out.wz) < 1e-6)
                phase = DiffPhase::drive;
        }
        if (phase == DiffPhase::drive) {
            ramp = {goal.x, goal.y, heading};
            const double progress = (pose.x - straight_origin.x) * std::cos(straight_path_bearing) +
                                    (pose.y - straight_origin.y) * std::sin(straight_path_bearing);
            linear_position = straight_direction * progress;
            linear_target = straight_direction * straight_length;
            const double remaining = straight_length - progress;
            if (remaining <= capture_position) {
                if (std::abs(out.vx) < 1e-6)
                    phase = distance <= capture_position ? DiffPhase::final_align : DiffPhase::path_error;
                linear_position = linear_target = 0.0;
            } else if (std::abs(path_yaw_error) > std::max(capture_rotation, 0.05)) {
                linear_target = linear_position;  // stop translating while the heading is re-aligned
            }
        }
        if (phase == DiffPhase::final_align) {
            ramp = goal;
            angular_displacement = yaw_error;
        } else if (phase == DiffPhase::path_error) {
            ramp = goal;
            linear_position = linear_target = angular_displacement = 0.0;
        }
        Regulated linear;
        if (!position_regulator(linear_position, out.vx, out_acceleration.vx, linear_target,
                                limits.linear_velocity, limits.linear_acceleration,
                                limits.linear_deceleration, limits.linear_reversal_deceleration,
                                limits.linear_jerk, dt, linear))
            return false;
        if (linear.velocity * straight_direction < 0)
            linear = Regulated{};  // never reverse on a fixed segment
        Regulated angular;
        if (!position_regulator(0.0, out.wz, out_acceleration.wz, angular_displacement,
                                limits.angular_velocity, limits.angular_acceleration,
                                limits.angular_deceleration, limits.angular_reversal_deceleration,
                                limits.angular_jerk, dt, angular))
            return false;
        if (phase == DiffPhase::final_align && distance <= capture_position &&
            std::abs(yaw_error) <= capture_rotation && std::abs(angular.velocity) < 1e-6) {
            angular = Regulated{};
            if (std::abs(linear.velocity) < 1e-6) settled = goal;
        }
        out_acceleration = {linear.acceleration, 0.0, angular.acceleration};
        body = {linear.velocity, 0.0, angular.velocity};
        return true;
    }

    ChassisOutput step(double now) {
        if (state == ChassisState::configured) return snapshot(now);
        const double dt = now - last_step.value_or(now);
        last_step = now;
        if (state != ChassisState::active) return snapshot(now);
        if (dt < 0 || dt > maximum_scheduler_dt()) {
            fault(ChassisReason::scheduler_discontinuity);
            return snapshot(now);
        }
        advance_pose(now);
        if (pending_period) {
            period = pending_period;
            pending_period.reset();
            return snapshot(now);
        }
        if (state != ChassisState::active) return snapshot(now);
        if (!command_fresh(now)) {
            clear_motion();
            reason = ChassisReason::command_timeout;
            return snapshot(now);
        }
        if (dt <= 0.0) return snapshot(now);  // no elapsed time: hold the current output
        const Twist2D nominal = pose_velocity(dt);
        target = nominal;
        if (pose_target) {
            Twist2D body;
            const bool ok = config.kinematics == ChassisKinematics::differential
                                ? differential_pose_step(dt, body)
                                : omni_pose_step(nominal, dt, body);
            if (!ok) {
                out = planned_velocity = out_acceleration = Twist2D{};
                output_valid_until.reset();
                ramp = pose;
                path_goal.reset();
                reason = ChassisReason::trajectory_error;
                return snapshot(now);
            }
            out = body;
            error.clear();
            output_valid_until = *command_stamp + config.command_timeout_s;
            planned_velocity = out;
            reason = config.kinematics == ChassisKinematics::differential && phase == DiffPhase::path_error
                         ? ChassisReason::path_error
                         : ChassisReason::tracking;
            return snapshot(now);
        }
        // Velocity mode: first-order limiter with the configured maxima.
        const auto& c = config;
        double vx = nominal.vx;
        double vy = c.kinematics == ChassisKinematics::omni ? nominal.vy : 0.0;
        double scale = std::min(1.0, c.max_linear / std::max(std::hypot(vx, vy), 1e-15));
        vx *= scale;
        vy *= scale;
        const double wz = std::clamp(nominal.wz, -c.max_angular, c.max_angular);
        const double dx = vx - out.vx;
        const double dy = vy - out.vy;
        scale = std::min(1.0, c.max_linear_acceleration * dt / std::max(std::hypot(dx, dy), 1e-15));
        const double dw =
            std::clamp(wz - out.wz, -c.max_angular_acceleration * dt, c.max_angular_acceleration * dt);
        out = {out.vx + dx * scale, out.vy + dy * scale, out.wz + dw};
        planned_velocity = out;
        // The first-order limiter has no acceleration state; handing its step rate to pose mode overshoots.
        out_acceleration = Twist2D{};
        output_valid_until = *command_stamp + c.command_timeout_s;
        reason = ChassisReason::tracking;
        return snapshot(now);
    }
};

Result<ChassisController> ChassisController::create(ChassisConfig config) {
    if (auto status = config.validate(); !status) return std::move(status).error();
    auto impl = std::make_unique<Impl>();
    impl->config = std::move(config);
    const auto& c = impl->config;
    auto& l = impl->limits;
    l.linear_velocity = c.max_linear;
    l.linear_acceleration = c.max_linear_acceleration;
    l.linear_deceleration = c.max_linear_deceleration;
    l.linear_reversal_deceleration = c.max_linear_reversal_deceleration;
    l.linear_jerk = c.max_linear_jerk;
    l.angular_velocity = c.max_angular;
    l.angular_acceleration = c.max_angular_acceleration;
    l.angular_deceleration = c.max_angular_deceleration;
    l.angular_reversal_deceleration = c.max_angular_reversal_deceleration;
    l.angular_jerk = c.max_angular_jerk;
    l.planner_mode = c.kinematics == ChassisKinematics::differential ? ChassisPlannerMode::straight
                                                                     : ChassisPlannerMode::axis_phase;
    return ChassisController(std::move(impl));
}

ChassisController::ChassisController(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
ChassisController::~ChassisController() = default;
ChassisController::ChassisController(ChassisController&&) noexcept = default;
ChassisController& ChassisController::operator=(ChassisController&&) noexcept = default;

const ChassisConfig& ChassisController::config() const noexcept { return impl_->config; }
const ChassisPoseLimits& ChassisController::pose_limits() const noexcept { return impl_->limits; }

Status ChassisController::tune_pose_limits(const ChassisPoseLimits& limits) {
    if (auto status = limits.validate(); !status) return status;
    if ((impl_->config.kinematics == ChassisKinematics::differential) !=
        (limits.planner_mode == ChassisPlannerMode::straight))
        return make_error(
            ErrorCode::invalid_argument,
            "chassis: planner_mode must be straight for differential, axis_phase or axis_time for omni");
    impl_->limits = limits;
    return {};
}

Status ChassisController::set_scheduler_period(double period) {
    if (auto status = require_positive(period, "scheduler period"); !status) return status;
    auto& m = *impl_;
    if (m.state == ChassisState::active && m.period && *m.period != period) {
        m.pending_period = period;
    } else {
        m.period = period;
        m.pending_period.reset();
    }
    return {};
}

std::optional<double> ChassisController::scheduler_period() const noexcept { return impl_->period; }

Status ChassisController::start(double now) {
    auto& m = *impl_;
    if (m.state != ChassisState::configured)
        return make_error(ErrorCode::invalid_argument, "chassis: can only start from configured");
    if (!m.period) return make_error(ErrorCode::invalid_argument, "chassis: scheduler period is not set");
    m.last_step = m.integrated_until = now;
    m.state = ChassisState::active;
    m.reason = ChassisReason::ready;
    return {};
}

Status ChassisController::pause(double /*now*/) {
    auto& m = *impl_;
    if (m.state != ChassisState::active)
        return make_error(ErrorCode::invalid_argument, "chassis: can only pause while active");
    m.state = ChassisState::paused;
    m.reason = ChassisReason::paused;
    m.clear_motion();
    return {};
}

Status ChassisController::resynchronize(double now) {
    auto& m = *impl_;
    if (m.state != ChassisState::paused && m.state != ChassisState::fault)
        return make_error(ErrorCode::invalid_argument,
                          "chassis: can only resynchronize while paused or faulted");
    m.clear_motion();
    m.last_step = m.integrated_until = now;
    m.state = ChassisState::active;
    m.reason = ChassisReason::resynchronized;
    return {};
}

void ChassisController::stop(double now) {
    auto& m = *impl_;
    m.state = ChassisState::stopped;
    m.reason = ChassisReason::stopped;
    m.clear_motion();
    m.last_step = now;
}

Result<bool> ChassisController::submit_velocity(const Twist2D& twist, double now, const std::string& source,
                                                int priority) {
    auto& m = *impl_;
    if (auto status = m.validate_command(finite(twist), source, priority); !status)
        return std::move(status).error();
    if (!m.accept_command(now, source, priority)) return false;
    m.target = twist;
    m.pose_target.reset();
    return true;
}

Result<bool> ChassisController::submit_pose(const Pose2D& pose, double now, const std::string& source,
                                            int priority) {
    auto& m = *impl_;
    if (auto status = m.validate_command(finite(pose), source, priority); !status)
        return std::move(status).error();
    if (!m.accept_command(now, source, priority)) return false;
    if (!m.pose_target) {
        m.ramp = m.pose;
        m.target = Twist2D{};
    }
    m.pose_target = pose;
    return true;
}

bool ChassisController::clear_velocity(std::optional<int> max_priority) {
    auto& m = *impl_;
    if (max_priority && m.command_priority > *max_priority) return false;
    m.clear_motion();
    return true;
}

Result<bool> ChassisController::submit_odometry(const Pose2D& pose, double stamp, double now) {
    auto& m = *impl_;
    if (!finite(pose) || !std::isfinite(stamp))
        return make_error(ErrorCode::invalid_argument, "chassis: odometry must be finite");
    if (m.state == ChassisState::stopped) return false;
    if (now - stamp < 0 || now - stamp > m.config.odom_timeout_s) return false;
    if (m.odom_stamp && stamp <= *m.odom_stamp) return false;
    m.advance_pose(now);
    m.pose = pose;
    m.odom_stamp = stamp;
    return true;
}

ChassisOutput ChassisController::service(double now, bool solve_enabled) {
    auto& m = *impl_;
    if (m.state != ChassisState::active) return m.snapshot(now);
    if (!solve_enabled) {
        m.clear_motion();
        m.last_step = m.integrated_until = now;
        m.reason = ChassisReason::solve_paused;
    } else if (!m.command_fresh(now)) {
        m.clear_motion();
        m.reason = ChassisReason::command_timeout;
    }
    return m.snapshot(now);
}

ChassisOutput ChassisController::step(double now) { return impl_->step(now); }
ChassisOutput ChassisController::snapshot(double now) const { return impl_->snapshot(now); }

ChassisCommandSnapshot ChassisController::command_snapshot() const {
    const auto& m = *impl_;
    ChassisCommandSnapshot snapshot;
    snapshot.pose_mode = m.pose_target.has_value();
    snapshot.velocity = m.target;
    if (m.pose_target) snapshot.pose = *m.pose_target;
    snapshot.timestamp = m.command_stamp.value_or(0.0);
    snapshot.expires_at = m.command_stamp ? *m.command_stamp + m.config.command_timeout_s : 0.0;
    snapshot.source = m.command_source;
    snapshot.priority = m.command_priority;
    return snapshot;
}

ChassisState ChassisController::state() const noexcept { return impl_->state; }
ChassisReason ChassisController::reason() const noexcept { return impl_->reason; }
const std::string& ChassisController::last_error() const noexcept { return impl_->error; }
const Pose2D& ChassisController::pose() const noexcept { return impl_->pose; }
const Pose2D& ChassisController::ramp_pose() const noexcept { return impl_->ramp; }
const Twist2D& ChassisController::output() const noexcept { return impl_->out; }
const Twist2D& ChassisController::nominal_target() const noexcept { return impl_->target; }
DiffPhase ChassisController::diff_phase() const noexcept { return impl_->phase; }
double ChassisController::maximum_scheduler_dt() const noexcept { return impl_->maximum_scheduler_dt(); }

}  // namespace kstone
