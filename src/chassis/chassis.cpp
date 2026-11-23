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

// Acceleration cap for one axis: deceleration while moving against the displacement or at the goal.
double acceleration_cap(double velocity, double displacement, double acceleration, double deceleration) {
    if (velocity * displacement < -1e-12 || std::abs(displacement) < 1e-9) return deceleration;
    return acceleration;
}

// Distance one axis can cover in about 1000 s from rest; farther goals are planned toward this horizon and
// re-planned every cycle, since Ruckig rejects trajectories longer than ~7.6e3 s.
double planning_horizon(double velocity, double acceleration) {
    constexpr double t = 1000.0;
    double horizon = std::numeric_limits<double>::infinity();
    if (std::isfinite(velocity)) horizon = std::min(horizon, velocity * t / 2);
    if (std::isfinite(acceleration)) horizon = std::min(horizon, acceleration * t * t / 8);
    return horizon;
}

double clamp_magnitude(double value, double limit) { return std::clamp(value, -limit, limit); }

// Pose mode drives a Ruckig profile planned on its own reference pose (integrated from the profile, not
// overwritten by odometry while it moves), so the base's command-to-odometry delay does not matter: any
// feedback on a delayed odometry overshoots by about gain x delay x distance. The odometry is checked
// against the reference the way ros2_control's joint_trajectory_controller checks a trajectory
// (constraints.trajectory during the motion, goal tolerance and goal_time at the end):
// - path: the odometry must stay within kPathPosition / kPathYaw of the reference as it was at some time in
//   the last kTrackLagS (the base may lag the reference by up to that). Leaving that tube means another
//   source moves the base (remote control, a push): the goal is taken as done where the base is.
// - goal: when the profile ends inside the tolerance the goal is latched (see Impl::arrived); outside it
//   (slip, a base that lagged or overshot) the remaining error is planned again once the odometry has been
//   still (within kRestPosition / kRestYaw) for kRestS, from where the base really stopped.
constexpr double kPathPosition = 0.1;
constexpr double kPathYaw = 0.2;
constexpr double kTrackLagS = 1.0;
constexpr double kRestS = 0.2;
constexpr double kRestPosition = 1e-4;
constexpr double kRestYaw = 2e-4;

double lateral_or(double lateral, double linear) { return lateral > 0.0 ? lateral : linear; }

// Largest world-axis box whose corners stay inside the body-frame ellipse (vx/x)^2 + (vy/y)^2 <= 1 at the
// heading (c, s) = (cos yaw, sin yaw). Equal x and y give the yaw-independent x/sqrt(2) of the planar norm.
std::array<double, 2> world_axis_bounds(double x, double y, double c, double s) {
    const double ix = 1.0 / (x * x);  // 0 for an unbounded axis
    const double iy = 1.0 / (y * y);
    const double qxx = c * c * ix + s * s * iy;
    const double qyy = s * s * ix + c * c * iy;
    const double qxy = std::abs(c * s * (ix - iy));
    const double rho = qxx > 0.0 && qyy > 0.0 ? std::min(1.0, qxy / std::sqrt(qxx * qyy)) : 0.0;
    const double k = 2.0 * (1.0 + rho);
    return {qxx > 0.0 ? 1.0 / std::sqrt(k * qxx) : kInfinity,
            qyy > 0.0 ? 1.0 / std::sqrt(k * qyy) : kInfinity};
}

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
        case ChassisReason::yielded:
            return "yielded";
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
        {odom_timeout_s, "odom_timeout_s"},
        {max_lateral_velocity, "max_lateral_velocity"},
        {max_lateral_acceleration, "max_lateral_acceleration"},
        {max_lateral_deceleration, "max_lateral_deceleration"},
    };
    for (const auto& [value, name] : nonnegative)
        if (auto status = require_nonnegative(value, name); !status) return status;
    const std::pair<double, const char*> positive[] = {
        {command_timeout_s, "command_timeout_s"},
        {max_dt_s, "max_dt_s"},
        {max_linear_velocity, "max_linear_velocity"},
        {max_angular_velocity, "max_angular_velocity"},
        {max_linear_acceleration, "max_linear_acceleration"},
        {max_linear_deceleration, "max_linear_deceleration"},
        {max_angular_acceleration, "max_angular_acceleration"},
        {max_angular_deceleration, "max_angular_deceleration"},
        {position_tolerance, "position_tolerance"},
        {orientation_tolerance, "orientation_tolerance"},
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
    for (double value : {max_linear_velocity, max_linear_acceleration, max_linear_deceleration,
                         max_lateral_velocity, max_lateral_acceleration, max_lateral_deceleration,
                         max_angular_velocity, max_angular_acceleration, max_angular_deceleration})
        if (auto status = require_nonnegative(value, "pose limit"); !status) return status;
    switch (planner_mode) {
        case ChassisPlannerMode::axis_phase:
        case ChassisPlannerMode::axis_time:
            return {};
    }
    return make_error(ErrorCode::invalid_argument, "chassis: unknown planner_mode");
}

bool ChassisPoseLimits::bypass() const noexcept {
    for (double value : {max_linear_velocity, max_linear_acceleration, max_linear_deceleration,
                         max_lateral_velocity, max_lateral_acceleration, max_lateral_deceleration,
                         max_angular_velocity, max_angular_acceleration, max_angular_deceleration})
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
    // Last odometry and whether the pose holds it (outside pose mode, or latched at the goal).
    std::optional<Pose2D> odom_pose;
    bool odom_applied{false};
    std::optional<double> still_since;
    // Reference poses of the last kTrackLagS (fixed storage: the cycle must not allocate).
    static constexpr std::size_t kHistory = 256;
    std::array<std::pair<double, Pose2D>, kHistory> history{};
    std::size_t history_head{0}, history_size{0};
    // Pose mode latched at this goal: the output stays zero until a goal outside the tolerance of it comes,
    // so a reached goal is not dithered at speeds the base cannot follow, and a base moved by someone else
    // (remote control, a push) is not driven back.
    std::optional<Pose2D> arrived;
    bool yielded{false};  // the latch came from leaving the path, not from reaching the goal
    std::string command_source;
    int command_priority{0};
    Pose2D pose, ramp;
    std::optional<Pose2D> pose_target, planned_pose_override;
    Twist2D target, out, planned_velocity, out_acceleration;
    // Reused generator: fixed-DOF Ruckig keeps its state in std::array.
    ruckig::Ruckig<3> omni{0.01};
    ruckig::InputParameter<3> omni_input;
    ruckig::OutputParameter<3> omni_output;

    double maximum_scheduler_dt() const {
        double cycle = period.value_or(0.0);
        if (pending_period) cycle = std::max(cycle, *pending_period);
        return std::max(config.max_dt_s, cycle * static_cast<double>(config.allowed_missed_cycles));
    }

    bool command_fresh(double now) const {
        return command_stamp && now - *command_stamp >= 0 && now - *command_stamp <= config.command_timeout_s;
    }

    double odom_timeout() const {
        return config.odom_timeout_s > 0 ? config.odom_timeout_s : std::numeric_limits<double>::infinity();
    }

    bool odom_fresh(double now) const {
        return odom_stamp && now - *odom_stamp >= 0 && now - *odom_stamp <= odom_timeout();
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
        arrived.reset();
        yielded = false;
        history_size = 0;
    }

    void record_reference(double now) {
        if (history_size > 0 && history[(history_head + history_size - 1) % kHistory].first >= now) return;
        history[(history_head + history_size) % kHistory] = {now, pose};
        if (history_size < kHistory)
            ++history_size;
        else
            history_head = (history_head + 1) % kHistory;
        while (history_size > 1 && history[(history_head + 1) % kHistory].first <= now - kTrackLagS) {
            history_head = (history_head + 1) % kHistory;
            --history_size;
        }
    }

    // The odometry lies within the path tolerance of the current reference or of one from the last kTrackLagS.
    bool on_path(const Pose2D& measured, double stamp) const {
        const auto near = [&](const Pose2D& reference) {
            return std::hypot(reference.x - measured.x, reference.y - measured.y) <= kPathPosition &&
                   std::abs(wrap_angle(reference.yaw - measured.yaw)) <= kPathYaw;
        };
        if (near(pose)) return true;
        for (std::size_t i = 0; i < history_size; ++i) {
            const auto& [t, reference] = history[(history_head + i) % kHistory];
            if (t >= stamp - kTrackLagS && near(reference)) return true;
        }
        return false;
    }

    // Take the goal as done where the base is (another source moved it).
    void yield_goal(const Pose2D& measured) {
        arrived = *pose_target;
        yielded = true;
        reason = ChassisReason::yielded;
        planned_pose_override.reset();
        pose = ramp = measured;
        out = planned_velocity = out_acceleration = Twist2D{};
        odom_applied = true;
        history_size = 0;
    }

    bool within_tolerance(const Pose2D& from) const {
        const Pose2D& goal = *pose_target;
        const double dx = goal.x - from.x;
        const double dy = goal.y - from.y;
        // differential: the part abeam of the heading is never driven out
        const double position = config.kinematics == ChassisKinematics::differential
                                    ? std::abs(dx * std::cos(from.yaw) + dy * std::sin(from.yaw))
                                    : std::hypot(dx, dy);
        return position <= config.position_tolerance &&
               std::abs(wrap_angle(goal.yaw - from.yaw)) <= config.orientation_tolerance;
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
        output.source = odom_applied && odom_fresh(now) ? PoseSource::odometry : PoseSource::internal;
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

    // The omni Ruckig sample is the exact reference pose at the step's end; only the step itself takes it
    // (an odometry callback in between integrates up to its own time).
    void advance_pose(double now, bool at_step = false) {
        const auto begin_stamp = integrated_until;
        integrated_until = now;
        if (!begin_stamp || state != ChassisState::active) return;
        double begin = *begin_stamp;
        if (now < begin || now - begin > maximum_scheduler_dt()) {
            fault(ChassisReason::scheduler_discontinuity);
            return;
        }
        if (odom_stamp && odom_applied) begin = std::max(begin, *odom_stamp + odom_timeout());
        const double end = output_valid_until ? std::min(now, *output_valid_until) : now;
        const double dt = output_valid_until ? std::max(0.0, end - begin) : 0.0;
        if (at_step && pose_target && planned_pose_override)
            pose = *planned_pose_override;
        else
            pose = integrate_se2(pose, pose_target ? planned_velocity : out, dt);
        if (at_step) planned_pose_override.reset();
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
        if (distance > config.position_tolerance) {
            double speed = distance / step_dt;
            if (limits.max_linear_velocity > 0) speed = std::min(speed, limits.max_linear_velocity);
            if (limits.max_linear_acceleration > 0)
                speed = std::min(speed, std::sqrt(2 * limits.max_linear_acceleration * distance));
            wx = speed * dx / distance;
            wy = speed * dy / distance;
        }
        const double c = std::cos(pose.yaw);
        const double s = std::sin(pose.yaw);
        Twist2D result{c * wx + s * wy, -s * wx + c * wy, 0.0};
        if (config.kinematics == ChassisKinematics::differential) result.vy = 0.0;
        const double error_yaw = wrap_angle(goal.yaw - pose.yaw);
        if (std::abs(error_yaw) > config.orientation_tolerance) {
            double rate = std::abs(error_yaw) / step_dt;
            if (limits.max_angular_velocity > 0) rate = std::min(rate, limits.max_angular_velocity);
            if (limits.max_angular_acceleration > 0)
                rate = std::min(rate, std::sqrt(2 * limits.max_angular_acceleration * std::abs(error_yaw)));
            result.wz = std::copysign(rate, error_yaw);
        }
        return result;
    }

    bool pose_step(const Twist2D& nominal, double dt, Twist2D& body) {
        const Pose2D& goal = *pose_target;
        if (limits.bypass()) {
            out_acceleration = Twist2D{};
            ramp = goal;
            body = nominal;
            return true;
        }
        return config.kinematics == ChassisKinematics::differential ? differential_pose_step(dt, body)
                                                                    : omni_pose_step(dt, body);
    }

    // Differential: the same synchronised planner on (distance along the current heading, unused, yaw). The
    // goal's position is projected onto the heading and the plan is redone every cycle, so the base drives
    // forward/backward as far as the goal lies ahead while it turns to the goal yaw; it never moves sideways.
    bool differential_pose_step(double dt, Twist2D& body) {
        const Pose2D& goal = *pose_target;
        const Pose2D start = pose;
        const double c = std::cos(start.yaw);
        const double s = std::sin(start.yaw);
        const double lv = or_infinity(limits.max_linear_velocity);
        const double av = or_infinity(limits.max_angular_velocity);
        const double la = or_infinity(limits.max_linear_acceleration);
        const double ld = limits.max_linear_deceleration > 0 ? limits.max_linear_deceleration : la;
        const double aa = or_infinity(limits.max_angular_acceleration);
        const double ad = limits.max_angular_deceleration > 0 ? limits.max_angular_deceleration : aa;
        const double yaw_goal = start.yaw + wrap_angle(goal.yaw - start.yaw);
        const double along = (goal.x - start.x) * c + (goal.y - start.y) * s;
        std::array<double, 3> velocity{planned_velocity.vx, 0.0, planned_velocity.wz};
        std::array<double, 3> acceleration{out_acceleration.vx, 0.0, out_acceleration.wz};
        const std::array<double, 3> position{0.0, 0.0, start.yaw};
        std::array<double, 3> goal_position{along, 0.0, yaw_goal};
        for (std::size_t i : {std::size_t{0}, std::size_t{2}})
            if (std::abs(goal_position[i] - position[i]) < 1e-9 && std::abs(velocity[i]) < 1e-8)
                velocity[i] = acceleration[i] = 0.0;
        const std::array<double, 3> max_velocity{lv, 1.0, av};
        const std::array<double, 3> max_acceleration{
            acceleration_cap(velocity[0], along, la, ld), 1.0,
            acceleration_cap(velocity[2], yaw_goal - start.yaw, aa, ad)};
        for (std::size_t i = 0; i < 3; ++i) {
            snap(velocity[i], max_velocity[i]);
            snap(acceleration[i], max_acceleration[i]);
        }
        goal_position[0] = clamp_magnitude(along, planning_horizon(lv, la));
        goal_position[2] = start.yaw + clamp_magnitude(yaw_goal - start.yaw, planning_horizon(av, aa));
        if (!plan(position, velocity, acceleration, goal_position, max_velocity, max_acceleration, dt))
            return false;
        const auto& v = omni_output.new_velocity;
        const auto& a = omni_output.new_acceleration;
        const auto& p = omni_output.new_position;
        body = {zero_small(v[0]), 0.0, zero_small(v[2])};
        out_acceleration = {a[0], 0.0, a[2]};
        ramp = {start.x + p[0] * c, start.y + p[0] * s, wrap_angle(p[2])};
        return true;
    }

    bool plan(const std::array<double, 3>& position, const std::array<double, 3>& velocity,
              const std::array<double, 3>& acceleration, const std::array<double, 3>& goal_position,
              const std::array<double, 3>& max_velocity, const std::array<double, 3>& max_acceleration,
              double dt) {
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
        // No jerk limit: the base firmware smooths its own acceleration.
        const bool phase_sync = limits.planner_mode != ChassisPlannerMode::axis_time;
        input.synchronization = phase_sync ? ruckig::Synchronization::Phase : ruckig::Synchronization::Time;
        if (!run_ruckig(omni, input, omni_output, dt, "chassis Ruckig update failed: ", error)) {
            if (!phase_sync) return false;
            input.synchronization = ruckig::Synchronization::Time;
            if (!run_ruckig(omni, input, omni_output, dt, "chassis Ruckig update failed: ", error))
                return false;
        }
        return true;
    }

    bool omni_pose_step(double dt, Twist2D& body) {
        const Pose2D start = pose;
        const double c = std::cos(start.yaw);
        const double s = std::sin(start.yaw);
        const Twist2D& ff = planned_velocity;
        std::array<double, 3> velocity{c * ff.vx - s * ff.vy, s * ff.vx + c * ff.vy, ff.wz};
        std::array<double, 3> acceleration{c * out_acceleration.vx - s * out_acceleration.vy,
                                           s * out_acceleration.vx + c * out_acceleration.vy,
                                           out_acceleration.wz};
        // Body x uses the linear limits, body y the lateral ones (0 = linear); both map to world-axis bounds.
        const double lv = or_infinity(limits.max_linear_velocity);
        const double av = or_infinity(limits.max_angular_velocity);
        const double la = or_infinity(limits.max_linear_acceleration);
        const double ld = limits.max_linear_deceleration > 0 ? limits.max_linear_deceleration : la;
        const double yv = or_infinity(lateral_or(limits.max_lateral_velocity, limits.max_linear_velocity));
        const double ya =
            or_infinity(lateral_or(limits.max_lateral_acceleration, limits.max_linear_acceleration));
        const double yd_set = lateral_or(limits.max_lateral_deceleration, limits.max_linear_deceleration);
        const double yd = yd_set > 0 ? yd_set : ya;
        const auto wv = world_axis_bounds(lv, yv, c, s);
        const auto wa = world_axis_bounds(la, ya, c, s);
        const auto wd = world_axis_bounds(ld, yd, c, s);
        const double aa = or_infinity(limits.max_angular_acceleration);
        const double ad = limits.max_angular_deceleration > 0 ? limits.max_angular_deceleration : aa;
        const Pose2D& goal = *pose_target;
        const double yaw_goal = start.yaw + wrap_angle(goal.yaw - start.yaw);
        const std::array<double, 3> position{start.x, start.y, start.yaw};
        std::array<double, 3> goal_position{goal.x, goal.y, yaw_goal};
        for (std::size_t i = 0; i < 3; ++i)
            if (std::abs(goal_position[i] - position[i]) < 1e-9 && std::abs(velocity[i]) < 1e-8)
                velocity[i] = acceleration[i] = 0.0;
        const double dx = goal.x - start.x;
        const double dy = goal.y - start.y;
        const std::array<double, 3> max_velocity{wv[0], wv[1], av};
        const std::array<double, 3> max_acceleration{
            acceleration_cap(velocity[0], dx, wa[0], wd[0]), acceleration_cap(velocity[1], dy, wa[1], wd[1]),
            acceleration_cap(velocity[2], yaw_goal - start.yaw, aa, ad)};
        for (std::size_t i = 0; i < 3; ++i) {
            snap(velocity[i], max_velocity[i]);
            snap(acceleration[i], max_acceleration[i]);
        }
        const double reach = std::min(planning_horizon(wv[0], wa[0]), planning_horizon(wv[1], wa[1]));
        const double distance = std::hypot(dx, dy);
        if (distance > reach) {
            goal_position[0] = start.x + dx * reach / distance;
            goal_position[1] = start.y + dy * reach / distance;
        }
        goal_position[2] = start.yaw + clamp_magnitude(yaw_goal - start.yaw, planning_horizon(av, aa));
        if (!plan(position, velocity, acceleration, goal_position, max_velocity, max_acceleration, dt))
            return false;
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

    ChassisOutput step(double now) {
        if (state == ChassisState::configured) return snapshot(now);
        const double dt = now - last_step.value_or(now);
        last_step = now;
        if (state != ChassisState::active) return snapshot(now);
        if (dt < 0 || dt > maximum_scheduler_dt()) {
            fault(ChassisReason::scheduler_discontinuity);
            return snapshot(now);
        }
        advance_pose(now, true);
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
            output_valid_until = *command_stamp + config.command_timeout_s;
            reason = ChassisReason::tracking;
            const bool measured = odom_pose && odom_fresh(now);
            if (arrived &&
                std::hypot(pose_target->x - arrived->x, pose_target->y - arrived->y) <=
                    config.position_tolerance &&
                std::abs(wrap_angle(pose_target->yaw - arrived->yaw)) <= config.orientation_tolerance) {
                out = planned_velocity = out_acceleration = Twist2D{};
                ramp = pose;
                // Moved off a reached goal afterwards: someone else moved the base (it only latches inside).
                if (measured && !within_tolerance(*odom_pose)) yielded = true;
                if (yielded) reason = ChassisReason::yielded;
                return snapshot(now);
            }
            if (arrived) {  // a new goal: plan from where the base is
                arrived.reset();
                yielded = false;
                odom_applied = false;
                if (measured) pose = *odom_pose;
                history_size = 0;
            }
            Twist2D body;
            if (!pose_step(nominal, dt, body)) {
                out = planned_velocity = out_acceleration = Twist2D{};
                output_valid_until.reset();
                ramp = pose;
                reason = ChassisReason::trajectory_error;
                return snapshot(now);
            }
            error.clear();
            planned_velocity = body;
            if (body.vx == 0.0 && body.vy == 0.0 && body.wz == 0.0 && within_tolerance(pose)) {
                // The plan is done. Latch if the base is inside the tolerance; otherwise wait for it to come
                // to rest and plan the remaining error from there.
                out = out_acceleration = Twist2D{};
                if (!measured || within_tolerance(*odom_pose)) {
                    arrived = *pose_target;
                } else if (still_since && now - *still_since >= kRestS) {
                    planned_pose_override.reset();
                    pose = ramp = *odom_pose;
                    history_size = 0;
                }
                return snapshot(now);
            }
            out = body;
            return snapshot(now);
        }
        // Velocity mode: the command is only clamped to the configured maxima; the base firmware ramps it.
        const auto& c = config;
        double vx = nominal.vx;
        double vy = c.kinematics == ChassisKinematics::omni ? nominal.vy : 0.0;
        const double vy_max = lateral_or(c.max_lateral_velocity, c.max_linear_velocity);
        const double scale =
            std::min(1.0, 1.0 / std::max(std::hypot(vx / c.max_linear_velocity, vy / vy_max), 1e-15));
        const double wz = std::clamp(nominal.wz, -c.max_angular_velocity, c.max_angular_velocity);
        out = {vx * scale, vy * scale, wz};
        planned_velocity = out;
        ramp = pose;  // no trajectory in velocity mode: the planned pose is where the base is
        // Velocity mode has no acceleration state; handing its step rate to pose mode overshoots.
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
    l.max_linear_velocity = c.max_linear_velocity;
    l.max_linear_acceleration = c.max_linear_acceleration;
    l.max_linear_deceleration = c.max_linear_deceleration;
    l.max_lateral_velocity = c.max_lateral_velocity;
    l.max_lateral_acceleration = c.max_lateral_acceleration;
    l.max_lateral_deceleration = c.max_lateral_deceleration;
    l.max_angular_velocity = c.max_angular_velocity;
    l.max_angular_acceleration = c.max_angular_acceleration;
    l.max_angular_deceleration = c.max_angular_deceleration;
    l.planner_mode = ChassisPlannerMode::axis_phase;
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

Status ChassisController::start(double now, const std::optional<Pose2D>& pose) {
    auto& m = *impl_;
    if (m.state != ChassisState::configured)
        return make_error(ErrorCode::invalid_argument, "chassis: can only start from configured");
    if (!m.period) return make_error(ErrorCode::invalid_argument, "chassis: scheduler period is not set");
    if (pose && !finite(*pose))
        return make_error(ErrorCode::invalid_argument, "chassis: pose must be finite");
    if (pose) m.pose = m.ramp = *pose;
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

Status ChassisController::resynchronize(double now, const std::optional<Pose2D>& pose) {
    auto& m = *impl_;
    if (m.state != ChassisState::paused && m.state != ChassisState::fault)
        return make_error(ErrorCode::invalid_argument,
                          "chassis: can only resynchronize while paused or faulted");
    if (pose && !finite(*pose))
        return make_error(ErrorCode::invalid_argument, "chassis: pose must be finite");
    if (pose) m.pose = *pose;
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
    if (m.pose_target) {  // a velocity command takes over a pose goal: drop its trajectory state with it
        m.pose_target.reset();
        m.ramp = m.pose;
        m.planned_pose_override.reset();
        m.history_size = 0;
        // `arrived` is kept: a goal reached before the base was driven away is not chased again.
    }
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
    if (now - stamp < 0 || now - stamp > m.odom_timeout()) return false;
    if (m.odom_stamp && stamp <= *m.odom_stamp) return false;
    m.advance_pose(now);
    const bool still = m.odom_pose &&
                       std::hypot(pose.x - m.odom_pose->x, pose.y - m.odom_pose->y) <= kRestPosition &&
                       std::abs(wrap_angle(pose.yaw - m.odom_pose->yaw)) <= kRestYaw;
    if (!still)
        m.still_since.reset();
    else if (!m.still_since)
        m.still_since = stamp;
    m.odom_pose = pose;
    m.odom_stamp = stamp;
    if (m.pose_target && !m.arrived && m.state == ChassisState::active && !m.on_path(pose, stamp))
        m.yield_goal(pose);
    // Outside pose mode the pose follows the odometry; in pose mode only once latched at the goal.
    m.odom_applied = !m.pose_target || m.arrived.has_value();
    if (m.odom_applied) m.pose = pose;
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

ChassisOutput ChassisController::step(double now) {
    ChassisOutput output = impl_->step(now);
    if (impl_->pose_target && !impl_->arrived) impl_->record_reference(now);
    return output;
}
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
double ChassisController::maximum_scheduler_dt() const noexcept { return impl_->maximum_scheduler_dt(); }

}  // namespace kstone
