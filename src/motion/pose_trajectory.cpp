#include "kstone/motion/pose_trajectory.hpp"

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

Eigen::Vector3d clip_norm(const Eigen::Vector3d& value, double limit) {
    const double norm = value.norm();
    if (limit <= 0 || norm <= limit || norm < 1e-15) return value;
    return value * (limit / norm);
}

struct Step {
    Eigen::Vector3d position, velocity, acceleration;
};

struct Limits {
    double velocity, acceleration, deceleration, reversal, jerk;
};

Limits resolved(const MotionLimits& limits) {
    Limits r{limits.velocity, limits.acceleration > 0 ? limits.acceleration : kInfinity, 0, 0, limits.jerk};
    r.deceleration = limits.deceleration > 0 ? limits.deceleration : r.acceleration;
    r.reversal = limits.reversal_deceleration > 0 ? limits.reversal_deceleration : r.deceleration;
    return r;
}

// Constant-speed step (no acceleration limit); `per_axis` caps each world axis, otherwise the norm.
Step constant_speed(const Eigen::Vector3d& error, double dt, double velocity, bool per_axis) {
    Eigen::Vector3d next = error / dt;
    next =
        per_axis ? Eigen::Vector3d(next.cwiseMax(-velocity).cwiseMin(velocity)) : clip_norm(next, velocity);
    return {next * dt, next, Eigen::Vector3d::Zero()};
}

bool ruckig_ok(ruckig::Result result) {
    return result == ruckig::Result::Working || result == ruckig::Result::Finished;
}

// Braking step toward `error` that never crosses the target (used when Ruckig rejects the input).
Step safe_stop(const Eigen::Vector3d& error, const Eigen::Vector3d& velocity,
               const Eigen::Vector3d& acceleration, double dt, const Limits& limits) {
    const double distance = error.norm(), speed = velocity.norm();
    Eigen::Vector3d direction;
    if (distance > 1e-12)
        direction = error / distance;
    else if (speed > 1e-12)
        direction = velocity / speed;
    else
        return {error, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    const double along_velocity = velocity.dot(direction), along_acceleration = acceleration.dot(direction);
    const Eigen::Vector3d guidance =
        clip_norm(error / std::max(4 * dt, limits.velocity / limits.acceleration), limits.velocity);
    const double brake = velocity.dot(guidance) < -1e-12 ? limits.reversal : limits.deceleration;
    const double velocity_epsilon = std::max(1e-6, std::abs(along_velocity) * 1e-3);
    if (std::abs(along_velocity) < velocity_epsilon && std::abs(along_acceleration) < 1e-6)
        return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    double next_velocity = 0.0, next_acceleration = 0.0, step = 0.0;
    if (limits.jerk > 0) {
        const double desired =
            along_velocity > velocity_epsilon ? -brake
            : along_velocity < -velocity_epsilon
                ? brake
                : -std::copysign(std::min(std::abs(along_acceleration), brake), along_acceleration);
        const double change = std::clamp(desired - along_acceleration, -limits.jerk * dt, limits.jerk * dt);
        next_acceleration = along_acceleration + change;
        next_velocity = along_velocity + 0.5 * (along_acceleration + next_acceleration) * dt;
        step = along_velocity * dt + 0.5 * along_acceleration * dt * dt + change * dt * dt / 6;
    } else {
        const double change = along_velocity > velocity_epsilon    ? -std::min(along_velocity, brake * dt)
                              : along_velocity < -velocity_epsilon ? std::min(-along_velocity, brake * dt)
                                                                   : 0.0;
        next_velocity = along_velocity + change;
        next_acceleration = change / dt;
        step = 0.5 * (along_velocity + next_velocity) * dt;
    }
    if (along_velocity > velocity_epsilon) {
        next_velocity = std::max(0.0, next_velocity);
        step = std::clamp(step, 0.0, distance);
        if (step >= distance - 1e-12) next_velocity = next_acceleration = 0;
    } else if (along_velocity < -velocity_epsilon) {
        next_velocity = std::min(0.0, next_velocity);
        if (step > 0) {
            step = 0;
            next_velocity = std::min(0.0, along_velocity);
            next_acceleration = 0;
        }
    }
    return {direction * step, direction * next_velocity, direction * next_acceleration};
}

Eigen::Matrix3d basis_along(const Eigen::Vector3d& direction) {
    const Eigen::Vector3d x = direction.normalized();
    const Eigen::Vector3d helper =
        std::abs(x.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d y = x.cross(helper).normalized();
    Eigen::Matrix3d basis;
    basis << x, y, x.cross(y);
    return basis;
}

bool is_rigid(const Eigen::Isometry3d& pose) {
    const Eigen::Matrix4d& m = pose.matrix();
    if (!m.allFinite() || m.row(3) != Eigen::RowVector4d(0, 0, 0, 1)) return false;
    const Eigen::Matrix3d R = m.topLeftCorner<3, 3>();
    return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
           std::abs(R.determinant() - 1.0) < 1e-6;
}

// One cached synchronised three-axis plan (translation or rotation), resampled until its goal moves.
struct AxisPlan {
    ruckig::Trajectory<3> trajectory;
    Eigen::Matrix3d basis{Eigen::Matrix3d::Identity()};  // plan coordinates -> world
    Eigen::Matrix4d goal{Eigen::Matrix4d::Identity()};
    double elapsed{0.0};
    bool valid{false};
};

// Plan toward `error` from the current relative state. Translation uses world axes with per-axis limits;
// rotation uses an error-aligned basis whose primary axis may use the reversal limit and whose lateral axes use
// max(a, d, r).
bool plan_axis(AxisPlan& plan, ruckig::Ruckig<3>& generator, const Eigen::Vector3d& error,
               const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration, double dt,
               const Limits& limits, bool phase, bool directional, const Eigen::Matrix4d& goal) {
    plan.goal = goal;
    plan.basis.setIdentity();
    plan.elapsed = 0.0;
    plan.valid = false;
    if (directional) {
        const Eigen::Vector3d reference = error.norm() > 1e-12 ? error : velocity;
        if (reference.norm() > 1e-12) plan.basis = basis_along(reference);
    }
    const Eigen::Vector3d target = plan.basis.transpose() * error;
    const Eigen::Vector3d v = plan.basis.transpose() * velocity;
    const Eigen::Vector3d a = plan.basis.transpose() * acceleration;
    std::array<double, 3> max_acceleration{}, min_acceleration{};
    if (directional) {
        const double lateral = std::max({limits.acceleration, limits.deceleration, limits.reversal});
        const double primary = v[0] < -1e-12 ? limits.reversal : limits.acceleration;
        max_acceleration = {primary, lateral, lateral};
        min_acceleration = {-std::max(limits.deceleration, limits.reversal), -lateral, -lateral};
    } else {
        const double bound = std::max(limits.acceleration, limits.deceleration);
        max_acceleration.fill(bound);
        min_acceleration.fill(-bound);
    }
    for (std::size_t i = 0; i < 3; ++i) {
        const auto e = static_cast<Eigen::Index>(i);
        max_acceleration[i] = std::max(max_acceleration[i], a[e]);
        min_acceleration[i] = std::min(min_acceleration[i], a[e]);
    }
    ruckig::InputParameter<3> input;  // fixed-size: lives on the stack
    input.current_position = {0.0, 0.0, 0.0};
    input.current_velocity = {v[0], v[1], v[2]};
    input.target_position = {target[0], target[1], target[2]};
    input.max_velocity = {limits.velocity, limits.velocity, limits.velocity};
    input.min_velocity = std::array<double, 3>{-limits.velocity, -limits.velocity, -limits.velocity};
    if (limits.jerk > 0) input.current_acceleration = {a[0], a[1], a[2]};
    const std::array<ruckig::Synchronization, 2> order{ruckig::Synchronization::Phase,
                                                       ruckig::Synchronization::Time};
    generator.delta_time = dt;
    // Ruckig can reject a move that lands a rounding error past a profile-shape boundary (0.03 m at a 1.5,
    // j 15); slightly tighter limits move it off the boundary and stay within.
    for (const double scale : {1.0, 1.0 - 1e-6}) {
        std::array<double, 3> upper{}, lower{};
        for (std::size_t i = 0; i < 3; ++i) {
            upper[i] = max_acceleration[i] * scale;
            lower[i] = min_acceleration[i] * scale;
        }
        input.max_acceleration = upper;
        input.min_acceleration = lower;
        if (limits.jerk > 0) input.max_jerk = {limits.jerk * scale, limits.jerk * scale, limits.jerk * scale};
        for (std::size_t k = phase ? 0 : 1; k < order.size(); ++k) {
            input.synchronization = order[k];
            try {
                if (ruckig_ok(generator.calculate(input, plan.trajectory))) {
                    plan.valid = true;
                    return true;
                }
            } catch (const std::exception&) {
            }
        }
    }
    return false;
}

Step sample(const AxisPlan& plan, double from, double to) {
    std::array<double, 3> p0{}, p1{}, v1{}, a1{};
    plan.trajectory.at_time(from, p0);
    plan.trajectory.at_time(to, p1, v1, a1);
    return {plan.basis * (Eigen::Vector3d(p1.data()) - Eigen::Vector3d(p0.data())),
            plan.basis * Eigen::Vector3d(v1.data()), plan.basis * Eigen::Vector3d(a1.data())};
}

Step advance_axis(AxisPlan& plan, ruckig::Ruckig<3>& generator, const Eigen::Vector3d& error,
                  const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration,
                  const MotionLimits& configured, double dt, bool phase, bool rotation,
                  const Eigen::Matrix4d& goal) {
    if (!configured.enabled()) {
        plan.valid = false;
        return {error, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    }
    const Limits limits = resolved(configured);
    if (!std::isfinite(limits.acceleration)) {
        plan.valid = false;
        return constant_speed(error, dt, limits.velocity, !rotation);
    }
    if (!plan.valid || (plan.goal - goal).cwiseAbs().maxCoeff() > 1e-12) {
        if (!plan_axis(plan, generator, error, velocity, acceleration, dt, limits, phase, rotation, goal))
            return safe_stop(error, velocity, acceleration, dt, limits);
    }
    const double duration = plan.trajectory.get_duration();
    const double next = std::min(plan.elapsed + dt, duration);
    Step step = sample(plan, std::min(plan.elapsed, duration), next);
    plan.elapsed = next;
    if (next >= duration) {  // land exactly on the goal
        step.position = error;
        step.velocity.setZero();
        step.acceleration.setZero();
    }
    return step;
}

}  // namespace

const char* to_string(PoseSync value) noexcept {
    switch (value) {
        case PoseSync::axis_phase:
            return "axis_phase";
        case PoseSync::axis_time:
            return "axis_time";
    }
    return "unknown";
}

bool MotionLimits::enabled() const noexcept { return velocity > 0.0; }

Status MotionLimits::validate() const {
    for (double value : {velocity, acceleration, deceleration, reversal_deceleration, jerk})
        if (!std::isfinite(value) || value < 0.0)
            return make_error(ErrorCode::invalid_argument, "motion limits must be finite and >= 0");
    return {};
}

Status PoseTrajectoryConfig::validate() const {
    if (auto status = translation.validate(); !status) return status;
    if (auto status = rotation.validate(); !status) return status;
    if (!std::isfinite(lead_speed_scale) || lead_speed_scale < 0.0)
        return make_error(ErrorCode::invalid_argument,
                          "pose trajectory: lead_speed_scale must be finite and >= 0");
    return {};
}

struct PoseTrajectory::Impl {
    PoseTrajectoryConfig config;
    ruckig::Ruckig<3> generator{0.01};
    AxisPlan translation_plan, rotation_plan;
    Eigen::Isometry3d last{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d previous_observed{Eigen::Isometry3d::Identity()};
    Eigen::Vector3d translation_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d translation_acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rotation_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rotation_acceleration{Eigen::Vector3d::Zero()};
    bool initialized{false};
    bool has_observed{false};

    void zero_motion() noexcept {
        translation_velocity.setZero();
        translation_acceleration.setZero();
        rotation_velocity.setZero();
        rotation_acceleration.setZero();
    }
};

Result<PoseTrajectory> PoseTrajectory::create(PoseTrajectoryConfig config) {
    if (auto status = config.validate(); !status) return std::move(status).error();
    auto impl = std::make_unique<Impl>();
    impl->config = config;
    return PoseTrajectory(std::move(impl));
}

PoseTrajectory::PoseTrajectory(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
PoseTrajectory::~PoseTrajectory() = default;
PoseTrajectory::PoseTrajectory(PoseTrajectory&&) noexcept = default;
PoseTrajectory& PoseTrajectory::operator=(PoseTrajectory&&) noexcept = default;

const PoseTrajectoryConfig& PoseTrajectory::config() const noexcept { return impl_->config; }

Status PoseTrajectory::tune(const PoseTrajectoryConfig& config) {
    if (auto status = config.validate(); !status) return status;
    Impl& s = *impl_;
    const bool sync_changed = config.sync != s.config.sync;
    s.config = config;
    s.translation_plan.valid = false;
    s.rotation_plan.valid = false;
    if (sync_changed) s.zero_motion();
    return {};
}

void PoseTrajectory::reset() noexcept {
    Impl& s = *impl_;
    s.initialized = false;
    s.has_observed = false;
    s.translation_plan.valid = false;
    s.rotation_plan.valid = false;
    s.zero_motion();
}

Status PoseTrajectory::reset(const Eigen::Isometry3d& pose) {
    if (!is_rigid(pose)) return make_error(ErrorCode::invalid_argument, "pose trajectory: pose is not rigid");
    reset();
    impl_->last = pose;
    impl_->initialized = true;
    return {};
}

Status PoseTrajectory::step(const Eigen::Isometry3d& observed, const Eigen::Isometry3d& target, double dt) {
    if (!std::isfinite(dt) || dt < 0.0)
        return make_error(ErrorCode::invalid_argument, "pose trajectory: dt must be finite and >= 0");
    if (!is_rigid(observed) || !is_rigid(target))
        return make_error(ErrorCode::invalid_argument, "pose trajectory: pose is not rigid");
    Impl& s = *impl_;
    if (!s.initialized) {
        s.last = observed;
        s.initialized = true;
    }
    if (dt == 0.0) return {};  // hold: no time elapsed
    Eigen::Vector3d observed_linear = Eigen::Vector3d::Zero();
    if (s.has_observed) observed_linear = (observed.translation() - s.previous_observed.translation()) / dt;
    s.previous_observed = observed;
    s.has_observed = true;

    const PoseTrajectoryConfig& c = s.config;
    const bool phase = c.sync == PoseSync::axis_phase;
    Eigen::Isometry3d next = s.last;

    Eigen::Matrix4d translation_goal = Eigen::Matrix4d::Identity();
    translation_goal.topRightCorner<3, 1>() = target.translation();
    const Eigen::Vector3d linear_error = target.translation() - s.last.translation();
    const Step translation =
        advance_axis(s.translation_plan, s.generator, linear_error, s.translation_velocity,
                     s.translation_acceleration, c.translation, dt, phase, false, translation_goal);
    next.translation() += translation.position;
    s.translation_velocity = translation.velocity;
    s.translation_acceleration = translation.acceleration;

    Eigen::Matrix4d rotation_goal = Eigen::Matrix4d::Identity();
    rotation_goal.topLeftCorner<3, 3>() = target.linear();
    const Eigen::Matrix3d last_rotation = s.last.linear();
    const Eigen::AngleAxisd difference(Eigen::Matrix3d(last_rotation.transpose() * target.linear()));
    const Eigen::Vector3d rotation_error = last_rotation * difference.axis() * difference.angle();
    const Step rotation = advance_axis(s.rotation_plan, s.generator, rotation_error, s.rotation_velocity,
                                       s.rotation_acceleration, c.rotation, dt, phase, true, rotation_goal);
    s.rotation_velocity = rotation.velocity;
    s.rotation_acceleration = rotation.acceleration;
    const double angle = rotation.position.norm();
    if (angle > 0.0)
        next.linear() =
            Eigen::AngleAxisd(angle, rotation.position / angle).toRotationMatrix() * last_rotation;

    // Leash: keep the ramp within lead_speed_scale * velocity * lead_period of the observed frame on each world
    // axis, but only where it leads the frame in the direction still to go. A clamped axis takes the frame's own
    // velocity (capped at the limit) and the translation plan is rebuilt from the clamped pose.
    if (c.lead_speed_scale > 0 && c.translation.enabled()) {
        const double velocity = c.translation.velocity;
        const double bound = c.lead_speed_scale * velocity * PoseTrajectoryConfig::lead_period;
        const Eigen::Vector3d lead = next.translation() - observed.translation();
        const Eigen::Vector3d remaining = target.translation() - next.translation();
        bool clamped = false;
        for (Eigen::Index axis = 0; axis < 3; ++axis) {
            if (std::abs(lead[axis]) <= bound || lead[axis] * remaining[axis] <= 0) continue;
            next.translation()[axis] = observed.translation()[axis] + std::copysign(bound, lead[axis]);
            s.translation_velocity[axis] = std::clamp(observed_linear[axis], -velocity, velocity);
            s.translation_acceleration[axis] = 0;
            clamped = true;
        }
        if (clamped) s.translation_plan.valid = false;
    }
    s.last = next;
    return {};
}

bool PoseTrajectory::initialized() const noexcept { return impl_->initialized; }
const Eigen::Isometry3d& PoseTrajectory::ramp_pose() const noexcept { return impl_->last; }
const Eigen::Vector3d& PoseTrajectory::linear_velocity() const noexcept {
    return impl_->translation_velocity;
}
const Eigen::Vector3d& PoseTrajectory::angular_velocity() const noexcept { return impl_->rotation_velocity; }

bool PoseTrajectory::stopped() const noexcept {
    const Impl& s = *impl_;
    return s.translation_velocity.norm() <= 1e-9 && s.translation_acceleration.norm() <= 1e-9 &&
           s.rotation_velocity.norm() <= 1e-9 && s.rotation_acceleration.norm() <= 1e-9;
}

}  // namespace kstone
