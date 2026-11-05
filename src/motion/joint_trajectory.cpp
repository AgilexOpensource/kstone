#include "kstone/motion/joint_trajectory.hpp"

#include <ruckig/ruckig.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <utility>

namespace kstone {

namespace {

constexpr double kTwoPi = 6.283185307179586;

bool finite_positive(double value) { return std::isfinite(value) && value > 0.0; }

bool same(const JointTrajectoryConfig& a, const JointTrajectoryConfig& b) {
    return a.trigger == b.trigger && a.sync == b.sync && a.acceleration_factor == b.acceleration_factor &&
           a.jerk_factor == b.jerk_factor && a.velocity_scale == b.velocity_scale &&
           a.velocity_cap.size() == b.velocity_cap.size() && a.velocity_cap == b.velocity_cap;
}

ruckig::Synchronization to_ruckig(JointSync sync) {
    switch (sync) {
        case JointSync::phase:
            return ruckig::Synchronization::Phase;
        case JointSync::none:
            return ruckig::Synchronization::None;
        case JointSync::time:
            break;
    }
    return ruckig::Synchronization::Time;
}

}  // namespace

const char* to_string(JointTrigger value) noexcept {
    switch (value) {
        case JointTrigger::direct:
            return "direct";
        case JointTrigger::threshold:
            return "threshold";
        case JointTrigger::continuous:
            return "continuous";
    }
    return "unknown";
}

const char* to_string(JointSync value) noexcept {
    switch (value) {
        case JointSync::time:
            return "time";
        case JointSync::phase:
            return "phase";
        case JointSync::none:
            return "none";
    }
    return "unknown";
}

Status JointTrajectoryConfig::validate(Eigen::Index size) const {
    if (!finite_positive(acceleration_factor) || !finite_positive(jerk_factor))
        return make_error(ErrorCode::invalid_argument, "joint trajectory: factors must be finite and > 0");
    if (!std::isfinite(threshold) || threshold < 0.0)
        return make_error(ErrorCode::invalid_argument, "joint trajectory: threshold must be finite and >= 0");
    if (!std::isfinite(velocity_scale) || velocity_scale <= 0.0 || velocity_scale > 1.0)
        return make_error(ErrorCode::invalid_argument, "joint trajectory: velocity_scale must be in (0, 1]");
    if (velocity_cap.size() != 0 && velocity_cap.size() != size)
        return make_error(ErrorCode::invalid_argument,
                          "joint trajectory: velocity_cap needs one entry per joint");
    for (Eigen::Index i = 0; i < velocity_cap.size(); ++i)
        if (!(velocity_cap[i] > 0.0))
            return make_error(ErrorCode::invalid_argument,
                              "joint trajectory: velocity_cap entries must be > 0");
    return {};
}

struct JointTrajectory::Impl {
    Impl(std::size_t n) : generator(n, 0.01), input(n), output(n) {}

    JointTrajectoryConfig config;
    std::vector<bool> wrap;
    ruckig::Ruckig<ruckig::DynamicDOFs> generator;
    ruckig::InputParameter<ruckig::DynamicDOFs> input;
    ruckig::OutputParameter<ruckig::DynamicDOFs> output;
    Eigen::VectorXd reference, velocity, acceleration, limits, error;
    bool active{false};
    bool seeded_limits{false};
    double tau{0.0};

    void reset() noexcept {
        active = false;
        seeded_limits = false;
        tau = 0.0;
        velocity.setZero();
        acceleration.setZero();
    }

    Status update(double dt) {
        generator.delta_time = dt;
        generator.reset();  // replan from the previous reference state every cycle
        ruckig::Result result = ruckig::Result::Error;
        try {
            result = generator.update(input, output);
            if (result == ruckig::Result::ErrorSynchronizationCalculation &&
                input.synchronization !=
                    ruckig::Synchronization::None) {  // e.g. entering above the scaled cruise
                input.synchronization = ruckig::Synchronization::None;
                generator.reset();
                result = generator.update(input, output);
            }
        } catch (const std::exception& e) {
            return make_error(ErrorCode::internal, std::string("joint trajectory: Ruckig: ") + e.what());
        }
        if (result != ruckig::Result::Working && result != ruckig::Result::Finished)
            return make_error(ErrorCode::internal, "joint trajectory: Ruckig update failed: " +
                                                       std::to_string(static_cast<int>(result)));
        return {};
    }
};

Result<JointTrajectory> JointTrajectory::create(Eigen::Index size, JointTrajectoryConfig config,
                                                std::vector<bool> wrap) {
    if (size <= 0) return make_error(ErrorCode::invalid_argument, "joint trajectory: size must be > 0");
    if (!wrap.empty() && static_cast<Eigen::Index>(wrap.size()) != size)
        return make_error(ErrorCode::invalid_argument, "joint trajectory: wrap needs one entry per joint");
    if (auto status = config.validate(size); !status) return std::move(status).error();
    auto impl = std::make_unique<Impl>(static_cast<std::size_t>(size));
    impl->config = std::move(config);
    impl->wrap = wrap.empty() ? std::vector<bool>(static_cast<std::size_t>(size), false) : std::move(wrap);
    impl->reference = Eigen::VectorXd::Zero(size);
    impl->velocity = Eigen::VectorXd::Zero(size);
    impl->acceleration = Eigen::VectorXd::Zero(size);
    impl->limits = Eigen::VectorXd::Zero(size);
    impl->error = Eigen::VectorXd::Zero(size);
    return JointTrajectory(std::move(impl));
}

JointTrajectory::JointTrajectory(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
JointTrajectory::~JointTrajectory() = default;
JointTrajectory::JointTrajectory(JointTrajectory&&) noexcept = default;
JointTrajectory& JointTrajectory::operator=(JointTrajectory&&) noexcept = default;

Status JointTrajectory::configure(const JointTrajectoryConfig& config) {
    if (auto status = config.validate(size()); !status) return status;
    if (!same(impl_->config, config)) impl_->reset();
    impl_->config = config;
    return {};
}

const JointTrajectoryConfig& JointTrajectory::config() const noexcept { return impl_->config; }
void JointTrajectory::reset() noexcept { impl_->reset(); }

Status JointTrajectory::step(const Eigen::Ref<const Eigen::VectorXd>& measured,
                             const Eigen::Ref<const Eigen::VectorXd>& target,
                             const Eigen::Ref<const Eigen::VectorXd>& measured_velocity,
                             const Eigen::Ref<const Eigen::VectorXd>& velocity_limit, double dt) {
    Impl& s = *impl_;
    const Eigen::Index n = size();
    if (measured.size() != n || target.size() != n || measured_velocity.size() != n ||
        velocity_limit.size() != n)
        return make_error(ErrorCode::invalid_argument, "joint trajectory: input size mismatch");
    if (!measured.allFinite() || !target.allFinite() || !measured_velocity.allFinite())
        return make_error(ErrorCode::invalid_argument, "joint trajectory: inputs must be finite");
    if (!finite_positive(dt))
        return make_error(ErrorCode::invalid_argument, "joint trajectory: dt must be > 0");
    const JointTrajectoryConfig& c = s.config;
    if (c.trigger == JointTrigger::direct) {
        s.reference = target;
        return {};
    }
    s.tau = 0.0;
    for (Eigen::Index i = 0; i < n; ++i) {
        const double limit = velocity_limit[i];
        const double acceleration = limit * c.acceleration_factor;
        if (!finite_positive(limit) || !finite_positive(acceleration) ||
            !finite_positive(acceleration * c.jerk_factor))
            return make_error(ErrorCode::invalid_argument,
                              "joint trajectory: velocity limits must be finite and > 0");
        const double d = measured[i] - target[i];
        s.error[i] = s.wrap[static_cast<std::size_t>(i)] ? std::remainder(d, kTwoPi) : d;
        s.tau = std::max(s.tau, std::abs(s.error[i]) / limit);
    }
    const bool changed = !s.seeded_limits || s.limits != velocity_limit;
    if (c.trigger == JointTrigger::threshold) {  // engage past the threshold, let go within half a cycle
        if (s.active && s.tau < 0.5 * dt) {
            const double tau = s.tau;
            s.reset();
            s.tau = tau;
        }
        if (!s.active && s.tau <= c.threshold) {
            s.reference = target;
            return {};
        }
    }
    if (!s.active || changed) {
        s.reference = measured;
        s.velocity = measured_velocity;
        s.acceleration.setZero();
        s.limits = velocity_limit;
        s.seeded_limits = true;
        s.active = true;
    }
    s.input.synchronization = to_ruckig(c.sync);
    for (Eigen::Index i = 0; i < n; ++i) {
        const auto k = static_cast<std::size_t>(i);
        s.input.current_position[k] = s.reference[i];
        s.input.current_velocity[k] = s.velocity[i];
        s.input.current_acceleration[k] = s.acceleration[i];
        // A wrapped joint targets the shortest arc from where it is, expressed around the reference.
        s.input.target_position[k] =
            s.wrap[k] ? s.reference[i] + std::remainder(measured[i] - s.reference[i], kTwoPi) - s.error[i]
                      : target[i];
        double cruise = c.velocity_scale * s.limits[i];
        if (c.velocity_cap.size() != 0) cruise = std::min(cruise, c.velocity_cap[i]);
        s.input.max_velocity[k] = cruise;
        s.input.max_acceleration[k] = c.acceleration_factor * s.limits[i];
        s.input.max_jerk[k] = c.jerk_factor * s.input.max_acceleration[k];
    }
    if (auto status = s.update(dt); !status) return status;
    for (Eigen::Index i = 0; i < n; ++i) {
        const auto k = static_cast<std::size_t>(i);
        s.reference[i] = s.output.new_position[k];
        s.velocity[i] = s.output.new_velocity[k];
        s.acceleration[i] = s.output.new_acceleration[k];
    }
    return {};
}

Eigen::Index JointTrajectory::size() const noexcept { return impl_->reference.size(); }
bool JointTrajectory::active() const noexcept { return impl_->active; }
double JointTrajectory::tau() const noexcept { return impl_->tau; }
const Eigen::VectorXd& JointTrajectory::reference() const noexcept { return impl_->reference; }
const Eigen::VectorXd& JointTrajectory::velocity() const noexcept { return impl_->velocity; }
const Eigen::VectorXd& JointTrajectory::acceleration() const noexcept { return impl_->acceleration; }

}  // namespace kstone
