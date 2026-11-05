#include "sim_robot.hpp"

#include "kstone/model/model.hpp"

#include <cmath>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace kstone::test {

struct SimRobot::Impl {
    std::shared_ptr<const Model> model;
    SimRobotOptions options;
    std::shared_ptr<const Clock> clock;
    double dt{0.01};
    double alpha{1.0};
    Eigen::VectorXd q, v, target, step, next, noise;
    std::vector<Eigen::VectorXd> pending;  ///< Ring of delayed command q.
    std::size_t head{0};
    std::mt19937_64 rng;
    std::normal_distribution<double> normal{0.0, 1.0};
};

SimRobot::SimRobot(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
SimRobot::~SimRobot() = default;

Result<std::shared_ptr<SimRobot>> SimRobot::create(std::shared_ptr<const Model> model, Eigen::VectorXd q,
                                                   const SimRobotOptions& options,
                                                   std::shared_ptr<const Clock> clock) {
    const auto bad = [](const std::string& what) {
        return make_error(ErrorCode::invalid_argument, "sim robot: " + what);
    };
    if (!model) return bad("model is null");
    if (!(options.rate_hz > 0.0) || !std::isfinite(options.rate_hz)) return bad("rate_hz must be > 0");
    for (const auto& [name, value] :
         {std::pair<const char*, double>{"time_constant_s", options.time_constant_s},
          {"latency_s", options.latency_s},
          {"position_noise", options.position_noise},
          {"velocity_noise", options.velocity_noise}})
        if (!(value >= 0.0) || !std::isfinite(value))
            return bad(std::string(name) + " must be finite and >= 0");
    if (auto s = model->validate(q); !s) return bad(s.error().message);
    const double delay = std::round(options.latency_s * options.rate_hz);
    if (delay > 100000.0) return bad("latency_s * rate_hz must be <= 100000 cycles");

    auto d = std::make_unique<Impl>();
    d->options = options;
    d->clock = clock ? std::move(clock) : make_steady_clock();
    d->dt = 1.0 / options.rate_hz;
    d->alpha = options.time_constant_s > 0.0 ? 1.0 - std::exp(-d->dt / options.time_constant_s) : 1.0;
    d->q = q;
    d->target = q;
    d->next = q;
    d->v = Eigen::VectorXd::Zero(model->nv());
    d->step = d->v;
    d->noise = d->v;
    d->pending.assign(static_cast<std::size_t>(delay), q);
    d->rng.seed(static_cast<std::uint64_t>(static_cast<std::int64_t>(options.seed)));
    d->model = std::move(model);
    return std::shared_ptr<SimRobot>(new SimRobot(std::move(d)));
}

Status SimRobot::read(RobotState& state) {
    Impl& d = *impl_;
    state.q.resize(d.q.size());
    state.v.resize(d.v.size());
    if (d.options.position_noise > 0.0) {
        for (Eigen::Index i = 0; i < d.noise.size(); ++i)
            d.noise[i] = d.options.position_noise * d.normal(d.rng);
        if (auto s = d.model->integrate(d.q, d.noise, state.q); !s) return s;
    } else {
        state.q = d.q;
    }
    state.v = d.v;
    if (d.options.velocity_noise > 0.0)
        for (Eigen::Index i = 0; i < state.v.size(); ++i)
            state.v[i] += d.options.velocity_noise * d.normal(d.rng);
    state.stamp_ns = d.clock->now_ns();
    return {};
}

Status SimRobot::write(const RobotCommand& command) {
    Impl& d = *impl_;
    if (command.q.size() != d.q.size() || command.v.size() != d.v.size())
        return make_error(ErrorCode::invalid_argument, "sim robot: command size mismatch");
    if (!command.q.allFinite())
        return make_error(ErrorCode::invalid_argument, "sim robot: command is not finite");
    if (d.pending.empty()) {
        d.target = command.q;
    } else {
        d.target = d.pending[d.head];
        d.pending[d.head] = command.q;
        d.head = (d.head + 1) % d.pending.size();
    }
    if (auto s = d.model->difference(d.q, d.target, d.step); !s) return s;
    d.step *= d.alpha;
    if (auto s = d.model->integrate(d.q, d.step, d.next); !s) return s;
    d.q.swap(d.next);
    d.v = d.step / d.dt;
    return {};
}

}  // namespace kstone::test
