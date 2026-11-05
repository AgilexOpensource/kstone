#include "kstone/runtime/snapshot.hpp"

#include <limits>
#include <utility>

namespace kstone {

TelemetrySink::~TelemetrySink() = default;

void take_snapshot(const Engine& engine, EngineSnapshot& out) {
    out.schema_version = snapshot_schema_version;
    out.stats = engine.stats();
    out.cycle_p50_s = out.stats.cycle_hist.quantile(0.5);
    out.cycle_p99_s = out.stats.cycle_hist.quantile(0.99);
    out.cycle_p999_s = out.stats.cycle_hist.quantile(0.999);
    out.paused = engine.paused();
    out.fresh = engine.fresh();
    out.written = engine.written();
    out.state.q = engine.state().q;
    out.state.v = engine.state().v;
    out.state.stamp_ns = engine.state().stamp_ns;
    out.command.q = engine.command().q;
    out.command.v = engine.command().v;
    out.command.fresh = engine.command().fresh;
    out.ideal = engine.ideal();
    const Eigen::VectorXd* measured = engine.measured();
    out.has_measured = measured != nullptr;
    if (measured) out.measured = *measured;

    const SolveResult& result = engine.result();
    out.quality = result.quality;
    out.held = result.held;
    out.step_scale = result.step_scale;
    if (out.residuals.size() != result.residuals.size()) out.residuals.resize(result.residuals.size());
    for (std::size_t i = 0; i < result.residuals.size(); ++i) {
        out.residuals[i].name = result.residuals[i].name;
        out.residuals[i].level = result.residuals[i].level;
        out.residuals[i].residual = result.residuals[i].residual;
    }

    const auto& mods = engine.modules().modules();
    if (out.modules.size() != mods.size()) out.modules.resize(mods.size());
    for (std::size_t i = 0; i < mods.size(); ++i) {
        const Module& m = *mods[i];
        ModuleSnapshot& ms = out.modules[i];
        ms.id = m.id();
        ms.type = m.type();
        ms.enabled = m.enabled();
        ms.running = m.running();
        ms.preempted = m.preempted();
        ms.home_state = m.home_state();
        ms.held_by = -1;
        if (const std::string* h = m.held_by())
            for (std::size_t k = 0; k < mods.size(); ++k)
                if (mods[k]->id() == *h) ms.held_by = static_cast<int>(k);
        const auto& infos = m.ports();
        if (ms.ports.size() != infos.size()) {
            ms.ports.resize(infos.size());
            for (std::size_t p = 0; p < infos.size(); ++p)
                if (infos[p].type == PortType::vector) ms.ports[p].vector.setZero(infos[p].size());
        }
        for (std::size_t p = 0; p < infos.size(); ++p) (void)m.read_port(p, ms.ports[p]);
    }
}

namespace {

class TelemetryObserver final : public EngineObserver {
  public:
    TelemetryObserver(std::shared_ptr<TelemetrySink> sink, double publish_hz)
        : sink_(std::move(sink)), period_(publish_hz > 0.0 ? 1.0 / publish_hz : 0.0) {}

    void after_step(const Engine& engine) override {
        const double t = engine.stats().time;
        if (t < next_ - 1e-9 * period_) return;
        next_ = next_ + period_ > t ? next_ + period_ : t + period_;
        take_snapshot(engine, snapshot_);
        sink_->publish(snapshot_);
    }

  private:
    std::shared_ptr<TelemetrySink> sink_;
    double period_;
    double next_{-std::numeric_limits<double>::infinity()};
    EngineSnapshot snapshot_;
};

}  // namespace

std::shared_ptr<EngineObserver> make_telemetry_observer(std::shared_ptr<TelemetrySink> sink,
                                                        double publish_hz) {
    if (!sink) return nullptr;
    return std::make_shared<TelemetryObserver>(std::move(sink), publish_hz);
}

}  // namespace kstone
