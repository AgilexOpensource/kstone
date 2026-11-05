#pragma once

#include "kstone/control/solver.hpp"
#include "kstone/core/export.hpp"
#include "kstone/module/module.hpp"
#include "kstone/runtime/engine.hpp"
#include "kstone/runtime/robot.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace kstone {

/// Bumped only when an EngineSnapshot field is removed or changes meaning; appended fields keep the version.
inline constexpr std::uint32_t snapshot_schema_version = 1;

struct ModuleSnapshot {
    std::string id;
    std::string type;
    bool enabled{false};
    bool running{false};
    int held_by{-1};  ///< Index in EngineSnapshot::modules of the preempting module; -1 = not held.
    std::uint64_t preempted{0};       ///< Times this module was preempted.
    const char* home_state{nullptr};  ///< Module::home_state(); nullptr if not homeable.
    std::vector<PortValue> ports;     ///< Same order as Module::ports().
};

/// Read-only view of the last engine cycle, shared by every telemetry consumer.
struct EngineSnapshot {
    std::uint32_t schema_version{snapshot_schema_version};
    EngineStats stats;
    double cycle_p50_s{0.0};
    double cycle_p99_s{0.0};
    double cycle_p999_s{0.0};
    bool paused{false};
    bool fresh{false};
    bool written{false};
    RobotState state;       ///< Last read.
    RobotCommand command;   ///< Last guarded command.
    Eigen::VectorXd ideal;  ///< Commanded state the next solve starts from.
    bool has_measured{false};
    Eigen::VectorXd measured;  ///< Measured estimate; meaningful only when has_measured.
    SolveQuality quality{SolveQuality::nominal};
    bool held{false};
    double step_scale{1.0};
    std::vector<TaskResidual> residuals;
    std::vector<ModuleSnapshot> modules;
};

/// Fills `out` from the engine's last cycle, reusing its buffers: no allocation once sizes and names settle.
/// Call from an EngineObserver or while the engine is not running.
KSTONE_EXPORT void take_snapshot(const Engine& engine, EngineSnapshot& out);

/// Telemetry consumer (recorder, bus publisher, plotter).
class KSTONE_EXPORT TelemetrySink {
  public:
    virtual ~TelemetrySink();
    /// Called on the control thread; must not block or allocate. Copy what is needed into the sink's own queue.
    virtual void publish(const EngineSnapshot& snapshot) = 0;
};

/// Observer that snapshots the engine after a cycle and passes it to `sink`, at most `publish_hz` times per
/// second of engine time (<= 0: every cycle, including paused ones). Register with Engine::add_observer.
KSTONE_EXPORT std::shared_ptr<EngineObserver> make_telemetry_observer(std::shared_ptr<TelemetrySink> sink,
                                                                      double publish_hz = 0.0);

}  // namespace kstone
