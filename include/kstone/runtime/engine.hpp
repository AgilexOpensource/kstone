#pragma once

#include "kstone/control/solver.hpp"
#include "kstone/core/context.hpp"
#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/core/histogram.hpp"
#include "kstone/module/module.hpp"
#include "kstone/runtime/robot.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace kstone {

enum class GuardMode {
    off,     ///< No checks; the solver output is written as is.
    reject,  ///< Any violation fails the cycle and nothing is written.
    clamp,   ///< Limit violations are clamped; wrong sizes and non-finite values still fail the cycle.
};

/// Last check of every command before RobotInterface::write, against the Model limits (not solver overrides).
/// Velocity: every dof whose model limit is finite and > 0 (URDFs without one load as 0 = no limit).
/// Position: scalar joints (nq == nv == 1) with Model::has_position_limit; a joint whose commanded (or measured)
/// start is outside its range may hold or move back but not further out. Quaternion and (cos, sin) coordinates are only velocity-checked.
struct CommandGuardOptions {
    GuardMode mode{GuardMode::reject};
    double velocity_scale{1.0};  ///< > 0; multiplies every model velocity limit.
    double tolerance{1e-6};      ///< Slack on position and velocity comparisons.
};

/// Real-time setup applied when Engine::run starts (Linux only). A setting the OS refuses is logged as a warning
/// and run() continues without it; priority and cpus are restored when run() returns.
struct ThreadOptions {
    /// 0 = keep the current policy; otherwise SCHED_FIFO at this priority (1..99, higher preempts lower), which
    /// needs CAP_SYS_NICE or an RLIMIT_RTPRIO (`ulimit -r`) at least this high.
    int priority{0};
    /// CPU indices to pin the calling thread to; empty = keep the current affinity.
    std::vector<int> cpus;
    /// mlockall(MCL_CURRENT | MCL_FUTURE) for the whole process, never undone. Needs CAP_IPC_LOCK or a large
    /// enough `ulimit -l`; with a small limit, later allocations beyond it fail.
    bool lock_memory{false};
};

struct EngineOptions {
    double rate_hz{100.0};
    /// Sleep to each deadline of Context::clock (which must follow wall time); otherwise run back to back.
    /// Either way `now` advances by exactly 1 / rate_hz per cycle (Engine::set_rate_hz changes it at runtime).
    bool realtime{true};
    /// > 0: feedback with an unknown RobotState::stamp_ns or older than this (s) is stale; 0 = always fresh.
    /// Default 0.25 s, so drivers must stamp their reads or set 0.
    /// Stale feedback only drops the measured estimate (closed-loop terms use the commanded state); the command
    /// is still written with RobotCommand::fresh = false, and the driver or caller decides whether to send it.
    double feedback_timeout_s{0.25};
    /// Scalar joints whose tracking error (measured - commanded) is held between samples to form the measured
    /// estimate used by closed-loop constraints and homing alignment. Names or fnmatch patterns ('*', '?',
    /// '[...]'), each must select a joint; empty = every scalar joint. Mimic followers always follow their leader.
    std::vector<std::string> feedback_joints;
    /// >= 1; run() returns the error once this many consecutive cycles failed, and keeps cycling before that.
    int max_failed_cycles{5};
    CommandGuardOptions safety;
    ThreadOptions thread;
};

struct EngineStats {
    std::uint64_t cycles{0};
    std::uint64_t overruns{0};        ///< Realtime cycles that finished after the next deadline.
    std::uint64_t repeated_reads{0};  ///< Reads whose known stamp_ns equals the previous read's.
    std::uint64_t guard_rejects{0};   ///< Cycles failed by the command guard.
    std::uint64_t guard_clamps{0};    ///< Cycles whose command the guard clamped.
    std::uint64_t held_cycles{0};     ///< Paused cycles (no solve, no write).
    std::uint64_t stale_cycles{0};    ///< Written cycles with stale feedback (RobotCommand::fresh = false).
    std::uint64_t resyncs{0};         ///< Resynchronizations to the measured state (resume).
    std::uint64_t failed_cycles{0};   ///< Failed step() calls (nothing written).
    double time{0.0};                 ///< `now` of the last cycle (s).
    double last_cycle_s{0.0};         ///< Wall time of the last read-solve-write.
    double max_cycle_s{0.0};
    double max_lateness_s{0.0};   ///< Largest wall-clock lateness past a realtime deadline (s).
    LatencyHistogram cycle_hist;  ///< Every successful cycle's last_cycle_s.
};

class Engine;

/// Per-cycle hooks on the control thread; must not block or allocate in steady state.
class KSTONE_EXPORT EngineObserver {
  public:
    virtual ~EngineObserver();
    /// Before the robot read; may change module inputs (ports, enable flags).
    virtual void before_step(Engine&) {}
    /// After every successful cycle, including paused ones that write nothing (see Engine::written).
    virtual void after_step(const Engine&) {}
};

/// Fixed-rate loop: robot read -> ModuleSet::step -> command guard -> robot write.
/// Open loop: the modules solve from the commanded (ideal) state, not from the measured one. The measured
/// state only enters on create and resume (resynchronization), through closed-loop
/// constraints and through homing alignment.
class KSTONE_EXPORT Engine {
  public:
    /// Configures `modules` if needed and checks the robot's state sizes once.
    static Result<Engine> create(ModuleSet modules, std::shared_ptr<RobotInterface> robot,
                                 EngineOptions options = {}, const Context& context = Context::defaults());

    ~Engine();
    Engine(Engine&&) noexcept;
    Engine& operator=(Engine&&) noexcept;
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    /// One cycle at time `now` (s, non-decreasing; a non-finite or backwards `now` fails). Nothing is written
    /// when it fails; no stop command is sent, so the driver must enforce its own command timeout and safe
    /// state. Failures are also logged (error).
    /// A failure after the modules ran resynchronizes them (targets kept, homing aborted), so the next cycle
    /// continues from the last written command instead of jumping over the missed cycles.
    Status step(double now);
    /// Cycles for `duration_s` (<= 0: until *stop) or until *stop becomes true; returns the error of the
    /// EngineOptions::max_failed_cycles-th consecutive failed cycle. A failed cycle writes nothing.
    /// Time continues from the previous run. `now` is logical time t0 + k * dt (per rate segment, see
    /// set_rate_hz) and lags wall time after
    /// overruns (the deadline re-anchors instead of catching up).
    Status run(double duration_s, const std::atomic<bool>* stop = nullptr);

    /// Paused cycles only read: no solve and no write, so the driver holds its last command (or applies its own
    /// command timeout). Module time stops while paused. Resuming resynchronizes to the measured state, keeping
    /// module targets. Thread-safe.
    void set_paused(bool paused) noexcept;
    bool paused() const noexcept;
    /// While paused, the next `cycles` cycles solve and write from the commanded state without resynchronizing
    /// (requests add up; set_paused clears them). Thread-safe.
    void step_paused(std::uint64_t cycles) noexcept;
    /// Switches EngineOptions::rate_hz (and dt) at the start of the next cycle; run() keeps `now` continuous.
    /// Robot plugins keep the rate they were created with. Thread-safe.
    Status set_rate_hz(double rate_hz);

    /// Called in add order by every step(); not while running.
    void add_observer(std::shared_ptr<EngineObserver> observer);

    double dt() const noexcept;
    const EngineOptions& options() const noexcept;
    const Context& context() const noexcept;
    const EngineStats& stats() const noexcept;
    /// Zeroes counters, maxima and histograms; keeps `time` so run() continues. Not while running.
    void reset_stats() noexcept;
    const RobotState& state() const noexcept;    ///< Last read.
    const SolveResult& result() const noexcept;  ///< Last successful solve, before the guard.
    /// Last guarded command; unspecified after a failed step().
    const RobotCommand& command() const noexcept;
    bool written() const noexcept;  ///< Whether the last successful cycle wrote (false when paused).
    bool fresh() const noexcept;    ///< Whether the last read was fresh (see feedback_timeout_s).
    const Eigen::VectorXd& ideal() const noexcept;  ///< Commanded state the next solve starts from.
    /// Last measured estimate (ideal + held tracking error); nullptr while stale.
    const Eigen::VectorXd* measured() const noexcept;
    ModuleSet& modules() noexcept;
    const ModuleSet& modules() const noexcept;
    RobotInterface& robot() noexcept;

  private:
    struct Impl;
    explicit Engine(std::unique_ptr<Impl> impl) noexcept;
    Status step_impl(double now);
    std::unique_ptr<Impl> impl_;
};

}  // namespace kstone
