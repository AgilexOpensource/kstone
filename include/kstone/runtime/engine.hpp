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

/// Real-time setup applied when Engine::run starts with realtime on (Linux only); ignored otherwise. A setting the OS refuses is logged as a warning
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

/// Privilege helper (kstone run calls it when realtime) for a process that will run with `thread`: if RLIMIT_RTPRIO (or, with lock_memory,
/// RLIMIT_MEMLOCK) is too low and the matching capability is missing, runs `sudo prlimit --pid <self> ...`, which
/// may prompt for a password on the controlling terminal. The process itself never gains root. Returns ok when
/// nothing was needed or the limits were raised; otherwise an error the caller should log and continue past.
/// Call before Engine::run, from a process with a terminal. Linux only; elsewhere returns unsupported.
KSTONE_EXPORT Status elevate_thread_limits(const ThreadOptions& thread, const Logger& log);

/// Joints whose feedback is stamped, checked for staleness and attached together (e.g. arms vs. an odometry base).
struct FeedbackGroup {
    std::string name;  ///< Unique, non-empty, not "default".
    /// Names or fnmatch patterns ('*', '?', '[...]'), each must select a joint; a joint belongs to at most one
    /// group. Mimic followers always join their leader's group (an exact follower name is an error).
    std::vector<std::string> joints;
};

/// Resolved feedback groups: EngineOptions::feedback_groups in order, then "default" with every joint in no
/// group (only when there is one; with no groups configured it holds every joint).
struct FeedbackLayout {
    std::vector<std::string> names;
    /// Per JointId: index into names; names.size() for the universe joint.
    std::vector<std::size_t> joint_group;
};

KSTONE_EXPORT Result<FeedbackLayout> resolve_feedback_groups(const Model& model,
                                                             const std::vector<FeedbackGroup>& groups);

struct EngineOptions {
    double rate_hz{100.0};
    /// Apply `thread` in run() (callers such as kstone run also call elevate_thread_limits first). run() always
    /// sleeps to each deadline of Context::clock; `now` advances by exactly 1 / rate_hz per cycle.
    bool realtime{false};
    /// > 0: a feedback group whose stamp is unknown or older than this (s) is stale; 0 = always fresh.
    /// Default 0.25 s, so drivers must stamp their reads or set 0.
    /// A stale group detaches; it reattaches once its feedback has stayed fresh for this long (see
    /// Engine::group_attached()). While detached its tracking error leaves the measured estimate (closed-loop
    /// terms use the commanded state there) and commands are still written with RobotCommand::fresh = false;
    /// the driver decides whether to send them.
    double feedback_timeout_s{0.25};
    /// Empty = one "default" group with every joint. Otherwise each group has its own stamp
    /// (RobotState::group_stamps_ns), staleness, settle window and attached state; a stale group only detaches
    /// and later resynchronizes its own joints and the modules driving them (see Engine::attached()).
    std::vector<FeedbackGroup> feedback_groups;
    /// Scalar joints whose tracking error (measured - commanded) is held between samples to form the measured
    /// estimate used by closed-loop constraints and homing alignment. Names or fnmatch patterns ('*', '?',
    /// '[...]'), each must select a joint; empty = every scalar joint. Mimic followers always follow their leader.
    std::vector<std::string> feedback_joints;
    /// >= 1; run() returns the error once this many consecutive cycles failed, and keeps cycling before that.
    int max_failed_cycles{5};
    /// Out-of-limit scalar synchronization starts at raw feedback and uses JointTrajectory's continuous
    /// Ruckig profile (acceleration = 2 * guarded model velocity, jerk = 10 * acceleration).
    /// Relevant modules are held/rebased until arrival; other feedback groups remain independent.
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
    std::uint64_t stale_cycles{0};    ///< Written cycles with any feedback group stale.
    std::uint64_t resyncs{
        0};  ///< Group resynchronizations to the measured state (resume, fresh after stale).
    std::uint64_t attaches{0};  ///< Group attachments (Engine::group_attached()), including the first ones.
    std::uint64_t failed_cycles{0};  ///< Failed step() calls (nothing written).
    double time{0.0};                ///< `now` of the last cycle (s).
    double last_cycle_s{0.0};        ///< Wall time of the last read-solve-write.
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
    /// Syncs attached group `group` (all_groups: every group) to its latest measurement on the next unpaused
    /// cycle, as resuming does, e.g. after a driver ignored commands behind a closed gate. `rebase` drops the
    /// held targets of the modules driving it, so the command holds the measurement; otherwise they are kept
    /// and reached from it. Groups that are stale or still attaching already follow feedback and skip it.
    /// Requests add up until consumed. Fails for an unknown group or more than 32 groups. Thread-safe.
    static constexpr std::size_t all_groups = static_cast<std::size_t>(-1);
    Status request_resync(std::size_t group = all_groups, bool rebase = true);
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
    bool fresh() const
        noexcept;  ///< Whether every feedback group was fresh in the last read (see feedback_timeout_s).
    /// Whether every feedback group is attached (see group_attached()).
    bool attached() const noexcept;
    std::size_t group_count() const noexcept;
    /// Empty when i >= group_count().
    const std::string& group_name(std::size_t i) const noexcept;
    bool group_fresh(std::size_t i) const noexcept;  ///< Whether group i was fresh in the last read.
    /// Whether group i's joints have been synced to fresh feedback and have stayed fresh since; always true
    /// when feedback_timeout_s = 0. Starts false and turns false on a stale read of the group. The first fresh,
    /// unpaused cycle after that rebases the modules driving the group's joints (held targets dropped); from
    /// then on every fresh, unpaused cycle syncs those joints to the latest measurement (targets set meanwhile
    /// kept) until the group has stayed fresh for feedback_timeout_s, when that cycle's sync attaches.
    /// RobotCommand::group_attached and RobotCommand::fresh (all groups) follow it for every written command,
    /// so a command built on a state measured before the window is never flagged attached.
    bool group_attached(std::size_t i) const noexcept;
    /// Largest |measured - commanded| over group i's position coordinates in the last cycle; 0 when i >= group_count().
    double group_deviation(std::size_t i) const noexcept;
    const Eigen::VectorXd& ideal() const noexcept;  ///< Commanded state the next solve starts from.
    /// Last measured estimate (ideal + held tracking error of the attached groups); nullptr while no group is attached.
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
