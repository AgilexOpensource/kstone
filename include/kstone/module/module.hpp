#pragma once

#include "kstone/collision/geometry.hpp"
#include "kstone/control/solver.hpp"
#include "kstone/core/context.hpp"
#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/core/mask.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"
#include "kstone/module/config.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace kstone {

/// Per-cycle view handed to module hooks.
struct Cycle {
    double now;
    double dt;
    std::uint64_t index;  ///< +1 per ModuleSet::step; skips one after resynchronize().
    Eigen::Ref<Eigen::VectorXd>
        q;  ///< Configuration solved from (the engine's ideal state); writable in prepare() only.
    const Eigen::VectorXd& v;    ///< Velocity at q (size nv; the engine passes the last command).
    const Workspace& workspace;  ///< Updated to q before reset() and contribute(); stale in prepare().
    const Eigen::VectorXd* measured{
        nullptr};  ///< Estimated robot configuration (size nq); nullptr if unknown.
    Mask* aligned{
        nullptr};        ///< prepare(): set the bits (size nv) of coordinates whose q was set to measured.
    bool rebase{false};  ///< reset() after resynchronize(true): drop held targets, hold where the robot is.
    /// Effective position limits (size nq): model limits narrowed by the solver options; +-inf where unbounded.
    const Eigen::VectorXd* lower_limit{nullptr};
    const Eigen::VectorXd* upper_limit{nullptr};
    std::uint64_t limits_revision{0};  ///< Changes whenever the solver options may have changed the limits.
};

/// What a running module does when it becomes held.
enum class OnHeld {
    wait,   ///< Pause; reset() and resume when free again.
    abort,  ///< Module::on_preempted() is called once, then it is held as usual.
};

KSTONE_EXPORT const char* to_string(OnHeld on_held) noexcept;

/// Exclusive request for velocity coordinates. joints == nullptr (or no bit set) = no claim.
struct Claim {
    const Mask* joints{nullptr};  ///< Size nv; must stay valid until the next prepare().
    int priority{0};
    OnHeld on_held{OnHeld::wait};  ///< Read even without joints (a module held by driven overlap).
    bool exclusive{false};  ///< While granted, other modules overlapping these joints reject new input.
};

/// Conventional claim priorities.
namespace priority {
inline constexpr int preemptive = 100;
inline constexpr int yielding = -100;
}  // namespace priority

class Module;
class ModuleSet;

enum class PortType { number, vector, pose, flag };

KSTONE_EXPORT const char* to_string(PortType type) noexcept;

/// A typed, externally writable input of a module (e.g. a target). Declared in configure().
struct PortInfo {
    std::string name;
    PortType type{PortType::number};
    std::vector<std::string> labels;  ///< vector: one per element.
    Eigen::VectorXd lower, upper;     ///< number/vector: bounds per element (may be infinite); empty = none.
    std::string frame;                ///< pose: the frame it drives.
    std::string reference;            ///< pose: expressed in this frame; empty = world.
    double timeout{0.0};              ///< > 0: a write lapses after this many seconds unless rewritten.
    /// Elements of a vector port; 1 for number/flag.
    Eigen::Index size() const noexcept {
        return type == PortType::vector ? static_cast<Eigen::Index>(labels.size()) : 1;
    }
};

/// Value of one port; only the member matching the port type is used.
struct PortValue {
    double number{0.0};
    Eigen::VectorXd vector;
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    bool flag{false};
};

/// A shape a viewer may draw, attached to a frame. Declared in configure().
struct VisualInfo {
    std::string name;
    std::string frame;                                           ///< Empty = world.
    Eigen::Isometry3d placement{Eigen::Isometry3d::Identity()};  ///< In `frame`.
    Shape shape;
};

enum class OutputType { number, flag, point, segment, polygon, visuals, text };

KSTONE_EXPORT const char* to_string(OutputType type) noexcept;

/// What an output means to a viewer, so it can be grouped and styled without knowing its name.
enum class OutputRole {
    value,     ///< Plain reading.
    margin,    ///< number: distance to a safety limit; <= 0 means the limit is reached.
    boundary,  ///< The limit itself (e.g. a support polygon).
    warning,   ///< Inside the warning band of a limit.
    alert,     ///< At or closest to a limit (e.g. the closest pair).
};

KSTONE_EXPORT const char* to_string(OutputRole role) noexcept;

/// A read-only display value recomputed by Module::update_outputs(). Declared in configure().
struct OutputInfo {
    std::string name;
    OutputType type{OutputType::number};
    Eigen::Index size{0};  ///< polygon: vertices; visuals: most indices; ignored otherwise.
    OutputRole role{OutputRole::value};
    std::string unit{};  ///< SI unit symbol of a number output; empty = none.
};

/// Value of one output; only the members matching the type are used. Points are in the world frame.
struct OutputValue {
    double number{0.0};
    bool flag{false};
    Eigen::Matrix3Xd points;   ///< point: 3 x 1; segment: 3 x 2; polygon: 3 x size.
    std::vector<int> visuals;  ///< Indices into Module::visuals(), at most size.
    std::string text;          ///< Short status word; 64 characters are reserved, longer allocates.
};

/// A task registered by one module; only that module can switch it.
class TaskHandle {
  public:
    TaskHandle() = default;
    bool valid() const noexcept { return owner_ != nullptr; }

  private:
    friend class ModuleSetup;
    friend class Module;
    TaskHandle(const Module* owner, std::size_t index) noexcept : owner_(owner), index_(index) {}
    const Module* owner_{nullptr};
    std::size_t index_{0};
};

/// Registration surface passed to Module::configure. Names are prefixed with "<module id>/".
class KSTONE_EXPORT ModuleSetup {
  public:
    const ModelPtr& model() const noexcept;
    const Context& context() const noexcept;
    /// Configured initial posture (size nq): ModuleSet::set_initial(), else model neutral.
    const Eigen::VectorXd& initial() const noexcept;
    /// Effective position limits as in Cycle::lower_limit / upper_limit, at configure time.
    const Eigen::VectorXd& lower_limit() const noexcept;
    const Eigen::VectorXd& upper_limit() const noexcept;
    /// Tasks are held (disabled in the solver) whenever the module is not running.
    Result<TaskHandle> add_task(const std::string& name, std::shared_ptr<Task> task,
                                TaskOptions options = {});
    /// Constraints stay active while the module is held; only set_enabled(false) removes them.
    Status add_constraint(const std::string& name, std::shared_ptr<Constraint> constraint);
    /// Velocity coordinates the module's tasks move (size nv). A grant overlapping them holds the module.
    Status set_driven(Mask joints);
    /// Velocity coordinates the solver must not move (size nv), e.g. a root driven outside the QP.
    Status lock(const Mask& joints);

  private:
    friend class ModuleSet;
    struct State;
    explicit ModuleSetup(State& state) noexcept : state_(state) {}
    State& state_;
};

/// One reusable unit of solver behaviour. Hooks run on the control thread; prepare/claim/reset/
/// contribute must not allocate in steady state.
class KSTONE_EXPORT Module {
  public:
    explicit Module(std::string id) : id_(std::move(id)) {}
    virtual ~Module() = default;
    Module(const Module&) = delete;
    Module& operator=(const Module&) = delete;

    const std::string& id() const noexcept { return id_; }
    virtual const char* type() const noexcept = 0;
    /// Module ids that must be configured (and stepped) before this one.
    virtual std::vector<std::string> dependencies() const { return {}; }

    virtual Status configure(ModuleSetup& setup) = 0;
    /// Every enabled module, before arbitration. May write q (e.g. the root pose).
    virtual Status prepare(Cycle&) { return {}; }
    /// Read after prepare().
    virtual Claim claim() const { return {}; }
    /// Running again (or for the first time) after being held or disabled.
    virtual Status reset(const Cycle&) { return {}; }
    /// Once on the running -> held transition when the last claim() had on_held == abort; after
    /// arbitration, before any contribute(). Not called on disable.
    virtual Status on_preempted(const Cycle&, const std::string& /*by*/) { return {}; }
    /// Once on the enabled -> disabled transition, at the start of that step; e.g. abort a running homing.
    virtual Status on_disabled(const Cycle&) { return {}; }
    /// A module driving some of the claimed joints received new input (request_control()); return true to
    /// drop the claim, which is read again. Called after prepare(), before arbitration.
    virtual bool yield_control() { return false; }
    /// Running modules only, after the workspace update; set task targets here.
    virtual Status contribute(const Cycle&) { return {}; }

    /// Takes effect on the next ModuleSet::step.
    void set_enabled(bool enabled) noexcept { enabled_ = enabled; }
    bool enabled() const noexcept { return enabled_; }
    /// Enabled and not held by a higher-priority claim, as of the last step.
    bool running() const noexcept { return running_; }
    /// Id of the module whose grant held this one, as of the last step; nullptr if none.
    const std::string* held_by() const noexcept { return held_by_; }
    /// Id of the module whose granted exclusive claim overlaps this one, as of the last step; nullptr if none.
    const std::string* locked_by() const noexcept { return locked_by_; }
    /// Running -> held transitions so far (either on_held policy).
    std::uint64_t preempted() const noexcept { return preempted_; }

  protected:
    /// New user input: on the next step, claimants on this module's driven joints may yield_control().
    void request_control() noexcept { control_requested_ = true; }
    /// Error naming the locking module, or ok if not locked_by(); call from input entry points.
    Status check_unlocked() const;

  public:
    /// Valid after configure().
    const std::vector<PortInfo>& ports() const noexcept { return ports_; }
    /// Index into ports(); -1 if absent.
    int port_index(const std::string& name) const noexcept;
    /// Control-thread only. Steady-state reads into a value of matching size do not allocate.
    virtual Status read_port(std::size_t index, PortValue& out) const;
    virtual Status write_port(std::size_t index, const PortValue& value);
    /// unsupported unless the module can home.
    virtual Status request_home();
    /// True if request_home() is supported.
    virtual bool homeable() const noexcept { return false; }
    /// Homing state name (idle, waiting, executing, succeeded, timed_out, aborted); nullptr if not homeable.
    virtual const char* home_state() const noexcept { return nullptr; }

    /// Valid after configure().
    const std::vector<VisualInfo>& visuals() const noexcept { return visuals_; }
    const std::vector<OutputInfo>& outputs() const noexcept { return outputs_; }
    /// Control-thread only, no allocation. Current placement of visuals()[i] in its frame (the declared
    /// one by default); false hides the visual.
    virtual bool visual_placement(std::size_t i, Eigen::Isometry3d& placement) const;
    /// Control-thread only, no steady-state allocation. Recomputes the outputs at `at` (updated; any
    /// configuration, e.g. the measured one), independently of the solve.
    virtual Status update_outputs(Workspace& at);
    /// After update_outputs(). Steady-state reads into a value of matching size and capacity do not allocate.
    virtual Status read_output(std::size_t index, OutputValue& out) const;

    /// Control-thread only. Changes the tunable parameters present in `params`; on error nothing changes.
    virtual Status tune(const ConfigNode& params);
    /// Current tunable parameters as a map; null if the module has none.
    virtual ConfigNode tunables() const { return {}; }
    /// JSON Schema of tune(); defaults are the construction values. Null if the module has none.
    virtual ConfigNode tune_schema() const { return {}; }

  protected:
    /// From configure() only.
    void declare_port(PortInfo port) { ports_.push_back(std::move(port)); }
    void declare_visual(VisualInfo visual) { visuals_.push_back(std::move(visual)); }
    void declare_output(OutputInfo output) { outputs_.push_back(std::move(output)); }
    /// Switch one of this module's tasks while running; all start active. Applied before the solve.
    /// Handles from other modules are ignored.
    void set_task_active(TaskHandle task, bool active) noexcept {
        const std::size_t k = task.index_;
        if (task.owner_ == this && k < task_active_.size() && static_cast<bool>(task_active_[k]) != active) {
            task_active_[k] = active;
            tasks_dirty_ = true;
        }
    }
    /// Lock one of this module's tasks while running (TaskOptions::locked); all start as configured.
    void set_task_locked(TaskHandle task, bool locked) noexcept {
        const std::size_t k = task.index_;
        if (task.owner_ == this && k < task_locked_.size() && static_cast<bool>(task_locked_[k]) != locked) {
            task_locked_[k] = locked;
            tasks_dirty_ = true;
        }
    }

  private:
    friend class ModuleSet;
    std::vector<char> task_active_, task_locked_;
    bool tasks_dirty_{false};
    std::vector<PortInfo> ports_;
    std::vector<VisualInfo> visuals_;
    std::vector<OutputInfo> outputs_;
    std::string id_;
    bool enabled_{true};
    bool running_{false};
    bool control_requested_{false};
    const std::string* held_by_{nullptr};
    const std::string* locked_by_{nullptr};
    std::uint64_t preempted_{0};
};

inline constexpr const char* kModuleIdRule = "letters, digits, '_' or '-', starting with a letter or '_'";

/// True if `id` follows kModuleIdRule; ids appear in task names, config paths and the debug protocol.
inline bool valid_module_id(const std::string& id) {
    auto alpha = [](char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_'; };
    if (id.empty() || !alpha(id[0])) return false;
    for (char c : id)
        if (!alpha(c) && !(c >= '0' && c <= '9') && c != '-') return false;
    return true;
}

/// Owns a solver and steps a fixed set of modules: prepare -> arbitrate -> reset -> contribute -> solve.
class KSTONE_EXPORT ModuleSet {
  public:
    static Result<ModuleSet> create(ModelPtr model, SolverOptions options = {},
                                    const Context& context = Context::defaults());

    ~ModuleSet();
    ModuleSet(ModuleSet&&) noexcept;
    ModuleSet& operator=(ModuleSet&&) noexcept;
    ModuleSet(const ModuleSet&) = delete;
    ModuleSet& operator=(const ModuleSet&) = delete;

    /// Before configure() only; ids are unique and non-empty.
    Status add(std::shared_ptr<Module> module);
    /// Before configure() only; finite, size nq. Reference for "initial" postures (default: model neutral).
    Status set_initial(const Eigen::VectorXd& q);
    /// Orders modules by dependency (ties keep add order) and configures them once.
    /// not_found on a missing dependency, invalid_argument on a cycle.
    Status configure();
    bool configured() const noexcept;

    /// One cycle from q (size nq) and v (size nv); out.q_next includes module writes. `measured` (size nq)
    /// feeds Cycle::measured and the solver's closed-loop constraints.
    Status step(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v,
                double now, double dt, SolveResult& out);
    Status step(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v,
                const Eigen::VectorXd* measured, double now, double dt, SolveResult& out);
    /// The next step() skips one Cycle::index (running homings abort) and calls reset() on every running
    /// module, as after being held. rebase: modules also drop their held targets (Cycle::rebase).
    void resynchronize(bool rebase = false) noexcept;
    /// Coordinates (size nv) set to measured by the last step()'s prepare().
    const Mask& aligned() const noexcept;

    /// Null if not found.
    std::shared_ptr<Module> module(const std::string& id) const;
    /// Step order after configure(), add order before.
    const std::vector<std::shared_ptr<Module>>& modules() const noexcept;
    /// Full solver task names ("<id>/<task>") added by module `id` in configure(); empty if unknown.
    const std::vector<std::string>& tasks(const std::string& id) const;
    /// Distinct configured TaskOptions::level of those tasks, ascending; empty if unknown or none.
    std::vector<int> task_levels(const std::string& id) const;
    /// Velocity coordinates module `id` set with ModuleSetup::set_driven(); empty if unknown or none.
    const Mask& driven(const std::string& id) const;
    const ModelPtr& model() const noexcept;
    /// set_initial() posture, else model neutral.
    const Eigen::VectorXd& initial() const noexcept;
    Solver& solver() noexcept;

    /// Changes only the given solver options (keys of solver_tune_schema()); on error nothing changes.
    /// Rebuilds the QP backend (allocates, drops the warm start); not during step().
    Status tune_solver(const ConfigNode& params);
    ConfigNode solver_params() const;
    /// JSON Schema of tune_solver(); defaults are the options given to create().
    ConfigNode solver_tune_schema() const;
    /// Run-time joint limits on top of the model's, tighten only, all-or-nothing:
    /// {reset: bool, velocity: {<joint>: v}, position: {lower: {<joint>: q}, upper: {<joint>: q}}}.
    /// reset drops every override first; velocity 0 or a position equal to the model bound drops that one.
    /// Writes SolverOptions velocity/position limits, so like tune_solver(); not during step().
    Status tune_joint_limits(const ConfigNode& params);
    /// Overrides in force, shaped like tune_joint_limits() without reset.
    ConfigNode joint_limits() const;

  private:
    struct Impl;
    explicit ModuleSet(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

}  // namespace kstone
