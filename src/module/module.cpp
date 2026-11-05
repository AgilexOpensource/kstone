#include "kstone/module/module.hpp"

#include "builtin_schemas.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace kstone {

namespace {

Error with_context(const Error& e, const std::string& what) {
    return make_error(e.code, what + ": " + e.message);
}

bool intersects(const Mask& a, const Mask& b) noexcept {
    const std::size_t n = std::min(a.size(), b.size());
    for (std::size_t i = 0; i < n; ++i)
        if (a.test(i) && b.test(i)) return true;
    return false;
}

void merge(Mask& into, const Mask& from) noexcept {
    const std::size_t n = std::min(into.size(), from.size());
    for (std::size_t i = 0; i < n; ++i)
        if (from.test(i)) into.set(i);
}

}  // namespace

const char* to_string(PortType type) noexcept {
    switch (type) {
        case PortType::number:
            return "number";
        case PortType::vector:
            return "vector";
        case PortType::pose:
            return "pose";
        case PortType::flag:
            return "flag";
    }
    return "unknown";
}

const char* to_string(OutputType type) noexcept {
    switch (type) {
        case OutputType::number:
            return "number";
        case OutputType::flag:
            return "flag";
        case OutputType::point:
            return "point";
        case OutputType::segment:
            return "segment";
        case OutputType::polygon:
            return "polygon";
        case OutputType::visuals:
            return "visuals";
        case OutputType::text:
            return "text";
    }
    return "unknown";
}

const char* to_string(OutputRole role) noexcept {
    switch (role) {
        case OutputRole::value:
            return "value";
        case OutputRole::margin:
            return "margin";
        case OutputRole::boundary:
            return "boundary";
        case OutputRole::warning:
            return "warning";
        case OutputRole::alert:
            return "alert";
    }
    return "unknown";
}

const char* to_string(OnHeld on_held) noexcept {
    switch (on_held) {
        case OnHeld::wait:
            return "wait";
        case OnHeld::abort:
            return "abort";
    }
    return "unknown";
}

int Module::port_index(const std::string& name) const noexcept {
    for (std::size_t i = 0; i < ports_.size(); ++i)
        if (ports_[i].name == name) return static_cast<int>(i);
    return -1;
}

Status Module::read_port(std::size_t, PortValue&) const {
    return make_error(ErrorCode::out_of_range, "module '" + id_ + "': no such port");
}

Status Module::write_port(std::size_t, const PortValue&) {
    return make_error(ErrorCode::out_of_range, "module '" + id_ + "': no such port");
}

bool Module::visual_placement(std::size_t i, Eigen::Isometry3d& placement) const {
    if (i >= visuals_.size()) return false;
    placement = visuals_[i].placement;
    return true;
}

Status Module::update_outputs(Workspace&) { return {}; }

Status Module::read_output(std::size_t, OutputValue&) const {
    return make_error(ErrorCode::out_of_range, "module '" + id_ + "': no such output");
}

Status Module::check_unlocked() const {
    if (!locked_by_) return {};
    return make_error(ErrorCode::invalid_argument, "module '" + id_ + "': locked by '" + *locked_by_ + "'");
}

Status Module::request_home() {
    return make_error(ErrorCode::unsupported, "module '" + id_ + "': cannot home");
}

Status Module::tune(const ConfigNode&) {
    return make_error(ErrorCode::unsupported, "module '" + id_ + "': nothing to tune");
}

struct ModuleSetup::State {
    const ModelPtr& model;
    const Context& context;
    const Eigen::VectorXd& initial;
    const Eigen::VectorXd& lower_limit;
    const Eigen::VectorXd& upper_limit;
    Solver& solver;
    const Module& module;
    const std::string& id;
    std::vector<std::string>& tasks;
    std::vector<TaskOptions>& task_options;
    std::vector<std::string>& constraints;
    Mask& driven;
    Mask& locked;
};

const ModelPtr& ModuleSetup::model() const noexcept { return state_.model; }
const Context& ModuleSetup::context() const noexcept { return state_.context; }
const Eigen::VectorXd& ModuleSetup::initial() const noexcept { return state_.initial; }
const Eigen::VectorXd& ModuleSetup::lower_limit() const noexcept { return state_.lower_limit; }
const Eigen::VectorXd& ModuleSetup::upper_limit() const noexcept { return state_.upper_limit; }

Result<TaskHandle> ModuleSetup::add_task(const std::string& name, std::shared_ptr<Task> task,
                                         TaskOptions options) {
    std::string full = state_.id + "/" + name;
    TaskOptions held = options;
    held.enabled = false;
    if (Status s = state_.solver.add_task(full, std::move(task), held); !s) return std::move(s).error();
    state_.tasks.push_back(std::move(full));
    state_.task_options.push_back(options);
    return TaskHandle(&state_.module, state_.tasks.size() - 1);
}

Status ModuleSetup::add_constraint(const std::string& name, std::shared_ptr<Constraint> constraint) {
    std::string full = state_.id + "/" + name;
    if (Status s = state_.solver.add_constraint(full, std::move(constraint)); !s) return s;
    state_.constraints.push_back(std::move(full));
    return {};
}

Status ModuleSetup::set_driven(Mask joints) {
    if (joints.size() != static_cast<std::size_t>(state_.model->nv()))
        return make_error(ErrorCode::invalid_argument, "module '" + state_.id + "': driven mask size != nv");
    state_.driven = std::move(joints);
    return {};
}

Status ModuleSetup::lock(const Mask& joints) {
    if (joints.size() != static_cast<std::size_t>(state_.model->nv()))
        return make_error(ErrorCode::invalid_argument, "module '" + state_.id + "': lock mask size != nv");
    merge(state_.locked, joints);
    return {};
}

namespace {

struct Entry {
    std::shared_ptr<Module> module;
    std::vector<std::string> tasks;
    std::vector<TaskOptions> task_options;
    std::vector<std::string> constraints;
    Mask driven;
    Claim claim;
    std::uint64_t claim_since{0};
    std::size_t order{0};
    bool claimed{false};
    bool tasks_on{false};
    bool constraints_on{true};
};

}  // namespace

struct ModuleSet::Impl {
    Impl(ModelPtr m, Solver s, const Context& c)
        : model(std::move(m)), solver(std::move(s)), context(c), workspace(model) {}

    ModelPtr model;
    Solver solver;
    SolverOptions solver_defaults;
    Context context;
    Workspace workspace;
    std::vector<std::shared_ptr<Module>> modules;
    std::vector<Entry> entries;
    std::vector<std::size_t> claimants;
    Mask granted, aligned;
    Eigen::VectorXd q, v, initial;
    Eigen::VectorXd lower_limit, upper_limit;  // effective, see Cycle
    std::uint64_t limits_revision{~std::uint64_t{0}};
    std::uint64_t cycle{0};
    bool resync{false};
    bool rebase{false};
    bool configured{false};
    bool attempted{false};

    Status set_tasks(Entry& e, bool on) {
        Module& m = *e.module;
        m.tasks_dirty_ = false;
        for (std::size_t k = 0; k < e.tasks.size(); ++k) {
            TaskOptions o = e.task_options[k];
            o.enabled = o.enabled && on && m.task_active_[k];
            o.locked = o.locked || m.task_locked_[k];
            if (Status s = solver.set_task_options(e.tasks[k], o); !s) return s;
        }
        e.tasks_on = on;
        return {};
    }

    Status set_constraints(Entry& e, bool on) {
        for (const std::string& name : e.constraints)
            if (Status s = solver.set_constraint_enabled(name, on); !s) return s;
        e.constraints_on = on;
        return {};
    }

    Status arbitrate();

    // Allocation-free once sized; recomputed only after the solver options changed.
    void sync_limits() {
        if (limits_revision == solver.options_revision()) return;
        limits_revision = solver.options_revision();
        constexpr double kInf = std::numeric_limits<double>::infinity();
        const Model& m = *model;
        const SolverOptions& o = solver.options();
        const bool has_lo = o.lower_position_limit.size() == m.nq();
        const bool has_hi = o.upper_position_limit.size() == m.nq();
        lower_limit.setConstant(m.nq(), -kInf);
        upper_limit.setConstant(m.nq(), kInf);
        for (const JointInfo& j : m.joints()) {
            if (j.id == 0 || j.nq != 1 || j.nv != 1) continue;
            const bool limited = m.has_position_limit(j.id);
            double lo = limited ? m.lower_position_limit()[j.idx_q] : -kInf;
            double hi = limited ? m.upper_position_limit()[j.idx_q] : kInf;
            if (has_lo && std::isfinite(o.lower_position_limit[j.idx_q]))
                lo = std::max(lo, o.lower_position_limit[j.idx_q]);
            if (has_hi && std::isfinite(o.upper_position_limit[j.idx_q]))
                hi = std::min(hi, o.upper_position_limit[j.idx_q]);
            lower_limit[j.idx_q] = lo;
            upper_limit[j.idx_q] = hi;
        }
    }
};

ModuleSet::ModuleSet(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
ModuleSet::~ModuleSet() = default;
ModuleSet::ModuleSet(ModuleSet&&) noexcept = default;
ModuleSet& ModuleSet::operator=(ModuleSet&&) noexcept = default;

Result<ModuleSet> ModuleSet::create(ModelPtr model, SolverOptions options, const Context& context) {
    if (!model) return make_error(ErrorCode::invalid_argument, "module set: model is null");
    Result<Solver> solver = Solver::create(model, std::move(options));
    if (!solver) return std::move(solver).error();
    auto impl = std::make_unique<Impl>(model, std::move(*solver), context);
    impl->solver_defaults = impl->solver.options();
    const auto nv = static_cast<std::size_t>(model->nv());
    impl->granted = Mask(nv);
    impl->aligned = Mask(nv);
    impl->q.setZero(model->nq());
    impl->v.setZero(model->nv());
    impl->initial = model->neutral();
    return ModuleSet(std::move(impl));
}

Status ModuleSet::set_initial(const Eigen::VectorXd& q) {
    if (impl_->attempted)
        return make_error(ErrorCode::invalid_argument, "module set: set_initial after configure");
    if (q.size() != impl_->model->nq() || !q.allFinite())
        return make_error(ErrorCode::invalid_argument, "module set: initial must be finite with " +
                                                           std::to_string(impl_->model->nq()) + " values");
    impl_->initial = q;
    return {};
}

Status ModuleSet::add(std::shared_ptr<Module> module) {
    if (impl_->attempted) return make_error(ErrorCode::invalid_argument, "module set: add after configure");
    if (!module) return make_error(ErrorCode::invalid_argument, "module set: module is null");
    if (!valid_module_id(module->id()))
        return make_error(ErrorCode::invalid_argument,
                          "module set: invalid module id '" + module->id() + "' (" + kModuleIdRule + ")");
    if (this->module(module->id()))
        return make_error(ErrorCode::invalid_argument, "module set: duplicate module '" + module->id() + "'");
    impl_->modules.push_back(std::move(module));
    return {};
}

Status ModuleSet::configure() {
    Impl& s = *impl_;
    if (s.attempted) return make_error(ErrorCode::invalid_argument, "module set: configure called twice");
    s.attempted = true;

    // Kahn's algorithm; among ready modules the earliest added goes first.
    const std::size_t n = s.modules.size();
    std::vector<std::vector<std::size_t>> deps(n);
    for (std::size_t i = 0; i < n; ++i)
        for (const std::string& d : s.modules[i]->dependencies()) {
            auto it = std::find_if(s.modules.begin(), s.modules.end(),
                                   [&](const std::shared_ptr<Module>& m) { return m->id() == d; });
            if (it == s.modules.end())
                return make_error(ErrorCode::not_found,
                                  "module '" + s.modules[i]->id() + "': missing dependency '" + d + "'");
            deps[i].push_back(static_cast<std::size_t>(it - s.modules.begin()));
        }
    std::vector<std::size_t> order;
    std::vector<bool> done(n, false);
    while (order.size() < n) {
        std::size_t next = n;
        for (std::size_t i = 0; i < n && next == n; ++i)
            if (!done[i] &&
                std::all_of(deps[i].begin(), deps[i].end(), [&](std::size_t d) { return done[d]; }))
                next = i;
        if (next == n) {
            std::string ids;
            for (std::size_t i = 0; i < n; ++i)
                if (!done[i]) ids += (ids.empty() ? "" : ", ") + s.modules[i]->id();
            return make_error(ErrorCode::invalid_argument, "module set: dependency cycle among " + ids);
        }
        done[next] = true;
        order.push_back(next);
    }

    const auto nv = static_cast<std::size_t>(s.model->nv());
    SolverOptions options = s.solver.options();
    Mask locked = options.locked.size() == nv ? options.locked : Mask(nv);
    const Mask locked_before = locked;
    std::vector<std::shared_ptr<Module>> sorted;
    s.entries.reserve(n);
    s.sync_limits();
    for (std::size_t i : order) {
        Entry e;
        e.module = s.modules[i];
        e.order = sorted.size();
        e.driven = Mask(nv);
        ModuleSetup::State state{s.model,       s.context, s.initial,      s.lower_limit, s.upper_limit,
                                 s.solver,      *e.module, e.module->id(), e.tasks,       e.task_options,
                                 e.constraints, e.driven,  locked};
        ModuleSetup setup(state);
        if (Status st = e.module->configure(setup); !st)
            return with_context(st.error(), "module '" + e.module->id() + "'");
        e.module->task_active_.assign(e.tasks.size(), 1);
        e.module->task_locked_.assign(e.tasks.size(), 0);
        e.module->tasks_dirty_ = false;
        sorted.push_back(e.module);
        s.entries.push_back(std::move(e));
    }
    if (locked != locked_before) {
        options.locked = std::move(locked);
        if (Status st = s.solver.set_options(std::move(options)); !st) return st;
    }
    s.modules = std::move(sorted);
    s.claimants.reserve(n);
    s.configured = true;
    return {};
}

bool ModuleSet::configured() const noexcept { return impl_->configured; }

// Grant by (priority desc, claim start asc, step order asc); losers and overlapped modules are held.
Status ModuleSet::Impl::arbitrate() {
    claimants.clear();
    for (std::size_t i = 0; i < entries.size(); ++i) {
        Entry& e = entries[i];
        Module& m = *e.module;
        m.held_by_ = nullptr;
        m.locked_by_ = nullptr;
        const bool claiming = m.enabled_ && e.claim.joints && e.claim.joints->any();
        if (claiming && !e.claimed) e.claim_since = cycle;
        e.claimed = claiming;
        if (claiming) {
            if (e.claim.joints->size() != granted.size())
                return make_error(ErrorCode::invalid_argument,
                                  "module '" + m.id() + "': claim mask size != nv");
            claimants.push_back(i);
        }
    }
    std::sort(claimants.begin(), claimants.end(), [&](std::size_t a, std::size_t b) {
        const Entry& x = entries[a];
        const Entry& y = entries[b];
        if (x.claim.priority != y.claim.priority) return x.claim.priority > y.claim.priority;
        if (x.claim_since != y.claim_since) return x.claim_since < y.claim_since;
        return x.order < y.order;
    });
    granted.set_range(0, granted.size(), false);
    for (std::size_t i : claimants) {
        Entry& e = entries[i];
        const std::string* holder = nullptr;
        for (std::size_t j : claimants) {
            if (j == i) break;
            const Entry& w = entries[j];
            if (w.module->held_by_ == nullptr &&
                (intersects(*e.claim.joints, *w.claim.joints) || intersects(e.driven, *w.claim.joints) ||
                 intersects(*e.claim.joints, w.driven))) {
                holder = &w.module->id();
                break;
            }
        }
        if (holder) {
            e.module->held_by_ = holder;
        } else {
            merge(granted, *e.claim.joints);
        }
    }
    for (std::size_t i = 0; i < entries.size(); ++i) {
        Entry& e = entries[i];
        if (e.claimed || !e.module->enabled_ || !intersects(e.driven, granted)) continue;
        for (std::size_t j : claimants)
            if (entries[j].module->held_by_ == nullptr && intersects(e.driven, *entries[j].claim.joints)) {
                e.module->held_by_ = &entries[j].module->id();
                break;
            }
    }
    for (std::size_t j : claimants) {
        const Entry& w = entries[j];
        if (!w.claim.exclusive || w.module->held_by_ != nullptr) continue;
        for (Entry& e : entries)
            if (&e != &w && !e.module->locked_by_ &&
                (intersects(e.driven, *w.claim.joints) ||
                 (e.claim.joints && intersects(*e.claim.joints, *w.claim.joints))))
                e.module->locked_by_ = &w.module->id();
    }
    return {};
}

Status ModuleSet::step(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v,
                       double now, double dt, SolveResult& out) {
    return step(q, v, nullptr, now, dt, out);
}

Status ModuleSet::step(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::Ref<const Eigen::VectorXd>& v,
                       const Eigen::VectorXd* measured, double now, double dt, SolveResult& out) {
    Impl& s = *impl_;
    if (!s.configured) return make_error(ErrorCode::invalid_argument, "module set: not configured");
    if (q.size() != s.model->nq() || v.size() != s.model->nv())
        return make_error(ErrorCode::invalid_argument, "module set: q/v size mismatch");
    if (measured && measured->size() != s.model->nq())
        return make_error(ErrorCode::invalid_argument, "module set: measured size mismatch");
    if (!std::isfinite(now) || !std::isfinite(dt) || dt <= 0.0)
        return make_error(ErrorCode::invalid_argument, "module set: now must be finite and dt > 0");
    if (!q.allFinite() || !v.allFinite() || (measured && !measured->allFinite()))
        return make_error(ErrorCode::invalid_argument, "module set: q/v/measured not finite");
    s.q = q;
    if (Status st = s.model->normalize(s.q); !st) return with_context(st.error(), "module set");
    s.v = v;
    if (s.resync) ++s.cycle;
    s.aligned.set_range(0, s.aligned.size(), false);
    s.sync_limits();
    Cycle cycle{now,
                dt,
                s.cycle,
                s.q,
                s.v,
                s.workspace,
                measured,
                &s.aligned,
                s.resync && s.rebase,
                &s.lower_limit,
                &s.upper_limit,
                s.limits_revision};

    for (Entry& e : s.entries) {
        Module& m = *e.module;
        if (m.enabled_ != e.constraints_on) {
            if (!m.enabled_)
                if (Status st = m.on_disabled(cycle); !st)
                    return with_context(st.error(), "module '" + m.id() + "'");
            if (Status st = s.set_constraints(e, m.enabled_); !st) return st;
        }
        e.claim = {};
        if (!m.enabled_) continue;
        if (Status st = m.prepare(cycle); !st) return with_context(st.error(), "module '" + m.id() + "'");
        e.claim = m.claim();
    }
    for (Entry& r : s.entries) {
        if (!r.module->control_requested_) continue;
        r.module->control_requested_ = false;
        if (!r.module->enabled_) continue;
        for (Entry& e : s.entries)
            if (&e != &r && e.claim.joints && intersects(r.driven, *e.claim.joints) &&
                e.module->yield_control())
                e.claim = e.module->claim();
    }
    if (Status st = s.arbitrate(); !st) return st;

    if (Status st = s.workspace.update(s.q); !st) return st;
    // Running -> held, before any contribute() so an aborting module drops its work this cycle.
    for (Entry& e : s.entries) {
        Module& m = *e.module;
        if (!m.running_ || !m.enabled_ || m.held_by_ == nullptr) continue;
        ++m.preempted_;
        if (e.claim.on_held == OnHeld::abort)
            if (Status st = m.on_preempted(cycle, *m.held_by_); !st)
                return with_context(st.error(), "module '" + m.id() + "'");
    }
    for (Entry& e : s.entries) {
        Module& m = *e.module;
        const bool running = m.enabled_ && m.held_by_ == nullptr;
        const bool resumed = running && (!m.running_ || s.resync);
        m.running_ = running;
        if (running != e.tasks_on)
            if (Status st = s.set_tasks(e, running); !st) return st;
        if (resumed)
            if (Status st = m.reset(cycle); !st) return with_context(st.error(), "module '" + m.id() + "'");
        if (running) {
            if (Status st = m.contribute(cycle); !st)
                return with_context(st.error(), "module '" + m.id() + "'");
            if (m.tasks_dirty_)
                if (Status st = s.set_tasks(e, true); !st) return st;
        }
    }
    s.resync = false;
    s.rebase = false;
    ++s.cycle;
    return s.solver.solve(s.q, measured, dt, out);
}

void ModuleSet::resynchronize(bool rebase) noexcept {
    impl_->resync = true;
    impl_->rebase = impl_->rebase || rebase;
}

const Mask& ModuleSet::aligned() const noexcept { return impl_->aligned; }

std::shared_ptr<Module> ModuleSet::module(const std::string& id) const {
    for (const auto& m : impl_->modules)
        if (m->id() == id) return m;
    return nullptr;
}

const std::vector<std::shared_ptr<Module>>& ModuleSet::modules() const noexcept { return impl_->modules; }

const std::vector<std::string>& ModuleSet::tasks(const std::string& id) const {
    static const std::vector<std::string> none;
    for (const Entry& e : impl_->entries)
        if (e.module->id() == id) return e.tasks;
    return none;
}
std::vector<int> ModuleSet::task_levels(const std::string& id) const {
    std::vector<int> levels;
    for (const Entry& e : impl_->entries)
        if (e.module->id() == id)
            for (const TaskOptions& o : e.task_options)
                if (o.level && std::find(levels.begin(), levels.end(), *o.level) == levels.end())
                    levels.push_back(*o.level);
    std::sort(levels.begin(), levels.end());
    return levels;
}
const Mask& ModuleSet::driven(const std::string& id) const {
    static const Mask none;
    for (const Entry& e : impl_->entries)
        if (e.module->id() == id) return e.driven;
    return none;
}
const ModelPtr& ModuleSet::model() const noexcept { return impl_->model; }
const Eigen::VectorXd& ModuleSet::initial() const noexcept { return impl_->initial; }
Solver& ModuleSet::solver() noexcept { return impl_->solver; }

Status ModuleSet::tune_solver(const ConfigNode& params) {
    SolverOptions next = impl_->solver.options();
    if (Status st = builtin_schemas::solver().patch(params, next); !st) return st;
    return impl_->solver.set_options(std::move(next));
}

ConfigNode ModuleSet::solver_params() const {
    return builtin_schemas::solver().values(impl_->solver.options());
}

Status ModuleSet::tune_joint_limits(const ConfigNode& params) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    const Model& m = *impl_->model;
    auto fail = [](const std::string& what) {
        return make_error(ErrorCode::invalid_argument, "joint_limits: " + what);
    };
    if (params.kind() != ConfigNode::Kind::map) return fail("expected a map");
    SolverOptions next = impl_->solver.options();
    const ConfigNode* reset = params.find("reset");
    if (reset && reset->kind() != ConfigNode::Kind::boolean) return fail("reset must be a bool");
    if (reset && reset->as_bool()) {
        next.velocity_limit = impl_->solver_defaults.velocity_limit;
        next.lower_position_limit = impl_->solver_defaults.lower_position_limit;
        next.upper_position_limit = impl_->solver_defaults.upper_position_limit;
    }
    if (next.velocity_limit.size() != m.nv()) next.velocity_limit.setConstant(m.nv(), kInf);
    if (next.lower_position_limit.size() != m.nq()) next.lower_position_limit.setConstant(m.nq(), -kInf);
    if (next.upper_position_limit.size() != m.nq()) next.upper_position_limit.setConstant(m.nq(), kInf);
    auto joint = [&](const std::string& name) -> const JointInfo* {
        Result<JointId> id = m.joint_id(name);
        return id ? &m.joints()[static_cast<std::size_t>(*id)] : nullptr;
    };
    auto number = [](const ConfigNode& n, double& out) {
        if (n.kind() != ConfigNode::Kind::number || !std::isfinite(n.as_number())) return false;
        out = n.as_number();
        return true;
    };
    for (const auto& [key, section] : params.entries()) {
        if (key == "reset") continue;
        if (key != "velocity" && key != "position")
            return fail("unknown key " + key + "; expected reset, velocity or position");
        if (section.kind() != ConfigNode::Kind::map) return fail(key + " must be a map");
        if (key == "velocity") {
            for (const auto& [name, node] : section.entries()) {
                const JointInfo* j = joint(name);
                if (!j || j->nv == 0) return fail("unknown joint or no own velocity: " + name);
                double v = 0;
                if (!number(node, v) || v < 0) return fail("velocity." + name + " must be finite and >= 0");
                const double model_v = m.velocity_limit()[j->idx_v];
                const bool bounded = model_v > 1e-10 && model_v < 1e20;
                if (bounded && v > model_v)
                    return fail("velocity." + name + " exceeds the model limit " + std::to_string(model_v));
                next.velocity_limit.segment(j->idx_v, j->nv).setConstant(v == 0 ? kInf : v);
            }
        } else if (key == "position") {
            for (const auto& [side, joints] : section.entries()) {
                if (side != "lower" && side != "upper")
                    return fail("position." + side + ": expected lower or upper");
                if (joints.kind() != ConfigNode::Kind::map)
                    return fail("position." + side + " must be a map");
                const bool lower = side == "lower";
                for (const auto& [name, node] : joints.entries()) {
                    const JointInfo* j = joint(name);
                    if (!j || j->nq != 1 || j->nv != 1) return fail("unknown or non-scalar joint: " + name);
                    double q = 0;
                    if (!number(node, q)) return fail("position." + side + "." + name + " must be finite");
                    const bool limited = m.has_position_limit(j->id);
                    const double bound =
                        lower ? m.lower_position_limit()[j->idx_q] : m.upper_position_limit()[j->idx_q];
                    if (limited && (lower ? q < bound : q > bound))
                        return fail("position." + side + "." + name + " is outside the model range");
                    const bool drop = limited && q == bound;
                    (lower ? next.lower_position_limit : next.upper_position_limit)[j->idx_q] =
                        drop ? (lower ? -kInf : kInf) : q;
                }
            }
        } else {
            return fail("unknown key " + key + "; expected reset, velocity or position");
        }
    }
    for (const JointInfo& j : m.joints()) {
        if (j.nq != 1 || j.nv != 1) continue;
        const bool limited = m.has_position_limit(j.id);
        const double lo =
            std::max(next.lower_position_limit[j.idx_q], limited ? m.lower_position_limit()[j.idx_q] : -kInf);
        const double hi =
            std::min(next.upper_position_limit[j.idx_q], limited ? m.upper_position_limit()[j.idx_q] : kInf);
        if (!(lo < hi)) return fail("joint " + j.name + ": lower must stay below upper");
    }
    return impl_->solver.set_options(std::move(next));
}

ConfigNode ModuleSet::joint_limits() const {
    const Model& m = *impl_->model;
    const SolverOptions& o = impl_->solver.options();
    ConfigNode velocity = ConfigNode::map(), lower = ConfigNode::map(), upper = ConfigNode::map();
    for (const JointInfo& j : m.joints()) {
        if (j.nv > 0 && o.velocity_limit.size() == m.nv() && std::isfinite(o.velocity_limit[j.idx_v]))
            velocity.set(j.name, o.velocity_limit[j.idx_v]);
        if (j.nq != 1 || j.nv != 1) continue;
        if (o.lower_position_limit.size() == m.nq() && std::isfinite(o.lower_position_limit[j.idx_q]))
            lower.set(j.name, o.lower_position_limit[j.idx_q]);
        if (o.upper_position_limit.size() == m.nq() && std::isfinite(o.upper_position_limit[j.idx_q]))
            upper.set(j.name, o.upper_position_limit[j.idx_q]);
    }
    ConfigNode position = ConfigNode::map();
    position.set("lower", std::move(lower));
    position.set("upper", std::move(upper));
    ConfigNode out = ConfigNode::map();
    out.set("velocity", std::move(velocity));
    out.set("position", std::move(position));
    return out;
}

ConfigNode ModuleSet::solver_tune_schema() const {
    return builtin_schemas::solver().tune_schema(impl_->solver_defaults);
}

}  // namespace kstone
