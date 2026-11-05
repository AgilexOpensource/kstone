#include "kstone/control/solver.hpp"

#include "kstone/model/workspace.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace kstone {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr int kCutPasses = 4;
constexpr int kClipBisections = 8;
// Clip trial scale = backoff x estimated admitted fraction; below the minimum scale the step holds.
constexpr double kClipBackoff = 0.9;
constexpr double kClipMinScale = 1.0 / 256.0;
// Exit flags reported when the QP claims success but the solution violates the hard part.
constexpr int kExitBoxViolated = -100;
constexpr int kExitRowsViolated = -101;

struct TaskEntry {
    std::string name;
    std::shared_ptr<Task> task;
    TaskOptions options;
};

struct ConstraintEntry {
    std::string name;
    std::shared_ptr<Constraint> constraint;
    bool enabled{true};
};

// Mimic pair between two scalar joints (coupled pinocchio mimics have no coordinates of their own).
struct ScalarMimic {
    Index follower_q, leader_q;
    Index follower_v, leader_v;
    double multiplier, offset;
};

struct Stage {
    int level{-1};
    std::vector<const TaskEntry*> tasks;
};

Status validate_options(const Model& model, const SolverOptions& o) {
    const Index nq = model.nq(), nv = model.nv();
    if (!std::isfinite(o.damping) || o.damping <= 0)
        return make_error(ErrorCode::invalid_argument, "solver: damping must be finite and > 0");
    if (!std::isfinite(o.limit_gain) || o.limit_gain <= 0 || o.limit_gain > 1)
        return make_error(ErrorCode::invalid_argument, "solver: limit_gain must be in (0, 1]");
    if (o.velocity_limit.size() != 0) {
        if (o.velocity_limit.size() != nv)
            return make_error(ErrorCode::invalid_argument, "solver: velocity_limit must be empty or size nv");
        for (Index i = 0; i < nv; ++i) {
            const double v = o.velocity_limit[i];
            if (std::isfinite(v) && v < 0)
                return make_error(ErrorCode::invalid_argument, "solver: velocity_limit entries must be >= 0");
        }
    }
    for (const Eigen::VectorXd* limit : {&o.lower_position_limit, &o.upper_position_limit}) {
        if (limit->size() != 0 && limit->size() != nq)
            return make_error(ErrorCode::invalid_argument,
                              "solver: position limits must be empty or size nq");
        if (limit->array().isNaN().any())
            return make_error(ErrorCode::invalid_argument, "solver: position limits must not be NaN");
    }
    if (o.locked.size() != 0 && o.locked.size() != static_cast<std::size_t>(nv))
        return make_error(ErrorCode::invalid_argument, "solver: locked mask must be empty or size nv");
    if (!std::isfinite(o.feasibility_tol) || o.feasibility_tol <= 0)
        return make_error(ErrorCode::invalid_argument, "solver: feasibility_tol must be finite and > 0");
    return {};
}

Status validate_task_options(const TaskOptions& o) {
    if (o.level && *o.level < 0)
        return make_error(ErrorCode::invalid_argument, "solver: task level must be >= 0");
    return {};
}

Result<std::unique_ptr<qp::Backend>> make_backend(const SolverOptions& o) {
    if (!o.backend) return std::unique_ptr<qp::Backend>(std::make_unique<qp::DaqpBackend>(o.daqp));
    std::unique_ptr<qp::Backend> backend = o.backend();
    if (!backend) return make_error(ErrorCode::invalid_argument, "solver: backend factory returned null");
    return backend;
}

// Intersect [low, high] with [other_low, other_high]; disjoint ranges collapse onto the nearest end.
void intersect(double& low, double& high, double other_low, double other_high) {
    if (other_low > high) {
        low = high;
    } else if (other_high < low) {
        high = low;
    } else {
        low = std::max(low, other_low);
        high = std::min(high, other_high);
    }
}

Error with_context(const Error& e, const std::string& what) {
    return make_error(e.code, what + ": " + e.message);
}

// Grow-only scratch: resizing an Eigen object to a different size reallocates.
void ensure(Eigen::VectorXd& v, Index n) {
    if (v.size() < n) v.resize(n);
}
void ensure(Eigen::MatrixXd& a, Index rows, Index cols) {
    if (a.cols() != cols)
        a.resize(rows, cols);
    else if (a.rows() < rows)
        a.resize(rows, cols);
}

}  // namespace

const char* to_string(SolveMode mode) noexcept {
    switch (mode) {
        case SolveMode::weighted:
            return "weighted";
        case SolveMode::lexicographic:
            return "lexicographic";
        case SolveMode::layered_independent:
            return "layered_independent";
        case SolveMode::layered_coupled:
            return "layered_coupled";
    }
    return "unknown";
}

const char* to_string(SolveQuality quality) noexcept {
    switch (quality) {
        case SolveQuality::nominal:
            return "nominal";
        case SolveQuality::degraded:
            return "degraded";
    }
    return "unknown";
}

SolveQuality solve_quality(const SolveResult& result) noexcept {
    if (result.held || result.step_scale < 1.0) return SolveQuality::degraded;
    for (const StageReport& s : result.stages)
        if (s.held || s.status != qp::SolveStatus::optimal) return SolveQuality::degraded;
    return SolveQuality::nominal;
}

struct Solver::Impl {
    ModelPtr model;
    SolverOptions options;
    std::uint64_t options_revision{0};
    std::unique_ptr<qp::Backend> backend;
    Workspace workspace;
    Workspace feedback_ws;  // closed-loop constraints, valid while `closed` is set
    bool closed{false};
    std::vector<TaskEntry> tasks;
    std::vector<ConstraintEntry> constraints;
    std::vector<ScalarMimic> mimics;

    // Scratch, reused across steps so a warm solve does not allocate.
    qp::Problem problem;
    qp::Solution solution;
    qp::LexicographicWorkspace lexicographic;
    std::vector<const TaskEntry*> active;
    std::vector<Stage> stages;  // first `stage_count` in use; inner vectors keep their capacity
    std::size_t stage_count{0};
    std::vector<int> levels;
    Mask locked, free_mask, owned_mask;  // owned: free in some stage of this solve
    std::vector<Mask> later_masks;       // layered_independent: joints free in stages after stage i
    const Mask* later{nullptr};          // later_masks entry of the stage being solved, or null

    // Sized for every registered task, so toggling tasks between steps does not allocate.
    void reserve_task_scratch() {
        const std::size_t n = tasks.size();
        active.reserve(n);
        levels.reserve(n);
        if (stages.size() < n) stages.resize(n);
        for (Stage& s : stages) s.tasks.reserve(n);
    }
    Eigen::MatrixXd weighted_jacobian;
    Eigen::VectorXd row_scratch, drift_scratch, x_step;
    Eigen::VectorXd box_lo, box_hi;  // start-of-step box, displacement units
    Eigen::VectorXd step_limit;      // velocity limit * dt, before constraint tightening
    Eigen::VectorXd x_total, q_work, q_feedback;
    Eigen::MatrixXd hard_A;
    Eigen::VectorXd hard_lo, hard_hi;
    Index hard_rows{0};  // rows of `problem` that are hard (constraints + mimic)
    Eigen::VectorXd x_saved;
    std::vector<StageReport> saved_stages;

    Impl(ModelPtr m, SolverOptions o, std::unique_ptr<qp::Backend> b)
        : model(std::move(m)),
          options(std::move(o)),
          backend(std::move(b)),
          workspace(model),
          feedback_ws(model) {
        for (const MimicRelation& rel : model->mimics()) {
            Result<JointId> f = model->joint_id(rel.follower);
            Result<JointId> l = model->joint_id(rel.leader);
            if (!f || !l) continue;
            const JointInfo& fj = model->joints()[*f];
            const JointInfo& lj = model->joints()[*l];
            if (fj.nv != 1 || lj.nv != 1 || fj.nq != 1 || lj.nq != 1) continue;
            mimics.push_back({fj.idx_q, lj.idx_q, fj.idx_v, lj.idx_v, rel.multiplier, rel.offset});
        }
    }

    TaskEntry* find_task(const std::string& name) {
        for (TaskEntry& e : tasks)
            if (e.name == name) return &e;
        return nullptr;
    }
    ConstraintEntry* find_constraint(const std::string& name) {
        for (ConstraintEntry& e : constraints)
            if (e.name == name) return &e;
        return nullptr;
    }

    Status solve(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::VectorXd* feedback, double dt,
                 SolveResult& out);

  private:
    void build_box(const Eigen::Ref<const Eigen::VectorXd>& q, double dt);
    void build_stages();
    Stage& next_stage(int level);
    void collect_levels(const std::vector<const TaskEntry*>& entries);
    void build_free_set(const Stage& stage);
    Status gather_hard_rows(const Tick& tick, const Mask& free);
    Status solve_stage(const Stage& stage, const Mask& free, double dt, StageReport& report);
    void build_weighted(const Stage& stage);
    void build_lexicographic(const Stage& stage);
    int check_feasible() const;  // 0 or a kExit* flag
    Status check_finite(const Eigen::VectorXd& x, const char* what) const;
    void widen_to_hold();
    bool hold_admissible() const;  // x = 0 meets the box and every hard row
};

void Solver::Impl::build_box(const Eigen::Ref<const Eigen::VectorXd>& q, double dt) {
    const Model& m = *model;
    const Index nv = m.nv();
    box_lo.setConstant(nv, -kInf);
    box_hi.setConstant(nv, kInf);
    const Eigen::VectorXd& model_vel = m.velocity_limit();
    for (Index i = 0; i < nv; ++i) {
        double limit = kInf;
        if (options.velocity_limit.size() == nv && std::isfinite(options.velocity_limit[i])) {
            limit = options.velocity_limit[i];
        } else if (model_vel[i] > 1e-10 && model_vel[i] < 1e20) {
            limit = model_vel[i];
        }
        box_lo[i] = -limit;
        box_hi[i] = limit;
    }
    step_limit = box_hi * dt;
    Tick tick{workspace, dt};
    Tick feedback_tick{feedback_ws, dt};
    for (const ConstraintEntry& c : constraints) {
        if (!c.enabled) continue;
        if (closed && c.constraint->closed_loop())
            c.constraint->tighten_bounds(feedback_tick, box_lo, box_hi);
        c.constraint->tighten_bounds(tick, box_lo, box_hi);
    }
    box_lo *= dt;
    box_hi *= dt;

    const Eigen::VectorXd& model_lo = m.lower_position_limit();
    const Eigen::VectorXd& model_hi = m.upper_position_limit();
    const bool has_lo = options.lower_position_limit.size() == m.nq();
    const bool has_hi = options.upper_position_limit.size() == m.nq();
    for (const JointInfo& j : m.joints()) {
        if (j.id == 0 || j.nq != 1 || j.nv != 1) continue;
        const bool limited = m.has_position_limit(j.id);
        double lo = limited ? model_lo[j.idx_q] : -kInf;
        double hi = limited ? model_hi[j.idx_q] : kInf;
        if (has_lo && std::isfinite(options.lower_position_limit[j.idx_q]))
            lo = std::max(lo, options.lower_position_limit[j.idx_q]);
        if (has_hi && std::isfinite(options.upper_position_limit[j.idx_q]))
            hi = std::min(hi, options.upper_position_limit[j.idx_q]);
        if (!std::isfinite(lo) && !std::isfinite(hi)) continue;
        const double qi = q[j.idx_q];
        const double low = std::isfinite(lo) ? -options.limit_gain * (qi - lo) : -kInf;
        const double high = std::isfinite(hi) ? options.limit_gain * (hi - qi) : kInf;
        intersect(box_lo[j.idx_v], box_hi[j.idx_v], std::min(low, high), std::max(low, high));
    }
}

// Unleveled tasks join only the last stage (layered) or follow the levels (lexicographic).
// Sorted distinct levels of the leveled entries.
void Solver::Impl::collect_levels(const std::vector<const TaskEntry*>& entries) {
    levels.clear();
    for (const TaskEntry* e : entries)
        if (e->options.level) levels.push_back(*e->options.level);
    std::sort(levels.begin(), levels.end());
    levels.erase(std::unique(levels.begin(), levels.end()), levels.end());
}

Stage& Solver::Impl::next_stage(int level) {
    if (stages.size() <= stage_count) stages.emplace_back();
    Stage& s = stages[stage_count++];
    s.level = level;
    s.tasks.clear();
    return s;
}

void Solver::Impl::build_stages() {
    stage_count = 0;
    collect_levels(active);
    const auto layer = [&](int level, bool coupled, bool last) {
        Stage& s = next_stage(level);
        for (const TaskEntry* e : active) {
            const std::optional<int>& l = e->options.level;
            if (l ? (coupled ? *l <= level : *l == level) : last) s.tasks.push_back(e);
        }
    };
    const bool single = options.mode == SolveMode::weighted || options.mode == SolveMode::lexicographic;
    switch (single || levels.empty() ? SolveMode::weighted : options.mode) {
        case SolveMode::weighted:
        case SolveMode::lexicographic: {
            Stage& s = next_stage(levels.empty() ? -1 : levels.back());
            s.tasks.insert(s.tasks.end(), active.begin(), active.end());
            break;
        }
        case SolveMode::layered_independent:
            for (int level : levels) layer(level, false, level == levels.back());
            break;
        case SolveMode::layered_coupled:
            for (int level : levels) layer(level, true, level == levels.back());
            break;
    }
}

void Solver::Impl::build_free_set(const Stage& stage) {
    const auto nv = static_cast<std::size_t>(model->nv());
    Mask& free = free_mask;
    if (free.size() != nv) free = Mask(nv);
    free.set_range(0, nv, false);
    // An unmasked unleveled task beside leveled tasks frees nothing.
    const bool leveled = std::any_of(stage.tasks.begin(), stage.tasks.end(),
                                     [](const TaskEntry* e) { return e->options.level.has_value(); });
    for (const TaskEntry* e : stage.tasks) {
        const Mask& mask = e->task->mask();
        if (mask.size() == 0 && !e->options.level && leveled) continue;
        if (mask.size() == 0) {
            free.set_range(0, nv, true);
        } else {
            free |= mask;
        }
    }
    for (const ScalarMimic& mm : mimics) {
        const auto f = static_cast<std::size_t>(mm.follower_v);
        const auto l = static_cast<std::size_t>(mm.leader_v);
        if (free.test(f) || free.test(l)) {
            free.set(f);
            free.set(l);
        }
    }
    free -= locked;
}

Status Solver::Impl::gather_hard_rows(const Tick& tick, const Mask& free) {
    const Index nv = model->nv();
    Index rows = 0;
    // A closed-loop constraint always owns a second block (measured rows), unbounded without fresh feedback.
    for (const ConstraintEntry& c : constraints)
        if (c.enabled) rows += c.constraint->rows() * (c.constraint->closed_loop() ? 2 : 1);
    // One row per mimic whether or not it is active, so the QP layout stays fixed across stages.
    const auto mimic_rows = static_cast<Index>(mimics.size());
    hard_A.setZero(rows + mimic_rows, nv);
    hard_lo.setConstant(rows + mimic_rows, -kInf);
    hard_hi.setConstant(rows + mimic_rows, kInf);
    Index r = 0;
    const Tick feedback_tick{feedback_ws, tick.dt};
    const auto emit = [&](const ConstraintEntry& c, const Tick& at, Index k) -> Status {
        auto A = hard_A.middleRows(r, k);
        auto lo = hard_lo.segment(r, k);
        auto hi = hard_hi.segment(r, k);
        if (Status s = c.constraint->evaluate(at, A, lo, hi); !s)
            return with_context(s.error(), "constraint '" + c.name + "'");
        // Velocity rows -> displacement rows.
        lo *= tick.dt;
        hi *= tick.dt;
        // Rows sharing the step budget already spent part of it in earlier stages.
        if (c.constraint->shares_step_budget() && !x_total.isZero(0.0)) {
            ensure(row_scratch, k);
            auto spent = row_scratch.head(k);
            spent.noalias() = A * x_total;
            lo -= spent;
            hi -= spent;
        }
        // A row this stage cannot move but another stage can is left to that stage; a row no stage moves is checked.
        const Mask* yields = c.constraint->defers_to_later_stages() ? later : nullptr;
        for (Index i = 0; i < k; ++i) {
            bool touched = false, owned = false;
            // layered_independent: the bounds widen by what joints only later stages move can still yield.
            double yield_down = 0.0, yield_up = 0.0;
            for (Index j = 0; j < nv; ++j) {
                const double a = A(i, j);
                if (a == 0.0) continue;
                const auto b = static_cast<std::size_t>(j);
                touched = touched || free.test(b);
                owned = owned || owned_mask.test(b);
                if (yields && yields->test(b) && !free.test(b)) {
                    yield_down += std::max(-a * box_lo[j], -a * box_hi[j]);
                    yield_up += std::max(a * box_hi[j], a * box_lo[j]);
                }
            }
            if (!touched && owned) {
                lo[i] = -kInf;
                hi[i] = kInf;
            } else {
                hi[i] += yield_down;
                lo[i] -= yield_up;
            }
        }
        r += k;
        return {};
    };
    for (const ConstraintEntry& c : constraints) {
        if (!c.enabled) continue;
        const Index k = c.constraint->rows();
        if (k == 0) continue;
        // Measured first, so the constraint's last evaluation (its report) is the commanded one.
        if (c.constraint->closed_loop()) {
            if (closed) {
                if (Status s = emit(c, feedback_tick, k); !s) return s;
            } else {
                r += k;
            }
        }
        if (Status s = emit(c, tick, k); !s) return s;
    }
    // Mimic velocity coupling x_f = m x_l. Position drift (or an inconsistent start) is removed by
    // the end-of-step projection, so these rows never ask for more than the box allows.
    // An inactive mimic (follower fixed by the box) keeps its row unbounded.
    for (const ScalarMimic& mm : mimics) {
        hard_A(r, mm.follower_v) = 1.0;
        if (free.test(static_cast<std::size_t>(mm.leader_v))) hard_A(r, mm.leader_v) = -mm.multiplier;
        if (free.test(static_cast<std::size_t>(mm.follower_v))) {
            hard_lo[r] = 0.0;
            hard_hi[r] = 0.0;
        }
        ++r;
    }
    return {};
}

void Solver::Impl::build_weighted(const Stage& stage) {
    const Index nv = model->nv();
    problem.H.setIdentity(nv, nv);
    problem.H *= options.damping;
    problem.g.setZero(nv);
    const bool coupled = options.mode == SolveMode::layered_coupled;
    const bool independent = options.mode == SolveMode::layered_independent;
    for (const TaskEntry* e : stage.tasks) {
        const Task& t = *e->task;
        const Index k = t.rows();
        if (k == 0) continue;
        ensure(weighted_jacobian, k, nv);
        ensure(row_scratch, k);
        auto WJ = weighted_jacobian.topRows(k);
        auto desired = row_scratch.head(k);
        WJ.noalias() = t.weights().asDiagonal() * t.jacobian();
        // Coupled: earlier levels only get what their own stage left of gain * error at q (first order).
        // Independent: drift from earlier stages this step is removed in full, only the error at q is scaled by gain.
        desired = -t.gain() * t.weights().cwiseProduct(t.error());
        const bool drift = independent || (coupled && e->options.level && *e->options.level < stage.level);
        // LM damping from gain * error at q (first order), so drift does not damp its own removal.
        double residual = t.weights().cwiseProduct(desired).norm();
        if (drift) {
            ensure(drift_scratch, k);
            auto jx = drift_scratch.head(k);
            jx.noalias() = WJ * x_total;
            residual = t.weights().cwiseProduct(desired + t.gain() * jx).norm();
            desired -= (1.0 - t.gain()) * jx;
        }
        problem.H.noalias() += WJ.transpose() * WJ;
        if (t.lm_damping() > 0) problem.H.diagonal().array() += t.lm_damping() * residual;
        problem.g.noalias() -= WJ.transpose() * desired;
    }
    problem.A = hard_A;
    problem.lo = hard_lo;
    problem.hi = hard_hi;
    problem.levels.clear();
    problem.level_damping.clear();
    hard_rows = hard_A.rows();
}

void Solver::Impl::build_lexicographic(const Stage& stage) {
    const Index nv = model->nv();
    problem.H.setIdentity(nv, nv);
    problem.H *= options.damping;
    problem.g.setZero(nv);
    hard_rows = hard_A.rows();

    // Order: hard rows, then task levels ascending, then unleveled tasks.
    collect_levels(stage.tasks);

    Index total = hard_rows;
    for (const TaskEntry* e : stage.tasks)
        for (Index i = 0; i < e->task->rows(); ++i)
            if (e->task->weights()[i] > 0) ++total;
    problem.A.setZero(total, nv);
    problem.lo.resize(total);
    problem.hi.resize(total);
    problem.A.topRows(hard_rows) = hard_A;
    problem.lo.head(hard_rows) = hard_lo;
    problem.hi.head(hard_rows) = hard_hi;
    problem.levels.assign(1, hard_rows);
    problem.level_damping.assign(1, 0.0);
    Index r = hard_rows;
    double mu = 0.0;  // Levenberg-Marquardt damping of the level being emitted, as in build_weighted
    const auto emit = [&](const Task& t) {
        if (t.lm_damping() > 0)
            mu += t.lm_damping() * t.gain() *
                  t.weights().cwiseProduct(t.weights()).cwiseProduct(t.error()).norm();
        for (Index i = 0; i < t.rows(); ++i) {
            const double w = t.weights()[i];
            if (!(w > 0)) continue;
            problem.A.row(r) = w * t.jacobian().row(i);
            problem.lo[r] = problem.hi[r] = -t.gain() * w * t.error()[i];
            ++r;
        }
    };
    const auto close_level = [&]() {
        if (r > problem.levels.back()) {
            problem.levels.push_back(r);
            problem.level_damping.push_back(mu);
        }
        mu = 0.0;
    };
    for (int level : levels) {
        for (const TaskEntry* e : stage.tasks)
            if (e->options.level == level) emit(*e->task);
        close_level();
    }
    for (const TaskEntry* e : stage.tasks)
        if (!e->options.level) emit(*e->task);
    close_level();
    if (problem.levels.size() == 1) {  // no soft rows: plain QP
        problem.levels.clear();
        problem.level_damping.clear();
    }
}

// Pinocchio asserts on non-finite input in debug builds, so velocities are checked before integrate().
Status Solver::Impl::check_finite(const Eigen::VectorXd& x, const char* what) const {
    for (Index i = 0; i < x.size(); ++i) {
        if (std::isfinite(x[i])) continue;
        std::string dof = "variable " + std::to_string(i);
        if (i < model->nv()) {
            const JointInfo& j = model->joints()[model->velocity_owner()[static_cast<std::size_t>(i)]];
            dof = "joint " + j.name + (j.nv == 1 ? "" : "[" + std::to_string(i - j.idx_v) + "]");
        }
        return make_error(ErrorCode::internal, std::string(what) + " a non-finite velocity step (" + dof +
                                                   " = " + std::to_string(x[i]) + ")");
    }
    return {};
}

int Solver::Impl::check_feasible() const {
    const double tol = options.feasibility_tol;
    const Eigen::VectorXd& x = solution.x;
    if (x.size() != problem.variables() || !x.allFinite()) return kExitBoxViolated;
    for (Index i = 0; i < x.size(); ++i)
        if (x[i] < problem.lb[i] - tol || x[i] > problem.ub[i] + tol) return kExitBoxViolated;
    for (Index i = 0; i < hard_rows; ++i) {
        const double ax = problem.A.row(i).dot(x);
        const double lo = problem.lo[i], hi = problem.hi[i];
        if (std::isfinite(lo) && ax < lo - tol * std::max(1.0, std::abs(lo))) return kExitRowsViolated;
        if (std::isfinite(hi) && ax > hi + tol * std::max(1.0, std::abs(hi))) return kExitRowsViolated;
    }
    return 0;
}

bool Solver::Impl::hold_admissible() const {
    const double tol = options.feasibility_tol;
    for (Index i = 0; i < problem.variables(); ++i)
        if (problem.lb[i] > tol || problem.ub[i] < -tol) return false;
    for (Index i = 0; i < hard_rows; ++i)
        if (problem.lo[i] > tol || problem.hi[i] < -tol) return false;
    return true;
}

// Make x = 0 admissible for the box and the inequality rows; equalities (mimic, user) stay.
void Solver::Impl::widen_to_hold() {
    problem.lb = problem.lb.cwiseMin(0.0);
    problem.ub = problem.ub.cwiseMax(0.0);
    for (Index i = 0; i < hard_rows; ++i) {
        if (problem.lo[i] == problem.hi[i]) continue;
        problem.lo[i] = std::min(problem.lo[i], 0.0);
        problem.hi[i] = std::max(problem.hi[i], 0.0);
    }
}

Status Solver::Impl::solve_stage(const Stage& stage, const Mask& free, double dt, StageReport& report) {
    const Index nv = model->nv();
    solution.x.setZero(nv);
    Tick tick{workspace, dt};
    for (const TaskEntry* e : stage.tasks)
        if (Status s = e->task->evaluate(tick); !s) return with_context(s.error(), "task '" + e->name + "'");
    if (Status s = gather_hard_rows(tick, free); !s) return s;
    if (options.mode == SolveMode::lexicographic) {
        build_lexicographic(stage);
    } else {
        build_weighted(stage);
    }
    problem.lb = box_lo - x_total;
    problem.ub = box_hi - x_total;
    for (Index i = 0; i < nv; ++i) {
        if (!free.test(static_cast<std::size_t>(i))) {
            problem.lb[i] = 0.0;
            problem.ub[i] = 0.0;
        } else if (problem.lb[i] > problem.ub[i]) {
            problem.lb[i] = problem.ub[i];
        }
    }

    const auto attempt = [&]() -> Status {
        if (Status s = qp::solve(*backend, problem, solution, lexicographic); !s) return s;
        report.status = solution.status;
        report.exitflag = solution.exitflag;
        report.iterations = solution.iterations;
        if (solution.ok()) {
            if (solution.x.size() != problem.variables())
                return make_error(ErrorCode::internal, std::string("backend '") + backend->name() +
                                                           "' returned " + std::to_string(solution.x.size()) +
                                                           " variables, expected " +
                                                           std::to_string(problem.variables()));
            if (Status s = check_finite(solution.x, "returned"); !s)
                return with_context(s.error(), std::string("backend '") + backend->name() + "'");
        }
        if (solution.ok() && options.check_feasibility) {
            if (int flag = check_feasible(); flag != 0) {
                report.status = qp::SolveStatus::infeasible;
                report.exitflag = flag;
            }
        }
        return {};
    };
    const auto succeeded = [&] {
        return report.status == qp::SolveStatus::optimal || report.status == qp::SolveStatus::inexact;
    };

    if (Status s = attempt(); !s) return with_context(s.error(), "qp");
    if (!succeeded() && options.hold_retry) {
        // A recovery demand (lo > 0 or hi < 0) beyond reach makes the stage infeasible; shrink it before
        // falling back to hold, which would give the recovery up entirely. hard_lo/hard_hi are the unrelaxed rows.
        for (const double scale : {0.5, 0.25}) {
            bool relaxed = false;
            for (Index i = 0; i < hard_rows; ++i) {
                if (hard_lo[i] == hard_hi[i]) continue;
                if (hard_lo[i] > 0.0) {
                    problem.lo[i] = scale * hard_lo[i];
                    relaxed = true;
                }
                if (hard_hi[i] < 0.0) {
                    problem.hi[i] = scale * hard_hi[i];
                    relaxed = true;
                }
            }
            if (!relaxed) break;
            if (Status s = attempt(); !s) return with_context(s.error(), "qp");
            if (succeeded()) break;
        }
        report.held = succeeded();
    }
    if (!succeeded() && options.hold_retry) {
        widen_to_hold();
        if (Status s = attempt(); !s) return with_context(s.error(), "qp");
        report.held = succeeded();
    }
    if (!succeeded() && options.hold_retry && hold_admissible()) {
        // Degenerate active sets (many near-parallel contact rows) can defeat the backend even though x = 0 fits.
        solution.x.setZero(nv);
        report.held = true;
        return {};
    }
    if (!succeeded()) {
        const bool infeasible =
            report.status == qp::SolveStatus::infeasible || report.exitflag <= kExitBoxViolated;
        solution.x.setZero(nv);
        return make_error(infeasible ? ErrorCode::infeasible : ErrorCode::internal,
                          std::string("solver: stage at level ") + std::to_string(stage.level) + " " +
                              qp::to_string(report.status) + " (exitflag " + std::to_string(report.exitflag) +
                              ")");
    }
    return {};
}

Status Solver::Impl::solve(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::VectorXd* feedback,
                           double dt, SolveResult& out) {
    const Model& m = *model;
    const Index nv = m.nv();
    out.v.setZero(nv);
    out.q_next = q;
    out.stages.clear();
    out.held = false;
    out.step_scale = 1.0;
    out.quality = SolveQuality::nominal;
    out.stages.reserve(std::max<std::size_t>(tasks.size(), 1));
    out.residuals.reserve(tasks.size());
    // Residuals are cleared only on the way out, so their name strings keep their capacity.
    const auto fail = [&](const Error& e) -> Status {
        out.v.setZero(nv);
        out.q_next = q;
        out.held = false;
        out.step_scale = 1.0;
        out.residuals.clear();
        return e;
    };
    if (Status s = m.validate(q); !s) return fail(s.error());
    if (!std::isfinite(dt) || dt <= 0)
        return fail(make_error(ErrorCode::invalid_argument, "solver: dt must be > 0"));
    closed = false;
    if (feedback) {
        if (Status s = m.validate(*feedback); !s) return fail(with_context(s.error(), "solver feedback"));
        for (const ConstraintEntry& c : constraints)
            closed = closed || (c.enabled && c.constraint->closed_loop());
        // Identical to q: the commanded workspace already serves.
        if (closed) closed = !(*feedback - q).isZero(1e-12);
    }

    const auto nvs = static_cast<std::size_t>(nv);
    if (locked.size() != nvs) locked = Mask(nvs);
    locked.set_range(0, nvs, false);
    if (options.locked.size() == nvs) locked |= options.locked;
    active.clear();
    for (const TaskEntry& e : tasks) {
        if (!e.options.enabled) continue;
        if (e.options.locked) {
            if (e.task->mask().size() == nvs) locked |= e.task->mask();
            continue;
        }
        active.push_back(&e);
    }
    if (active.empty()) {
        out.residuals.clear();
        return {};
    }

    if (Status s = workspace.update(q); !s) return fail(s.error());
    if (closed) {
        if (Status s = feedback_ws.update(*feedback); !s) return fail(s.error());
        q_feedback.resize(m.nq());
    }
    build_box(q, dt);
    build_stages();
    const auto nv_bits = static_cast<std::size_t>(nv);
    if (owned_mask.size() != nv_bits) owned_mask = Mask(nv_bits);
    owned_mask.set_range(0, nv_bits, false);
    if (later_masks.size() < stage_count) later_masks.resize(stage_count);
    for (std::size_t i = stage_count; i-- > 0;) {
        Mask& m_later = later_masks[i];
        if (m_later.size() != nv_bits) m_later = Mask(nv_bits);
        m_later.set_range(0, nv_bits, false);
        if (i + 1 < stage_count) {
            m_later |= later_masks[i + 1];
            build_free_set(stages[i + 1]);
            m_later |= free_mask;
        }
    }
    for (std::size_t i = 0; i < stage_count; ++i) {
        build_free_set(stages[i]);
        owned_mask |= free_mask;
    }
    const bool defer_rows = options.mode == SolveMode::layered_independent;
    const bool layered =
        options.mode == SolveMode::layered_independent || options.mode == SolveMode::layered_coupled;
    for (const ConstraintEntry& c : constraints)
        if (c.enabled) c.constraint->begin_step(workspace, false);

    // All stages from q into x_total / out.stages. A re-solve first moves the workspaces back to the start.
    bool at_start = true;
    const auto run_stages = [&]() -> Status {
        x_total.setZero(nv);
        q_work = q;
        out.stages.clear();
        out.held = false;
        for (std::size_t i = 0; i < stage_count; ++i) {
            if (i > 0 || !at_start) {
                if (Status s = m.integrate(q, x_total, q_work); !s) return s;
                if (Status s = workspace.update(q_work); !s) return s;
                if (closed) {
                    if (Status s = m.integrate(*feedback, x_total, q_feedback); !s) return s;
                    if (Status s = feedback_ws.update(q_feedback); !s) return s;
                }
            }
            StageReport report;
            report.level = stages[i].level;
            const auto started = std::chrono::steady_clock::now();
            build_free_set(stages[i]);
            later = defer_rows ? &later_masks[i] : nullptr;
            const Status s = solve_stage(stages[i], free_mask, dt, report);
            later = nullptr;
            report.solve_s =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            out.stages.push_back(report);
            if (!s) return s;
            x_total += solution.x;
            if (Status f = check_finite(x_total, "solver: stages sum to"); !f) return f;
            out.held = out.held || report.held;
        }
        at_start = false;
        return {};
    };
    if (Status s = run_stages(); !s) return fail(s.error());

    // The configuration the step actually commands: x_total from q, then the mimic projection and limit clamp.
    const auto finish = [&](Eigen::VectorXd& q_end) -> Status {
        // The backend meets the velocity box only to its primal tolerance, which is large after dividing by dt.
        x_total = x_total.cwiseMax(-step_limit).cwiseMin(step_limit);
        if (Status s = m.integrate(q, x_total, q_end); !s) return s;
        // Followers exactly track their leaders (no drift from the linearized rows).
        for (const ScalarMimic& mm : mimics)
            if (!locked.test(static_cast<std::size_t>(mm.follower_v)))
                q_end[mm.follower_q] = mm.multiplier * q_end[mm.leader_q] + mm.offset;
        // Box rows hold only to the QP tolerance; clamp so long saturated runs never drift past a
        // limit. A start outside the limits is never pulled further in than where it began.
        const Eigen::VectorXd& lower = m.lower_position_limit();
        const Eigen::VectorXd& upper = m.upper_position_limit();
        for (const JointInfo& j : m.joints()) {
            if (j.id == 0 || j.nq != 1 || !m.has_position_limit(j.id)) continue;
            const double lo = lower[j.idx_q], hi = upper[j.idx_q];
            if (!(hi > lo + 1e-6)) continue;
            const double start = q[j.idx_q];
            q_end[j.idx_q] = std::clamp(q_end[j.idx_q], std::min(lo, start), std::max(hi, start));
        }
        return {};
    };

    // Cutting planes: the hard rows are linear at each stage start, so a curved step can end inside a
    // constraint. Re-solve with the rows relinearized at the rejected end; keep the last good solve if
    // a tighter one fails.
    for (int pass = 0; pass < kCutPasses; ++pass) {
        bool refining = false;
        for (const ConstraintEntry& c : constraints)
            refining = refining || (c.enabled && c.constraint->refines());
        // A stage that solved unheld can still meet a cut; when every stage gave up recovery, none can.
        bool any_unheld = false;
        for (const StageReport& r : out.stages) any_unheld = any_unheld || !r.held;
        if (!refining || !any_unheld) break;
        if (Status s = finish(q_work); !s) return fail(s.error());
        if (Status s = workspace.update(q_work); !s) return fail(s.error());
        const Tick end{workspace, dt};
        Index added = 0;
        for (const ConstraintEntry& c : constraints)
            if (c.enabled && c.constraint->refines()) added += c.constraint->add_cuts(end);
        if (added == 0) break;
        x_saved = x_total;
        saved_stages.assign(out.stages.begin(), out.stages.end());
        const bool saved_held = out.held;
        for (const ConstraintEntry& c : constraints)
            if (c.enabled) c.constraint->begin_step(workspace, true);
        if (Status s = run_stages(); !s) {
            x_total = x_saved;
            out.stages.assign(saved_stages.begin(), saved_stages.end());
            out.held = saved_held;
            break;
        }
    }

    Eigen::VectorXd& q_next = out.q_next;
    if (Status s = finish(q_next); !s) return fail(s.error());
    if (Status s = m.difference(q, q_next, out.v); !s) return fail(s.error());

    // Endpoint clip: only when some constraint cannot rule out an unchecked end violation.
    bool check_end = false;
    for (const ConstraintEntry& c : constraints)
        check_end = check_end || (c.enabled && c.constraint->checks_end(q_next, out.v));
    if (check_end) {
        double fraction = 1.0;
        const auto admitted = [&](const Eigen::VectorXd& q_end) -> Result<bool> {
            if (workspace.q() != q_end)
                if (Status s = workspace.update(q_end); !s) return s.error();
            const Tick end{workspace, dt};
            bool ok = true;
            fraction = 1.0;
            // No short-circuit: every rejecting constraint contributes its admitted fraction.
            for (const ConstraintEntry& c : constraints)
                if (c.enabled && !c.constraint->admits(end)) {
                    ok = false;
                    fraction = std::min(fraction, c.constraint->admitted_fraction());
                }
            return ok;
        };
        Result<bool> full = admitted(q_next);
        if (!full) return fail(full.error());
        if (!*full) {
            // Shrink toward the constraints' admitted-fraction estimate; s = 0 (the start) is assumed admitted.
            x_saved = x_total;
            double lo = 0.0, tested = 1.0;
            for (int i = 0; i < kClipBisections; ++i) {
                const double mid = tested * kClipBackoff * fraction;
                if (mid < kClipMinScale) break;
                x_total = mid * x_saved;
                if (Status s = finish(q_next); !s) return fail(s.error());
                Result<bool> ok = admitted(q_next);
                if (!ok) return fail(ok.error());
                if (*ok) {
                    lo = mid;
                    break;
                }
                tested = mid;
            }
            x_total = lo * x_saved;
            if (Status s = finish(q_next); !s) return fail(s.error());
            if (Status s = m.difference(q, q_next, out.v); !s) return fail(s.error());
            out.step_scale = lo;
        }
    }
    out.v /= dt;

    // Residuals at the start configuration for the full step.
    if (layered && stage_count > 1) {
        if (Status s = workspace.update(q); !s) return fail(s.error());
        Tick tick{workspace, dt};
        for (const TaskEntry* e : active)
            if (Status s = e->task->evaluate(tick); !s)
                return fail(with_context(s.error(), "task '" + e->name + "'"));
    }
    x_step = out.v * dt;
    out.residuals.resize(active.size());
    for (std::size_t i = 0; i < active.size(); ++i) {
        const TaskEntry* e = active[i];
        const Task& t = *e->task;
        const Index k = t.rows();
        double residual = 0.0;
        if (k > 0) {
            ensure(row_scratch, k);
            auto jx = row_scratch.head(k);
            jx.noalias() = t.jacobian() * x_step;
            residual = t.weights().cwiseProduct(jx + t.gain() * t.error()).norm();
        }
        TaskResidual& out_residual = out.residuals[i];
        out_residual.name = e->name;
        out_residual.level = e->options.level.value_or(-1);
        out_residual.residual = residual;
    }
    out.quality = solve_quality(out);
    return {};
}

// ---- Solver --------------------------------------------------------------------------------

Solver::Solver(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Solver::~Solver() = default;
Solver::Solver(Solver&&) noexcept = default;
Solver& Solver::operator=(Solver&&) noexcept = default;

Result<Solver> Solver::create(ModelPtr model, SolverOptions options) {
    if (!model) return make_error(ErrorCode::invalid_argument, "solver: model is null");
    if (model->nv() == 0)
        return make_error(ErrorCode::invalid_argument, "solver: model has no degrees of freedom (nv == 0)");
    if (Status s = validate_options(*model, options); !s) return s.error();
    Result<std::unique_ptr<qp::Backend>> backend = make_backend(options);
    if (!backend) return backend.error();
    return Solver(std::make_unique<Impl>(std::move(model), std::move(options), std::move(*backend)));
}

const ModelPtr& Solver::model() const noexcept { return impl_->model; }

Status Solver::add_task(std::string name, std::shared_ptr<Task> task, TaskOptions options) {
    if (name.empty()) return make_error(ErrorCode::invalid_argument, "solver: task name is empty");
    if (!task) return make_error(ErrorCode::invalid_argument, "solver: task '" + name + "' is null");
    if (impl_->find_task(name))
        return make_error(ErrorCode::invalid_argument, "solver: duplicate task '" + name + "'");
    if (Status s = validate_task_options(options); !s) return s;
    if (Status s = task->bind(impl_->model); !s) return with_context(s.error(), "task '" + name + "'");
    impl_->tasks.push_back({std::move(name), std::move(task), options});
    impl_->reserve_task_scratch();
    return {};
}

Status Solver::remove_task(const std::string& name) {
    auto& v = impl_->tasks;
    auto it = std::find_if(v.begin(), v.end(), [&](const TaskEntry& e) { return e.name == name; });
    if (it == v.end()) return make_error(ErrorCode::not_found, "solver: no task '" + name + "'");
    v.erase(it);
    return {};
}

Status Solver::set_task_options(const std::string& name, TaskOptions options) {
    TaskEntry* e = impl_->find_task(name);
    if (!e) return make_error(ErrorCode::not_found, "solver: no task '" + name + "'");
    if (Status s = validate_task_options(options); !s) return s;
    e->options = options;
    return {};
}

std::shared_ptr<Task> Solver::task(const std::string& name) const {
    const TaskEntry* e = impl_->find_task(name);
    return e ? e->task : nullptr;
}

Status Solver::add_constraint(std::string name, std::shared_ptr<Constraint> constraint) {
    if (name.empty()) return make_error(ErrorCode::invalid_argument, "solver: constraint name is empty");
    if (!constraint)
        return make_error(ErrorCode::invalid_argument, "solver: constraint '" + name + "' is null");
    if (impl_->find_constraint(name))
        return make_error(ErrorCode::invalid_argument, "solver: duplicate constraint '" + name + "'");
    if (Status s = constraint->bind(impl_->model); !s)
        return with_context(s.error(), "constraint '" + name + "'");
    impl_->constraints.push_back({std::move(name), std::move(constraint)});
    return {};
}

Status Solver::remove_constraint(const std::string& name) {
    auto& v = impl_->constraints;
    auto it = std::find_if(v.begin(), v.end(), [&](const ConstraintEntry& e) { return e.name == name; });
    if (it == v.end()) return make_error(ErrorCode::not_found, "solver: no constraint '" + name + "'");
    v.erase(it);
    return {};
}

Status Solver::set_constraint_enabled(const std::string& name, bool enabled) {
    ConstraintEntry* e = impl_->find_constraint(name);
    if (!e) return make_error(ErrorCode::not_found, "solver: no constraint '" + name + "'");
    e->enabled = enabled;
    return {};
}

std::shared_ptr<Constraint> Solver::constraint(const std::string& name) const {
    const ConstraintEntry* e = impl_->find_constraint(name);
    return e ? e->constraint : nullptr;
}

const SolverOptions& Solver::options() const noexcept { return impl_->options; }

Status Solver::set_options(SolverOptions options) {
    if (Status s = validate_options(*impl_->model, options); !s) return s;
    Result<std::unique_ptr<qp::Backend>> backend = make_backend(options);
    if (!backend) return backend.error();
    impl_->backend = std::move(*backend);
    impl_->options = std::move(options);
    ++impl_->options_revision;
    return {};
}

std::uint64_t Solver::options_revision() const noexcept { return impl_->options_revision; }

Status Solver::solve(const Eigen::Ref<const Eigen::VectorXd>& q, double dt, SolveResult& out) {
    return impl_->solve(q, nullptr, dt, out);
}

Status Solver::solve(const Eigen::Ref<const Eigen::VectorXd>& q, const Eigen::VectorXd* feedback, double dt,
                     SolveResult& out) {
    return impl_->solve(q, feedback, dt, out);
}

}  // namespace kstone
