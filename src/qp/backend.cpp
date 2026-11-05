#include "kstone/qp/backend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace kstone::qp {

Backend::~Backend() = default;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kProx = 1e-4;  // proximal weight on x, relative to the largest soft row norm^2
constexpr int kProxSteps = 50;
constexpr double kProxTol = 1e-8;   // slack change that ends the proximal iteration, above DAQP precision
constexpr double kFixTol = 1e-9;    // relative widening of a settled level, at the solver tolerance
constexpr double kRetryTol = 1e-6;  // widening used once a stage fails on near-dependent settled rows

SolveStatus worse(SolveStatus a, SolveStatus b) { return static_cast<int>(a) > static_cast<int>(b) ? a : b; }

bool bounded(const Problem& p, Eigen::Index i) { return std::isfinite(p.lo[i]) || std::isfinite(p.hi[i]); }

}  // namespace

// Per soft level, plain QPs over (x, s) with one slack per bounded row:
//   min ||s||^2 + mu ||x - x_start||^2 + rho ||x - x_prev||^2  s.t. box, level 0, settled levels, lo <= a x - s <= hi,
// repeated (proximal point) until s stops changing. A tiny weight on x instead would leave
// the dual too ill-conditioned. The optimal slack is unique, so the level is then settled to
// the bounds reached. A last QP minimizes the base objective over everything settled.
Status solve_lexicographic(Backend& backend, const Problem& p, Solution& out, LexicographicWorkspace& ws) {
    if (!p.hierarchical()) {
        return backend.solve(p, out);
    }
    if (Status s = validate(p); !s.ok()) {
        return s;
    }
    const Eigen::Index n = p.variables();
    const Eigen::Index h = p.levels.front();

    // Rows constraining x alone: level 0, then settled levels with their reached bounds.
    std::vector<Eigen::Index>& fixed = ws.fixed;
    std::vector<Eigen::Index>& soft = ws.soft;
    Eigen::VectorXd& fixed_lo = ws.fixed_lo;
    Eigen::VectorXd& fixed_hi = ws.fixed_hi;
    fixed.clear();
    fixed_lo.resize(p.rows());
    fixed_hi.resize(p.rows());
    ws.slack.resize(p.rows());
    for (Eigen::Index i = 0; i < h; ++i) {
        if (bounded(p, i)) {
            fixed.push_back(i);
            fixed_lo[i] = p.lo[i];
            fixed_hi[i] = p.hi[i];
        }
    }

    out.status = SolveStatus::optimal;
    out.iterations = 0;
    out.exitflag = 0;
    out.x.setZero(n);

    // Each stage owns its sub-problem so repeated calls keep every size (no reallocation).
    std::size_t used = 0;
    Problem* sub = nullptr;
    Solution* step = nullptr;
    // Builds the next stage over x and `soft.size()` slacks (none: the base objective).
    auto build = [&](bool with_soft, double rho, double mu) {
        if (ws.stages.size() <= used) ws.stages.emplace_back();
        sub = &ws.stages[used].problem;
        step = &ws.stages[used].solution;
        ++used;
        const auto ns = with_soft ? static_cast<Eigen::Index>(soft.size()) : Eigen::Index{0};
        const auto nf = static_cast<Eigen::Index>(fixed.size());
        sub->resize(n + ns, nf + ns);
        sub->H.setZero();
        sub->g.setZero();
        if (ns > 0) {
            sub->H.topLeftCorner(n, n).diagonal().setConstant(2.0 * (rho + mu));
            sub->H.bottomRightCorner(ns, ns).diagonal().setConstant(2.0);
        } else {
            sub->H = p.H;
            sub->g = p.g;
        }
        sub->lb.head(n) = p.lb;
        sub->ub.head(n) = p.ub;
        sub->lb.tail(ns).setConstant(-kInf);
        sub->ub.tail(ns).setConstant(kInf);
        sub->A.setZero();
        for (Eigen::Index r = 0; r < nf; ++r) {
            const Eigen::Index i = fixed[static_cast<std::size_t>(r)];
            sub->A.row(r).head(n) = p.A.row(i);
            sub->lo[r] = fixed_lo[i];
            sub->hi[r] = fixed_hi[i];
        }
        for (Eigen::Index j = 0; j < ns; ++j) {
            const Eigen::Index i = soft[static_cast<std::size_t>(j)];
            sub->A.row(nf + j).head(n) = p.A.row(i);
            sub->A(nf + j, n + j) = -1.0;
            sub->lo[nf + j] = p.lo[i];
            sub->hi[nf + j] = p.hi[i];
        }
        sub->levels.clear();
    };

    // Returns false on an error or a failed stage, leaving out.x/status/exitflag untouched.
    auto run = [&](Status& error) {
        if (error = backend.solve(*sub, *step); !error.ok()) {
            return false;
        }
        out.iterations += step->iterations;
        if (!step->ok()) {
            return false;
        }
        out.exitflag = step->exitflag;
        out.status = worse(out.status, step->status);
        out.x = step->x.head(n);
        return true;
    };
    // After a failed stage: a stage has succeeded, every settled row holds at out.x, so the
    // failure is numerical (near-degenerate settled ranges); keep that x as inexact.
    auto settle = [&](bool usable) {
        if (usable) {
            out.status = worse(out.status, SolveStatus::inexact);
        } else {
            out.status = step->status;
            out.exitflag = step->exitflag;
        }
    };

    // Widens the settled soft rows once so the failed stage can be retried; later levels settle as wide.
    bool widened = false;
    auto widen = [&]() {
        if (widened) return false;
        bool any = false;
        for (Eigen::Index i : fixed) {
            if (i < h) continue;
            any = true;
            if (std::isfinite(fixed_lo[i])) fixed_lo[i] -= kRetryTol * (1.0 + std::abs(fixed_lo[i]));
            if (std::isfinite(fixed_hi[i])) fixed_hi[i] += kRetryTol * (1.0 + std::abs(fixed_hi[i]));
        }
        widened = any;
        return any;
    };

    Status error;
    bool staged = false;  // a slack stage succeeded, so out.x satisfies every settled row
    for (std::size_t k = 1; k < p.levels.size(); ++k) {
        soft.clear();
        for (Eigen::Index i = p.levels[k - 1]; i < p.levels[k]; ++i) {
            if (bounded(p, i)) {
                soft.push_back(i);
            }
        }
        if (soft.empty()) {
            continue;
        }
        double row_scale = 0.0;
        for (Eigen::Index i : soft) row_scale = std::max(row_scale, p.A.row(i).squaredNorm());
        const double rho = kProx * std::max(row_scale, 1e-12);
        const double mu = p.level_damping.empty() ? 0.0 : p.level_damping[k];
        const auto ns = static_cast<Eigen::Index>(soft.size());
        auto slack = ws.slack.head(ns);
        slack.setZero();
        ws.start = out.x;
        bool converged = false;
        build(true, rho, mu);
        for (int it = 0; it < kProxSteps && !converged; ++it) {
            sub->g.head(n) = -2.0 * rho * out.x - 2.0 * mu * ws.start;
            if (!run(error)) {
                if (!error.ok()) return error;
                if (it == 0 && widen()) {
                    --used;
                    build(true, rho, mu);
                    --it;
                    continue;
                }
                settle(staged || it > 0);
                return error;
            }
            const auto s_new = step->x.tail(ns);
            double change = 0.0;
            for (Eigen::Index j = 0; j < ns; ++j) change = std::max(change, std::abs(s_new[j] - slack[j]));
            converged = it > 0 && change <= kProxTol * (1.0 + slack.norm());
            slack = s_new;
        }
        if (!converged) {
            out.status = worse(out.status, SolveStatus::inexact);
        }
        staged = true;
        const double tol = widened ? kRetryTol : kFixTol;
        for (Eigen::Index i : soft) {
            const double ax = p.A.row(i).dot(out.x);
            const double lo = std::min(p.lo[i], ax), hi = std::max(p.hi[i], ax);
            fixed.push_back(i);
            fixed_lo[i] = std::isfinite(lo) ? lo - tol * (1.0 + std::abs(lo)) : lo;
            fixed_hi[i] = std::isfinite(hi) ? hi + tol * (1.0 + std::abs(hi)) : hi;
        }
    }
    build(false, 0.0, 0.0);
    if (!run(error) && error.ok()) {
        if (widen()) {
            --used;
            build(false, 0.0, 0.0);
            if (run(error) || !error.ok()) return error;
        }
        settle(staged);
    }
    return error;
}

Status solve_lexicographic(Backend& backend, const Problem& problem, Solution& out) {
    LexicographicWorkspace workspace;
    return solve_lexicographic(backend, problem, out, workspace);
}

Status solve(Backend& backend, const Problem& problem, Solution& out, LexicographicWorkspace& workspace) {
    return solve_lexicographic(backend, problem, out, workspace);
}

Status solve(Backend& backend, const Problem& problem, Solution& out) {
    return solve_lexicographic(backend, problem, out);
}

}  // namespace kstone::qp
