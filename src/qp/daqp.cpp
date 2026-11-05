#include "kstone/qp/daqp.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

extern "C" {
#include "api.h"
#include "auxiliary.h"
#include "constants.h"
#include "utils.h"
}

namespace kstone::qp {

namespace {

using RowMajor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

constexpr int kSenseEquality = DAQP_ACTIVE | DAQP_IMMUTABLE;

double clamp_inf(double v) { return std::clamp(v, -DAQP_INF, DAQP_INF); }

SolveStatus map_exitflag(int flag) {
    switch (flag) {
        case DAQP_EXIT_OPTIMAL:
        case DAQP_EXIT_SOFT_OPTIMAL:
            return SolveStatus::optimal;
        case DAQP_EXIT_OPTIMAL_INEXACT:
        case DAQP_EXIT_FEASIBLE:
            return SolveStatus::inexact;
        case DAQP_EXIT_INFEASIBLE:
            return SolveStatus::infeasible;
        default:
            return SolveStatus::failed;
    }
}

// One DAQP workspace for a fixed (n, m) layout.
struct Slot {
    long last_use = 0;

    bool built = false;
    Eigen::Index n = 0;
    Eigen::Index m = 0;

    // Buffers referenced by DAQP; never resized while built.
    std::vector<double> H, f, A, bu, bl;
    std::vector<int> sense;
    std::vector<double> x, lam;
    DAQPProblem qp{};
    DAQPWorkspace work{};

    Slot() = default;
    Slot(const Slot&) = delete;
    Slot& operator=(const Slot&) = delete;
    ~Slot() { release(); }

    void release() noexcept {
        if (built) {
            free_daqp_ldp(&work);
            free_daqp_workspace(&work);
        }
        work = DAQPWorkspace{};
        built = false;
    }

    bool same_layout(const Problem& p) const { return built && p.variables() == n && p.rows() == m; }

    int fill(const Problem& p) {
        int mask = DAQP_UPDATE_v | DAQP_UPDATE_d;
        Eigen::Map<RowMajor> hessian(H.data(), n, n);
        const auto symmetric = 0.5 * (p.H + p.H.transpose());
        // Compare exactly: tiny matrix changes can matter for near-dependent rows.
        if (!(hessian.array() == symmetric.array()).all()) {
            mask |= DAQP_UPDATE_Rinv | DAQP_UPDATE_M;
        }
        hessian = symmetric;
        Eigen::Map<Eigen::VectorXd>(f.data(), n) = p.g;
        if (m > 0) {
            Eigen::Map<RowMajor> constraints(A.data(), m, n);
            if (!(constraints.array() == p.A.array()).all()) {
                mask |= DAQP_UPDATE_M;
            }
            constraints = p.A;
        }
        auto put = [&](std::size_t k, double lo, double hi) {
            bl[k] = clamp_inf(lo);
            bu[k] = clamp_inf(hi);
            const int s = (lo == hi && std::isfinite(lo)) ? kSenseEquality : 0;
            if (sense[k] != s) {
                sense[k] = s;
                mask |= DAQP_UPDATE_sense;
            }
        };
        for (Eigen::Index i = 0; i < n; ++i) {
            put(static_cast<std::size_t>(i), p.lb[i], p.ub[i]);
        }
        for (Eigen::Index i = 0; i < m; ++i) {
            put(static_cast<std::size_t>(n + i), p.lo[i], p.hi[i]);
        }
        return mask;
    }

    int build(const Problem& p, const DaqpSettings& settings) {
        release();
        n = p.variables();
        m = p.rows();
        const auto un = static_cast<std::size_t>(n);
        const auto total = static_cast<std::size_t>(n + m);
        const auto rows = static_cast<std::size_t>(m);
        H.assign(un * un, 0.0);
        f.assign(un, 0.0);
        A.assign(std::max<std::size_t>(rows * un, 1), 0.0);
        bu.assign(total, 0.0);
        bl.assign(total, 0.0);
        sense.assign(total, -1);
        x.assign(un, 0.0);
        lam.assign(total, 0.0);
        fill(p);

        qp = DAQPProblem{};
        qp.n = static_cast<int>(n);
        qp.m = static_cast<int>(total);
        qp.ms = static_cast<int>(n);
        qp.H = H.data();
        qp.f = f.data();
        qp.A = A.data();
        qp.bupper = bu.data();
        qp.blower = bl.data();
        qp.sense = sense.data();
        qp.break_points = nullptr;
        qp.nh = 0;
        qp.problem_type = 0;

        work = DAQPWorkspace{};
        const int flag = setup_daqp(&qp, &work, nullptr);
        if (flag < 0) {
            work = DAQPWorkspace{};  // setup already freed everything
            return flag;
        }
        built = true;
        work.settings->eq_reduction = DAQP_EQ_REDUCTION_OFF;
        work.settings->iter_limit = settings.iteration_limit;
        work.settings->primal_tol = settings.primal_tol;
        return 0;
    }
};

}  // namespace

struct DaqpBackend::Impl {
    DaqpSettings settings;
    long rebuilds = 0;
    long clock = 0;
    std::vector<std::unique_ptr<Slot>> slots;

    explicit Impl(DaqpSettings s) : settings(s) {}

    void release() noexcept {
        for (auto& slot : slots) slot->release();
    }

    // The slot holding this layout, else a free or the least recently used one.
    Slot& select(const Problem& p) {
        Slot* pick = nullptr;
        for (auto& slot : slots) {
            if (slot->same_layout(p)) {
                pick = slot.get();
                break;
            }
        }
        if (pick == nullptr) {
            for (auto& slot : slots) {
                if (!slot->built) {
                    pick = slot.get();
                    break;
                }
            }
        }
        if (pick == nullptr) {
            if (slots.size() < static_cast<std::size_t>(std::max(settings.cached_layouts, 1))) {
                slots.push_back(std::make_unique<Slot>());
                pick = slots.back().get();
            } else {
                pick = slots.front().get();
                for (auto& slot : slots)
                    if (slot->last_use < pick->last_use) pick = slot.get();
            }
        }
        pick->last_use = ++clock;
        return *pick;
    }
};

DaqpBackend::DaqpBackend(DaqpSettings settings) : impl_(std::make_unique<Impl>(settings)) {}
DaqpBackend::~DaqpBackend() = default;
DaqpBackend::DaqpBackend(DaqpBackend&&) noexcept = default;
DaqpBackend& DaqpBackend::operator=(DaqpBackend&&) noexcept = default;

const DaqpSettings& DaqpBackend::settings() const noexcept { return impl_->settings; }
long DaqpBackend::rebuilds() const noexcept { return impl_->rebuilds; }
void DaqpBackend::reset() noexcept { impl_->release(); }

Status DaqpBackend::solve(const Problem& problem, Solution& out) {
    if (Status s = validate(problem); !s.ok()) {
        return s;
    }
    if (problem.hierarchical()) {
        return make_error(ErrorCode::unsupported, "daqp: hierarchical problems go through qp::solve()");
    }
    Slot& d = impl_->select(problem);

    int flag = 0;
    bool warm = d.same_layout(problem);
    if (warm) {
        const int mask = d.fill(problem);
        flag = daqp_update_ldp(mask, &d.work, &d.qp);
        if (flag < 0) {
            ++impl_->rebuilds;
            flag = d.build(problem, impl_->settings);
            warm = false;
        }
    } else {
        ++impl_->rebuilds;
        flag = d.build(problem, impl_->settings);
    }

    out.x.resize(d.n);
    out.iterations = 0;
    if (flag < 0) {
        d.release();
        out.exitflag = flag;
        out.status = map_exitflag(flag);
        out.x.setZero();
        return {};
    }
    DAQPResult res{};
    res.x = d.x.data();
    res.lam = d.lam.data();
    daqp_solve(&res, &d.work);
    if (res.exitflag < 0 && warm) {
        // A working set inherited from a previous problem can make DAQP report infeasible on a
        // feasible, near-degenerate problem. Retry once from the equalities alone.
        daqp_deactivate_constraints(&d.work);
        daqp_activate_constraints(&d.work);
        const int iterations = res.iter;
        res = DAQPResult{};
        res.x = d.x.data();
        res.lam = d.lam.data();
        daqp_solve(&res, &d.work);
        res.iter += iterations;
    }
    out.exitflag = res.exitflag;
    out.iterations = res.iter;
    out.status = map_exitflag(res.exitflag);
    out.x = Eigen::Map<const Eigen::VectorXd>(d.x.data(), d.n);
    if (res.exitflag < 0) {
        // Do not warm start from a broken working set; the factorization itself stays valid.
        daqp_deactivate_constraints(&d.work);
        daqp_activate_constraints(&d.work);
    }
    return {};
}

}  // namespace kstone::qp
