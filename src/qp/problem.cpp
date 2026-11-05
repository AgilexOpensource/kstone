#include "kstone/qp/problem.hpp"

#include "kstone/qp/backend.hpp"

#include <cmath>
#include <string>

namespace kstone::qp {

void Problem::resize(Eigen::Index n, Eigen::Index m) {
    H.resize(n, n);
    g.resize(n);
    lb.resize(n);
    ub.resize(n);
    A.resize(m, n);
    lo.resize(m);
    hi.resize(m);
}

namespace {

bool no_nan_order(const Eigen::VectorXd& lo, const Eigen::VectorXd& hi) {
    for (Eigen::Index i = 0; i < lo.size(); ++i) {
        if (std::isnan(lo[i]) || std::isnan(hi[i]) || lo[i] > hi[i]) {
            return false;
        }
    }
    return true;
}

}  // namespace

Status validate(const Problem& p) {
    const Eigen::Index n = p.H.rows();
    const Eigen::Index m = p.A.rows();
    if (n == 0 || p.H.cols() != n || p.g.size() != n || p.lb.size() != n || p.ub.size() != n) {
        return make_error(ErrorCode::invalid_argument, "H, g, lb, ub must all have n > 0 variables");
    }
    if (p.A.cols() != n || p.lo.size() != m || p.hi.size() != m) {
        return make_error(ErrorCode::invalid_argument, "qp: A must be m x n with lo/hi of size m");
    }
    if (!p.H.allFinite() || !p.g.allFinite() || !p.A.allFinite()) {
        return make_error(ErrorCode::invalid_argument, "qp: H, g and A must be finite");
    }
    if (!no_nan_order(p.lb, p.ub) || !no_nan_order(p.lo, p.hi)) {
        return make_error(ErrorCode::invalid_argument, "qp: bounds must be non-NaN with lower <= upper");
    }
    if (!p.levels.empty()) {
        Eigen::Index prev = 0;
        for (std::size_t k = 0; k < p.levels.size(); ++k) {
            const Eigen::Index end = p.levels[k];
            if (end < prev || (k > 0 && end == prev)) {
                return make_error(ErrorCode::invalid_argument,
                                  "qp: levels must be increasing (only level 0 may be empty)");
            }
            prev = end;
        }
        if (prev != m) {
            return make_error(ErrorCode::invalid_argument, "qp: levels.back() must equal A.rows()");
        }
    }
    if (!p.level_damping.empty()) {
        if (p.level_damping.size() != p.levels.size())
            return make_error(ErrorCode::invalid_argument, "qp: level_damping must have one entry per level");
        for (double mu : p.level_damping)
            if (!std::isfinite(mu) || mu < 0)
                return make_error(ErrorCode::invalid_argument, "qp: level_damping must be finite and >= 0");
    }
    return {};
}

const char* to_string(SolveStatus status) noexcept {
    switch (status) {
        case SolveStatus::optimal:
            return "optimal";
        case SolveStatus::inexact:
            return "inexact";
        case SolveStatus::infeasible:
            return "infeasible";
        case SolveStatus::failed:
            return "failed";
    }
    return "unknown";
}

}  // namespace kstone::qp
