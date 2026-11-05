#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"

#include <Eigen/Core>

#include <vector>

namespace kstone::qp {

// Dense convex QP over x in R^n:
//   minimize   0.5 x'Hx + g'x
//   subject to lb <= x <= ub, lo <= A x <= hi
// Infinite bounds are allowed; rows with lo == hi are equalities.
//
// Hierarchy: when `levels` is non-empty the rows of A are split into
// consecutive priority levels, `levels[k]` being the exclusive end row of
// level k. Level 0 (together with the box) is hard; every later level is
// soft and solved lexicographically (higher levels first), H/g only pick the
// solution among the remaining freedom. `levels.back()` must equal A.rows().
// `level_damping` (empty, or one entry per level, entry 0 unused) adds
// mu_k ||x - x_k||^2 to soft level k, x_k being the solution of the levels above.
struct KSTONE_EXPORT Problem {
    Eigen::MatrixXd H;
    Eigen::VectorXd g;
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;
    Eigen::MatrixXd A;
    Eigen::VectorXd lo;
    Eigen::VectorXd hi;
    std::vector<Eigen::Index> levels;
    std::vector<double> level_damping;

    Eigen::Index variables() const noexcept { return H.rows(); }
    Eigen::Index rows() const noexcept { return A.rows(); }
    bool hierarchical() const noexcept { return !levels.empty(); }

    // Size every member for n variables and m rows (values are unspecified).
    void resize(Eigen::Index n, Eigen::Index m);
};

// Checks sizes, finiteness of H/g/A, ordering of bounds and levels.
KSTONE_EXPORT Status validate(const Problem& problem);

enum class SolveStatus {
    optimal,     // solved to tolerance
    inexact,     // solved, tolerance not fully met
    infeasible,  // hard part (box + level 0) infeasible
    failed,      // iteration limit, numerical trouble, ...
};

KSTONE_EXPORT const char* to_string(SolveStatus status) noexcept;

struct Solution {
    SolveStatus status = SolveStatus::failed;
    int exitflag = 0;  // backend specific raw code
    int iterations = 0;
    Eigen::VectorXd x;

    bool ok() const noexcept { return status == SolveStatus::optimal || status == SolveStatus::inexact; }
};

}  // namespace kstone::qp
