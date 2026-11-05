#pragma once

#include "kstone/qp/problem.hpp"

#include <vector>

namespace kstone::qp {

// A persistent QP solver. Implementations may keep factorizations and
// warm-start data between calls with the same structure (n, m, levels).
// Solve returns an error Status only for invalid input or internal faults;
// infeasibility and iteration limits are reported in Solution::status.
class KSTONE_EXPORT Backend {
  public:
    virtual ~Backend();

    virtual const char* name() const noexcept = 0;
    // Solves a plain problem; hierarchical problems go through qp::solve().
    virtual Status solve(const Problem& problem, Solution& out) = 0;
    // Drop cached state; the next solve rebuilds from scratch.
    virtual void reset() noexcept = 0;
};

// Scratch for solve_lexicographic(). Keeping one alive across calls with the
// same structure makes repeated solves allocation free (each stage keeps its
// own sub-problem, so its size never changes between calls).
struct LexicographicWorkspace {
    struct Stage {
        Problem problem;
        Solution solution;
    };
    std::vector<Stage> stages;
    std::vector<Eigen::Index> fixed, soft;
    Eigen::VectorXd fixed_lo, fixed_hi, slack, start;
};

// Solve a hierarchical problem through a sequence of plain QPs on `backend`
// (one slack QP per soft level minimizing the squared violation, then the base
// objective). Soft rows may be equalities, ranges or one-sided. Plain problems
// are forwarded unchanged. The overload without a workspace allocates one per call.
KSTONE_EXPORT Status solve_lexicographic(Backend& backend, const Problem& problem, Solution& out,
                                         LexicographicWorkspace& workspace);
KSTONE_EXPORT Status solve_lexicographic(Backend& backend, const Problem& problem, Solution& out);

// Plain problems go to the backend, hierarchical ones through solve_lexicographic().
KSTONE_EXPORT Status solve(Backend& backend, const Problem& problem, Solution& out,
                           LexicographicWorkspace& workspace);
KSTONE_EXPORT Status solve(Backend& backend, const Problem& problem, Solution& out);

}  // namespace kstone::qp
