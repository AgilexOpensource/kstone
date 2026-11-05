// Randomized lexicographic-optimality checks of the hierarchical fallback.
#include "kstone/qp/daqp.hpp"

#include <doctest/doctest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

using namespace kstone;
using namespace kstone::qp;

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

enum class RowKind { equality, range, lower, upper };

// Box [-1, 1]; level 0 holds hard rows feasible at a random interior point; soft levels mix
// equalities with (optionally) ranges and one-sided rows whose targets are often unreachable.
Problem random_problem(std::mt19937& rng, bool with_inequalities) {
    std::uniform_int_distribution<int> n_dist(2, 6), lvl_dist(1, 4), rows_dist(1, 3), hard_dist(0, 2);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    std::uniform_int_distribution<int> kind_dist(0, with_inequalities ? 3 : 0);
    const Eigen::Index n = n_dist(rng);
    const int soft_levels = lvl_dist(rng);
    const int hard = hard_dist(rng);
    std::vector<int> rows_per_level;
    Eigen::Index m = hard;
    for (int k = 0; k < soft_levels; ++k) {
        rows_per_level.push_back(rows_dist(rng));
        m += rows_per_level.back();
    }

    Problem p;
    p.resize(n, m);
    const Eigen::MatrixXd R = Eigen::MatrixXd::NullaryExpr(n, n, [&] { return u(rng); });
    p.H = R.transpose() * R + 0.1 * Eigen::MatrixXd::Identity(n, n);
    p.g = Eigen::VectorXd::NullaryExpr(n, [&] { return u(rng); });
    p.lb.setConstant(-1.0);
    p.ub.setConstant(1.0);
    p.A = Eigen::MatrixXd::NullaryExpr(m, n, [&] { return u(rng); });

    const Eigen::VectorXd interior = 0.5 * Eigen::VectorXd::NullaryExpr(n, [&] { return u(rng); });
    for (Eigen::Index i = 0; i < hard; ++i) {
        const double ax = p.A.row(i).dot(interior);
        p.lo[i] = ax - 0.3 - 0.2 * std::abs(u(rng));
        p.hi[i] = ax + 0.3 + 0.2 * std::abs(u(rng));
    }
    p.levels = {hard};
    Eigen::Index r = hard;
    for (int k = 0; k < soft_levels; ++k) {
        for (int j = 0; j < rows_per_level[static_cast<std::size_t>(k)]; ++j, ++r) {
            const double target = 2.0 * u(rng);  // often outside what the box allows
            switch (static_cast<RowKind>(kind_dist(rng))) {
                case RowKind::equality:
                    p.lo[r] = p.hi[r] = target;
                    break;
                case RowKind::range:
                    p.lo[r] = target;
                    p.hi[r] = target + 0.5 * std::abs(u(rng));
                    break;
                case RowKind::lower:
                    p.lo[r] = target;
                    p.hi[r] = kInf;
                    break;
                case RowKind::upper:
                    p.lo[r] = -kInf;
                    p.hi[r] = target;
                    break;
            }
        }
        p.levels.push_back(r);
    }
    return p;
}

double violation(const Problem& p, Eigen::Index i, const Eigen::VectorXd& x) {
    const double ax = p.A.row(i).dot(x);
    if (ax < p.lo[i]) return p.lo[i] - ax;
    if (ax > p.hi[i]) return ax - p.hi[i];
    return 0.0;
}

// Violation norm of each soft level.
std::vector<double> level_residual(const Problem& p, const Eigen::VectorXd& x) {
    std::vector<double> out;
    for (std::size_t k = 1; k < p.levels.size(); ++k) {
        double sum = 0.0;
        for (Eigen::Index i = p.levels[k - 1]; i < p.levels[k]; ++i) sum += std::pow(violation(p, i, x), 2);
        out.push_back(std::sqrt(sum));
    }
    return out;
}

bool hard_ok(const Problem& p, const Eigen::VectorXd& x, double tol) {
    for (Eigen::Index i = 0; i < x.size(); ++i)
        if (x[i] < p.lb[i] - tol || x[i] > p.ub[i] + tol) return false;
    for (Eigen::Index i = 0; i < p.levels.front(); ++i)
        if (violation(p, i, x) > tol) return false;
    return true;
}

// True if candidate c beats the solution x lexicographically: no level worse beyond `slack`
// before the first level where c is better by more than `tol`.
bool beats(const std::vector<double>& c, const std::vector<double>& x, double slack, double tol) {
    for (std::size_t k = 0; k < c.size(); ++k) {
        if (c[k] < x[k] - tol * (1.0 + x[k])) return true;
        if (c[k] > x[k] + slack) return false;
    }
    return false;
}

struct Tally {
    int optimal = 0;
    int beaten = 0;  // a sampled hard-feasible point was lexicographically better
};

// No native reference: probe the fallback's answer with hard-feasible points on segments
// toward random box points and with small perturbations. By convexity a better point would
// show up along such segments if the answer were not lexicographically optimal.
Tally probe(bool with_inequalities, int trials, unsigned seed) {
    std::mt19937 rng(seed), sampler(seed + 1000u);  // separate streams: problems do not depend on probing
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    Tally tally;
    for (int t = 0; t < trials; ++t) {
        const Problem p = random_problem(rng, with_inequalities);
        CAPTURE(t);
        DaqpBackend daqp;
        Solution s;
        REQUIRE(solve(daqp, p, s).ok());
        // Level 0 is feasible by construction, so the fallback always returns a usable point.
        CHECK(s.ok());
        CHECK(hard_ok(p, s.x, 1e-6));
        tally.optimal += s.status == SolveStatus::optimal ? 1 : 0;
        const std::vector<double> ref = level_residual(p, s.x);
        const Eigen::Index n = p.variables();
        bool beaten = false;
        for (int j = 0; j < 200 && !beaten; ++j) {
            const Eigen::VectorXd y = Eigen::VectorXd::NullaryExpr(n, [&] { return u(sampler); });
            for (double step : {1.0, 1e-1, 1e-2, 1e-3}) {
                const Eigen::VectorXd c = s.x + step * (y - s.x);
                if (!hard_ok(p, c, 0.0)) continue;
                if (beats(level_residual(p, c), ref, 1e-9, 1e-5)) {
                    beaten = true;
                    MESSAGE("beaten at t=" << t << ", step " << step);
                    break;
                }
            }
        }
        tally.beaten += beaten ? 1 : 0;
    }
    MESSAGE("optimal " << tally.optimal << "/" << trials << ", beaten " << tally.beaten);
    return tally;
}
}  // namespace

TEST_CASE("random hierarchies are lexicographically optimal, equality soft rows") {
    const Tally tally = probe(false, 300, 1u);
    CHECK(tally.beaten == 0);
}

TEST_CASE("random hierarchies are lexicographically optimal, inequality soft rows") {
    const Tally tally = probe(true, 300, 2u);
    CHECK(tally.beaten == 0);
}

// Near-degenerate settled ranges occasionally make a later stage numerically infeasible.
TEST_CASE("random hierarchies always return a hard-feasible point") {
    std::mt19937 rng(3u);
    int failed = 0;
    for (int t = 0; t < 10000; ++t) {
        const Problem p = random_problem(rng, true);
        DaqpBackend daqp;
        Solution s;
        REQUIRE(solve(daqp, p, s).ok());
        if (!s.ok() || !hard_ok(p, s.x, 1e-6)) {
            ++failed;
            MESSAGE("t=" << t << " status " << static_cast<int>(s.status) << " exitflag " << s.exitflag);
        }
    }
    CHECK(failed == 0);
}
