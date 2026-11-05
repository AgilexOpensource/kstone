#include "kstone/qp/daqp.hpp"

#include <doctest/doctest.h>

#include <limits>

using namespace kstone;
using namespace kstone::qp;

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

// n variables, box [-1, 1], small Tikhonov objective.
Problem box_problem(Eigen::Index n, Eigen::Index m) {
    Problem p;
    p.resize(n, m);
    p.H = 1e-4 * Eigen::MatrixXd::Identity(n, n);
    p.g.setZero();
    p.lb.setConstant(-1.0);
    p.ub.setConstant(1.0);
    p.A.setZero();
    p.lo.setConstant(-kInf);
    p.hi.setConstant(kInf);
    return p;
}

// level 0: x0 + x1 + x2 <= 1.5   (hard)
// level 1: x0 = 0.8
// level 2: x1 = 0.9
// level 3: x0 + x1 = 2           (conflicts with 1 and 2, must not win)
Problem stacked() {
    Problem p = box_problem(3, 4);
    p.A.row(0) << 1, 1, 1;
    p.hi[0] = 1.5;
    p.A.row(1) << 1, 0, 0;
    p.lo[1] = p.hi[1] = 0.8;
    p.A.row(2) << 0, 1, 0;
    p.lo[2] = p.hi[2] = 0.9;
    p.A.row(3) << 1, 1, 0;
    p.lo[3] = p.hi[3] = 2.0;
    p.levels = {1, 2, 3, 4};
    return p;
}

Eigen::VectorXd solved(Backend& b, const Problem& p) {
    Solution s;
    const Status st = solve(b, p, s);
    REQUIRE_MESSAGE(st.ok(), (st.ok() ? "" : st.error().message));
    REQUIRE_MESSAGE(s.ok(), to_string(s.status), " exitflag ", s.exitflag);
    return s.x;
}
}  // namespace

TEST_CASE("validate rejects malformed problems") {
    Problem p = box_problem(2, 1);
    CHECK(validate(p).ok());
    p.lo[0] = 1.0;
    p.hi[0] = 0.0;
    CHECK_FALSE(validate(p).ok());
    p = box_problem(2, 2);
    p.levels = {1};
    CHECK_FALSE(validate(p).ok());
    p.levels = {1, 1, 2};
    CHECK_FALSE(validate(p).ok());
    p.levels = {0, 2};
    CHECK(validate(p).ok());
    p.g[0] = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(validate(p).ok());
    p = Problem{};
    CHECK_FALSE(validate(p).ok());
}

TEST_CASE("plain QP with box and equality") {
    Problem p = box_problem(2, 1);
    p.H.setIdentity();
    p.g << -3.0, 0.0;  // unconstrained optimum (3, 0)
    p.A.row(0) << 1, -1;
    p.lo[0] = p.hi[0] = 0.5;
    DaqpBackend daqp;
    const Eigen::VectorXd x = solved(daqp, p);
    CHECK(x[0] == doctest::Approx(1.0).epsilon(1e-6));
    CHECK(x[1] == doctest::Approx(0.5).epsilon(1e-6));
}

TEST_CASE("infeasible hard constraints are reported, not an error") {
    Problem p = box_problem(2, 1);
    p.A.row(0) << 1, 1;
    p.lo[0] = 3.0;  // box allows at most 2
    DaqpBackend daqp;
    Solution s;
    REQUIRE(daqp.solve(p, s).ok());
    CHECK(s.status == SolveStatus::infeasible);
    // the backend recovers on the next feasible problem
    p.lo[0] = 1.0;
    REQUIRE(daqp.solve(p, s).ok());
    CHECK(s.ok());
}

TEST_CASE("hierarchy respects priorities") {
    DaqpBackend daqp;
    const Eigen::VectorXd x = solved(daqp, stacked());
    CHECK(x[0] == doctest::Approx(0.8).epsilon(1e-4));
    CHECK(x[1] == doctest::Approx(0.9).epsilon(1e-4));
    CHECK(x[2] == doctest::Approx(-0.2).epsilon(1e-4));

    // the backend itself only takes plain problems
    Solution s;
    CHECK_FALSE(daqp.solve(stacked(), s).ok());
}

TEST_CASE("soft target beyond the box is clamped") {
    Problem p = box_problem(2, 2);
    p.A.row(0) << 1, 0;
    p.lo[0] = p.hi[0] = 3.0;
    p.A.row(1) << 0, 1;
    p.lo[1] = p.hi[1] = -0.25;
    p.levels = {0, 1, 2};
    DaqpBackend b;
    const Eigen::VectorXd x = solved(b, p);
    CHECK(x[0] == doctest::Approx(1.0).epsilon(1e-4));
    CHECK(x[1] == doctest::Approx(-0.25).epsilon(1e-4));
}

TEST_CASE("row scaling weights conflicting rows of one level") {
    // One level: k*x0 = k and x0 = -1. The level minimizes k^2 (x0 - 1)^2 + (x0 + 1)^2,
    // so x0 = (k^2 - 1) / (k^2 + 1): 0 for k = 1, 0.8 for k = 3.
    for (double k : {1.0, 3.0}) {
        CAPTURE(k);
        Problem p = box_problem(2, 2);
        p.A.row(0) << k, 0;
        p.lo[0] = p.hi[0] = k;
        p.A.row(1) << 1, 0;
        p.lo[1] = p.hi[1] = -1.0;
        p.levels = {0, 2};
        DaqpBackend b;
        const Eigen::VectorXd x = solved(b, p);
        CHECK(x[0] == doctest::Approx((k * k - 1.0) / (k * k + 1.0)).epsilon(1e-4));
    }
}

TEST_CASE("level damping shrinks a level's step from the levels above") {
    // Undamped x1 = 1. One damped level: grad of (x0+x1-1)^2 + (x1-1)^2 + |x|^2 gives (0.2, 0.6).
    // Two levels: level 2 slides along x0 + x1 = 1 from (0.5, 0.5), min (t-0.5)^2 + 2t^2 gives x1 = 2/3.
    Problem p = box_problem(2, 2);
    p.A.row(0) << 1, 1;
    p.lo[0] = p.hi[0] = 1.0;
    p.A.row(1) << 0, 1;
    p.lo[1] = p.hi[1] = 1.0;
    p.levels = {0, 1, 2};
    DaqpBackend b;
    CHECK(solved(b, p)[1] == doctest::Approx(1.0).epsilon(1e-4));
    p.levels = {0, 2};
    p.level_damping = {0.0, 1.0};
    const Eigen::VectorXd one = solved(b, p);
    CHECK(one[0] == doctest::Approx(0.2).epsilon(1e-4));
    CHECK(one[1] == doctest::Approx(0.6).epsilon(1e-4));
    p.levels = {0, 1, 2};
    p.level_damping = {0.0, 0.0, 1.0};
    const Eigen::VectorXd two = solved(b, p);
    CHECK(two[1] == doctest::Approx(2.0 / 3.0).epsilon(1e-4));
    p.level_damping = {0.0, 1.0};
    CHECK_FALSE(validate(p).ok());
    p.level_damping = {0.0, 0.0, -1.0};
    CHECK_FALSE(validate(p).ok());
}

TEST_CASE("hierarchical hard infeasibility is detected") {
    Problem p = stacked();
    p.lo[0] = 5.0;  // x0 + x1 + x2 >= 5 is outside the box (max 3)
    p.hi[0] = kInf;
    DaqpBackend b;
    Solution s;
    REQUIRE(solve(b, p, s).ok());
    CHECK(s.status == SolveStatus::infeasible);
}

TEST_CASE("a zero row that becomes non-zero on a warm solve is enforced") {
    // Fixed-size constraints (e.g. collision rows) leave unused rows all zero; DAQP then ignores
    // them, and must stop ignoring them once they are filled in.
    DaqpBackend daqp;
    Problem p = box_problem(1, 2);
    p.H.setIdentity();
    p.g << -1.0;  // optimum x = 1 (the box edge)
    REQUIRE(solved(daqp, p)[0] == doctest::Approx(1.0));
    p.A(0, 0) = -1.0;
    p.lo[0] = 0.5;  // -x >= 0.5
    const Eigen::VectorXd x = solved(daqp, p);
    CHECK(x[0] == doctest::Approx(-0.5).epsilon(1e-9));
    CHECK(daqp.rebuilds() == 1);
}

// DAQP that reports infeasible when a two-sided row range is narrower than `precision`,
// standing in for near-dependent settled rows it cannot resolve.
class CoarseBackend final : public Backend {
  public:
    explicit CoarseBackend(double precision) : precision_(precision) {}
    const char* name() const noexcept override { return "coarse"; }
    Status solve(const Problem& p, Solution& out) override {
        for (Eigen::Index i = 0; i < p.rows(); ++i) {
            const double width = p.hi[i] - p.lo[i];
            if (width > 0.0 && width < precision_) {
                out.status = SolveStatus::infeasible;
                out.exitflag = -1;
                return {};
            }
        }
        return daqp_.solve(p, out);
    }
    void reset() noexcept override { daqp_.reset(); }

  private:
    double precision_;
    DaqpBackend daqp_;
};

TEST_CASE("a level failing on near-dependent settled rows is retried once with wider rows") {
    Problem single = box_problem(2, 1);
    single.A.row(0) << 1, 0;
    single.lo[0] = single.hi[0] = 0.8;
    single.levels = {0, 1};
    for (const Problem& p : {stacked(), single}) {
        CoarseBackend retried(1e-8);
        Solution s;
        REQUIRE(solve(retried, p, s).ok());
        CHECK(s.status == SolveStatus::optimal);
        CHECK(s.x[0] == doctest::Approx(0.8).epsilon(1e-4));
        if (p.levels.size() == 4) CHECK(s.x[1] == doctest::Approx(0.9).epsilon(1e-4));

        // Still failing after the retry: the last good x is kept as inexact.
        CoarseBackend coarse(1e-3);
        REQUIRE(solve(coarse, p, s).ok());
        CHECK(s.status == SolveStatus::inexact);
        CHECK(s.x[0] == doctest::Approx(0.8).epsilon(1e-4));
    }
}

TEST_CASE("repeated solves reuse the workspace") {
    DaqpBackend daqp;
    Problem p = box_problem(2, 2);
    p.H.setIdentity();
    p.A.row(0) << 1, 0;
    p.A.row(1) << 0, 1;
    for (int tick = 0; tick < 20; ++tick) {
        p.lo[0] = p.hi[0] = 0.8 - 0.01 * tick;
        p.lo[1] = p.hi[1] = 0.1 + 0.02 * tick;
        const Eigen::VectorXd x = solved(daqp, p);
        CHECK(x[0] == doctest::Approx(p.lo[0]).epsilon(1e-6));
        CHECK(x[1] == doctest::Approx(p.lo[1]).epsilon(1e-6));
    }
    CHECK(daqp.rebuilds() == 1);

    // a different size is a new layout
    p = box_problem(3, 2);
    Solution s;
    REQUIRE(daqp.solve(p, s).ok());
    CHECK(daqp.rebuilds() == 2);
}
