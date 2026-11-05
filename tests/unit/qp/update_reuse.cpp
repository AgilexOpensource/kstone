#include "kstone/qp/daqp.hpp"

#include <doctest/doctest.h>

#include <cmath>

using namespace kstone::qp;

TEST_CASE("partial updates match cold solves across matrix and vector mutations") {
    DaqpBackend warm;
    Problem p;
    p.resize(4, 3);
    p.H.setIdentity();
    p.A.setZero();
    p.A.leftCols(3).setIdentity();
    p.lo.setConstant(-0.5);
    p.hi.setConstant(0.5);
    p.lb.setConstant(-2.0);
    p.ub.setConstant(2.0);
    p.g.setZero();
    for (int tick = 0; tick < 400; ++tick) {
        INFO("tick=" << tick);
        const int phase = tick % 10;
        p.g[0] = std::sin(static_cast<double>(tick) * 0.13);
        p.g[3] = std::cos(static_cast<double>(tick) * 0.17);
        if (phase == 1) p.H(0, 0) = 0.2 + 0.01 * static_cast<double>(tick);
        if (phase == 2) p.A(1, 0) = 0.1 * std::sin(static_cast<double>(tick));
        if (phase == 3) p.lo[1] = p.hi[1] = 0.0;
        if (phase == 4) {
            p.lo[1] = -0.5;
            p.hi[1] = 0.5;
        }
        if (phase == 5) p.A.row(2).setZero();
        if (phase == 6) p.A(2, 2) = 1.0;
        if (phase == 7) {
            p.H.setIdentity();
            p.H(3, 3) = 1e-8;
        }
        if (phase == 8) {
            p.A.row(2).setZero();
            p.hi[2] = -0.2;
        }
        if (phase == 9) {
            p.A(2, 2) = 1.0;
            p.hi[2] = 0.5;
            p.H.setIdentity();
        }
        DaqpBackend cold;
        Solution actual, expected;
        REQUIRE(warm.solve(p, actual).ok());
        REQUIRE(cold.solve(p, expected).ok());
        CHECK(actual.status == expected.status);
        if (expected.status == SolveStatus::optimal) {
            CHECK((actual.x - expected.x).lpNorm<Eigen::Infinity>() < 1e-6);
        }
    }
}
