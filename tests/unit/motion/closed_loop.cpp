// Closed loop: trajectory -> task target -> Solver::solve -> integrate q -> measured state fed back.
#include "kstone/control/solver.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"
#include "kstone/motion/joint_trajectory.hpp"
#include "kstone/motion/pose_trajectory.hpp"

#include <Eigen/Geometry>
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace kstone;
namespace fs = std::filesystem;

namespace {

const fs::path kFixtures = KSTONE_FIXTURES;
constexpr double kDt = 0.01;

ModelPtr load_arm() {
    auto m = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), ModelOptions{},
                         Context::defaults());
    REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
    return *m;
}

Eigen::Isometry3d tool_pose(const ModelPtr& m, Workspace& ws, const Eigen::VectorXd& q) {
    REQUIRE(ws.update(q));
    auto id = m->frame_id("tool");
    REQUIRE(id);
    Eigen::Isometry3d pose;
    REQUIRE(ws.frame_pose(*id, pose));
    return pose;
}

// Configuration inside the joint limits with a margin, so its tool pose is reachable.
Eigen::VectorXd random_q(const ModelPtr& m, std::mt19937& rng) {
    std::uniform_real_distribution<double> u(0.15, 0.85);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(m->nv());
    Eigen::VectorXd q = m->neutral();
    for (const JointInfo& j : m->joints()) {
        if (j.nv != 1) continue;
        if (j.nq == 1 && m->has_position_limit(j.id)) {
            const double lo = m->lower_position_limit()[j.idx_q], hi = m->upper_position_limit()[j.idx_q];
            q[j.idx_q] = lo + u(rng) * (hi - lo);
        } else if (j.nq == 2) {
            v[j.idx_v] = 6.0 * (u(rng) - 0.5);
        }
    }
    Eigen::VectorXd out(m->nq());
    REQUIRE(m->integrate(q, v, out));
    return out;
}

double pose_error(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
    return std::max((a.translation() - b.translation()).norm(),
                    Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle());
}

struct Watch {
    double worst_speed_ratio{0.0};  // max |v_i| / limit_i
    double worst_limit_excess{0.0};
    bool finite{true};
    int failures{0};

    void check(const ModelPtr& m, const SolveResult& r) {
        finite = finite && r.v.allFinite() && r.q_next.allFinite();
        const Eigen::VectorXd& vmax = m->velocity_limit();
        for (Index i = 0; i < m->nv(); ++i)
            if (std::isfinite(vmax[i]) && vmax[i] > 0)
                worst_speed_ratio = std::max(worst_speed_ratio, std::abs(r.v[i]) / vmax[i]);
        for (const JointInfo& j : m->joints()) {
            if (j.nq != 1 || !m->has_position_limit(j.id)) continue;
            const double x = r.q_next[j.idx_q];
            worst_limit_excess = std::max({worst_limit_excess, m->lower_position_limit()[j.idx_q] - x,
                                           x - m->upper_position_limit()[j.idx_q]});
        }
    }
};

struct PoseLoop {
    ModelPtr model = load_arm();
    Workspace ws{model};
    std::shared_ptr<FrameTask> frame;
    Solver solver;
    PoseTrajectory trajectory;
    Eigen::VectorXd q;
    SolveResult result;
    Watch watch;

    static Solver make_solver(const ModelPtr& m) {
        auto s = Solver::create(m);
        REQUIRE(s);
        return std::move(*s);
    }
    static PoseTrajectory make_trajectory(const PoseTrajectoryConfig& c) {
        auto t = PoseTrajectory::create(c);
        REQUIRE(t);
        return std::move(*t);
    }

    explicit PoseLoop(const PoseTrajectoryConfig& config)
        : solver(make_solver(model)), trajectory(make_trajectory(config)), q(model->neutral()) {
        frame = std::make_shared<FrameTask>("tool", tool_pose(model, ws, q));
        REQUIRE(solver.add_task("tool", frame, TaskOptions{}));
    }

    // One control cycle; returns the ramp pose commanded this cycle.
    Eigen::Isometry3d cycle(const Eigen::Isometry3d& goal) {
        const Eigen::Isometry3d observed = tool_pose(model, ws, q);
        REQUIRE(trajectory.step(observed, goal, kDt));
        REQUIRE(frame->set_target(trajectory.ramp_pose()));
        if (!solver.solve(q, kDt, result)) {
            ++watch.failures;
            return trajectory.ramp_pose();
        }
        watch.check(model, result);
        q = result.q_next;
        return trajectory.ramp_pose();
    }
};

PoseTrajectoryConfig shaped(PoseSync sync) {
    PoseTrajectoryConfig c;
    c.sync = sync;
    c.translation = {0.3, 1.0, 0.0, 0.0, 10.0};
    c.rotation = {1.0, 3.0, 0.0, 0.0, 20.0};
    return c;
}

}  // namespace

// Reachable random goals: the arm tracks the ramp, settles on each goal and never exceeds joint limits.
TEST_CASE("closed loop: pose trajectory drives the arm to reachable goals") {
    for (PoseSync sync : {PoseSync::axis_phase, PoseSync::axis_time}) {
        CAPTURE(std::string(to_string(sync)));
        PoseLoop loop(shaped(sync));
        std::mt19937 rng(7);
        double worst_final = 0.0, worst_ramp_speed = 0.0;
        for (int goal_index = 0; goal_index < 8; ++goal_index) {
            const Eigen::Isometry3d goal = tool_pose(loop.model, loop.ws, random_q(loop.model, rng));
            Eigen::Isometry3d previous = loop.cycle(goal);
            for (int k = 0; k < 600; ++k) {
                const Eigen::Isometry3d ramp = loop.cycle(goal);
                const double speed =
                    (ramp.translation() - previous.translation()).cwiseAbs().maxCoeff() / kDt;
                worst_ramp_speed = std::max(worst_ramp_speed, speed);
                previous = ramp;
            }
            CHECK(loop.trajectory.stopped());
            worst_final = std::max(worst_final, pose_error(tool_pose(loop.model, loop.ws, loop.q), goal));
        }
        CHECK(loop.watch.finite);
        CHECK(loop.watch.failures == 0);
        CHECK(worst_final < 1e-3);
        CHECK(worst_ramp_speed <= 0.3 + 1e-6);
        CHECK(loop.watch.worst_speed_ratio <= 1.0 + 1e-6);
        CHECK(loop.watch.worst_limit_excess <= 1e-6);
    }
}

// Goal switched back and forth mid-flight: the ramp velocity stays continuous (bounded by acceleration * dt).
TEST_CASE("closed loop: mid-flight reversals keep the ramp smooth") {
    PoseLoop loop(shaped(PoseSync::axis_phase));
    std::mt19937 rng(11);
    const Eigen::Isometry3d a = tool_pose(loop.model, loop.ws, random_q(loop.model, rng));
    const Eigen::Isometry3d b = tool_pose(loop.model, loop.ws, random_q(loop.model, rng));
    Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
    double worst_jump = 0.0;
    for (int k = 0; k < 1500; ++k) {
        (void)loop.cycle((k / 37) % 2 == 0 ? a : b);  // switch every 0.37 s, well before arrival
        const Eigen::Vector3d v = loop.trajectory.linear_velocity();
        worst_jump = std::max(worst_jump, (v - previous_velocity).cwiseAbs().maxCoeff());
        previous_velocity = v;
    }
    for (int k = 0; k < 800; ++k) (void)loop.cycle(a);
    CHECK(loop.watch.finite);
    CHECK(loop.watch.failures == 0);
    CHECK(worst_jump <= 1.0 * kDt + 1e-9);  // translation acceleration 1.0
    CHECK(pose_error(tool_pose(loop.model, loop.ws, loop.q), a) < 1e-3);
    CHECK(loop.watch.worst_speed_ratio <= 1.0 + 1e-6);
    CHECK(loop.watch.worst_limit_excess <= 1e-6);
}

// Joint trajectory feeding a JointTask, measured state from the integrated q; j3 is continuous (wrapped).
TEST_CASE("closed loop: joint trajectory drives the arm joints") {
    ModelPtr m = load_arm();
    const std::vector<std::string> names{"j1", "j2", "j3", "j4"};
    std::vector<JointInfo> joints;
    for (const std::string& n : names) {
        auto id = m->joint_id(n);
        REQUIRE(id);
        joints.push_back(m->joints()[static_cast<std::size_t>(*id)]);
    }
    const auto read = [&](const Eigen::VectorXd& q, const Eigen::VectorXd& v, Eigen::VectorXd& pos,
                          Eigen::VectorXd& vel) {
        for (std::size_t i = 0; i < joints.size(); ++i) {
            const JointInfo& j = joints[i];
            const auto r = static_cast<Index>(i);
            pos[r] = j.nq == 1 ? q[j.idx_q] : std::atan2(q[j.idx_q + 1], q[j.idx_q]);
            vel[r] = v[j.idx_v];
        }
    };
    Eigen::VectorXd limit(4);
    for (std::size_t i = 0; i < joints.size(); ++i)
        limit[static_cast<Index>(i)] = m->velocity_limit()[joints[i].idx_v];

    for (JointTrigger trigger : {JointTrigger::threshold, JointTrigger::continuous}) {
        for (JointSync sync : {JointSync::time, JointSync::phase, JointSync::none}) {
            CAPTURE(std::string(to_string(trigger)));
            CAPTURE(std::string(to_string(sync)));
            JointTrajectoryConfig config;
            config.trigger = trigger;
            config.sync = sync;
            auto created = JointTrajectory::create(4, config, {false, false, true, false});
            REQUIRE(created);
            JointTrajectory trajectory = std::move(*created);
            Eigen::VectorXd target(4);
            target << 0.8, -0.6, 3.0, 0.07;
            auto task = std::make_shared<JointTask>(names, target);
            auto s = Solver::create(m);
            REQUIRE(s);
            Solver solver = std::move(*s);
            REQUIRE(solver.add_task("joints", task, TaskOptions{}));

            Eigen::VectorXd q = m->neutral(), v = Eigen::VectorXd::Zero(m->nv());
            Eigen::VectorXd pos(4), vel(4);
            SolveResult r;
            Watch watch;
            std::mt19937 rng(static_cast<unsigned>(3 + static_cast<int>(sync)));
            std::uniform_real_distribution<double> u(0.0, 1.0);
            double worst_final = 0.0;
            for (int goal = 0; goal < 6; ++goal) {
                if (goal > 0)
                    target << -1.6 + 3.2 * u(rng), -1.2 + 2.4 * u(rng), -6.0 + 12.0 * u(rng), 0.1 * u(rng);
                for (int k = 0; k < 500; ++k) {
                    read(q, v, pos, vel);
                    REQUIRE(trajectory.step(pos, target, vel, limit, kDt));
                    REQUIRE(task->set_target(trajectory.reference()));
                    if (!solver.solve(q, kDt, r)) {
                        ++watch.failures;
                        continue;
                    }
                    watch.check(m, r);
                    q = r.q_next;
                    v = r.v;
                }
                read(q, v, pos, vel);
                Eigen::VectorXd e = pos - target;
                e[2] = std::remainder(e[2], 2.0 * M_PI);
                worst_final = std::max(worst_final, e.cwiseAbs().maxCoeff());
            }
            CHECK(watch.finite);
            CHECK(watch.failures == 0);
            CHECK(worst_final < 1e-4);
            CHECK(watch.worst_speed_ratio <= 1.0 + 1e-6);
            CHECK(watch.worst_limit_excess <= 1e-6);
        }
    }
}

// Long random run: goals partly unreachable, limits and sync retuned on the fly, observed pose lagging.
TEST_CASE("closed loop: randomized stress") {
    PoseLoop loop(shaped(PoseSync::axis_phase));
    std::mt19937 rng(2024);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    Eigen::Isometry3d goal = loop.trajectory.ramp_pose();
    for (int k = 0; k < 6000; ++k) {
        if (k % 97 == 0) {
            goal = tool_pose(loop.model, loop.ws, random_q(loop.model, rng));
            if (u(rng) < 0.3) goal.translation() += Eigen::Vector3d(0.5, 0.5, 0.5);  // out of reach
        }
        if (k % 401 == 0) {
            PoseTrajectoryConfig c = shaped(u(rng) < 0.5 ? PoseSync::axis_phase : PoseSync::axis_time);
            c.translation.velocity = 0.05 + 0.5 * u(rng);
            c.translation.jerk = u(rng) < 0.3 ? 0.0 : c.translation.jerk;
            c.rotation.deceleration = 1.0 + 4.0 * u(rng);
            c.lead_speed_scale = u(rng) < 0.5 ? 0.0 : 2.0;
            REQUIRE(loop.trajectory.tune(c));
        }
        const Eigen::Isometry3d ramp = loop.cycle(goal);
        REQUIRE(ramp.matrix().allFinite());
    }
    CHECK(loop.watch.finite);
    CHECK(loop.watch.failures == 0);
    CHECK(loop.watch.worst_speed_ratio <= 1.0 + 1e-6);
    CHECK(loop.watch.worst_limit_excess <= 1e-6);
}
