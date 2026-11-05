// Full control-cycle benchmark: FK of the observed pose + PoseTrajectory::step + FrameTask::set_target +
// Solver::solve, per frame, timed as one cycle. Goals jump every 200 cycles so the trajectory keeps replanning.
// Usage: kstone_cycle_bench [cycles]   (build with -DKSTONE_BUILD_BENCH=ON, Release)
#include "kstone/collision/constraint.hpp"
#include "kstone/control/solver.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"
#include "kstone/module/builtins.hpp"
#include "kstone/motion/pose_trajectory.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace kstone;

namespace {

struct Case {
    const char* file;
    RootJoint root;
    std::vector<std::string> frames;
};

struct Stats {
    std::vector<double> us;
    void add(double v) { us.push_back(v); }
    double at(double p) const {
        return us[std::min(us.size() - 1, static_cast<std::size_t>(p * static_cast<double>(us.size())))];
    }
};

int run(const Case& c, SolveMode mode, const char* mode_name, int cycles, bool collision = false) {
    constexpr double dt = 0.01;
    ModelOptions mo;
    mo.root = c.root;
    auto m =
        Model::load(ModelSource::file(std::string(KSTONE_FIXTURES) + "/" + c.file), mo, Context::defaults());
    if (!m) {
        std::fprintf(stderr, "%s: %s\n", c.file, m.error().message.c_str());
        return 1;
    }
    SolverOptions so;
    so.mode = mode;
    auto created = Solver::create(*m, so);
    if (!created) {
        std::fprintf(stderr, "%s: %s\n", c.file, created.error().message.c_str());
        return 1;
    }
    Solver solver = std::move(*created);
    Workspace ws(*m);
    Eigen::VectorXd q = (*m)->neutral();
    if (!ws.update(q)) return 1;

    std::vector<FrameId> ids;
    std::vector<std::shared_ptr<FrameTask>> tasks;
    std::vector<PoseTrajectory> trajectories;
    std::vector<Eigen::Isometry3d> home, goals;
    PoseTrajectoryConfig pc;
    pc.translation = {0.3, 1.0, 0.0, 0.0, 10.0};
    pc.rotation = {1.0, 3.0, 0.0, 0.0, 20.0};
    int level = 0;
    for (const std::string& f : c.frames) {
        auto id = (*m)->frame_id(f);
        Eigen::Isometry3d pose;
        if (!id || !ws.frame_pose(*id, pose)) return 1;
        auto t = PoseTrajectory::create(pc);
        if (!t) return 1;
        ids.push_back(*id);
        home.push_back(pose);
        goals.push_back(pose);
        trajectories.push_back(std::move(*t));
        tasks.push_back(std::make_shared<FrameTask>(f, pose));
        TaskOptions o;
        o.level = level++;
        if (!solver.add_task(f, tasks.back(), o)) return 1;
    }
    std::shared_ptr<CollisionConstraint> self;
    if (collision) {
        const std::string path = std::string(KSTONE_FIXTURES) + "/" + c.file;
        auto cm = CollisionModel::from_urdf(*m, ModelSource::file(path), CollisionModelOptions{},
                                            Context::defaults());
        if (!cm) {
            std::fprintf(stderr, "%s: %s\n", c.file, cm.error().message.c_str());
            return 1;
        }
        CollisionConstraintOptions co;
        co.activation_margin = 0.05;
        auto con = CollisionConstraint::create(*cm, nullptr, co);
        if (!con || !solver.add_constraint("self_collision", *con)) return 1;
        self = *con;
    }

    std::mt19937 rng(1);
    std::uniform_real_distribution<double> u(-0.1, 0.1);
    Stats total, traj, solve;
    SolveResult r;
    int failures = 0;
    const int warmup = 50;
    for (int i = 0; i < warmup + cycles; ++i) {
        if (i % 200 == 0)
            for (std::size_t k = 0; k < goals.size(); ++k) {
                goals[k] = home[k];
                goals[k].translation() += Eigen::Vector3d(u(rng), u(rng), u(rng));
            }
        const auto t0 = std::chrono::steady_clock::now();
        bool ok = static_cast<bool>(ws.update(q));
        for (std::size_t k = 0; ok && k < ids.size(); ++k) {
            Eigen::Isometry3d observed;
            ok = ws.frame_pose(ids[k], observed) && trajectories[k].step(observed, goals[k], dt) &&
                 tasks[k]->set_target(trajectories[k].ramp_pose());
        }
        const auto t1 = std::chrono::steady_clock::now();
        ok = ok && solver.solve(q, dt, r);
        const auto t2 = std::chrono::steady_clock::now();
        if (!ok) {
            ++failures;
            continue;
        }
        q = r.q_next;
        if (i >= warmup) {
            traj.add(std::chrono::duration<double, std::micro>(t1 - t0).count());
            solve.add(std::chrono::duration<double, std::micro>(t2 - t1).count());
            total.add(std::chrono::duration<double, std::micro>(t2 - t0).count());
        }
    }
    if (total.us.empty()) {
        std::printf("%-28s %-16s all cycles failed\n", c.file, mode_name);
        return 1;
    }
    for (Stats* s : {&total, &traj, &solve}) std::sort(s->us.begin(), s->us.end());
    std::printf(
        "%-28s %-16s%s nv=%-3d cycle p50 %7.1f p99 %7.1f max %7.1f | traj+fk p50 %6.1f p99 %6.1f | "
        "solve p50 %7.1f p99 %7.1f us  failures %d",
        c.file, mode_name, collision ? "+coll" : "     ", static_cast<int>((*m)->nv()), total.at(0.5),
        total.at(0.99), total.us.back(), traj.at(0.5), traj.at(0.99), solve.at(0.5), solve.at(0.99),
        failures);
    if (self)
        std::printf("  pairs %d rows %d nearest %.3f m (%s)",
                    static_cast<int>(self->query().model()->pairs().size()),
                    static_cast<int>(self->report().rows), self->report().nearest.distance,
                    self->query().pair_name(self->report().nearest.pair).c_str());
    std::printf("\n");
    return failures == 0 ? 0 : 1;
}

// Module layer: chassis + two frame tasks + a left-arm joint target that claims and releases every 300 cycles.
int run_modules(const Case& c, SolveMode mode, const char* mode_name, int cycles) {
    constexpr double dt = 0.01;
    ModelOptions mo;
    mo.root = c.root;
    auto m =
        Model::load(ModelSource::file(std::string(KSTONE_FIXTURES) + "/" + c.file), mo, Context::defaults());
    if (!m) return 1;
    SolverOptions so;
    so.mode = mode;
    auto created = ModuleSet::create(*m, so);
    if (!created) return 1;
    ModuleSet set = std::move(*created);

    ChassisModuleConfig cc;
    cc.period = dt;
    auto chassis = std::make_shared<ChassisModule>("chassis", cc);
    PoseTrajectoryConfig pc;
    pc.translation = {0.3, 1.0, 0.0, 0.0, 10.0};
    pc.rotation = {1.0, 3.0, 0.0, 0.0, 20.0};
    std::vector<std::shared_ptr<FrameTaskModule>> frames;
    int level = 0;
    for (const std::string& f : c.frames) {
        FrameTaskModuleConfig fc;
        fc.frame = f;
        fc.trajectory = pc;
        fc.task.level = level++;
        frames.push_back(std::make_shared<FrameTaskModule>(f, fc));
    }
    JointTargetModuleConfig jc;
    for (int i = 1; i <= 7; ++i) jc.joints.push_back("left_arm_joint_" + std::to_string(i));
    jc.task.level = level;
    auto arm = std::make_shared<JointTargetModule>("left_arm", jc);
    if (!set.add(chassis)) return 1;
    for (const auto& f : frames)
        if (!set.add(f)) return 1;
    if (!set.add(arm) || !set.configure()) return 1;

    Workspace ws(*m);
    Eigen::VectorXd q = (*m)->neutral(), v = Eigen::VectorXd::Zero((*m)->nv());
    if (!ws.update(q)) return 1;
    std::vector<Eigen::Isometry3d> home;
    for (const std::string& f : c.frames) {
        auto id = (*m)->frame_id(f);
        Eigen::Isometry3d pose;
        if (!id || !ws.frame_pose(*id, pose)) return 1;
        home.push_back(pose);
    }
    Eigen::VectorXd arm_goal = Eigen::VectorXd::Constant(7, 0.2);

    std::mt19937 rng(1);
    std::uniform_real_distribution<double> u(-0.1, 0.1);
    // Keyed by solve stage count: a held module drops a level, so mixing them blurs the percentiles.
    std::map<std::size_t, Stats> by_stages;
    SolveResult r;
    int failures = 0;
    const int warmup = 50;
    double now = 0.0;
    for (int i = 0; i < warmup + cycles; ++i) {
        now += dt;
        bool ok = static_cast<bool>(chassis->controller()->submit_pose({0.2 * std::sin(now), 0.1, 0.1}, now));
        if (i % 200 == 0)
            for (std::size_t k = 0; ok && k < frames.size(); ++k) {
                Eigen::Isometry3d goal = home[k];
                goal.translation() += Eigen::Vector3d(u(rng), u(rng), u(rng));
                ok = static_cast<bool>(frames[k]->set_target(goal));
            }
        if (i % 300 == 0) {
            arm_goal *= -1.0;
            ok = ok && arm->set_target(arm_goal);
        }
        const auto t0 = std::chrono::steady_clock::now();
        ok = ok && set.step(q, v, now, dt, r);
        const auto t1 = std::chrono::steady_clock::now();
        if (!ok) {
            ++failures;
            continue;
        }
        q = r.q_next;
        v = r.v;
        if (i >= warmup)
            by_stages[r.stages.size()].add(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    if (by_stages.empty()) {
        std::printf("%-28s %-16s+mod  all cycles failed\n", c.file, mode_name);
        return 1;
    }
    for (auto& [stages, s] : by_stages) {
        std::sort(s.us.begin(), s.us.end());
        std::printf(
            "%-28s %-16s+mod  nv=%-3d stages %zu n=%-5zu cycle p50 %7.1f p99 %7.1f max %7.1f us  failures "
            "%d\n",
            c.file, mode_name, static_cast<int>((*m)->nv()), stages, s.us.size(), s.at(0.5), s.at(0.99),
            s.us.back(), failures);
    }
    return failures == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    const int cycles = argc > 1 ? std::max(1, std::atoi(argv[1])) : 5000;
    const std::vector<Case> cases = {
        {"arm.urdf", RootJoint::fixed, {"tool"}},
        {"floating_base.urdf", RootJoint::free_flyer, {"tool"}},
        {"omni_dual_arm_gripper.urdf", RootJoint::free_flyer, {"left_tool_frame", "right_tool_frame"}},
    };
    int rc = 0;
    for (const Case& c : cases) {
        rc |= run(c, SolveMode::weighted, "weighted", cycles);
        rc |= run(c, SolveMode::layered_coupled, "layered_coupled", cycles);
    }
    // Self-collision over the URDF <collision> geometry (SRDF-free default filtering).
    rc |= run(cases.back(), SolveMode::weighted, "weighted", cycles, true);
    rc |= run(cases.back(), SolveMode::layered_coupled, "layered_coupled", cycles, true);
    rc |= run_modules(cases.back(), SolveMode::weighted, "weighted", cycles);
    rc |= run_modules(cases.back(), SolveMode::layered_coupled, "layered_coupled", cycles);
    return rc;
}
