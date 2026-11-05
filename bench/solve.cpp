// Solve-time benchmark: median / p99 / max of Solver::solve per model and mode.
// Usage: kstone_bench [steps]   (build with -DKSTONE_BUILD_BENCH=ON, Release)
#include "kstone/control/solver.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

using namespace kstone;

namespace {

struct Case {
    const char* file;
    RootJoint root;
    std::vector<std::string> frames;
};

struct Mode {
    const char* name;
    SolveMode mode;
};

Eigen::Isometry3d pose_at(const ModelPtr& m, const Eigen::VectorXd& q, const std::string& frame) {
    Workspace ws(m);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    auto id = m->frame_id(frame);
    if (ws.update(q) && id) (void)ws.frame_pose(*id, pose);
    return pose;
}

int run(const Case& c, const Mode& mode, int steps) {
    ModelOptions mo;
    mo.root = c.root;
    auto m =
        Model::load(ModelSource::file(std::string(KSTONE_FIXTURES) + "/" + c.file), mo, Context::defaults());
    if (!m) {
        std::fprintf(stderr, "%s: %s\n", c.file, m.error().message.c_str());
        return 1;
    }
    SolverOptions so;
    so.mode = mode.mode;
    auto created = Solver::create(*m, so);
    if (!created) {
        std::fprintf(stderr, "%s: %s\n", c.file, created.error().message.c_str());
        return 1;
    }
    Solver solver = std::move(*created);
    const Eigen::VectorXd q0 = (*m)->neutral();
    // One frame task per level (levels 0, 1, ...), posture one level below.
    int level = 0;
    for (const std::string& f : c.frames) {
        Eigen::Isometry3d target = pose_at(*m, q0, f);
        target.translation() += Eigen::Vector3d(0.2, 0.05, 0.05);
        TaskOptions o;
        o.level = level++;
        (void)solver.add_task(f, std::make_shared<FrameTask>(f, target), o);
    }
    TaskOptions low;
    low.level = level;
    (void)solver.add_task("posture", std::make_shared<PostureTask>(q0), low);

    SolveResult r;
    Eigen::VectorXd q = q0;
    std::vector<double> us;
    us.reserve(static_cast<std::size_t>(steps));
    int failures = 0;
    for (int i = 0; i < 50 + steps; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        const Status s = solver.solve(q, 0.01, r);
        const auto t1 = std::chrono::steady_clock::now();
        if (!s) {
            if (failures == 0)
                std::fprintf(stderr, "%s %s: %s\n", c.file, mode.name, s.error().message.c_str());
            ++failures;
            continue;
        }
        q = r.q_next;
        if (i >= 50) us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    if (us.empty()) {
        std::printf("%-28s %-20s all %d solves failed\n", c.file, mode.name, failures);
        return 1;
    }
    std::sort(us.begin(), us.end());
    const auto at = [&](double p) {
        return us[std::min(us.size() - 1, static_cast<std::size_t>(p * static_cast<double>(us.size())))];
    };
    std::printf("%-28s %-20s nv=%-3d median %8.1f us  p99 %8.1f us  max %8.1f us  failures %d\n", c.file,
                mode.name, static_cast<int>((*m)->nv()), at(0.5), at(0.99), us.back(), failures);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const int steps = argc > 1 ? std::max(1, std::atoi(argv[1])) : 2000;
    const std::vector<Case> cases = {
        {"arm.urdf", RootJoint::fixed, {"tool"}},
        {"diff_chassis.urdf", RootJoint::free_flyer, {"tool"}},
        {"floating_base.urdf", RootJoint::free_flyer, {"tool"}},
        {"omni_dual_arm_gripper.urdf", RootJoint::free_flyer, {"left_tool_frame", "right_tool_frame"}},
    };
    const std::vector<Mode> modes = {{"weighted", SolveMode::weighted},
                                     {"layered_independent", SolveMode::layered_independent},
                                     {"layered_coupled", SolveMode::layered_coupled},
                                     {"lexicographic", SolveMode::lexicographic}};
    int rc = 0;
    for (const Case& c : cases)
        for (const Mode& md : modes) rc |= run(c, md, steps);
    return rc;
}
