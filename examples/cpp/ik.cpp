// Offline IK: move the left gripper of the omni dual-arm robot 10 cm forward with its left arm only.
//   ./kstone_example_ik examples/omni_dual_arm/omni_dual_arm.urdf
#include "kstone/control/solver.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace kstone;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <omni_dual_arm.urdf>\n", argv[0]);
        return 2;
    }
    auto loaded = Model::load(ModelSource::file(argv[1]), {}, Context::defaults());
    if (!loaded) {
        std::fprintf(stderr, "load: %s\n", loaded.error().message.c_str());
        return 1;
    }
    const ModelPtr model = *loaded;

    // Bent start posture; a fully stretched arm is singular.
    Eigen::VectorXd q = model->neutral();
    const std::vector<std::pair<std::string, double>> start{
        {"left_arm_joint_2", 1.57}, {"left_arm_joint_3", -1.57}, {"left_arm_joint_4", 1.57}};
    for (const auto& [name, value] : start) q[model->joints()[*model->joint_id(name)].idx_q] = value;

    std::vector<std::size_t> arm;
    for (int i = 1; i <= 7; ++i) {
        const auto id = model->joint_id("left_arm_joint_" + std::to_string(i));
        if (!id) return 1;
        arm.push_back(static_cast<std::size_t>(model->joints()[*id].idx_v));
    }

    Workspace ws(model);
    const auto tool = model->frame_id("left_tool_frame");
    Eigen::Isometry3d start_pose;
    if (!tool || !ws.update(q) || !ws.frame_pose(*tool, start_pose)) return 1;
    Eigen::Isometry3d target = start_pose;
    target.translation().x() += 0.1;

    auto solver = Solver::create(model);
    if (!solver) return 1;
    auto task = std::make_shared<FrameTask>("left_tool_frame", target);
    if (!task->set_gain(0.5) ||
        !task->set_mask(Mask::from_indices(static_cast<std::size_t>(model->nv()), arm)))
        return 1;
    if (!solver->add_task("left_tool", task)) return 1;

    SolveResult r;
    Eigen::Isometry3d pose;
    for (int i = 1; i <= 200; ++i) {
        if (const Status s = solver->solve(q, 0.01, r); !s) {
            std::fprintf(stderr, "solve: %s\n", s.error().message.c_str());
            return 1;
        }
        q = r.q_next;
        if (!ws.update(q) || !ws.frame_pose(*tool, pose)) return 1;
        const double error = (pose.translation() - target.translation()).norm();
        if (error < 1e-4) {
            std::printf("converged in %d steps, position error %.2e m\n", i, error);
            return 0;
        }
    }
    std::fprintf(stderr, "did not converge\n");
    return 1;
}
