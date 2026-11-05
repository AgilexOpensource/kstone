// Out-of-tree smoke test against an installed kstone: ./consumer <arm.urdf>
#include "kstone/chassis/chassis.hpp"
#include "kstone/control/solver.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/motion/joint_trajectory.hpp"
#include "kstone/motion/pose_trajectory.hpp"

#include <cmath>
#include <cstdio>
#include <memory>

using namespace kstone;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <arm.urdf>\n", argv[0]);
        return 2;
    }
    auto model = Model::load(ModelSource::file(argv[1]), {}, Context::defaults());
    if (!model) {
        std::fprintf(stderr, "load: %s\n", model.error().message.c_str());
        return 1;
    }
    auto solver = Solver::create(*model);
    if (!solver) {
        std::fprintf(stderr, "solver: %s\n", solver.error().message.c_str());
        return 1;
    }
    auto task =
        std::make_shared<JointTask>(std::vector<std::string>{"j1"}, Eigen::VectorXd::Constant(1, 0.5));
    if (!solver->add_task("j1", task)) return 1;

    const auto id = (*model)->joint_id("j1");
    if (!id) return 1;
    const Eigen::Index iq = (*model)->joints()[*id].idx_q;

    Eigen::VectorXd q = (*model)->neutral();
    SolveResult r;
    for (int i = 0; i < 200; ++i) {
        if (!solver->solve(q, 0.01, r)) {
            std::fprintf(stderr, "solve failed at step %d\n", i);
            return 1;
        }
        q = r.q_next;
    }
    std::printf("j1=%f\n", q[iq]);
    if (std::abs(q[iq] - 0.5) >= 1e-3) return 1;

    // Chassis module: Ruckig is compiled into kstone, so no extra dependency is needed here.
    auto chassis = ChassisController::create();
    if (!chassis || !chassis->set_scheduler_period(0.02) || !chassis->start(0.0)) return 1;
    double now = 0.0;
    for (int i = 0; i < 400; ++i) {
        now += 0.02;
        auto accepted = chassis->submit_pose({0.5, 0.0, 0.0}, now);
        if (!accepted || !*accepted) return 1;
        chassis->step(now);
    }
    std::printf("chassis x=%f\n", chassis->pose().x);
    if (std::abs(chassis->pose().x - 0.5) >= 0.015) return 1;

    // Motion module: joint and pose references shaped by the bundled Ruckig.
    JointTrajectoryConfig joint_cfg;
    joint_cfg.trigger = JointTrigger::continuous;
    auto joint = JointTrajectory::create(1, joint_cfg);
    if (!joint) return 1;
    Eigen::VectorXd jq = Eigen::VectorXd::Zero(1);
    const Eigen::VectorXd one = Eigen::VectorXd::Ones(1);
    for (int i = 0; i < 400; ++i) {
        if (!joint->step(jq, one, joint->velocity(), one, 0.01)) return 1;
        jq = joint->reference();
    }
    PoseTrajectoryConfig pose_cfg;
    pose_cfg.translation.velocity = 0.2;
    pose_cfg.translation.acceleration = 1.0;
    auto pose = PoseTrajectory::create(pose_cfg);
    if (!pose) return 1;
    Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
    goal.translation().x() = 0.3;
    for (int i = 0; i < 300; ++i)
        if (!pose->step(Eigen::Isometry3d::Identity(), goal, 0.01)) return 1;
    std::printf("joint=%f pose x=%f\n", jq[0], pose->ramp_pose().translation().x());
    return std::abs(jq[0] - 1.0) < 1e-6 && std::abs(pose->ramp_pose().translation().x() - 0.3) < 1e-6 ? 0 : 1;
}
