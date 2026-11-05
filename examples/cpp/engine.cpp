// Embed the engine in a host loop with a custom robot driver.
//   ./kstone_example_engine tests/fixtures/arm.urdf
#include "kstone/core/context.hpp"
#include "kstone/model/model.hpp"
#include "kstone/module/yaml.hpp"
#include "kstone/runtime/config.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>

using namespace kstone;

// Reports the previous cycle's command, like a driver with one cycle of latency. A real driver talks to hardware here.
class DelayedRobot final : public RobotInterface {
  public:
    DelayedRobot(Eigen::VectorXd q, Eigen::Index nv, std::shared_ptr<const Clock> clock)
        : state_{std::move(q), Eigen::VectorXd::Zero(nv)}, clock_(std::move(clock)) {}

    Status read(RobotState& state) override {
        state.q = state_.q;
        state.v = state_.v;
        state.stamp_ns = clock_->now_ns();  // Stamp when the sample was taken; the engine checks its age.
        return {};
    }
    Status write(const RobotCommand& command) override {
        state_.q = command.q;
        state_.v = command.v;
        ++writes;
        return {};
    }

    int writes{0};

  private:
    RobotState state_;
    std::shared_ptr<const Clock> clock_;
};

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <arm.urdf>\n", argv[0]);
        return 2;
    }
    const std::filesystem::path urdf = std::filesystem::absolute(argv[1]);
    auto model = Model::load(ModelSource::file(urdf.string()), {}, Context::defaults());
    if (!model) {
        std::fprintf(stderr, "load: %s\n", model.error().message.c_str());
        return 1;
    }

    const std::string text = "model: {file: " + urdf.filename().string() + R"(}
rate_hz: 100
modules:
  arm:
    type: joint_target
    joints: [j1, j2, j3]
)";
    auto config = parse_yaml(text, "inline.yaml");
    if (!config) return 1;
    auto robot =
        std::make_shared<DelayedRobot>((*model)->neutral(), (*model)->nv(), Context::defaults().clock);
    EngineLoadOptions options;
    options.robot = robot;
    auto engine = build_engine(*config, urdf.parent_path(), options);
    if (!engine) {
        std::fprintf(stderr, "engine: %s\n", engine.error().message.c_str());
        return 1;
    }

    auto arm = engine->modules().module("arm");
    PortValue target;
    target.vector = Eigen::Vector3d(0.5, -0.3, 1.0);
    if (!arm || !arm->write_port(static_cast<std::size_t>(arm->port_index("target")), target)) return 1;

    const auto& joints = (*model)->joints();
    const Eigen::Index j1 = joints[*(*model)->joint_id("j1")].idx_q;
    const Eigen::Index j2 = joints[*(*model)->joint_id("j2")].idx_q;
    // The host owns the clock; Engine::run() would pace the same loop by itself.
    for (int i = 0; i <= 300; ++i) {
        if (const Status s = engine->step(i * engine->dt()); !s) {
            std::fprintf(stderr, "step: %s\n", s.error().message.c_str());
            return 1;
        }
        const Eigen::VectorXd& q = engine->state().q;
        if (i % 50 == 0) std::printf("t=%.2fs j1=%.3f j2=%.3f\n", i * engine->dt(), q[j1], q[j2]);
    }
    const Eigen::VectorXd& q = engine->state().q;
    const bool reached = std::abs(q[j1] - 0.5) < 1e-3 && std::abs(q[j2] + 0.3) < 1e-3;
    std::printf("%s after %d writes\n", reached ? "reached" : "not reached", robot->writes);
    return reached ? 0 : 1;
}
