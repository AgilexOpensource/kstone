#pragma once

#include "kstone/core/clock.hpp"
#include "kstone/core/error.hpp"
#include "kstone/runtime/robot.hpp"

#include <Eigen/Core>

#include <memory>

namespace kstone {

class Model;

namespace test {

struct SimRobotOptions {
    double rate_hz{100.0};        ///< > 0; each write advances the simulation by 1 / rate_hz.
    double time_constant_s{0.0};  ///< >= 0; first-order position lag toward the command, 0 = exact tracking.
    double latency_s{0.0};        ///< >= 0; commands take effect after round(latency_s * rate_hz) writes.
    double position_noise{
        0.0};  ///< >= 0; std dev of Gaussian noise added to each read q, per dof (rad or m).
    double velocity_noise{0.0};  ///< >= 0; std dev of Gaussian noise added to each read v, per dof.
    int seed{0};                 ///< Noise generator seed; equal seeds give equal reads.
};

/// Test-only simulated robot for exercising closed-loop behaviour without hardware: commands are delayed, tracked through a
/// first-order lag on the model manifold, and read back with noise, stamped with `clock` (null = steady clock).
/// v is the simulated step / dt. Before the first delayed command arrives the robot holds q.
class SimRobot final : public RobotInterface {
  public:
    static Result<std::shared_ptr<SimRobot>> create(std::shared_ptr<const Model> model, Eigen::VectorXd q,
                                                    const SimRobotOptions& options = {},
                                                    std::shared_ptr<const Clock> clock = nullptr);
    ~SimRobot() override;
    Status read(RobotState& state) override;
    Status write(const RobotCommand& command) override;

  private:
    struct Impl;
    explicit SimRobot(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

}  // namespace test
}  // namespace kstone
