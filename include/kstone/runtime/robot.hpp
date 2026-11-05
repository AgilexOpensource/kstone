#pragma once

#include "kstone/core/clock.hpp"
#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <memory>

namespace kstone {

/// Measured joint state: q (size nq) and v (size nv) of the controlled model.
struct RobotState {
    Eigen::VectorXd q;
    Eigen::VectorXd v;
    /// When the measurement was taken, in the engine Context::clock domain (ns); -1 = unknown.
    std::int64_t stamp_ns{-1};
};

/// Command produced by one engine cycle: the configuration to reach after dt and the velocity to get there.
struct RobotCommand {
    Eigen::VectorXd q;
    Eigen::VectorXd v;
    /// false when this cycle's feedback was stale (EngineOptions::feedback_timeout_s); the driver may drop it.
    bool fresh{true};
};

/// Hardware or simulator boundary. Called from the control thread only; read/write must not allocate once
/// the vectors have their final size.
/// A failed engine cycle (including a command rejected by the guard, see CommandGuardOptions) writes nothing
/// and the engine never synthesizes a stop command: the driver must enforce its own command timeout and safe state.
class KSTONE_EXPORT RobotInterface {
  public:
    virtual ~RobotInterface();
    /// Fills state; an error aborts the cycle before solving. Unit-norm coordinates off by <= 1e-3 are renormalized.
    virtual Status read(RobotState& state) = 0;
    virtual Status write(const RobotCommand& command) = 0;
};

/// Ideal robot: every command is tracked exactly, so the next read returns it, stamped with `clock`
/// (null = steady clock, the Context::defaults() domain).
class KSTONE_EXPORT LoopbackRobot final : public RobotInterface {
  public:
    LoopbackRobot(Eigen::VectorXd q, Eigen::Index nv, std::shared_ptr<const Clock> clock = nullptr);
    Status read(RobotState& state) override;
    Status write(const RobotCommand& command) override;

  private:
    RobotState state_;
    std::shared_ptr<const Clock> clock_;
};

}  // namespace kstone
