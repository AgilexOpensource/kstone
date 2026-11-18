#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/core/mask.hpp"
#include "kstone/module/module.hpp"

#include <cstdint>
#include <string>

namespace kstone {

/// Return to a configured pose. Preemptive: claims over other modules and rejects new targets and input to
/// modules on its joints until it ends. It runs as long as it keeps closing in on home.
struct HomeConfig {
    double velocity_scale{0.25};  ///< (0, 1], applied to the module's motion limits.
    double timeout_s{2.0};        ///< > 0: times out after this long without progress toward home.
};

enum class HomeStatus { idle, waiting, executing, succeeded, timed_out, aborted, cancelled };
KSTONE_EXPORT const char* to_string(HomeStatus value) noexcept;

/// Homing state machine shared by modules; the module owns the home pose and the motion.
/// idle/finished -request()-> waiting -start()-> executing -finish()-> succeeded | aborted | timed_out | cancelled.
class KSTONE_EXPORT Homing {
  public:
    static Status validate(const HomeConfig& config);
    Status set_config(const HomeConfig& config);
    const HomeConfig& config() const noexcept { return config_; }

    HomeStatus status() const noexcept { return status_; }
    bool active() const noexcept {
        return status_ == HomeStatus::waiting || status_ == HomeStatus::executing;
    }
    /// Smallest drop in the error toward home that counts as progress (module units: rad, or m + rad).
    static constexpr double progress_epsilon = 1e-4;

    /// No-op while active.
    void request() noexcept;
    /// While active: the next update() ends it with cancelled (the module stops on its ramp and keeps the joints).
    /// No-op otherwise.
    void cancel() noexcept {
        if (active()) cancel_ = true;
    }
    /// The motion toward home began (start() switched to executing) in the current or last run.
    bool started() const noexcept { return started_; }
    /// Call from every prepare(). Ends a running homing with cancelled after cancel(), with aborted on a skipped
    /// step or Cycle::resynced, with timed_out after timeout_s without progress. True if it ended in this call.
    bool update(const Cycle& cycle) noexcept;
    /// While executing: report the remaining error to home; `arrived` (within tolerance) always counts as progress.
    /// An error more than progress_epsilon below the one at the last progress restarts the timeout.
    void progress(double now, double error, bool arrived) noexcept;
    /// Preemptive claim on joints while active, else none.
    Claim claim(const Mask& joints) const noexcept;
    /// waiting -> executing; true if it switched (begin the home motion now).
    bool start() noexcept;
    /// Ends the homing; the module chooses the target it keeps (home if succeeded, the ideal state if timed out).
    void finish(HomeStatus outcome) noexcept { status_ = outcome; }

  private:
    HomeConfig config_;
    HomeStatus status_{HomeStatus::idle};
    double progressed_{0.0};
    double anchor_{0.0};
    std::uint64_t last_index_{0};
    bool prepared_{false};
    bool timing_{false};
    bool cancel_{false};
    bool started_{false};
};

}  // namespace kstone
