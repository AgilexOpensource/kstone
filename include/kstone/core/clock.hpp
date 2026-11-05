#pragma once

#include "kstone/core/export.hpp"

#include <atomic>
#include <cstdint>
#include <memory>

namespace kstone {

/// Time source in integer nanoseconds. Implementations must be thread-safe.
/// The epoch is implementation-defined; only differences are meaningful unless documented otherwise.
class KSTONE_EXPORT Clock {
  public:
    virtual ~Clock();
    virtual std::int64_t now_ns() const noexcept = 0;
};

/// Monotonic clock backed by std::chrono::steady_clock.
KSTONE_EXPORT std::shared_ptr<Clock> make_steady_clock();

/// Externally driven clock (simulation, replay, tests).
class KSTONE_EXPORT ManualClock final : public Clock {
  public:
    explicit ManualClock(std::int64_t start_ns = 0) noexcept;
    std::int64_t now_ns() const noexcept override;
    void set_ns(std::int64_t t) noexcept;
    void advance_ns(std::int64_t dt) noexcept;

  private:
    std::atomic<std::int64_t> now_;
};

}  // namespace kstone
