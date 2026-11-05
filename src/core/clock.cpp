#include "kstone/core/clock.hpp"

#include <chrono>

namespace kstone {

Clock::~Clock() = default;

namespace {

class SteadyClock final : public Clock {
  public:
    std::int64_t now_ns() const noexcept override {
        using namespace std::chrono;
        return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    }
};

}  // namespace

std::shared_ptr<Clock> make_steady_clock() { return std::make_shared<SteadyClock>(); }

ManualClock::ManualClock(std::int64_t start_ns) noexcept : now_(start_ns) {}
std::int64_t ManualClock::now_ns() const noexcept { return now_.load(std::memory_order_acquire); }
void ManualClock::set_ns(std::int64_t t) noexcept { now_.store(t, std::memory_order_release); }
void ManualClock::advance_ns(std::int64_t dt) noexcept { now_.fetch_add(dt, std::memory_order_acq_rel); }

}  // namespace kstone
