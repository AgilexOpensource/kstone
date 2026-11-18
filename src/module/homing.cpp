#include "kstone/module/homing.hpp"

#include <cmath>
#include <limits>

namespace kstone {

const char* to_string(HomeStatus value) noexcept {
    switch (value) {
        case HomeStatus::idle:
            return "idle";
        case HomeStatus::waiting:
            return "waiting";
        case HomeStatus::executing:
            return "executing";
        case HomeStatus::succeeded:
            return "succeeded";
        case HomeStatus::timed_out:
            return "timed_out";
        case HomeStatus::aborted:
            return "aborted";
        case HomeStatus::cancelled:
            return "cancelled";
    }
    return "unknown";
}

Status Homing::validate(const HomeConfig& config) {
    if (!std::isfinite(config.velocity_scale) || config.velocity_scale <= 0.0 || config.velocity_scale > 1.0)
        return make_error(ErrorCode::invalid_argument, "home velocity_scale must be in (0, 1]");
    if (!std::isfinite(config.timeout_s) || config.timeout_s <= 0.0)
        return make_error(ErrorCode::invalid_argument, "home timeout_s must be > 0");
    return {};
}

Status Homing::set_config(const HomeConfig& config) {
    if (Status s = validate(config); !s) return s;
    config_ = config;
    return {};
}

void Homing::request() noexcept {
    if (active()) return;
    status_ = HomeStatus::waiting;
    prepared_ = false;
    timing_ = false;
    cancel_ = false;
    started_ = false;
}

bool Homing::update(const Cycle& cycle) noexcept {
    const bool skipped = prepared_ && (cycle.index != last_index_ + 1 || cycle.resynced);
    last_index_ = cycle.index;
    prepared_ = true;
    if (!active()) return false;
    if (cancel_) {
        cancel_ = false;
        status_ = HomeStatus::cancelled;
        return true;
    }
    if (!timing_) {
        progressed_ = cycle.now;
        anchor_ = std::numeric_limits<double>::infinity();
        timing_ = true;
    }
    if (skipped) {
        status_ = HomeStatus::aborted;
        return true;
    }
    if (status_ == HomeStatus::executing && cycle.now - progressed_ > config_.timeout_s) {
        status_ = HomeStatus::timed_out;
        return true;
    }
    return false;
}

void Homing::progress(double now, double error, bool arrived) noexcept {
    if (status_ != HomeStatus::executing) return;
    // Measured against the error at the last progress, so a slow steady approach still counts.
    if (arrived || error < anchor_ - progress_epsilon) {
        progressed_ = now;
        anchor_ = error;
    }
}

Claim Homing::claim(const Mask& joints) const noexcept {
    if (!active()) return {};
    return {&joints, priority::preemptive, OnHeld::wait, true};
}

bool Homing::start() noexcept {
    if (status_ != HomeStatus::waiting) return false;
    status_ = HomeStatus::executing;
    started_ = true;
    timing_ = false;  // the no-progress clock starts at the next update()
    return true;
}

}  // namespace kstone
