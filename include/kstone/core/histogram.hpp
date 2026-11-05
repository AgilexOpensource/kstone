#pragma once

#include "kstone/core/export.hpp"

#include <array>
#include <cstdint>

namespace kstone {

/// Fixed-size log-linear latency histogram: 8 linear sub-buckets per power of two from 1.024 us to 1.07 s,
/// plus an underflow and an overflow bucket. add() is O(1) and never allocates; relative error <= 12.5%.
class KSTONE_EXPORT LatencyHistogram {
  public:
    static constexpr int kSubBits = 3;
    static constexpr int kSub = 1 << kSubBits;
    static constexpr int kMinExp = 10;  ///< Smaller samples fall in the underflow bucket.
    static constexpr int kMaxExp = 30;  ///< Samples >= 2^kMaxExp ns fall in the overflow bucket.
    static constexpr int kBuckets = (kMaxExp - kMinExp) * kSub + 2;

    void add(std::int64_t ns) noexcept {
        if (ns < 0) ns = 0;
        counts_[static_cast<std::size_t>(index(ns))]++;
        count_++;
        if (ns > max_ns_) max_ns_ = ns;
    }
    void clear() noexcept { *this = LatencyHistogram{}; }

    std::uint64_t count() const noexcept { return count_; }
    std::int64_t max_ns() const noexcept { return max_ns_; }
    /// Upper bound (s) of the bucket holding the p-quantile (p clamped to [0, 1]), capped at the largest
    /// sample; 0 when empty.
    double quantile(double p) const noexcept;

    const std::array<std::uint64_t, kBuckets>& counts() const noexcept { return counts_; }
    static int index(std::int64_t ns) noexcept {
        if (ns < (std::int64_t{1} << kMinExp)) return 0;
        if (ns >= (std::int64_t{1} << kMaxExp)) return kBuckets - 1;
        const int e = floor_log2(static_cast<std::uint64_t>(ns));
        return 1 + (e - kMinExp) * kSub + static_cast<int>((ns >> (e - kSubBits)) & (kSub - 1));
    }
    /// Exclusive upper bound (ns) of bucket i; the overflow bucket returns max int64.
    static std::int64_t upper_ns(int i) noexcept;

  private:
    static int floor_log2(std::uint64_t x) noexcept {
#if defined(__GNUC__) || defined(__clang__)
        return 63 - __builtin_clzll(x);
#else
        int e = 0;
        while (x >>= 1) ++e;
        return e;
#endif
    }

    std::array<std::uint64_t, kBuckets> counts_{};
    std::uint64_t count_{0};
    std::int64_t max_ns_{0};
};

}  // namespace kstone
