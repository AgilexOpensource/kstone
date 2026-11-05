#include "kstone/core/histogram.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kstone {

std::int64_t LatencyHistogram::upper_ns(int i) noexcept {
    if (i <= 0) return std::int64_t{1} << kMinExp;
    if (i >= kBuckets - 1) return std::numeric_limits<std::int64_t>::max();
    const int k = i - 1;
    const int e = kMinExp + k / kSub;
    return (std::int64_t{1} << e) +
           static_cast<std::int64_t>(k % kSub + 1) * (std::int64_t{1} << (e - kSubBits));
}

double LatencyHistogram::quantile(double p) const noexcept {
    if (count_ == 0) return 0.0;
    if (!(p > 0.0)) p = 0.0;
    if (p > 1.0) p = 1.0;
    const auto rank =
        std::max<std::uint64_t>(1, static_cast<std::uint64_t>(std::ceil(p * static_cast<double>(count_))));
    std::uint64_t seen = 0;
    for (int i = 0; i < kBuckets; ++i) {
        seen += counts_[static_cast<std::size_t>(i)];
        if (seen >= rank) return static_cast<double>(std::min(upper_ns(i), max_ns_)) * 1e-9;
    }
    return static_cast<double>(max_ns_) * 1e-9;
}

}  // namespace kstone
