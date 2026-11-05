#pragma once

#include "kstone/core/export.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace kstone {

/// Fixed-length boolean selection over a vector space (typically velocity dimensions).
/// Binary operators on masks of different sizes treat missing entries as false and yield the larger size.
class KSTONE_EXPORT Mask {
  public:
    Mask() = default;
    explicit Mask(std::size_t size, bool value = false);

    static Mask all(std::size_t size) { return Mask(size, true); }
    static Mask none(std::size_t size) { return Mask(size, false); }
    static Mask from_indices(std::size_t size, const std::vector<std::size_t>& indices);

    std::size_t size() const noexcept { return bits_.size(); }
    std::size_t count() const noexcept;
    bool any() const noexcept { return count() > 0; }
    bool test(std::size_t i) const noexcept { return i < bits_.size() && bits_[i] != 0; }

    /// Out-of-range indices are ignored.
    Mask& set(std::size_t i, bool value = true) noexcept;
    Mask& set_range(std::size_t begin, std::size_t count, bool value = true) noexcept;

    std::vector<std::size_t> indices() const;

    Mask operator~() const;
    Mask& operator|=(const Mask& other);
    Mask& operator&=(const Mask& other);
    Mask& operator-=(const Mask& other);  ///< set difference
    friend Mask operator|(Mask a, const Mask& b) { return a |= b; }
    friend Mask operator&(Mask a, const Mask& b) { return a &= b; }
    friend Mask operator-(Mask a, const Mask& b) { return a -= b; }
    friend bool operator==(const Mask& a, const Mask& b) { return a.bits_ == b.bits_; }
    friend bool operator!=(const Mask& a, const Mask& b) { return !(a == b); }

  private:
    std::vector<std::uint8_t> bits_;
};

}  // namespace kstone
