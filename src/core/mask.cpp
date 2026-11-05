#include "kstone/core/mask.hpp"

#include <algorithm>

namespace kstone {

Mask::Mask(std::size_t size, bool value) : bits_(size, value ? 1 : 0) {}

Mask Mask::from_indices(std::size_t size, const std::vector<std::size_t>& indices) {
    Mask m(size);
    for (std::size_t i : indices) m.set(i);
    return m;
}

std::size_t Mask::count() const noexcept {
    return static_cast<std::size_t>(std::count(bits_.begin(), bits_.end(), std::uint8_t{1}));
}

Mask& Mask::set(std::size_t i, bool value) noexcept {
    if (i < bits_.size()) bits_[i] = value ? 1 : 0;
    return *this;
}

Mask& Mask::set_range(std::size_t begin, std::size_t count, bool value) noexcept {
    const std::size_t end = std::min(bits_.size(), begin + count);
    for (std::size_t i = begin; i < end; ++i) bits_[i] = value ? 1 : 0;
    return *this;
}

std::vector<std::size_t> Mask::indices() const {
    std::vector<std::size_t> out;
    out.reserve(count());
    for (std::size_t i = 0; i < bits_.size(); ++i)
        if (bits_[i]) out.push_back(i);
    return out;
}

Mask Mask::operator~() const {
    Mask m = *this;
    for (auto& b : m.bits_) b = b ? 0 : 1;
    return m;
}

Mask& Mask::operator|=(const Mask& other) {
    if (other.size() > size()) bits_.resize(other.size(), 0);
    for (std::size_t i = 0; i < other.size(); ++i) bits_[i] = (bits_[i] | other.bits_[i]) ? 1 : 0;
    return *this;
}

Mask& Mask::operator&=(const Mask& other) {
    if (other.size() > size()) bits_.resize(other.size(), 0);
    for (std::size_t i = 0; i < bits_.size(); ++i) bits_[i] = (bits_[i] && other.test(i)) ? 1 : 0;
    return *this;
}

Mask& Mask::operator-=(const Mask& other) {
    if (other.size() > size()) bits_.resize(other.size(), 0);
    for (std::size_t i = 0; i < bits_.size(); ++i) bits_[i] = (bits_[i] && !other.test(i)) ? 1 : 0;
    return *this;
}

}  // namespace kstone
