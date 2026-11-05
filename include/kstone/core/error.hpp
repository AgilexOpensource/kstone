#pragma once

#include "kstone/core/export.hpp"

#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace kstone {

/// Error categories reported at API boundaries. Values are stable so they can cross a C ABI.
enum class ErrorCode : int {
    invalid_argument = 1,  ///< Caller passed a malformed or inconsistent value.
    out_of_range = 2,      ///< Index, size or limit violation.
    not_found = 3,         ///< Named entity or resource does not exist.
    parse_error = 4,       ///< Input document could not be parsed.
    io_error = 5,          ///< Filesystem or stream failure.
    unsupported = 6,       ///< Feature not available in this build or for this input.
    internal = 7,          ///< Unexpected failure inside a dependency.
    infeasible = 8,        ///< Hard constraints admit no solution.
};

KSTONE_EXPORT const char* to_string(ErrorCode code) noexcept;

struct Error {
    ErrorCode code{ErrorCode::internal};
    std::string message;
};

/// Thrown only when a caller dereferences a Result that holds an error (a programming bug).
class KSTONE_EXPORT BadResultAccess : public std::logic_error {
  public:
    explicit BadResultAccess(const Error& error);
    ~BadResultAccess() override;
    BadResultAccess(const BadResultAccess&) = default;
    BadResultAccess& operator=(const BadResultAccess&) = default;
};

/// Value-or-error return type used by every fallible boundary API.
template <typename T>
class [[nodiscard]] Result {
    static_assert(!std::is_same_v<std::decay_t<T>, Error>, "Result<Error> is ambiguous");

  public:
    using value_type = T;

    Result(T value) : storage_(std::in_place_index<0>, std::move(value)) {}      // NOLINT(implicit)
    Result(Error error) : storage_(std::in_place_index<1>, std::move(error)) {}  // NOLINT(implicit)

    bool ok() const noexcept { return storage_.index() == 0; }
    explicit operator bool() const noexcept { return ok(); }

    T& value() & {
        check();
        return std::get<0>(storage_);
    }
    const T& value() const& {
        check();
        return std::get<0>(storage_);
    }
    T&& value() && {
        check();
        return std::get<0>(std::move(storage_));
    }

    T& operator*() & { return value(); }
    const T& operator*() const& { return value(); }
    T&& operator*() && { return std::move(*this).value(); }
    T* operator->() { return &value(); }
    const T* operator->() const { return &value(); }

    template <typename U>
    T value_or(U&& fallback) const& {
        return ok() ? std::get<0>(storage_) : static_cast<T>(std::forward<U>(fallback));
    }

    /// Precondition: !ok().
    const Error& error() const& { return std::get<1>(storage_); }
    Error&& error() && { return std::get<1>(std::move(storage_)); }

  private:
    void check() const {
        if (!ok()) throw BadResultAccess(std::get<1>(storage_));
    }

    std::variant<T, Error> storage_;
};

template <>
class [[nodiscard]] Result<void> {
  public:
    using value_type = void;

    Result() = default;
    Result(Error error) : error_(std::move(error)) {}  // NOLINT(implicit)

    bool ok() const noexcept { return !error_.has_value(); }
    explicit operator bool() const noexcept { return ok(); }

    void value() const {
        if (error_) throw BadResultAccess(*error_);
    }

    /// Precondition: !ok().
    const Error& error() const& { return *error_; }
    Error&& error() && { return std::move(*error_); }

  private:
    std::optional<Error> error_;
};

using Status = Result<void>;

inline Error make_error(ErrorCode code, std::string message) { return Error{code, std::move(message)}; }

}  // namespace kstone
