#include "kstone/core/error.hpp"

namespace kstone {

const char* to_string(ErrorCode code) noexcept {
    switch (code) {
        case ErrorCode::invalid_argument:
            return "invalid_argument";
        case ErrorCode::out_of_range:
            return "out_of_range";
        case ErrorCode::not_found:
            return "not_found";
        case ErrorCode::parse_error:
            return "parse_error";
        case ErrorCode::io_error:
            return "io_error";
        case ErrorCode::unsupported:
            return "unsupported";
        case ErrorCode::internal:
            return "internal";
        case ErrorCode::infeasible:
            return "infeasible";
    }
    return "unknown";
}

BadResultAccess::BadResultAccess(const Error& error)
    : std::logic_error(std::string("kstone: bad Result access (") + to_string(error.code) +
                       "): " + error.message) {}

BadResultAccess::~BadResultAccess() = default;

}  // namespace kstone
