#include "kstone/core/version.hpp"

namespace kstone {

Version library_version() noexcept {
    return {KSTONE_VERSION_MAJOR, KSTONE_VERSION_MINOR, KSTONE_VERSION_PATCH};
}

const char* library_version_string() noexcept { return KSTONE_VERSION_STRING; }

}  // namespace kstone
