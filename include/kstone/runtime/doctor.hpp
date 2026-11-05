#pragma once

#include "kstone/core/export.hpp"
#include "kstone/module/config.hpp"

#include <string>
#include <vector>

namespace kstone {

struct DoctorCheck {
    std::string name;
    bool ok{true};
    std::string detail;
};

/// A native library mapped into the process.
struct LoadedLibrary {
    std::string name;  ///< File name, e.g. libpinocchio_default.so.4.1.0.
    std::string path;
    std::string provider;  ///< pypi, ros, system or other.
};

struct DoctorReport {
    std::string version;
    std::string build;     ///< release or debug, plus sanitizer.
    bool shared{false};    ///< kstone built as a shared library.
    std::string location;  ///< File holding the kstone code (library, executable or extension).
    std::vector<std::pair<std::string, std::string>>
        compiled;                          ///< Dependency versions kstone was built against.
    std::vector<LoadedLibrary> libraries;  ///< Empty where the loaded-library scan is unsupported.
    std::vector<DoctorCheck> checks;

    bool ok() const noexcept {
        for (const auto& c : checks)
            if (!c.ok) return false;
        return true;
    }
};

/// Environment self-check: build info, loaded dependency libraries and a smoke solve / collision query.
KSTONE_EXPORT DoctorReport doctor();

/// Install prefix classification of a library path.
KSTONE_EXPORT std::string library_provider(const std::string& path);
/// Fails when one library is loaded from two files or Pinocchio's libraries come from different providers.
KSTONE_EXPORT DoctorCheck check_libraries(const std::vector<LoadedLibrary>& libraries);

KSTONE_EXPORT ConfigNode to_config_node(const DoctorReport& report);
KSTONE_EXPORT std::string to_text(const DoctorReport& report);

}  // namespace kstone
