#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"

#include <cstddef>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace kstone {

/// Where a configuration value came from; line/column are 1-based, 0 = unknown.
struct SourceLocation {
    std::string file;
    int line{0};
    int column{0};
};

/// Format-independent configuration tree (null, bool, number, string, list, map).
/// Maps keep insertion order; set() replaces an existing key.
class KSTONE_EXPORT ConfigNode {
  public:
    enum class Kind { null, boolean, number, string, list, map };

    ConfigNode() = default;
    ConfigNode(bool value);         // NOLINT(implicit)
    ConfigNode(int value);          // NOLINT(implicit)
    ConfigNode(double value);       // NOLINT(implicit)
    ConfigNode(const char* value);  // NOLINT(implicit)
    ConfigNode(std::string value);  // NOLINT(implicit)

    static ConfigNode list(std::vector<ConfigNode> items = {});
    static ConfigNode map(std::vector<std::pair<std::string, ConfigNode>> entries = {});

    Kind kind() const noexcept { return kind_; }
    bool is_null() const noexcept { return kind_ == Kind::null; }

    /// Valid only for the matching kind().
    bool as_bool() const noexcept { return bool_; }
    double as_number() const noexcept { return number_; }
    const std::string& as_string() const noexcept { return string_; }
    const std::vector<ConfigNode>& items() const noexcept { return items_; }
    const std::vector<std::pair<std::string, ConfigNode>>& entries() const noexcept { return entries_; }

    /// List or map element count; 0 otherwise.
    std::size_t size() const noexcept;
    /// Map lookup; null when absent or not a map.
    const ConfigNode* find(std::string_view key) const noexcept;

    /// Turns a null node into a list.
    ConfigNode& append(ConfigNode value);
    /// Turns a null node into a map.
    ConfigNode& set(std::string key, ConfigNode value);

    const SourceLocation& location() const noexcept { return location_; }
    ConfigNode& set_location(SourceLocation location);

  private:
    Kind kind_{Kind::null};
    bool bool_{false};
    double number_{0.0};
    std::string string_;
    std::vector<ConfigNode> items_;
    std::vector<std::pair<std::string, ConfigNode>> entries_;
    SourceLocation location_;
};

KSTONE_EXPORT const char* to_string(ConfigNode::Kind kind) noexcept;

/// Stable configuration diagnostic numbers, printed as "KST-Exxx". Never renumber; add new ones at the end
/// of their group.
enum class ConfigDiag : int {
    // Documents (E0xx).
    syntax = 1,          ///< Malformed YAML.
    duplicate_key = 2,   ///< Same key twice in one map.
    bad_key = 3,         ///< Non-scalar or merge (`<<`) map key.
    bad_tag = 4,         ///< Unsupported tag, or !include on a non-scalar.
    too_large = 5,       ///< Node, depth or include limit exceeded.
    include_failed = 6,  ///< Included (or top-level) file cannot be resolved or read.
    include_cycle = 7,   ///< A file includes itself, directly or indirectly.
    undefined_var = 8,   ///< ${name} without a value or default.
    bad_var = 9,         ///< Malformed ${...}.
    unused_var = 10,     ///< An explicitly set variable is never referenced.
    // Values (E1xx).
    wrong_type = 101,  ///< "expected X, got Y".
    unknown_key = 102,
    missing_key = 103,
    invalid_value = 104,   ///< Out of range, non-finite or otherwise inconsistent.
    invalid_choice = 105,  ///< Not one of the enum names.
    not_tunable = 106,     ///< Runtime patch of a field that is fixed after construction.
    // Modules (E2xx).
    unknown_module_type = 201,
    missing_module_type = 202,
    unknown_module = 203,  ///< Reference to a module id that does not exist.
    wrong_module_type = 204,
    module_cycle = 205,
    module_failed = 206,  ///< The module rejected its (well-formed) parameters.
    // Engine (E3xx).
    model_failed = 301,    ///< model.file cannot be loaded.
    robot_mismatch = 302,  ///< robot section does not fit the model.
    robot_failed = 303,    ///< Robot plugin cannot be loaded or refused to start.
};

/// "KST-E104".
KSTONE_EXPORT std::string to_string(ConfigDiag diag);

/// "file:line:col: path: message [KST-Exxx]" (location and path omitted when unknown).
KSTONE_EXPORT Error config_error(ErrorCode code, ConfigDiag diag, const SourceLocation& location,
                                 const std::string& path, const std::string& message);

/// JSON text; non-finite numbers become null. indent <= 0 writes a single line.
KSTONE_EXPORT std::string to_json(const ConfigNode& node, int indent = 2);

/// "a" + "b" -> "a.b"; "" + "b" -> "b".
KSTONE_EXPORT std::string config_path(const std::string& parent, const std::string& key);
/// "a" + 2 -> "a[2]".
KSTONE_EXPORT std::string config_path(const std::string& parent, std::size_t index);

}  // namespace kstone
