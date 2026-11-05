#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/module/config.hpp"

#include <filesystem>
#include <map>
#include <string>
#include <string_view>

namespace kstone {

class ResourceResolver;

/// Opt-in preprocessing for configuration files. Defaults disable both features, so untrusted
/// text (e.g. debug-server commands) never reads files or the environment.
struct YamlOptions {
    /// Expand `${name}` / `${name:-default}` in scalars; `$${` is a literal `${`. As in the shell, an empty
    /// value also takes the default.
    /// A plain scalar that is exactly one `${...}` is re-typed (bool/number); otherwise the result is a string.
    bool expand_vars{false};
    /// Looked up first; then the process environment when `environment` is set; then the default.
    /// Every entry must be referenced, otherwise parsing fails with KST-E010 (catches misspelled names).
    std::map<std::string, std::string> vars;
    bool environment{false};
    /// Replace `!include <path>` scalars with the referenced document. Relative paths resolve
    /// against the including file's directory, then `resources` (search dirs, schemes) when given.
    bool includes{false};
    const ResourceResolver* resources{nullptr};
    /// Directory used for includes in text parsed by parse_yaml (load_yaml uses the file's directory).
    std::filesystem::path base_dir;
    int max_include_depth{16};

    /// includes + vars + environment: the setting for configuration files on disk.
    static YamlOptions files() {
        YamlOptions o;
        o.expand_vars = true;
        o.environment = true;
        o.includes = true;
        return o;
    }
};

/// Parses one YAML document into a ConfigNode with source locations; `file` only labels them.
/// Quoted scalars stay strings; plain ones become null (~, null, empty), bool (true/false), number or string.
/// parse_error on malformed YAML, non-scalar or duplicate keys, bad tags, include or variable failures;
/// messages are "file:line:col: message [KST-E0xx]".
KSTONE_EXPORT Result<ConfigNode> parse_yaml(std::string_view text, const std::string& file = {},
                                            const YamlOptions& options = {});
/// io_error when the file cannot be read.
KSTONE_EXPORT Result<ConfigNode> load_yaml(const std::filesystem::path& path,
                                           const YamlOptions& options = {});

}  // namespace kstone
