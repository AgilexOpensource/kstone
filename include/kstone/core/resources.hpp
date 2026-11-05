#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"

#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace kstone {

/// Splits "scheme://rest". Returns empty scheme for plain paths (including Windows drive letters).
struct ParsedUri {
    std::string scheme;
    std::string rest;
};
KSTONE_EXPORT ParsedUri parse_uri(std::string_view uri);

/// Maps a scheme-specific remainder (the part after "scheme://") to a local path.
using SchemeHandler = std::function<Result<std::filesystem::path>(std::string_view rest)>;

/// Resolves resource references (model files, meshes, includes) to local filesystem paths.
///
/// Resolution order for an input `uri` with an optional `base_dir`:
///   1. "scheme://..." -> registered handler for that scheme ("file" is built in), else not_found.
///   2. absolute path  -> used as is if it exists.
///   3. relative path  -> base_dir / uri, then each search dir in insertion order.
///
/// The resolver itself has no knowledge of any particular ecosystem; hosts register handlers.
/// Not thread-safe for mutation; concurrent resolve() calls on an unmodified resolver are safe.
class KSTONE_EXPORT ResourceResolver {
  public:
    ResourceResolver();

    /// Replaces any existing handler for `scheme` (case-sensitive, without "://").
    ResourceResolver& add_scheme(std::string scheme, SchemeHandler handler);
    bool has_scheme(std::string_view scheme) const noexcept;
    ResourceResolver& add_search_dir(std::filesystem::path dir);
    const std::vector<std::filesystem::path>& search_dirs() const noexcept { return search_dirs_; }

    Result<std::filesystem::path> resolve(std::string_view uri,
                                          const std::filesystem::path& base_dir = {}) const;
    Result<std::string> read_text(std::string_view uri, const std::filesystem::path& base_dir = {}) const;

  private:
    struct Entry {
        std::string scheme;
        SchemeHandler handler;
    };
    std::vector<Entry> schemes_;
    std::vector<std::filesystem::path> search_dirs_;
};

/// Generic "scheme://<name>/<sub/path>" handler: looks for <root>/<name>/<sub/path> in each root,
/// in order. Works for any package-like layout; roots are supplied by the caller.
KSTONE_EXPORT SchemeHandler make_directory_scheme(std::vector<std::filesystem::path> roots);

/// Splits a PATH-style list (':' on POSIX, ';' on Windows) into paths, dropping empty entries.
KSTONE_EXPORT std::vector<std::filesystem::path> split_path_list(std::string_view list);

KSTONE_EXPORT Result<std::string> read_text_file(const std::filesystem::path& path);

}  // namespace kstone
