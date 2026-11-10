#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/resources.hpp"
#include "kstone/model/source.hpp"
#include "kstone/module/config.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace kstone::detail {

/// Files a remote client needs to render a URDF model (the document plus the meshes and the
/// materials/textures they reference), keyed by bundle-relative names. Only listed files are served.
struct ModelAssets {
    struct File {
        std::string name;            ///< Bundle-relative, '/'-separated.
        std::filesystem::path path;  ///< Empty for the rewritten document itself.
        std::string stamp;  ///< Changes when the file does: size and mtime, or a content hash for `model`.
        std::uint64_t size{0};
        std::filesystem::file_time_type mtime{};
    };

    std::string id;                    ///< Hash of the manifest; unchanged models keep their id.
    std::string model{"model.urdf"};   ///< Name of the rewritten document.
    std::string document;              ///< URDF with every filename="..." pointing into the bundle.
    std::vector<File> files;           ///< Sorted by name; includes `model`.
    std::vector<std::string> missing;  ///< References the resolver could not find.

    /// {id, model, files: {name: {stamp, size}}, missing: [...]}.
    ConfigNode describe() const;
    /// Up to `max_bytes` from `offset`; `total` is the file size. Errors for unlisted or changed files.
    Status read(const std::string& name, std::uint64_t offset, std::size_t max_bytes, std::string& out,
                std::uint64_t& total) const;
};

/// URDF sources only (file or text); mesh references resolve through `resolver` relative to the document.
Result<ModelAssets> build_model_assets(const ModelSource& source, const ResourceResolver& resolver);

std::string sha256_hex(const std::string& data);
std::string base64_encode(const std::string& data);

}  // namespace kstone::detail
