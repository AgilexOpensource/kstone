#pragma once

#include "kstone/core/export.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace kstone {

/// Defined in "kstone/model/pinocchio.hpp"; only loader authors need the full type.
class ModelLoader;

/// Format-tag -> loader table. Value type; copy builtin(), add loaders, store in Context::loaders.
class KSTONE_EXPORT ModelLoaderRegistry {
  public:
    ModelLoaderRegistry();
    ~ModelLoaderRegistry();
    ModelLoaderRegistry(const ModelLoaderRegistry&);
    ModelLoaderRegistry& operator=(const ModelLoaderRegistry&);
    ModelLoaderRegistry(ModelLoaderRegistry&&) noexcept;
    ModelLoaderRegistry& operator=(ModelLoaderRegistry&&) noexcept;

    /// Registry containing the loaders compiled into this build ("urdf", "mjcf").
    static ModelLoaderRegistry builtin();

    /// Later registrations take precedence for the same format or extension.
    ModelLoaderRegistry& add(std::shared_ptr<const ModelLoader> loader);

    std::shared_ptr<const ModelLoader> find_format(std::string_view format) const;
    /// `extension` with or without the leading dot, case-insensitive.
    std::shared_ptr<const ModelLoader> find_extension(std::string_view extension) const;
    std::vector<std::string> formats() const;

  private:
    std::vector<std::shared_ptr<const ModelLoader>> loaders_;
};

}  // namespace kstone
