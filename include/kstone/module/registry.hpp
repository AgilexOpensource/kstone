#pragma once

#include "kstone/core/context.hpp"
#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/model/model.hpp"
#include "kstone/module/config.hpp"
#include "kstone/module/module.hpp"
#include "kstone/module/schema.hpp"

#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace kstone {

/// What a factory may use besides its parameters.
struct BuildContext {
    ModelPtr model;
    const Context& context;
    ConfigReadContext refs;          ///< Resolves ids of modules built earlier.
    Eigen::VectorXd initial;         ///< ModuleSet::initial().
    std::filesystem::path base_dir;  ///< Relative file parameters resolve against it first; empty = none.
};

/// Builds one module from its parameter map; `path` prefixes error paths.
using ModuleFactory = std::function<Result<std::shared_ptr<Module>>(
    const std::string& id, const ConfigNode& params, const std::string& path, const BuildContext& build)>;
/// Lists the module ids a parameter map refers to, without resolving them.
using ModuleRefCollector =
    std::function<void(const ConfigNode& params, const std::string& path, std::vector<ModuleRef>& out)>;

/// Module types by name, each with a parameter schema (JSON Schema) and a factory.
class KSTONE_EXPORT ModuleRegistry {
  public:
    /// The 8 built-in types: frame_task, joint_target, gaze, center_of_mass, null_space_preference,
    /// self_collision, world_collision, chassis.
    static ModuleRegistry with_builtins();

    /// invalid_argument on an empty or duplicate type.
    Status add(std::string type, ConfigNode schema, ModuleFactory factory, ModuleRefCollector refs = {});

    /// Reads Cfg (defaults from Cfg{}) and calls make(id, cfg, build).
    template <typename Cfg, typename Make>
    Status add(std::string type, ParamSchema<Cfg> schema, Make make) {
        auto shared = std::make_shared<const ParamSchema<Cfg>>(std::move(schema));
        ConfigNode json = shared->json_schema();
        ModuleFactory factory = [shared, make = std::move(make)](
                                    const std::string& id, const ConfigNode& params, const std::string& path,
                                    const BuildContext& build) -> Result<std::shared_ptr<Module>> {
            Cfg cfg{};
            if (auto s = shared->read(params, cfg, path, build.refs); !s) return std::move(s).error();
            Result<std::shared_ptr<Module>> module = make(id, std::move(cfg), build);
            if (!module)
                return config_error(module.error().code, ConfigDiag::module_failed, params.location(), path,
                                    module.error().message);
            return module;
        };
        ModuleRefCollector refs = [shared](const ConfigNode& params, const std::string& path,
                                           std::vector<ModuleRef>& out) {
            shared->collect_refs(params, path, out);
        };
        return add(std::move(type), std::move(json), std::move(factory), std::move(refs));
    }

    /// M is constructed as M(id, Cfg).
    template <typename M, typename Cfg>
    Status add(std::string type, ParamSchema<Cfg> schema) {
        return add(
            std::move(type), std::move(schema),
            [](const std::string& id, Cfg cfg, const BuildContext&) -> Result<std::shared_ptr<Module>> {
                return std::shared_ptr<Module>(std::make_shared<M>(id, std::move(cfg)));
            });
    }

    bool contains(const std::string& type) const;
    /// Sorted.
    std::vector<std::string> types() const;

    /// not_found on an unknown type; the module's type() must equal `type`.
    Result<std::shared_ptr<Module>> create(const std::string& type, const std::string& id,
                                           const ConfigNode& params, const BuildContext& build,
                                           const std::string& path = {}) const;
    /// JSON Schema (draft 2020-12) of the parameter map.
    Result<ConfigNode> schema(const std::string& type) const;
    /// Module ids `params` refers to (unresolved); malformed values are left to create().
    Result<std::vector<ModuleRef>> references(const std::string& type, const ConfigNode& params,
                                              const std::string& path = {}) const;

    /// Builds every entry of `modules` (id -> {type, ...params}) into `set`, referenced modules first.
    /// Does not call set.configure(). not_found on an unknown type or id, invalid_argument on a cycle.
    Status populate(ModuleSet& set, const ConfigNode& modules, const Context& context,
                    const std::string& path = "modules", const std::filesystem::path& base_dir = {}) const;

  private:
    struct Entry {
        ConfigNode schema;
        ModuleFactory factory;
        ModuleRefCollector refs;
    };
    const Entry* find(const std::string& type) const;
    std::map<std::string, Entry> entries_;
};

}  // namespace kstone
