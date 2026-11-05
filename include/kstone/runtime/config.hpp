#pragma once

#include "kstone/core/context.hpp"
#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/module/config.hpp"
#include "kstone/module/registry.hpp"
#include "kstone/module/yaml.hpp"
#include "kstone/runtime/engine.hpp"
#include "kstone/runtime/robot.hpp"

#include <filesystem>
#include <memory>

namespace kstone {

struct EngineLoadOptions {
    /// Module types; null uses ModuleRegistry::with_builtins().
    const ModuleRegistry* registry{nullptr};
    /// Replaces the configured `robot` section (hardware drivers, external simulators).
    std::shared_ptr<RobotInterface> robot;
    /// Preprocessing used by load_engine (includes, ${vars}, environment). A null resolver uses the context's.
    YamlOptions yaml = YamlOptions::files();
};

/// Builds an engine from a configuration document:
///   model:   {file, format, root: fixed|free_flyer, mimic: independent|coupled}
///   rate_hz, realtime
///   solver:  {mode, damping, limit_gain, feasibility_tol, check_feasibility, hold_retry}
///   robot:   {type: loopback|plugin, initial: [q...], library, params: {...}}   (initial empty = model neutral;
///            plugin: see kstone/runtime/robot_plugin.h)
///   modules: {<id>: {type: <module type>, ...params}}
/// Relative model paths resolve against base_dir, then the context's search dirs.
KSTONE_EXPORT Result<Engine> build_engine(const ConfigNode& config, const std::filesystem::path& base_dir,
                                          const EngineLoadOptions& options = {},
                                          const Context& context = Context::defaults());

/// load_yaml(path, options.yaml) followed by build_engine relative to the file's directory.
KSTONE_EXPORT Result<Engine> load_engine(const std::filesystem::path& path,
                                         const EngineLoadOptions& options = {},
                                         const Context& context = Context::defaults());

}  // namespace kstone
