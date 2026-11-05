#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/model/model.hpp"
#include "kstone/runtime/robot.hpp"

#include <Eigen/Core>

#include <filesystem>
#include <memory>
#include <string>

namespace kstone {

struct RobotPluginOptions {
    /// Shared library implementing kstone/runtime/robot_plugin.h.
    std::filesystem::path library;
    /// JSON object handed to the plugin as params_json.
    std::string params_json = "{}";
    std::filesystem::path base_dir;
    double rate_hz{100.0};
    /// Initial q (size nq) offered to the plugin; empty = model neutral.
    Eigen::VectorXd initial;
};

/// Loads a C-ABI robot driver. The library stays loaded until the returned robot is destroyed.
/// Loading runs the library's code: only use trusted libraries.
KSTONE_EXPORT Result<std::shared_ptr<RobotInterface>> load_robot_plugin(const ModelPtr& model,
                                                                        const RobotPluginOptions& options);

}  // namespace kstone
