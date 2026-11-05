#pragma once

/// Optional direct access to the underlying Pinocchio objects, and the loader extension point.
/// Including this header makes the consumer depend on Pinocchio's headers (find_package(pinocchio)).

#if __has_include(<pinocchio/multibody.hpp>)  // Pinocchio >= 4
#include <pinocchio/multibody.hpp>
#else
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#endif

#include "kstone/core/context.hpp"
#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace kstone {

/// Input handed to a ModelLoader. For file sources `path` is already resolved and `text` is empty.
struct LoadRequest {
    const ModelSource& source;
    const ModelOptions& options;
    const Context& context;
    std::filesystem::path path;  ///< Resolved local path (Kind::file).
    std::string_view text;       ///< Document content (Kind::text).
};

struct LoadedModel {
    pinocchio::Model model;
    std::vector<MimicRelation> mimics;
};

/// Extension point for model formats. Implementations must be stateless or thread-safe.
class KSTONE_EXPORT ModelLoader {
  public:
    virtual ~ModelLoader();
    virtual std::string format() const = 0;                   ///< Lowercase tag, e.g. "sdf".
    virtual std::vector<std::string> extensions() const = 0;  ///< Without dot, lowercase.
    virtual Result<LoadedModel> load(const LoadRequest& request) const = 0;
};

/// Wraps an existing Pinocchio model. The root kind is inferred from joint 1: a free-flyer
/// joint attached to the universe is treated as the root joint, anything else as a fixed base.
/// Mimic relations naming unknown joints are rejected.
KSTONE_EXPORT Result<ModelPtr> make_model(pinocchio::Model model, std::vector<MimicRelation> mimics = {});

/// Helper for loaders: the Pinocchio root joint matching `root` (nullopt for RootJoint::fixed).
KSTONE_EXPORT std::optional<pinocchio::JointModel> root_joint_model(RootJoint root);

KSTONE_EXPORT const pinocchio::Model& pinocchio_model(const Model& model) noexcept;
KSTONE_EXPORT pinocchio::Data& pinocchio_data(Workspace& workspace) noexcept;
KSTONE_EXPORT const pinocchio::Data& pinocchio_data(const Workspace& workspace) noexcept;

}  // namespace kstone
