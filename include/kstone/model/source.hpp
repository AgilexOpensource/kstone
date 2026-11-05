#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <utility>

namespace kstone {

/// Where a model description comes from. The format is a lowercase tag ("urdf", "mjcf", ...);
/// empty means "infer from the file extension".
struct ModelSource {
    enum class Kind { file, text };

    Kind kind{Kind::file};
    std::string location;  ///< Kind::file: path or URI resolved through Context::resources.
    std::string content;   ///< Kind::text: the document itself.
    std::string format;
    std::filesystem::path base_dir;  ///< For relative references inside the document.

    static ModelSource file(std::string location, std::string format = {}) {
        ModelSource s;
        s.kind = Kind::file;
        s.location = std::move(location);
        s.format = std::move(format);
        return s;
    }
    static ModelSource text(std::string content, std::string format, std::filesystem::path base_dir = {}) {
        ModelSource s;
        s.kind = Kind::text;
        s.content = std::move(content);
        s.format = std::move(format);
        s.base_dir = std::move(base_dir);
        return s;
    }
};

enum class RootJoint {
    fixed,       ///< Model root welded to the world.
    free_flyer,  ///< 6-DoF. q stores (x, y, z, qx, qy, qz, qw).
};

enum class MimicMode {
    independent,  ///< Mimic followers stay ordinary DoFs; relations are reported via Model::mimics().
    coupled,      ///< Followers are driven by their leader and add no DoFs (format support permitting).
};

struct ModelOptions {
    RootJoint root{RootJoint::fixed};
    std::string root_joint_name{"root_joint"};
    MimicMode mimic{MimicMode::independent};
};

}  // namespace kstone
