#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "impl.hpp"
#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <random>
#include <set>
#include <urdf_parser/urdf_parser.h>

namespace kstone {

namespace {

std::string lowercase(std::string_view text) {
    std::string out(text);
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

// ---------------------------------------------------------------------------------------------
// URDF

// Joint names in the subtree below `link` (the joints to its children included).
void collect_joints(const ::urdf::Link& link, std::set<std::string>& out) {
    for (const auto& joint : link.child_joints) out.insert(joint->name);
    for (const auto& child : link.child_links) collect_joints(*child, out);
}

// urdfdom orders sibling joints by name and Pinocchio adds a coupled mimic joint only after its
// leader, so a follower may come first. Reorders the children of every link, stably, so that a
// subtree holding a leader precedes the sibling subtrees holding its followers.
Status order_mimic_leaders(::urdf::Link& link, const std::map<std::string, MimicRelation>& mimics) {
    const std::size_t k = link.child_links.size();
    if (k > 1) {
        std::vector<std::set<std::string>> joints(k);
        for (std::size_t i = 0; i < k; ++i) {
            const auto& child = *link.child_links[i];
            if (child.parent_joint) joints[i].insert(child.parent_joint->name);
            collect_joints(child, joints[i]);
        }
        // after[i][j]: subtree j holds a follower of a leader in subtree i.
        std::vector<std::vector<bool>> after(k, std::vector<bool>(k, false));
        std::vector<int> pending(k, 0);
        for (std::size_t j = 0; j < k; ++j) {
            for (const auto& name : joints[j]) {
                const auto it = mimics.find(name);
                if (it == mimics.end()) continue;
                for (std::size_t i = 0; i < k; ++i) {
                    if (i != j && !after[i][j] && joints[i].count(it->second.leader)) {
                        after[i][j] = true;
                        ++pending[j];
                    }
                }
            }
        }
        std::vector<std::size_t> order;
        std::vector<bool> placed(k, false);
        while (order.size() < k) {
            std::size_t next = k;
            for (std::size_t i = 0; i < k && next == k; ++i)
                if (!placed[i] && pending[i] == 0) next = i;
            if (next == k)
                return make_error(ErrorCode::invalid_argument, "mimic joints below link " + link.name +
                                                                   " depend on each other in a cycle");
            placed[next] = true;
            order.push_back(next);
            for (std::size_t j = 0; j < k; ++j)
                if (after[next][j]) --pending[j];
        }
        // urdfdom fills child_links and child_joints together; keep each child with its joint.
        std::vector<::urdf::LinkSharedPtr> links;
        std::vector<::urdf::JointSharedPtr> joints_out;
        for (std::size_t i : order) {
            links.push_back(link.child_links[i]);
            for (const auto& joint : link.child_joints)
                if (joint->child_link_name == link.child_links[i]->name) joints_out.push_back(joint);
        }
        if (joints_out.size() == link.child_joints.size()) link.child_joints = std::move(joints_out);
        link.child_links = std::move(links);
    }
    for (const auto& child : link.child_links) {
        if (Status s = order_mimic_leaders(*child, mimics); !s.ok()) return s;
    }
    return {};
}

class UrdfLoader final : public ModelLoader {
  public:
    std::string format() const override { return "urdf"; }
    std::vector<std::string> extensions() const override { return {"urdf"}; }

    Result<LoadedModel> load(const LoadRequest& request) const override {
        std::string text;
        if (request.source.kind == ModelSource::Kind::file) {
            auto read = read_text_file(request.path);
            if (!read) return std::move(read).error();
            text = std::move(*read);
        } else {
            text = std::string(request.text);
        }
        const ::urdf::ModelInterfaceSharedPtr tree = ::urdf::parseURDF(text);
        if (!tree) return make_error(ErrorCode::parse_error, "invalid URDF document");

        LoadedModel out;
        // std::map: deterministic order independent of the parser's container.
        std::map<std::string, MimicRelation> mimics;
        for (const auto& [name, joint] : tree->joints_) {
            if (!joint || !joint->mimic) continue;
            mimics[name] =
                MimicRelation{name, joint->mimic->joint_name, joint->mimic->multiplier, joint->mimic->offset};
        }
        const bool coupled = request.options.mimic == MimicMode::coupled;
        if (coupled && !mimics.empty() && tree->root_link_) {
            if (Status s = order_mimic_leaders(*tree->root_link_, mimics); !s.ok())
                return std::move(s).error();
        }
        for (auto& [name, relation] : mimics) out.mimics.push_back(std::move(relation));

        if (auto root = root_joint_model(request.options.root)) {
            pinocchio::urdf::buildModel(tree, *root, request.options.root_joint_name, out.model, false,
                                        coupled);
        } else {
            pinocchio::urdf::buildModel(tree, out.model, false, coupled);
        }
        return out;
    }
};

// ---------------------------------------------------------------------------------------------
// MJCF

/// Removes a temporary file on scope exit.
class TempFile {
  public:
    TempFile() {
        std::random_device rd;
        std::uniform_int_distribution<unsigned long long> dist;
        path_ = std::filesystem::temp_directory_path() / ("kstone-" + std::to_string(dist(rd)) + ".xml");
    }
    ~TempFile() {
        std::error_code ec;
        std::filesystem::remove(path_, ec);
    }
    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;
    const std::filesystem::path& path() const { return path_; }

  private:
    std::filesystem::path path_;
};

class MjcfLoader final : public ModelLoader {
  public:
    std::string format() const override { return "mjcf"; }
    std::vector<std::string> extensions() const override { return {"mjcf", "xml"}; }

    Result<LoadedModel> load(const LoadRequest& request) const override {
        const auto& log = request.context.log;
        if (request.options.mimic == MimicMode::coupled && log.enabled(LogLevel::debug))
            log.log(LogLevel::debug, "model.mjcf", "coupled mimic mode has no effect for MJCF sources");

        // The Pinocchio MJCF parser only reads files; text sources go through a private temp file.
        std::optional<TempFile> temp;
        std::filesystem::path path = request.path;
        if (request.source.kind == ModelSource::Kind::text) {
            if (!request.source.base_dir.empty() && log.enabled(LogLevel::warn))
                log.log(LogLevel::warn, "model.mjcf",
                        "base_dir is not applied to relative includes of MJCF text");
            temp.emplace();
            std::ofstream stream(temp->path(), std::ios::binary);
            stream.write(request.text.data(), static_cast<std::streamsize>(request.text.size()));
            stream.close();
            if (!stream) return make_error(ErrorCode::io_error, "cannot write temporary MJCF file");
            path = temp->path();
        }

        LoadedModel out;
        if (auto root = root_joint_model(request.options.root)) {
            pinocchio::mjcf::buildModel(path.string(), *root, request.options.root_joint_name, out.model,
                                        false);
        } else {
            pinocchio::mjcf::buildModel(path.string(), out.model, false);
        }
        return out;
    }
};

}  // namespace

// ---------------------------------------------------------------------------------------------
// registry

ModelLoader::~ModelLoader() = default;

ModelLoaderRegistry::ModelLoaderRegistry() = default;
ModelLoaderRegistry::~ModelLoaderRegistry() = default;
ModelLoaderRegistry::ModelLoaderRegistry(const ModelLoaderRegistry&) = default;
ModelLoaderRegistry& ModelLoaderRegistry::operator=(const ModelLoaderRegistry&) = default;
ModelLoaderRegistry::ModelLoaderRegistry(ModelLoaderRegistry&&) noexcept = default;
ModelLoaderRegistry& ModelLoaderRegistry::operator=(ModelLoaderRegistry&&) noexcept = default;

ModelLoaderRegistry ModelLoaderRegistry::builtin() {
    ModelLoaderRegistry registry;
    registry.add(std::make_shared<UrdfLoader>());
    registry.add(std::make_shared<MjcfLoader>());
    return registry;
}

ModelLoaderRegistry& ModelLoaderRegistry::add(std::shared_ptr<const ModelLoader> loader) {
    if (loader) loaders_.push_back(std::move(loader));
    return *this;
}

std::shared_ptr<const ModelLoader> ModelLoaderRegistry::find_format(std::string_view format) const {
    const std::string key = lowercase(format);
    for (auto it = loaders_.rbegin(); it != loaders_.rend(); ++it)
        if (lowercase((*it)->format()) == key) return *it;
    return nullptr;
}

std::shared_ptr<const ModelLoader> ModelLoaderRegistry::find_extension(std::string_view extension) const {
    if (!extension.empty() && extension.front() == '.') extension.remove_prefix(1);
    const std::string key = lowercase(extension);
    for (auto it = loaders_.rbegin(); it != loaders_.rend(); ++it)
        for (const auto& ext : (*it)->extensions())
            if (lowercase(ext) == key) return *it;
    return nullptr;
}

std::vector<std::string> ModelLoaderRegistry::formats() const {
    std::vector<std::string> out;
    for (const auto& loader : loaders_) {
        std::string f = lowercase(loader->format());
        if (std::find(out.begin(), out.end(), f) == out.end()) out.push_back(std::move(f));
    }
    return out;
}

// ---------------------------------------------------------------------------------------------
// Model::load

Result<std::shared_ptr<const Model>> Model::load(const ModelSource& source, const ModelOptions& options,
                                                 const Context& context) {
    LoadRequest request{source, options, context, {}, {}};
    std::shared_ptr<const ModelLoader> loader;

    if (source.kind == ModelSource::Kind::file) {
        auto path = context.resources.resolve(source.location, source.base_dir);
        if (!path) return std::move(path).error();
        request.path = std::move(*path);
        if (std::error_code ec; std::filesystem::is_directory(request.path, ec))
            return make_error(ErrorCode::io_error, request.path.string() + " is a directory");
        loader = source.format.empty() ? context.loaders.find_extension(request.path.extension().string())
                                       : context.loaders.find_format(source.format);
    } else {
        if (source.format.empty())
            return make_error(ErrorCode::invalid_argument, "text model sources require an explicit format");
        request.text = source.content;
        loader = context.loaders.find_format(source.format);
    }
    if (!loader) {
        const std::string what = !source.format.empty()
                                     ? "format '" + source.format + "'"
                                     : "extension '" + request.path.extension().string() + "'";
        return make_error(ErrorCode::unsupported, "no model loader for " + what);
    }

    Result<LoadedModel> loaded = make_error(ErrorCode::internal, "loader did not run");
    try {
        loaded = loader->load(request);
    } catch (const std::exception& e) {
        return make_error(ErrorCode::parse_error, loader->format() + ": " + e.what());
    }
    if (!loaded) return std::move(loaded).error();

    auto model = make_model(std::move(loaded->model), std::move(loaded->mimics));
    if (!model) return model;
    // A fixed request accepts whatever root the document declares (e.g. a URDF floating joint).
    if (options.root != RootJoint::fixed && (*model)->root() != options.root)
        return make_error(
            ErrorCode::unsupported,
            "loader '" + loader->format() +
                "' did not apply the requested root joint (the document may already define how its "
                "top-level body attaches to the world)");

    ModelSource resolved = source;
    resolved.format = lowercase(loader->format());
    if (source.kind == ModelSource::Kind::file) {
        std::error_code ec;
        std::filesystem::path abs = std::filesystem::absolute(request.path, ec);
        if (ec) abs = request.path;
        resolved.location = abs.lexically_normal().string();
        resolved.base_dir = abs.parent_path();
    }
    ModelFactory::set_source(**model, std::move(resolved));

    const auto& log = context.log;
    if (log.enabled(LogLevel::debug))
        log.log(LogLevel::debug, "model",
                "loaded '" + (*model)->name() + "' nq=" + std::to_string((*model)->nq()) +
                    " nv=" + std::to_string((*model)->nv()));
    return model;
}

}  // namespace kstone
