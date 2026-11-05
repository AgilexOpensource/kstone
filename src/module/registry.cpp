#include "kstone/module/registry.hpp"

#include <algorithm>

namespace kstone {

Status ModuleRegistry::add(std::string type, ConfigNode schema, ModuleFactory factory,
                           ModuleRefCollector refs) {
    if (type.empty()) return make_error(ErrorCode::invalid_argument, "module type must not be empty");
    if (!factory) return make_error(ErrorCode::invalid_argument, "module type '" + type + "' has no factory");
    if (entries_.count(type))
        return make_error(ErrorCode::invalid_argument, "duplicate module type '" + type + "'");
    entries_.emplace(std::move(type), Entry{std::move(schema), std::move(factory), std::move(refs)});
    return {};
}

const ModuleRegistry::Entry* ModuleRegistry::find(const std::string& type) const {
    auto it = entries_.find(type);
    return it == entries_.end() ? nullptr : &it->second;
}

bool ModuleRegistry::contains(const std::string& type) const { return find(type) != nullptr; }

std::vector<std::string> ModuleRegistry::types() const {
    std::vector<std::string> out;
    out.reserve(entries_.size());
    for (const auto& [type, entry] : entries_) out.push_back(type);
    return out;
}

namespace {
std::string unknown_type_message(const std::string& type, const std::vector<std::string>& known) {
    std::string out = "unknown module type '" + type + "' (known:";
    for (std::size_t i = 0; i < known.size(); ++i) out += (i ? ", " : " ") + known[i];
    return out + ")";
}

}  // namespace

Result<std::shared_ptr<Module>> ModuleRegistry::create(const std::string& type, const std::string& id,
                                                       const ConfigNode& params, const BuildContext& build,
                                                       const std::string& path) const {
    const Entry* entry = find(type);
    if (!entry) return make_error(ErrorCode::not_found, unknown_type_message(type, types()));
    if (id.empty()) return make_error(ErrorCode::invalid_argument, "module id must not be empty");
    auto module = entry->factory(id, params, path, build);
    if (!module) return module;
    if (!*module || (*module)->type() != type)
        return make_error(ErrorCode::internal, "factory for '" + type + "' built a module of another type");
    return module;
}

Result<ConfigNode> ModuleRegistry::schema(const std::string& type) const {
    const Entry* entry = find(type);
    if (!entry) return make_error(ErrorCode::not_found, unknown_type_message(type, types()));
    return entry->schema;
}

Result<std::vector<ModuleRef>> ModuleRegistry::references(const std::string& type, const ConfigNode& params,
                                                          const std::string& path) const {
    const Entry* entry = find(type);
    if (!entry) return make_error(ErrorCode::not_found, unknown_type_message(type, types()));
    std::vector<ModuleRef> out;
    if (entry->refs) entry->refs(params, path, out);
    return out;
}

Status ModuleRegistry::populate(ModuleSet& set, const ConfigNode& modules, const Context& context,
                                const std::string& path, const std::filesystem::path& base_dir) const {
    if (modules.is_null()) return {};
    if (modules.kind() != ConfigNode::Kind::map)
        return config_error(ErrorCode::invalid_argument, ConfigDiag::wrong_type, modules.location(), path,
                            "expected a map of module ids, got " + std::string(to_string(modules.kind())));

    struct Pending {
        const std::string* id;
        std::string type;
        ConfigNode params;
        std::string path;
        SourceLocation location;
        std::vector<ModuleRef> refs;
    };
    std::vector<Pending> pending;
    for (const auto& [id, node] : modules.entries()) {
        const std::string at = config_path(path, id);
        if (!valid_module_id(id))
            return config_error(ErrorCode::invalid_argument, ConfigDiag::invalid_value, node.location(), at,
                                "invalid module id '" + id + "' (" + kModuleIdRule + ")");
        if (node.kind() != ConfigNode::Kind::map)
            return config_error(ErrorCode::invalid_argument, ConfigDiag::wrong_type, node.location(), at,
                                "expected a map, got " + std::string(to_string(node.kind())));
        const ConfigNode* type = node.find("type");
        if (!type)
            return config_error(ErrorCode::invalid_argument, ConfigDiag::missing_module_type, node.location(),
                                at, "missing 'type'");
        if (type->kind() != ConfigNode::Kind::string)
            return config_error(ErrorCode::invalid_argument, ConfigDiag::wrong_type, type->location(),
                                config_path(at, "type"),
                                "expected a string, got " + std::string(to_string(type->kind())));
        if (!contains(type->as_string())) {
            return config_error(ErrorCode::not_found, ConfigDiag::unknown_module_type, type->location(),
                                config_path(at, "type"), unknown_type_message(type->as_string(), types()));
        }
        ConfigNode params = ConfigNode::map();
        params.set_location(node.location());
        for (const auto& [key, value] : node.entries())
            if (key != "type") params.set(key, value);
        auto refs = references(type->as_string(), params, at);
        if (!refs) return std::move(refs).error();
        for (const ModuleRef& ref : *refs)
            if (!modules.find(ref.id) && !set.module(ref.id))
                return config_error(ErrorCode::not_found, ConfigDiag::unknown_module, ref.location, ref.path,
                                    "no module '" + ref.id + "'");
        pending.push_back({&id, type->as_string(), std::move(params), at, node.location(), std::move(*refs)});
    }

    BuildContext build{set.model(), context, {}, set.initial(), base_dir};
    build.refs.find = [&set](const std::string& id) { return set.module(id); };
    // Repeated passes in document order: each builds the entries whose references already exist.
    while (!pending.empty()) {
        bool progressed = false;
        for (auto it = pending.begin(); it != pending.end();) {
            const bool ready = std::all_of(it->refs.begin(), it->refs.end(),
                                           [&](const ModuleRef& r) { return set.module(r.id) != nullptr; });
            if (!ready) {
                ++it;
                continue;
            }
            auto module = create(it->type, *it->id, it->params, build, it->path);
            if (!module) return std::move(module).error();
            if (auto s = set.add(std::move(*module)); !s)
                return config_error(s.error().code, ConfigDiag::module_failed, it->location, it->path,
                                    s.error().message);
            it = pending.erase(it);
            progressed = true;
        }
        if (!progressed) {
            std::string ids;
            for (const Pending& p : pending) ids += (ids.empty() ? "'" : ", '") + *p.id + "'";
            return config_error(ErrorCode::invalid_argument, ConfigDiag::module_cycle,
                                pending.front().location, pending.front().path,
                                "reference cycle among modules " + ids);
        }
    }
    return {};
}

}  // namespace kstone
