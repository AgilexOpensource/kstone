#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/module/config.hpp"
#include "kstone/module/module.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <functional>
#include <initializer_list>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace kstone {

/// A configuration value that names another module by id.
struct ModuleRef {
    std::string id;
    std::string type;  ///< Required module type.
    std::string path;
    SourceLocation location;
};

/// Resolves module references while reading; empty find = every reference is not_found.
struct ConfigReadContext {
    std::function<std::shared_ptr<Module>(const std::string& id)> find;
};

/// Name, documentation and value constraints of one schema field.
struct FieldMeta {
    std::string name;
    std::string doc;
    bool required{false};
    bool non_empty{false};       ///< Strings and lists.
    bool allow_infinite{false};  ///< Numbers are finite unless set.
    std::optional<double> minimum, maximum, exclusive_minimum,
        exclusive_maximum;             ///< Numbers and number lists.
    std::vector<std::string> choices;  ///< Enum names.
    std::string module_ref;            ///< Non-empty: values are ids of modules of this type.
    bool tunable{false};               ///< May be changed at runtime through ParamSchema::patch.
};

/// Chainable constraints on the field just added to a ParamSchema.
class FieldRef {
  public:
    explicit FieldRef(FieldMeta& meta) noexcept : meta_(&meta) {}
    FieldRef& doc(std::string text) {
        meta_->doc = std::move(text);
        return *this;
    }
    FieldRef& required() noexcept {
        meta_->required = true;
        return *this;
    }
    FieldRef& non_empty() noexcept {
        meta_->non_empty = true;
        return *this;
    }
    FieldRef& allow_infinite() noexcept {
        meta_->allow_infinite = true;
        return *this;
    }
    FieldRef& at_least(double v) noexcept {
        meta_->minimum = v;
        return *this;
    }
    FieldRef& at_most(double v) noexcept {
        meta_->maximum = v;
        return *this;
    }
    FieldRef& above(double v) noexcept {
        meta_->exclusive_minimum = v;
        return *this;
    }
    FieldRef& below(double v) noexcept {
        meta_->exclusive_maximum = v;
        return *this;
    }
    FieldRef& range(double lo, double hi) noexcept { return at_least(lo).at_most(hi); }
    FieldRef& tunable() noexcept {
        meta_->tunable = true;
        return *this;
    }

  private:
    FieldMeta* meta_;
};

namespace config_detail {

using NamePairs = std::vector<std::pair<std::string, std::string>>;
using Points2 = std::vector<Eigen::Vector2d>;
using NamedNumbers = std::vector<std::pair<std::string, double>>;  ///< A {name: number} map, in order.

KSTONE_EXPORT Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, bool& out);
KSTONE_EXPORT Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, int& out);
KSTONE_EXPORT Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                            std::ptrdiff_t& out);
KSTONE_EXPORT Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, double& out);
KSTONE_EXPORT Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                            std::string& out);
KSTONE_EXPORT Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                            std::vector<std::string>& out);
KSTONE_EXPORT Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                            std::vector<int>& out);
KSTONE_EXPORT Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                            Eigen::VectorXd& out);
KSTONE_EXPORT Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                            Eigen::Isometry3d& out);
KSTONE_EXPORT Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, Points2& out);
KSTONE_EXPORT Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, NamePairs& out);
KSTONE_EXPORT Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                            NamedNumbers& out);
/// Fixed-size number list into data[0..size).
KSTONE_EXPORT Status decode_fixed(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                                  Eigen::Index size, double* data);
/// Index into m.choices.
KSTONE_EXPORT Status decode_choice(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                                   std::size_t& index);
/// Looks up one module id; checks m.module_ref against the found module's type().
KSTONE_EXPORT Result<std::shared_ptr<Module>> resolve(const ConfigNode& n, const std::string& path,
                                                      const FieldMeta& m, const ConfigReadContext& ctx);

template <int N>
Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m,
              Eigen::Matrix<double, N, 1>& out) {
    static_assert(N > 0, "fixed-size vectors only");
    return decode_fixed(n, path, m, N, out.data());
}

template <typename T>
Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, std::optional<T>& out) {
    if (n.is_null()) {
        out.reset();
        return {};
    }
    T value{};
    if (auto s = decode(n, path, m, value); !s) return s;
    out = std::move(value);
    return {};
}

KSTONE_EXPORT ConfigNode encode(bool v);
KSTONE_EXPORT ConfigNode encode(int v);
KSTONE_EXPORT ConfigNode encode(std::ptrdiff_t v);
KSTONE_EXPORT ConfigNode encode(double v);
KSTONE_EXPORT ConfigNode encode(const std::string& v);
KSTONE_EXPORT ConfigNode encode(const std::vector<std::string>& v);
KSTONE_EXPORT ConfigNode encode(const std::vector<int>& v);
KSTONE_EXPORT ConfigNode encode(const Eigen::VectorXd& v);
KSTONE_EXPORT ConfigNode encode(const Eigen::Isometry3d& v);
KSTONE_EXPORT ConfigNode encode(const Points2& v);
KSTONE_EXPORT ConfigNode encode(const NamePairs& v);
KSTONE_EXPORT ConfigNode encode(const NamedNumbers& v);

template <int N>
ConfigNode encode(const Eigen::Matrix<double, N, 1>& v) {
    return encode(Eigen::VectorXd(v));
}

template <typename T>
ConfigNode encode(const std::optional<T>& v) {
    return v ? encode(*v) : ConfigNode{};
}

// JSON Schema of each value type; FieldMeta constraints are added by finish_field.
KSTONE_EXPORT ConfigNode json_type(const bool*);
KSTONE_EXPORT ConfigNode json_type(const int*);
KSTONE_EXPORT ConfigNode json_type(const std::ptrdiff_t*);
KSTONE_EXPORT ConfigNode json_type(const double*);
KSTONE_EXPORT ConfigNode json_type(const std::string*);
KSTONE_EXPORT ConfigNode json_type(const std::vector<std::string>*);
KSTONE_EXPORT ConfigNode json_type(const std::vector<int>*);
KSTONE_EXPORT ConfigNode json_type(const Eigen::VectorXd*);
KSTONE_EXPORT ConfigNode json_type(const Eigen::Isometry3d*);
KSTONE_EXPORT ConfigNode json_type(const Points2*);
KSTONE_EXPORT ConfigNode json_type(const NamePairs*);
KSTONE_EXPORT ConfigNode json_type(const NamedNumbers*);
KSTONE_EXPORT ConfigNode json_number_list(Eigen::Index size);

template <int N>
ConfigNode json_type(const Eigen::Matrix<double, N, 1>*) {
    return json_number_list(N);
}

template <typename T>
ConfigNode json_type(const std::optional<T>*) {
    return json_type(static_cast<const T*>(nullptr));
}

/// Adds description, default (unless null) and constraint keywords to a type schema.
KSTONE_EXPORT ConfigNode finish_field(ConfigNode type, const FieldMeta& m, ConfigNode default_value);
KSTONE_EXPORT ConfigNode object_schema(const std::string& description,
                                       std::vector<std::pair<std::string, ConfigNode>> properties,
                                       const std::vector<std::string>& required);
KSTONE_EXPORT ConfigNode array_of(ConfigNode items);
KSTONE_EXPORT Error expected(const ConfigNode& n, const std::string& path, const std::string& what);
KSTONE_EXPORT Error unknown_key(const ConfigNode& n, const std::string& path,
                                const std::vector<std::string>& known);
KSTONE_EXPORT Error missing_key(const ConfigNode& parent, const std::string& path,
                                const std::string& doc = {});
KSTONE_EXPORT Error not_tunable(const ConfigNode& n, const std::string& path);
KSTONE_EXPORT Error wrong_module(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                                 const Module& found);
KSTONE_EXPORT void collect_ref(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                               std::vector<ModuleRef>& out);

}  // namespace config_detail

/// Typed parameter schema built without macros:
///   ParamSchema<Cfg> s("…");
///   s.field("gain", &Cfg::gain).range(0, 1).doc("…");
/// read() rejects unknown keys; errors read "file:line:col: path: message".
template <typename Cfg>
class ParamSchema {
  public:
    explicit ParamSchema(std::string description = {}) : description_(std::move(description)) {}

    template <typename T, std::enable_if_t<!std::is_enum_v<T>, int> = 0>
    FieldRef field(std::string name, T Cfg::* member) {
        Field& f = push(std::move(name));
        f.read = [member](const ConfigNode& n, const std::string& p, const FieldMeta& m,
                          const ConfigReadContext&,
                          Cfg& c) { return config_detail::decode(n, p, m, c.*member); };
        f.json = [member](const FieldMeta& m, const Cfg& d) {
            return config_detail::finish_field(config_detail::json_type(static_cast<const T*>(nullptr)), m,
                                               config_detail::encode(d.*member));
        };
        f.value = [member](const Cfg& c) { return config_detail::encode(c.*member); };
        return FieldRef(f.meta);
    }

    template <typename E, std::enable_if_t<std::is_enum_v<E>, int> = 0>
    FieldRef field(std::string name, E Cfg::* member,
                   std::initializer_list<std::pair<const char*, E>> names) {
        Field& f = push(std::move(name));
        std::vector<E> values;
        for (const auto& [label, value] : names) {
            f.meta.choices.emplace_back(label);
            values.push_back(value);
        }
        f.read = [member, values](const ConfigNode& n, const std::string& p, const FieldMeta& m,
                                  const ConfigReadContext&, Cfg& c) -> Status {
            std::size_t i = 0;
            if (auto s = config_detail::decode_choice(n, p, m, i); !s) return s;
            c.*member = values[i];
            return {};
        };
        f.json = [member, values](const FieldMeta& m, const Cfg& d) {
            ConfigNode def;
            for (std::size_t i = 0; i < values.size(); ++i)
                if (values[i] == d.*member) def = ConfigNode(m.choices[i]);
            return config_detail::finish_field(ConfigNode::map({{"type", "string"}}), m, std::move(def));
        };
        f.value = [member, values, choices = f.meta.choices](const Cfg& c) {
            for (std::size_t i = 0; i < values.size(); ++i)
                if (values[i] == c.*member) return ConfigNode(choices[i]);
            return ConfigNode{};
        };
        return FieldRef(f.meta);
    }

    /// Nested map; defaults come from the parent's value.
    template <typename T, std::enable_if_t<std::is_class_v<T>, int> = 0>
    FieldRef field(std::string name, T Cfg::* member, ParamSchema<T> nested) {
        auto sub = std::make_shared<const ParamSchema<T>>(std::move(nested));
        Field& f = push(std::move(name));
        f.read = [member, sub](const ConfigNode& n, const std::string& p, const FieldMeta&,
                               const ConfigReadContext& ctx,
                               Cfg& c) { return sub->read(n, c.*member, p, ctx); };
        f.json = [member, sub](const FieldMeta& m, const Cfg& d) {
            return config_detail::finish_field(sub->json_schema(d.*member), m, ConfigNode{});
        };
        f.collect = [sub](const ConfigNode& n, const std::string& p, std::vector<ModuleRef>& out) {
            sub->collect_refs(n, p, out);
        };
        f.value = [member, sub](const Cfg& c) { return sub->encode(c.*member); };
        if (sub->has_tunables()) {
            f.patch_nested = [member, sub](const ConfigNode& n, const std::string& p, Cfg& c) {
                return sub->patch(n, c.*member, p);
            };
            f.values_nested = [member, sub](const Cfg& c) { return sub->values(c.*member); };
            f.tune_json_nested = [member, sub](const FieldMeta& m, const Cfg& d) {
                return config_detail::finish_field(sub->tune_schema(d.*member), m, ConfigNode{});
            };
        }
        return FieldRef(f.meta);
    }

    /// List of nested maps.
    template <typename T>
    FieldRef field(std::string name, std::vector<T> Cfg::* member, ParamSchema<T> item) {
        auto sub = std::make_shared<const ParamSchema<T>>(std::move(item));
        Field& f = push(std::move(name));
        f.read = [member, sub](const ConfigNode& n, const std::string& p, const FieldMeta& m,
                               const ConfigReadContext& ctx, Cfg& c) -> Status {
            if (n.is_null()) {
                (c.*member).clear();
                return {};
            }
            if (n.kind() != ConfigNode::Kind::list) return config_detail::expected(n, p, "a list");
            if (m.non_empty && n.size() == 0) return config_detail::expected(n, p, "a non-empty list");
            std::vector<T> values;
            values.reserve(n.size());
            for (std::size_t i = 0; i < n.items().size(); ++i) {
                T value{};
                if (auto s = sub->read(n.items()[i], value, config_path(p, i), ctx); !s) return s;
                values.push_back(std::move(value));
            }
            c.*member = std::move(values);
            return {};
        };
        f.json = [sub](const FieldMeta& m, const Cfg&) {
            return config_detail::finish_field(config_detail::array_of(sub->json_schema()), m, ConfigNode{});
        };
        f.collect = [sub](const ConfigNode& n, const std::string& p, std::vector<ModuleRef>& out) {
            if (n.kind() != ConfigNode::Kind::list) return;
            for (std::size_t i = 0; i < n.items().size(); ++i)
                sub->collect_refs(n.items()[i], config_path(p, i), out);
        };
        f.value = [member, sub](const Cfg& c) {
            ConfigNode out = ConfigNode::list({});
            for (const T& element : c.*member) out.append(sub->encode(element));
            return out;
        };
        return FieldRef(f.meta);
    }

    /// Id of another module of `type`, resolved through ConfigReadContext.
    template <typename M>
    FieldRef ref(std::string name, std::shared_ptr<M> Cfg::* member, std::string type) {
        Field& f = push(std::move(name));
        f.meta.module_ref = std::move(type);
        f.read = [member](const ConfigNode& n, const std::string& p, const FieldMeta& m,
                          const ConfigReadContext& ctx, Cfg& c) -> Status {
            if (n.is_null()) {
                c.*member = nullptr;
                return {};
            }
            auto found = config_detail::resolve(n, p, m, ctx);
            if (!found) return std::move(found).error();
            auto typed = std::dynamic_pointer_cast<M>(*found);
            if (!typed) return config_detail::wrong_module(n, p, m, **found);
            c.*member = std::move(typed);
            return {};
        };
        f.json = [](const FieldMeta& m, const Cfg&) {
            return config_detail::finish_field(ConfigNode::map({{"type", "string"}}), m, ConfigNode{});
        };
        f.value = [member](const Cfg& c) { return c.*member ? ConfigNode((c.*member)->id()) : ConfigNode{}; };
        return FieldRef(f.meta);
    }

    /// List of module ids of `type`.
    template <typename M>
    FieldRef ref(std::string name, std::vector<std::shared_ptr<M>> Cfg::* member, std::string type) {
        Field& f = push(std::move(name));
        f.meta.module_ref = std::move(type);
        f.read = [member](const ConfigNode& n, const std::string& p, const FieldMeta& m,
                          const ConfigReadContext& ctx, Cfg& c) -> Status {
            if (n.is_null()) {
                (c.*member).clear();
                return {};
            }
            if (n.kind() != ConfigNode::Kind::list)
                return config_detail::expected(n, p, "a list of module ids");
            if (m.non_empty && n.size() == 0) return config_detail::expected(n, p, "a non-empty list");
            std::vector<std::shared_ptr<M>> values;
            for (std::size_t i = 0; i < n.items().size(); ++i) {
                const auto& item = n.items()[i];
                auto found = config_detail::resolve(item, config_path(p, i), m, ctx);
                if (!found) return std::move(found).error();
                auto typed = std::dynamic_pointer_cast<M>(*found);
                if (!typed) return config_detail::wrong_module(item, config_path(p, i), m, **found);
                values.push_back(std::move(typed));
            }
            c.*member = std::move(values);
            return {};
        };
        f.json = [](const FieldMeta& m, const Cfg&) {
            return config_detail::finish_field(config_detail::array_of(ConfigNode::map({{"type", "string"}})),
                                               m, ConfigNode{});
        };
        f.value = [member](const Cfg& c) {
            ConfigNode out = ConfigNode::list({});
            for (const auto& item : c.*member) out.append(item ? ConfigNode(item->id()) : ConfigNode{});
            return out;
        };
        return FieldRef(f.meta);
    }

    /// Adds every field of `sub` at this level, reading into the member.
    template <typename Sub>
    ParamSchema& merge(Sub Cfg::* member, const ParamSchema<Sub>& sub) {
        for (const auto& src : sub.fields_) {
            Field& f = push(src->meta.name);
            f.meta = src->meta;
            f.read = [member, read = src->read](const ConfigNode& n, const std::string& p, const FieldMeta& m,
                                                const ConfigReadContext& ctx,
                                                Cfg& c) { return read(n, p, m, ctx, c.*member); };
            f.json = [member, json = src->json](const FieldMeta& m, const Cfg& d) {
                return json(m, d.*member);
            };
            f.collect = src->collect;
            f.value = [member, value = src->value](const Cfg& c) { return value(c.*member); };
            if (src->patch_nested) {
                f.patch_nested = [member, patch = src->patch_nested](const ConfigNode& n,
                                                                     const std::string& p, Cfg& c) {
                    return patch(n, p, c.*member);
                };
                f.values_nested = [member, values = src->values_nested](const Cfg& c) {
                    return values(c.*member);
                };
                f.tune_json_nested = [member, json = src->tune_json_nested](
                                         const FieldMeta& m, const Cfg& d) { return json(m, d.*member); };
            }
        }
        return *this;
    }

    Status read(const ConfigNode& node, Cfg& out, const std::string& path = {},
                const ConfigReadContext& ctx = {}) const {
        if (!node.is_null() && node.kind() != ConfigNode::Kind::map)
            return config_detail::expected(node, path, "a map");
        for (const auto& [key, value] : node.entries()) {
            const Field* f = find(key);
            if (!f) return config_detail::unknown_key(value, config_path(path, key), names());
            if (auto s = f->read(value, config_path(path, key), f->meta, ctx, out); !s) return s;
        }
        for (const auto& f : fields_)
            if (f->meta.required && !node.find(f->meta.name))
                return config_detail::missing_key(node, config_path(path, f->meta.name),
                                                  f->meta.doc.empty() && !f->meta.module_ref.empty()
                                                      ? "id of a " + f->meta.module_ref + " module"
                                                      : f->meta.doc);
        return {};
    }

    /// Module references in a parameter map; skips anything malformed (read() reports it).
    void collect_refs(const ConfigNode& node, const std::string& path, std::vector<ModuleRef>& out) const {
        for (const auto& [key, value] : node.entries()) {
            const Field* f = find(key);
            if (!f) continue;
            if (!f->meta.module_ref.empty())
                config_detail::collect_ref(value, config_path(path, key), f->meta, out);
            else if (f->collect)
                f->collect(value, config_path(path, key), out);
        }
    }

    /// JSON Schema (draft 2020-12) as a ConfigNode; defaults are taken from `defaults`.
    ConfigNode json_schema(const Cfg& defaults = Cfg{}) const {
        std::vector<std::pair<std::string, ConfigNode>> properties;
        std::vector<std::string> required;
        for (const auto& f : fields_) {
            properties.emplace_back(f->meta.name, f->json(f->meta, defaults));
            if (f->meta.required) required.push_back(f->meta.name);
        }
        return config_detail::object_schema(description_, std::move(properties), required);
    }

    /// Every field of `c` as a parameter map that read() accepts.
    ConfigNode encode(const Cfg& c) const {
        ConfigNode out = ConfigNode::map({});
        for (const auto& f : fields_) out.set(f->meta.name, f->value(c));
        return out;
    }

    /// True if any field, or any field of a nested map, is tunable.
    bool has_tunables() const {
        for (const auto& f : fields_)
            if (f->meta.tunable || f->patch_nested) return true;
        return false;
    }

    /// Overlays tunable keys of `node` onto `out`; all-or-nothing, no required-key check.
    Status patch(const ConfigNode& node, Cfg& out, const std::string& path = {}) const {
        if (node.kind() != ConfigNode::Kind::map) return config_detail::expected(node, path, "a map");
        Cfg next = out;
        for (const auto& [key, value] : node.entries()) {
            const Field* f = find(key);
            const std::string p = config_path(path, key);
            if (!f) return config_detail::unknown_key(value, p, tunable_names());
            if (f->meta.tunable) {
                if (auto s = f->read(value, p, f->meta, ConfigReadContext{}, next); !s) return s;
            } else if (f->patch_nested) {
                if (auto s = f->patch_nested(value, p, next); !s) return s;
            } else {
                return config_detail::not_tunable(value, p);
            }
        }
        out = std::move(next);
        return {};
    }

    /// Current values of the tunable fields only.
    ConfigNode values(const Cfg& c) const {
        ConfigNode out = ConfigNode::map({});
        for (const auto& f : fields_) {
            if (f->meta.tunable)
                out.set(f->meta.name, f->value(c));
            else if (f->patch_nested)
                out.set(f->meta.name, f->values_nested(c));
        }
        return out;
    }

    /// JSON Schema of the tunable fields only; defaults are taken from `defaults`.
    ConfigNode tune_schema(const Cfg& defaults = Cfg{}) const {
        std::vector<std::pair<std::string, ConfigNode>> properties;
        for (const auto& f : fields_) {
            if (f->meta.tunable)
                properties.emplace_back(f->meta.name, f->json(f->meta, defaults));
            else if (f->patch_nested)
                properties.emplace_back(f->meta.name, f->tune_json_nested(f->meta, defaults));
        }
        return config_detail::object_schema(description_, std::move(properties), {});
    }

    const std::string& description() const noexcept { return description_; }

  private:
    template <typename>
    friend class ParamSchema;

    struct Field {
        FieldMeta meta;
        std::function<Status(const ConfigNode&, const std::string&, const FieldMeta&,
                             const ConfigReadContext&, Cfg&)>
            read;
        std::function<ConfigNode(const FieldMeta&, const Cfg&)> json;
        std::function<void(const ConfigNode&, const std::string&, std::vector<ModuleRef>&)> collect;
        std::function<ConfigNode(const Cfg&)> value;
        // Set only on nested maps that contain tunable fields.
        std::function<Status(const ConfigNode&, const std::string&, Cfg&)> patch_nested;
        std::function<ConfigNode(const Cfg&)> values_nested;
        std::function<ConfigNode(const FieldMeta&, const Cfg&)> tune_json_nested;
    };

    Field& push(std::string name) {
        fields_.push_back(std::make_shared<Field>());
        fields_.back()->meta.name = std::move(name);
        return *fields_.back();
    }

    const Field* find(const std::string& key) const {
        for (const auto& f : fields_)
            if (f->meta.name == key) return f.get();
        return nullptr;
    }

    std::vector<std::string> names() const {
        std::vector<std::string> out;
        for (const auto& f : fields_) out.push_back(f->meta.name);
        return out;
    }

    std::vector<std::string> tunable_names() const {
        std::vector<std::string> out;
        for (const auto& f : fields_)
            if (f->meta.tunable || f->patch_nested) out.push_back(f->meta.name);
        return out;
    }

    std::string description_;
    std::vector<std::shared_ptr<Field>> fields_;  // Stable addresses for FieldRef.
};

}  // namespace kstone
