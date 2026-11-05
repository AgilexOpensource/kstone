#include "kstone/module/schema.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>

namespace kstone::config_detail {
namespace {

using Kind = ConfigNode::Kind;

Error invalid(const ConfigNode& n, const std::string& path, const std::string& msg,
              ConfigDiag diag = ConfigDiag::invalid_value) {
    return config_error(ErrorCode::invalid_argument, diag, n.location(), path, msg);
}

std::string describe(const ConfigNode& n) {
    if (n.kind() == Kind::list) return n.size() ? "list of " + std::to_string(n.size()) : "empty list";
    if (n.kind() == Kind::string) {
        const std::string& v = n.as_string();
        return "'" + (v.size() > 40 ? v.substr(0, 40) + "..." : v) + "'";
    }
    return to_string(n.kind());
}

std::string number_text(double v) {
    std::ostringstream s;
    s << v;
    return s.str();
}

Status check_number(const ConfigNode& n, const std::string& path, const FieldMeta& m, double v) {
    if (std::isnan(v) || (!m.allow_infinite && !std::isfinite(v))) return invalid(n, path, "must be finite");
    if (m.minimum && v < *m.minimum) return invalid(n, path, "must be >= " + number_text(*m.minimum));
    if (m.maximum && v > *m.maximum) return invalid(n, path, "must be <= " + number_text(*m.maximum));
    if (m.exclusive_minimum && v <= *m.exclusive_minimum)
        return invalid(n, path, "must be > " + number_text(*m.exclusive_minimum));
    if (m.exclusive_maximum && v >= *m.exclusive_maximum)
        return invalid(n, path, "must be < " + number_text(*m.exclusive_maximum));
    return {};
}

Status number(const ConfigNode& n, const std::string& path, const FieldMeta& m, double& out) {
    if (n.kind() != Kind::number) return expected(n, path, "a number");
    if (auto s = check_number(n, path, m, n.as_number()); !s) return s;
    out = n.as_number();
    return {};
}

Status integer(const ConfigNode& n, const std::string& path, const FieldMeta& m, double lo, double hi,
               double& out) {
    if (auto s = number(n, path, m, out); !s) return s;
    if (std::floor(out) != out || out < lo || out > hi) return expected(n, path, "an integer");
    return {};
}

Status list(const ConfigNode& n, const std::string& path, const FieldMeta& m, const char* what) {
    if (n.kind() != Kind::list) return expected(n, path, what);
    if (m.non_empty && n.size() == 0) return expected(n, path, "a non-empty list");
    return {};
}

Status numbers(const ConfigNode& n, const std::string& path, const FieldMeta& m, std::size_t size,
               double* data) {
    if (n.kind() != Kind::list || n.size() != size)
        return expected(n, path, "a list of " + std::to_string(size) + " numbers");
    for (std::size_t i = 0; i < size; ++i)
        if (auto s = number(n.items()[i], config_path(path, i), m, data[i]); !s) return s;
    return {};
}

ConfigNode number_list(const double* data, Eigen::Index size) {
    ConfigNode out = ConfigNode::list();
    for (Eigen::Index i = 0; i < size; ++i) out.append(data[i]);
    return out;
}

ConfigNode array_type(ConfigNode items) {
    return ConfigNode::map({{"type", "array"}, {"items", std::move(items)}});
}

ConfigNode fixed_array(ConfigNode items, std::size_t size) {
    ConfigNode out = array_type(std::move(items));
    out.set("minItems", static_cast<double>(size));
    out.set("maxItems", static_cast<double>(size));
    return out;
}

ConfigNode number_type() { return ConfigNode::map({{"type", "number"}}); }
ConfigNode string_type() { return ConfigNode::map({{"type", "string"}}); }

void add_bounds(ConfigNode& schema, const FieldMeta& m) {
    if (m.minimum) schema.set("minimum", *m.minimum);
    if (m.maximum) schema.set("maximum", *m.maximum);
    if (m.exclusive_minimum) schema.set("exclusiveMinimum", *m.exclusive_minimum);
    if (m.exclusive_maximum) schema.set("exclusiveMaximum", *m.exclusive_maximum);
}

bool has_bounds(const FieldMeta& m) {
    return m.minimum || m.maximum || m.exclusive_minimum || m.exclusive_maximum;
}

// Applies number bounds to the innermost number schema (scalar, list or list of lists).
void bound_numbers(ConfigNode& schema, const FieldMeta& m) {
    const ConfigNode* type = schema.find("type");
    if (!type || type->kind() != Kind::string) return;
    if (type->as_string() == "number" || type->as_string() == "integer") {
        add_bounds(schema, m);
        return;
    }
    if (type->as_string() != "array") return;
    const ConfigNode* items = schema.find("items");
    if (!items) return;
    ConfigNode copy = *items;
    bound_numbers(copy, m);
    schema.set("items", std::move(copy));
}

std::string join(const std::vector<std::string>& names) {
    std::string out;
    for (const auto& name : names) out += (out.empty() ? "" : ", ") + name;
    return out;
}

}  // namespace

Status decode(const ConfigNode& n, const std::string& path, const FieldMeta&, bool& out) {
    if (n.kind() != Kind::boolean) return expected(n, path, "a boolean");
    out = n.as_bool();
    return {};
}

Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, int& out) {
    double v = 0.0;
    if (auto s = integer(n, path, m, std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), v); !s)
        return s;
    out = static_cast<int>(v);
    return {};
}

Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, std::ptrdiff_t& out) {
    constexpr double limit = 9007199254740992.0;  // 2^53: exactly representable.
    double v = 0.0;
    if (auto s = integer(n, path, m, -limit, limit, v); !s) return s;
    out = static_cast<std::ptrdiff_t>(v);
    return {};
}

Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, double& out) {
    return number(n, path, m, out);
}

Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, std::string& out) {
    if (n.kind() != Kind::string) return expected(n, path, "a string");
    if (m.non_empty && n.as_string().empty()) return expected(n, path, "a non-empty string");
    out = n.as_string();
    return {};
}

Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m,
              std::vector<std::string>& out) {
    if (auto s = list(n, path, m, "a list of strings"); !s) return s;
    std::vector<std::string> values;
    values.reserve(n.size());
    for (std::size_t i = 0; i < n.size(); ++i) {
        const ConfigNode& item = n.items()[i];
        if (item.kind() != Kind::string || item.as_string().empty())
            return expected(item, config_path(path, i), "a non-empty string");
        values.push_back(item.as_string());
    }
    out = std::move(values);
    return {};
}

Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, std::vector<int>& out) {
    if (auto s = list(n, path, m, "a list of integers"); !s) return s;
    std::vector<int> values(n.size());
    for (std::size_t i = 0; i < n.size(); ++i)
        if (auto s = decode(n.items()[i], config_path(path, i), m, values[i]); !s) return s;
    out = std::move(values);
    return {};
}

Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, Eigen::VectorXd& out) {
    if (auto s = list(n, path, m, "a list of numbers"); !s) return s;
    Eigen::VectorXd values(static_cast<Eigen::Index>(n.size()));
    if (auto s = numbers(n, path, m, n.size(), values.data()); !s) return s;
    out = std::move(values);
    return {};
}

Status decode(const ConfigNode& n, const std::string& path, const FieldMeta&, Eigen::Isometry3d& out) {
    if (n.kind() != Kind::map) return expected(n, path, "a map with translation and quaternion or rpy");
    const FieldMeta plain;
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    bool has_rotation = false;
    for (const auto& [key, value] : n.entries()) {
        const std::string p = config_path(path, key);
        if (key == "translation") {
            if (auto s = numbers(value, p, plain, 3, translation.data()); !s) return s;
        } else if (key == "quaternion" || key == "rpy") {
            if (has_rotation) return invalid(value, p, "give either quaternion or rpy, not both");
            has_rotation = true;
            if (key == "quaternion") {
                double q[4];
                if (auto s = numbers(value, p, plain, 4, q); !s) return s;
                rotation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
                const double norm = rotation.norm();
                if (std::abs(norm - 1.0) > 1e-6)
                    return invalid(value, p, "must be a unit quaternion [w, x, y, z]");
                rotation.normalize();
            } else {
                double rpy[3];
                if (auto s = numbers(value, p, plain, 3, rpy); !s) return s;
                rotation = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
            }
        } else {
            return unknown_key(value, p, {"translation", "quaternion", "rpy"});
        }
    }
    out = Eigen::Isometry3d::Identity();
    out.linear() = rotation.toRotationMatrix();
    out.translation() = translation;
    return {};
}

Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, Points2& out) {
    if (auto s = list(n, path, m, "a list of [x, y] points"); !s) return s;
    Points2 values(n.size());
    for (std::size_t i = 0; i < n.size(); ++i)
        if (auto s = numbers(n.items()[i], config_path(path, i), m, 2, values[i].data()); !s) return s;
    out = std::move(values);
    return {};
}

Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, NamePairs& out) {
    if (auto s = list(n, path, m, "a list of [name, name] pairs"); !s) return s;
    NamePairs values;
    values.reserve(n.size());
    for (std::size_t i = 0; i < n.size(); ++i) {
        const ConfigNode& item = n.items()[i];
        const std::string p = config_path(path, i);
        if (item.kind() != Kind::list || item.size() != 2 || item.items()[0].kind() != Kind::string ||
            item.items()[1].kind() != Kind::string || item.items()[0].as_string().empty() ||
            item.items()[1].as_string().empty())
            return expected(item, p, "a [name, name] pair");
        values.emplace_back(item.items()[0].as_string(), item.items()[1].as_string());
    }
    out = std::move(values);
    return {};
}

Status decode(const ConfigNode& n, const std::string& path, const FieldMeta& m, NamedNumbers& out) {
    if (n.is_null()) {
        out.clear();
        return {};
    }
    if (n.kind() != Kind::map) return expected(n, path, "a {name: number} map");
    NamedNumbers values;
    values.reserve(n.size());
    for (const auto& [name, value] : n.entries()) {
        double x = 0.0;
        if (auto s = number(value, config_path(path, name), m, x); !s) return s;
        values.emplace_back(name, x);
    }
    out = std::move(values);
    return {};
}

Status decode_fixed(const ConfigNode& n, const std::string& path, const FieldMeta& m, Eigen::Index size,
                    double* data) {
    return numbers(n, path, m, static_cast<std::size_t>(size), data);
}

Status decode_choice(const ConfigNode& n, const std::string& path, const FieldMeta& m, std::size_t& index) {
    if (n.kind() == Kind::string)
        for (std::size_t i = 0; i < m.choices.size(); ++i)
            if (m.choices[i] == n.as_string()) {
                index = i;
                return {};
            }
    return invalid(n, path, "expected one of: " + join(m.choices) + ", got " + describe(n),
                   ConfigDiag::invalid_choice);
}

Result<std::shared_ptr<Module>> resolve(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                                        const ConfigReadContext& ctx) {
    if (n.kind() != Kind::string || n.as_string().empty()) return expected(n, path, "a module id");
    std::shared_ptr<Module> found = ctx.find ? ctx.find(n.as_string()) : nullptr;
    if (!found)
        return config_error(ErrorCode::not_found, ConfigDiag::unknown_module, n.location(), path,
                            "unknown module '" + n.as_string() + "'");
    if (!m.module_ref.empty() && m.module_ref != found->type()) return wrong_module(n, path, m, *found);
    return found;
}

ConfigNode encode(bool v) { return v; }
ConfigNode encode(int v) { return v; }
ConfigNode encode(std::ptrdiff_t v) { return static_cast<double>(v); }
ConfigNode encode(double v) { return v; }
ConfigNode encode(const std::string& v) { return v; }

ConfigNode encode(const std::vector<std::string>& v) {
    ConfigNode out = ConfigNode::list();
    for (const auto& s : v) out.append(s);
    return out;
}

ConfigNode encode(const std::vector<int>& v) {
    ConfigNode out = ConfigNode::list();
    for (int i : v) out.append(i);
    return out;
}

ConfigNode encode(const Eigen::VectorXd& v) { return number_list(v.data(), v.size()); }

ConfigNode encode(const Eigen::Isometry3d& v) {
    const Eigen::Quaterniond q(v.linear());
    const double wxyz[4] = {q.w(), q.x(), q.y(), q.z()};
    return ConfigNode::map(
        {{"translation", number_list(v.translation().data(), 3)}, {"quaternion", number_list(wxyz, 4)}});
}

ConfigNode encode(const Points2& v) {
    ConfigNode out = ConfigNode::list();
    for (const auto& p : v) out.append(number_list(p.data(), 2));
    return out;
}

ConfigNode encode(const NamePairs& v) {
    ConfigNode out = ConfigNode::list();
    for (const auto& [a, b] : v) out.append(ConfigNode::list({a, b}));
    return out;
}

ConfigNode encode(const NamedNumbers& v) {
    ConfigNode out = ConfigNode::map({});
    for (const auto& [name, x] : v) out.set(name, x);
    return out;
}

ConfigNode json_type(const bool*) { return ConfigNode::map({{"type", "boolean"}}); }
ConfigNode json_type(const int*) { return ConfigNode::map({{"type", "integer"}}); }
ConfigNode json_type(const std::ptrdiff_t*) { return ConfigNode::map({{"type", "integer"}}); }
ConfigNode json_type(const double*) { return number_type(); }
ConfigNode json_type(const std::string*) { return string_type(); }

ConfigNode json_type(const std::vector<std::string>*) {
    ConfigNode item = string_type();
    item.set("minLength", 1);
    return array_type(std::move(item));
}

ConfigNode json_type(const std::vector<int>*) { return array_type(ConfigNode::map({{"type", "integer"}})); }

ConfigNode json_type(const Eigen::VectorXd*) { return array_type(number_type()); }

ConfigNode json_type(const Eigen::Isometry3d*) {
    ConfigNode schema = object_schema(
        "Rigid transform; rotation as unit quaternion [w, x, y, z] or rpy [roll, "
        "pitch, yaw] (rad, extrinsic X-Y-Z), not both.",
        {{"translation", fixed_array(number_type(), 3)},
         {"quaternion", fixed_array(number_type(), 4)},
         {"rpy", fixed_array(number_type(), 3)}},
        {});
    return schema;
}

ConfigNode json_type(const Points2*) { return array_type(fixed_array(number_type(), 2)); }

ConfigNode json_type(const NamePairs*) {
    ConfigNode name = string_type();
    name.set("minLength", 1);
    return array_type(fixed_array(std::move(name), 2));
}

ConfigNode json_type(const NamedNumbers*) {
    return ConfigNode::map({{"type", "object"}, {"additionalProperties", number_type()}});
}

ConfigNode json_number_list(Eigen::Index size) {
    return fixed_array(number_type(), static_cast<std::size_t>(size));
}

ConfigNode finish_field(ConfigNode type, const FieldMeta& m, ConfigNode default_value) {
    if (!m.doc.empty()) type.set("description", m.doc);
    if (!m.module_ref.empty()) type.set("x-module-type", m.module_ref);
    if (!m.choices.empty()) type.set("enum", encode(m.choices));
    if (has_bounds(m)) bound_numbers(type, m);
    if (m.non_empty) {
        const ConfigNode* t = type.find("type");
        const std::string kind = t && t->kind() == Kind::string ? t->as_string() : std::string{};
        if (kind == "string") type.set("minLength", 1);
        if (kind == "array") type.set("minItems", 1);
    }
    if (m.tunable) type.set("tunable", true);
    if (!default_value.is_null()) type.set("default", std::move(default_value));
    return type;
}

ConfigNode object_schema(const std::string& description,
                         std::vector<std::pair<std::string, ConfigNode>> properties,
                         const std::vector<std::string>& required) {
    ConfigNode out = ConfigNode::map({{"type", "object"}});
    if (!description.empty()) out.set("description", description);
    out.set("properties", ConfigNode::map(std::move(properties)));
    if (!required.empty()) out.set("required", encode(required));
    out.set("additionalProperties", false);
    return out;
}

ConfigNode array_of(ConfigNode items) { return array_type(std::move(items)); }

// A quoted scalar stays a string, which is easy to miss after quoting a ${var} in a flow map.
bool looks_unquotable(const ConfigNode& n) {
    if (n.kind() != Kind::string) return false;
    const std::string& v = n.as_string();
    if (v == "true" || v == "false") return true;
    char* end = nullptr;
    const double d = std::strtod(v.c_str(), &end);
    return !v.empty() && end == v.c_str() + v.size() && std::isfinite(d);
}

Error expected(const ConfigNode& n, const std::string& path, const std::string& what) {
    std::string message = "expected " + what + ", got " + describe(n);
    if (looks_unquotable(n)) message += " (quoted scalars stay strings; remove the quotes)";
    return invalid(n, path, message, ConfigDiag::wrong_type);
}

Error unknown_key(const ConfigNode& n, const std::string& path, const std::vector<std::string>& known) {
    return invalid(n, path, "unknown key (allowed: " + join(known) + ")", ConfigDiag::unknown_key);
}

Error missing_key(const ConfigNode& parent, const std::string& path, const std::string& doc) {
    return invalid(parent, path,
                   doc.empty() ? "required key is missing" : "required key is missing (" + doc + ")",
                   ConfigDiag::missing_key);
}

Error not_tunable(const ConfigNode& n, const std::string& path) {
    return invalid(n, path, "not tunable at runtime", ConfigDiag::not_tunable);
}

Error wrong_module(const ConfigNode& n, const std::string& path, const FieldMeta& m, const Module& found) {
    return invalid(
        n, path,
        "module '" + found.id() + "' has type '" + found.type() + "', expected '" + m.module_ref + "'",
        ConfigDiag::wrong_module_type);
}

void collect_ref(const ConfigNode& n, const std::string& path, const FieldMeta& m,
                 std::vector<ModuleRef>& out) {
    if (n.kind() == Kind::string) {
        out.push_back({n.as_string(), m.module_ref, path, n.location()});
        return;
    }
    if (n.kind() != Kind::list) return;
    for (std::size_t i = 0; i < n.size(); ++i) {
        const ConfigNode& item = n.items()[i];
        if (item.kind() == Kind::string)
            out.push_back({item.as_string(), m.module_ref, config_path(path, i), item.location()});
    }
}

}  // namespace kstone::config_detail
