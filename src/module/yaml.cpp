#include "kstone/module/yaml.hpp"

#include "kstone/core/resources.hpp"

#include <yaml-cpp/depthguard.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace kstone {
namespace {

// Bounds alias expansion and nesting so hostile documents cannot exhaust memory or the stack.
// Both limits span every included file.
constexpr std::size_t kMaxNodes = 1u << 20;
constexpr int kMaxDepth = 256;
constexpr const char* kIncludeTag = "!include";

bool valid_var_name(const std::string& name) {
    if (name.empty()) return false;
    const auto alpha = [](char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_'; };
    if (!alpha(name[0])) return false;
    return std::all_of(name.begin(), name.end(),
                       [&](char c) { return alpha(c) || (c >= '0' && c <= '9') || c == '.'; });
}

ConfigNode type_plain(const std::string& text) {
    if (text == "true" || text == "True" || text == "TRUE") return ConfigNode(true);
    if (text == "false" || text == "False" || text == "FALSE") return ConfigNode(false);
    if (text.empty() || text == "~" || text == "null" || text == "Null" || text == "NULL")
        return ConfigNode{};
    // JSON encoders such as Python's json write infinity as a bare Infinity token.
    if (text == "Infinity" || text == "+Infinity") return ConfigNode(std::numeric_limits<double>::infinity());
    if (text == "-Infinity") return ConfigNode(-std::numeric_limits<double>::infinity());
    double number = 0.0;
    if (YAML::convert<double>::decode(YAML::Node(text), number)) return ConfigNode(number);
    return ConfigNode(text);
}

class Converter {
  public:
    Converter(std::string file, const YamlOptions& options)
        : file_(std::move(file)), base_dir_(options.base_dir), options_(options) {}

    void push_file(const std::filesystem::path& canonical) { stack_.push_back(canonical); }

    Result<ConfigNode> convert(const YAML::Node& node, int depth) {
        if (++count_ > kMaxNodes)
            return fail(node.Mark(), "document expands to too many nodes", ConfigDiag::too_large);
        if (depth > kMaxDepth)
            return fail(node.Mark(), "document is nested too deeply", ConfigDiag::too_large);
        if (options_.includes && node.Tag() == kIncludeTag) return include(node, depth);
        ConfigNode out;
        switch (node.Type()) {
            case YAML::NodeType::Undefined:
            case YAML::NodeType::Null:
                break;
            case YAML::NodeType::Scalar: {
                auto scalar = convert_scalar(node);
                if (!scalar) return scalar;
                out = std::move(*scalar);
                break;
            }
            case YAML::NodeType::Sequence:
                out = ConfigNode::list();
                for (const auto& item : node) {
                    auto child = convert(item, depth + 1);
                    if (!child) return child;
                    out.append(std::move(*child));
                }
                break;
            case YAML::NodeType::Map:
                out = ConfigNode::map();
                for (const auto& kv : node) {
                    if (!kv.first.IsScalar())
                        return fail(kv.first.Mark(), "map keys must be scalars", ConfigDiag::bad_key);
                    const std::string& key = kv.first.Scalar();
                    if (key == "<<" && kv.first.Tag() != "!")
                        return fail(kv.first.Mark(),
                                    "merge keys ('<<') are not supported; repeat the keys or use !include",
                                    ConfigDiag::bad_key);
                    if (out.find(key))
                        return fail(kv.first.Mark(), "duplicate key '" + key + "'",
                                    ConfigDiag::duplicate_key);
                    auto child = convert(kv.second, depth + 1);
                    if (!child) return child;
                    // Empty values have no position of their own (their mark is past the line); use the key's.
                    if (kv.second.IsNull() && kv.second.Scalar().empty())
                        child->set_location(location(kv.first.Mark()));
                    out.set(key, std::move(*child));
                }
                break;
        }
        out.set_location(location(node.Mark()));
        return out;
    }

    Error fail(const YAML::Mark& mark, const std::string& message, ConfigDiag diag,
               ErrorCode code = ErrorCode::parse_error) const {
        return config_error(code, diag, location(mark), {}, message);
    }

    /// Loads the root document text; YAML syntax errors are reported against the current file.
    Result<ConfigNode> convert_text(const std::string& text, int depth) {
        std::vector<YAML::Node> docs;
        try {
            docs = YAML::LoadAll(text);
        } catch (const YAML::DeepRecursion& e) {
            return fail(e.mark, "document is nested too deeply", ConfigDiag::too_large);
        } catch (const YAML::Exception& e) {
            std::string msg = e.msg;
            if (e.mark.line >= 0) {
                std::size_t begin = 0;
                for (int l = 0; l < e.mark.line && begin != std::string::npos; ++l) {
                    begin = text.find('\n', begin);
                    if (begin != std::string::npos) ++begin;
                }
                if (begin != std::string::npos &&
                    text.substr(begin, text.find('\n', begin) - begin).find("${") != std::string::npos)
                    msg +=
                        " (inside {...} or [...] quote ${...} references, \"${name}\", which keeps them "
                        "strings;"
                        " use block style for numbers)";
            }
            return fail(e.mark, msg, ConfigDiag::syntax);
        }
        // YAML::Load silently drops everything after the first document, e.g. `- a\nb: 1` loses `b`.
        for (std::size_t i = 1; i < docs.size(); ++i)
            if (!docs[i].IsNull())
                return fail(docs[i].Mark(), "unexpected content after the end of the document",
                            ConfigDiag::syntax);
        return convert(docs.empty() ? YAML::Node() : docs[0], depth);
    }

  private:
    Result<ConfigNode> convert_scalar(const YAML::Node& node) const {
        const std::string& tag = node.Tag();
        const bool quoted = tag == "!" || tag == "tag:yaml.org,2002:str";
        if (!quoted && tag != "?")
            return fail(node.Mark(), "unsupported tag '" + tag + "'", ConfigDiag::bad_tag);
        if (!options_.expand_vars) return quoted ? ConfigNode(node.Scalar()) : type_plain(node.Scalar());
        bool whole = false;
        bool changed = false;
        auto text = expand(node.Scalar(), node.Mark(), whole, changed);
        if (!text) return std::move(text).error();
        if (quoted) return ConfigNode(std::move(*text));
        // A plain scalar that is exactly one reference takes the value's type (`rate: ${RATE}`);
        // partial substitutions and escapes stay strings.
        if (!changed || whole) return type_plain(*text);
        return ConfigNode(std::move(*text));
    }

    /// Expands `${name}` / `${name:-default}`; `$${` is a literal `${`. `whole` is set when the
    /// scalar is exactly one reference, `changed` when anything was substituted or unescaped.
    Result<std::string> expand(const std::string& text, const YAML::Mark& mark, bool& whole,
                               bool& changed) const {
        std::string out;
        out.reserve(text.size());
        std::size_t refs = 0;
        bool literal = false;
        for (std::size_t i = 0; i < text.size();) {
            if (text.compare(i, 3, "$${") == 0) {
                out += "${";
                i += 3;
                literal = true;
                continue;
            }
            if (text.compare(i, 2, "${") != 0) {
                out += text[i++];
                continue;
            }
            const std::size_t close = text.find('}', i + 2);
            if (close == std::string::npos)
                return fail(mark, "unterminated '${' in '" + text + "'", ConfigDiag::bad_var);
            const std::string inner = text.substr(i + 2, close - i - 2);
            if (inner.find("${") != std::string::npos)
                return fail(mark, "nested '${' in '" + text + "' is not supported", ConfigDiag::bad_var);
            const std::size_t sep = inner.find(":-");
            const std::string name = inner.substr(0, sep);
            if (!valid_var_name(name))
                return fail(mark, "bad variable reference '${" + inner + "}'", ConfigDiag::bad_var);
            const std::string* value = nullptr;
            std::string env;
            if (auto it = options_.vars.find(name); it != options_.vars.end()) {
                value = &it->second;
                used_.insert(name);
            } else if (const char* e = options_.environment ? std::getenv(name.c_str()) : nullptr) {
                env = e;
                value = &env;
            } else if (sep != std::string::npos) {
                env = inner.substr(sep + 2);
                value = &env;
            } else {
                return fail(mark, "undefined variable '" + name + "'", ConfigDiag::undefined_var,
                            ErrorCode::not_found);
            }
            // Like the shell's ${name:-default}: an empty value also takes the default.
            if (value->empty() && sep != std::string::npos) {
                env = inner.substr(sep + 2);
                value = &env;
            }
            out += *value;
            ++refs;
            whole = i == 0 && close + 1 == text.size();
            i = close + 1;
        }
        if (refs != 1 || literal) whole = false;
        changed = refs > 0 || literal;
        return out;
    }

    Result<ConfigNode> include(const YAML::Node& node, int depth) {
        if (!node.IsScalar())
            return fail(node.Mark(), std::string(kIncludeTag) + " needs a path", ConfigDiag::bad_tag);
        std::string uri = node.Scalar();
        if (options_.expand_vars) {
            bool whole = false;
            bool changed = false;
            auto expanded = expand(uri, node.Mark(), whole, changed);
            if (!expanded) return std::move(expanded).error();
            uri = std::move(*expanded);
        }
        const auto include_error = [&](const Error& e) {
            return fail(node.Mark(), "cannot include '" + uri + "': " + e.message, ConfigDiag::include_failed,
                        e.code);
        };
        std::filesystem::path path;
        if (options_.resources != nullptr) {
            auto resolved = options_.resources->resolve(uri, base_dir_);
            if (!resolved) return include_error(resolved.error());
            path = std::move(*resolved);
        } else {
            path = std::filesystem::path(uri).is_absolute() ? std::filesystem::path(uri) : base_dir_ / uri;
        }
        std::error_code ec;
        std::filesystem::path canonical = std::filesystem::weakly_canonical(path, ec);
        if (ec) canonical = path;
        if (std::find(stack_.begin(), stack_.end(), canonical) != stack_.end())
            return fail(node.Mark(), "include cycle through '" + path.string() + "'",
                        ConfigDiag::include_cycle);
        if (include_depth_ >= options_.max_include_depth)
            return fail(node.Mark(),
                        "includes nested deeper than " + std::to_string(options_.max_include_depth),
                        ConfigDiag::too_large);
        auto text = read_text_file(path);
        if (!text) return include_error(text.error());

        std::string file = path.string();
        std::filesystem::path base = path.parent_path();
        std::swap(file_, file);
        std::swap(base_dir_, base);
        stack_.push_back(canonical);
        ++include_depth_;
        auto out = convert_text(*text, depth + 1);
        --include_depth_;
        stack_.pop_back();
        std::swap(file_, file);
        std::swap(base_dir_, base);
        // The included root has no useful position of its own (1:1); errors about it point at the !include.
        if (out) out->set_location(location(node.Mark()));
        return out;
    }

    SourceLocation location(const YAML::Mark& mark) const {
        if (mark.is_null() || mark.line < 0) return {file_, 0, 0};
        return {file_, mark.line + 1, mark.column + 1};
    }

    std::string file_;
    std::filesystem::path base_dir_;
    const YamlOptions& options_;
    std::vector<std::filesystem::path> stack_;
    int include_depth_{0};
    std::size_t count_{0};
    mutable std::set<std::string> used_;

  public:
    /// Explicit vars must be referenced somewhere, so a misspelled `--set` name is not silently ignored.
    Result<ConfigNode> check_used(Result<ConfigNode> out) const {
        if (!out || !options_.expand_vars) return out;
        for (const auto& kv : options_.vars)
            if (!used_.count(kv.first))
                return config_error(
                    ErrorCode::invalid_argument, ConfigDiag::unused_var, {file_, 0, 0}, {},
                    "variable '" + kv.first + "' is set but never referenced as ${" + kv.first + "}");
        return out;
    }
};

}  // namespace

Result<ConfigNode> parse_yaml(std::string_view text, const std::string& file, const YamlOptions& options) {
    Converter converter(file, options);
    return converter.check_used(converter.convert_text(std::string(text), 0));
}

Result<ConfigNode> load_yaml(const std::filesystem::path& path, const YamlOptions& options) {
    auto text = read_text_file(path);
    if (!text) return std::move(text).error();
    YamlOptions local = options;
    local.base_dir = path.parent_path();
    Converter converter(path.string(), local);
    std::error_code ec;
    const std::filesystem::path canonical = std::filesystem::weakly_canonical(path, ec);
    converter.push_file(ec ? path : canonical);
    return converter.check_used(converter.convert_text(*text, 0));
}

}  // namespace kstone
