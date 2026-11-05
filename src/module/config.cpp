#include "kstone/module/config.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>

namespace kstone {

ConfigNode::ConfigNode(bool value) : kind_(Kind::boolean), bool_(value) {}
ConfigNode::ConfigNode(int value) : kind_(Kind::number), number_(value) {}
ConfigNode::ConfigNode(double value) : kind_(Kind::number), number_(value) {}
ConfigNode::ConfigNode(const char* value) : kind_(Kind::string), string_(value) {}
ConfigNode::ConfigNode(std::string value) : kind_(Kind::string), string_(std::move(value)) {}

ConfigNode ConfigNode::list(std::vector<ConfigNode> items) {
    ConfigNode n;
    n.kind_ = Kind::list;
    n.items_ = std::move(items);
    return n;
}

ConfigNode ConfigNode::map(std::vector<std::pair<std::string, ConfigNode>> entries) {
    ConfigNode n;
    n.kind_ = Kind::map;
    for (auto& [key, value] : entries) n.set(std::move(key), std::move(value));
    return n;
}

std::size_t ConfigNode::size() const noexcept {
    if (kind_ == Kind::list) return items_.size();
    if (kind_ == Kind::map) return entries_.size();
    return 0;
}

const ConfigNode* ConfigNode::find(std::string_view key) const noexcept {
    if (kind_ != Kind::map) return nullptr;
    for (const auto& [k, v] : entries_)
        if (k == key) return &v;
    return nullptr;
}

ConfigNode& ConfigNode::append(ConfigNode value) {
    if (kind_ == Kind::null) kind_ = Kind::list;
    items_.push_back(std::move(value));
    return *this;
}

ConfigNode& ConfigNode::set(std::string key, ConfigNode value) {
    if (kind_ == Kind::null) kind_ = Kind::map;
    for (auto& [k, v] : entries_) {
        if (k == key) {
            v = std::move(value);
            return *this;
        }
    }
    entries_.emplace_back(std::move(key), std::move(value));
    return *this;
}

ConfigNode& ConfigNode::set_location(SourceLocation location) {
    location_ = std::move(location);
    return *this;
}

const char* to_string(ConfigNode::Kind kind) noexcept {
    switch (kind) {
        case ConfigNode::Kind::null:
            return "null";
        case ConfigNode::Kind::boolean:
            return "bool";
        case ConfigNode::Kind::number:
            return "number";
        case ConfigNode::Kind::string:
            return "string";
        case ConfigNode::Kind::list:
            return "list";
        case ConfigNode::Kind::map:
            return "map";
    }
    return "unknown";
}

std::string to_string(ConfigDiag diag) {
    const int n = static_cast<int>(diag);
    std::string digits = std::to_string(n);
    return "KST-E" + std::string(digits.size() < 3 ? 3 - digits.size() : 0, '0') + digits;
}

Error config_error(ErrorCode code, ConfigDiag diag, const SourceLocation& loc, const std::string& path,
                   const std::string& message) {
    std::string text;
    if (!loc.file.empty() || loc.line > 0) {
        text = loc.file.empty() ? "<input>" : loc.file;
        if (loc.line > 0) text += ":" + std::to_string(loc.line) + ":" + std::to_string(loc.column);
        text += ": ";
    }
    if (!path.empty()) text += path + ": ";
    text += message + " [" + to_string(diag) + "]";
    return make_error(code, std::move(text));
}

std::string config_path(const std::string& parent, const std::string& key) {
    return parent.empty() ? key : parent + "." + key;
}

std::string config_path(const std::string& parent, std::size_t index) {
    return parent + "[" + std::to_string(index) + "]";
}

namespace {

// Length of the well-formed UTF-8 sequence at s[i], or 0 if it is malformed.
std::size_t utf8_length(const std::string& s, std::size_t i) {
    const auto b = [&](std::size_t k) { return static_cast<unsigned char>(s[i + k]); };
    const unsigned char c = b(0);
    const std::size_t n = c >= 0xF5 ? 0 : c >= 0xF0 ? 4 : c >= 0xE0 ? 3 : c >= 0xC2 ? 2 : 0;
    if (n == 0 || i + n > s.size()) return 0;
    for (std::size_t k = 1; k < n; ++k)
        if ((b(k) & 0xC0) != 0x80) return 0;
    if (c == 0xE0 && b(1) < 0xA0) return 0;
    if (c == 0xED && b(1) > 0x9F) return 0;
    if (c == 0xF0 && b(1) < 0x90) return 0;
    if (c == 0xF4 && b(1) > 0x8F) return 0;
    return n;
}

void write_string(std::string& out, const std::string& s) {
    out += '"';
    for (std::size_t i = 0; i < s.size(); ++i) {
        const char c = s[i];
        if (static_cast<unsigned char>(c) >= 0x80) {
            // Malformed bytes would make the whole document invalid JSON.
            const std::size_t n = utf8_length(s, i);
            if (n == 0) {
                out += "\\ufffd";
            } else {
                out.append(s, i, n);
                i += n - 1;
            }
            continue;
        }
        switch (c) {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof buf, "\\u%04x",
                                  static_cast<unsigned>(static_cast<unsigned char>(c)));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    out += '"';
}

void write_number(std::string& out, double v) {
    if (!std::isfinite(v)) {
        out += "null";
        return;
    }
    char buf[32];
    auto r = std::to_chars(buf, buf + sizeof buf, v);
    out.append(buf, r.ptr);
}

void newline(std::string& out, int indent, int depth) {
    if (indent <= 0) return;
    out += '\n';
    out.append(static_cast<std::size_t>(indent * depth), ' ');
}

void write(std::string& out, const ConfigNode& n, int indent, int depth) {
    switch (n.kind()) {
        case ConfigNode::Kind::null:
            out += "null";
            return;
        case ConfigNode::Kind::boolean:
            out += n.as_bool() ? "true" : "false";
            return;
        case ConfigNode::Kind::number:
            write_number(out, n.as_number());
            return;
        case ConfigNode::Kind::string:
            write_string(out, n.as_string());
            return;
        case ConfigNode::Kind::list: {
            if (n.items().empty()) {
                out += "[]";
                return;
            }
            out += '[';
            bool first = true;
            for (const auto& item : n.items()) {
                if (!first) out += ',';
                first = false;
                newline(out, indent, depth + 1);
                write(out, item, indent, depth + 1);
            }
            newline(out, indent, depth);
            out += ']';
            return;
        }
        case ConfigNode::Kind::map: {
            if (n.entries().empty()) {
                out += "{}";
                return;
            }
            out += '{';
            bool first = true;
            for (const auto& [key, value] : n.entries()) {
                if (!first) out += ',';
                first = false;
                newline(out, indent, depth + 1);
                write_string(out, key);
                out += indent > 0 ? ": " : ":";
                write(out, value, indent, depth + 1);
            }
            newline(out, indent, depth);
            out += '}';
            return;
        }
    }
}

}  // namespace

std::string to_json(const ConfigNode& node, int indent) {
    std::string out;
    write(out, node, indent, 0);
    return out;
}

}  // namespace kstone
