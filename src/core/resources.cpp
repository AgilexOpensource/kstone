#include "kstone/core/resources.hpp"

#include <cerrno>
#include <fstream>
#include <sstream>
#include <system_error>

namespace kstone {

namespace fs = std::filesystem;

namespace {

bool is_scheme_char(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '+' ||
           c == '-' || c == '.';
}

bool path_exists(const fs::path& p) {
    std::error_code ec;
    return fs::exists(p, ec) && !ec;
}

}  // namespace

ParsedUri parse_uri(std::string_view uri) {
    const auto pos = uri.find("://");
    // Require at least two scheme characters so "C://x" is not taken as a scheme.
    if (pos == std::string_view::npos || pos < 2) return {std::string(), std::string(uri)};
    for (std::size_t i = 0; i < pos; ++i)
        if (!is_scheme_char(uri[i])) return {std::string(), std::string(uri)};
    return {std::string(uri.substr(0, pos)), std::string(uri.substr(pos + 3))};
}

ResourceResolver::ResourceResolver() {
    add_scheme("file", [](std::string_view rest) -> Result<fs::path> {
        fs::path p{std::string(rest)};
        if (path_exists(p)) return p;
        return make_error(ErrorCode::not_found, "file not found: " + p.string());
    });
}

ResourceResolver& ResourceResolver::add_scheme(std::string scheme, SchemeHandler handler) {
    for (auto& e : schemes_) {
        if (e.scheme == scheme) {
            e.handler = std::move(handler);
            return *this;
        }
    }
    schemes_.push_back(Entry{std::move(scheme), std::move(handler)});
    return *this;
}

bool ResourceResolver::has_scheme(std::string_view scheme) const noexcept {
    for (const auto& e : schemes_)
        if (e.scheme == scheme) return true;
    return false;
}

ResourceResolver& ResourceResolver::add_search_dir(fs::path dir) {
    search_dirs_.push_back(std::move(dir));
    return *this;
}

Result<fs::path> ResourceResolver::resolve(std::string_view uri, const fs::path& base_dir) const {
    if (uri.empty()) return make_error(ErrorCode::invalid_argument, "empty resource reference");
    const ParsedUri parsed = parse_uri(uri);
    if (!parsed.scheme.empty()) {
        for (const auto& e : schemes_) {
            if (e.scheme != parsed.scheme) continue;
            if (!e.handler) break;
            try {
                return e.handler(parsed.rest);
            } catch (const std::exception& ex) {
                return make_error(ErrorCode::internal,
                                  "scheme handler '" + parsed.scheme + "' threw: " + ex.what());
            }
        }
        return make_error(ErrorCode::not_found, "no handler registered for scheme '" + parsed.scheme + "'");
    }

    const fs::path p{std::string(uri)};
    if (p.is_absolute()) {
        if (path_exists(p)) return p;
        return make_error(ErrorCode::not_found, "file not found: " + p.string());
    }
    if (!base_dir.empty() && path_exists(base_dir / p)) return base_dir / p;
    for (const auto& dir : search_dirs_)
        if (path_exists(dir / p)) return dir / p;
    if (base_dir.empty() && path_exists(p)) return p;  // relative to the process working directory
    std::string tried;
    const auto add = [&](const fs::path& d) {
        std::error_code ec;
        tried += (tried.empty() ? "" : ", ") + fs::absolute(d / p, ec).lexically_normal().string();
    };
    if (!base_dir.empty()) add(base_dir);
    for (const auto& dir : search_dirs_) add(dir);
    if (base_dir.empty()) add({});
    return make_error(ErrorCode::not_found,
                      "cannot resolve '" + std::string(uri) + "' (tried " + tried + ")");
}

Result<std::string> ResourceResolver::read_text(std::string_view uri, const fs::path& base_dir) const {
    auto path = resolve(uri, base_dir);
    if (!path) return std::move(path).error();
    return read_text_file(*path);
}

SchemeHandler make_directory_scheme(std::vector<fs::path> roots) {
    return [roots = std::move(roots)](std::string_view rest) -> Result<fs::path> {
        const fs::path rel{std::string(rest)};
        bool escapes = rel.empty() || rel.has_root_path();
        for (const auto& part : rel) escapes = escapes || part == "..";
        if (escapes)
            return make_error(ErrorCode::invalid_argument,
                              "bad package reference '" + std::string(rest) + "'");
        for (const auto& root : roots)
            if (path_exists(root / rel)) return root / rel;
        return make_error(ErrorCode::not_found, "'" + std::string(rest) + "' not found in any root");
    };
}

std::vector<fs::path> split_path_list(std::string_view list) {
#ifdef _WIN32
    constexpr char sep = ';';
#else
    constexpr char sep = ':';
#endif
    std::vector<fs::path> out;
    std::size_t start = 0;
    while (start <= list.size()) {
        auto end = list.find(sep, start);
        if (end == std::string_view::npos) end = list.size();
        if (end > start) out.emplace_back(std::string(list.substr(start, end - start)));
        start = end + 1;
    }
    return out;
}

Result<std::string> read_text_file(const fs::path& path) {
    std::error_code ec;
    if (fs::is_directory(path, ec)) return make_error(ErrorCode::io_error, path.string() + " is a directory");
    errno = 0;
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        const int err = errno;
        return make_error(ErrorCode::io_error,
                          "cannot open " + path.string() +
                              (err ? ": " + std::generic_category().message(err) : std::string()));
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    if (in.bad()) return make_error(ErrorCode::io_error, "read failed: " + path.string());
    return ss.str();
}

}  // namespace kstone
