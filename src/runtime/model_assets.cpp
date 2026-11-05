#include "model_assets.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <system_error>

namespace kstone::detail {

namespace fs = std::filesystem;

namespace {

class Sha256 {
  public:
    void update(const unsigned char* data, std::size_t n) {
        length_ += n;
        while (n > 0) {
            const std::size_t take = std::min(n, buffer_.size() - used_);
            std::copy(data, data + take, buffer_.begin() + static_cast<std::ptrdiff_t>(used_));
            used_ += take;
            data += take;
            n -= take;
            if (used_ == buffer_.size()) {
                block(buffer_.data());
                used_ = 0;
            }
        }
    }

    std::string hex() {
        const std::uint64_t bits = length_ * 8;
        const unsigned char pad = 0x80;
        update(&pad, 1);
        const unsigned char zero = 0;
        while (used_ != 56) update(&zero, 1);
        unsigned char len[8];
        for (int i = 0; i < 8; ++i) len[i] = static_cast<unsigned char>(bits >> (56 - 8 * i));
        update(len, 8);
        std::string out;
        char buf[9];
        for (std::uint32_t v : h_) {
            std::snprintf(buf, sizeof buf, "%08x", v);
            out += buf;
        }
        return out;
    }

  private:
    static std::uint32_t rotr(std::uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

    void block(const unsigned char* p) {
        static constexpr std::array<std::uint32_t, 64> k{
            0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
            0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
            0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
            0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
            0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
            0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
            0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
            0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
        std::array<std::uint32_t, 64> w{};
        for (std::size_t i = 0; i < 16; ++i)
            w[i] = (std::uint32_t{p[4 * i]} << 24) | (std::uint32_t{p[4 * i + 1]} << 16) |
                   (std::uint32_t{p[4 * i + 2]} << 8) | std::uint32_t{p[4 * i + 3]};
        for (std::size_t i = 16; i < 64; ++i) {
            const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        std::uint32_t a = h_[0], b = h_[1], c = h_[2], d = h_[3], e = h_[4], f = h_[5], g = h_[6], h = h_[7];
        for (std::size_t i = 0; i < 64; ++i) {
            const std::uint32_t t1 =
                h + (rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25)) + ((e & f) ^ (~e & g)) + k[i] + w[i];
            const std::uint32_t t2 = (rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
            h = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }
        h_[0] += a;
        h_[1] += b;
        h_[2] += c;
        h_[3] += d;
        h_[4] += e;
        h_[5] += f;
        h_[6] += g;
        h_[7] += h;
    }

    std::array<std::uint32_t, 8> h_{0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::array<unsigned char, 64> buffer_{};
    std::size_t used_{0};
    std::uint64_t length_{0};
};

Status hash_file(const fs::path& path, std::string& sha, std::uint64_t& size) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return make_error(ErrorCode::io_error, "cannot open '" + path.string() + "'");
    Sha256 h;
    std::array<char, 1 << 16> buf{};
    size = 0;
    while (in) {
        in.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        const auto n = static_cast<std::size_t>(in.gcount());
        h.update(reinterpret_cast<const unsigned char*>(buf.data()), n);
        size += n;
    }
    if (in.bad()) return make_error(ErrorCode::io_error, "cannot read '" + path.string() + "'");
    sha = h.hex();
    return {};
}

std::string xml_unescape(const std::string& s) {
    static const std::array<std::pair<const char*, char>, 5> entities{
        {{"&amp;", '&'}, {"&lt;", '<'}, {"&gt;", '>'}, {"&quot;", '"'}, {"&apos;", '\''}}};
    std::string out;
    for (std::size_t i = 0; i < s.size();) {
        bool hit = false;
        if (s[i] == '&')
            for (const auto& [name, c] : entities)
                if (s.compare(i, std::char_traits<char>::length(name), name) == 0) {
                    out += c;
                    i += std::char_traits<char>::length(name);
                    hit = true;
                    break;
                }
        if (!hit) out += s[i++];
    }
    return out;
}

std::string xml_escape(const std::string& s) {
    std::string out;
    for (char c : s) {
        switch (c) {
            case '&':
                out += "&amp;";
                break;
            case '<':
                out += "&lt;";
                break;
            case '>':
                out += "&gt;";
                break;
            case '"':
                out += "&quot;";
                break;
            case '\'':
                out += "&apos;";
                break;
            default:
                out += c;
        }
    }
    return out;
}

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return {};
    return s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
}

// Files a mesh references relative to itself: OBJ materials, MTL and DAE textures.
std::vector<fs::path> mesh_dependencies(const fs::path& path) {
    const std::string ext = lower(path.extension().string());
    if (ext != ".obj" && ext != ".mtl" && ext != ".dae") return {};
    std::ifstream in(path, std::ios::binary);
    if (!in) return {};
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string text = ss.str();
    std::vector<std::string> names;
    if (ext == ".dae") {
        static const std::regex image(R"(<init_from>\s*([^<]+?)\s*</init_from>)");
        for (std::sregex_iterator it(text.begin(), text.end(), image), end; it != end; ++it)
            names.push_back((*it)[1]);
    } else {
        std::istringstream lines(text);
        for (std::string line; std::getline(lines, line);) {
            const std::string t = trim(line);
            if (ext == ".obj" && lower(t.substr(0, 7)) == "mtllib ") {
                names.push_back(trim(t.substr(7)));
            } else if (ext == ".mtl" && lower(t.substr(0, 4)) == "map_") {
                const auto sp = t.find_last_of(" \t");
                if (sp != std::string::npos) names.push_back(t.substr(sp + 1));
            }
        }
    }
    std::vector<fs::path> found;
    for (std::string name : names) {
        if (name.rfind("file://", 0) == 0) name.erase(0, 7);
        std::error_code ec;
        const fs::path candidate = fs::weakly_canonical(path.parent_path() / name, ec);
        if (!ec && fs::is_regular_file(candidate, ec)) found.push_back(candidate);
    }
    return found;
}

}  // namespace

std::string sha256_hex(const std::string& data) {
    Sha256 h;
    h.update(reinterpret_cast<const unsigned char*>(data.data()), data.size());
    return h.hex();
}

std::string base64_encode(const std::string& data) {
    static constexpr char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((data.size() + 2) / 3 * 4);
    std::size_t i = 0;
    for (; i + 2 < data.size(); i += 3) {
        const std::uint32_t v = (std::uint32_t{static_cast<unsigned char>(data[i])} << 16) |
                                (std::uint32_t{static_cast<unsigned char>(data[i + 1])} << 8) |
                                std::uint32_t{static_cast<unsigned char>(data[i + 2])};
        out += table[(v >> 18) & 63];
        out += table[(v >> 12) & 63];
        out += table[(v >> 6) & 63];
        out += table[v & 63];
    }
    if (i < data.size()) {
        std::uint32_t v = std::uint32_t{static_cast<unsigned char>(data[i])} << 16;
        if (i + 1 < data.size()) v |= std::uint32_t{static_cast<unsigned char>(data[i + 1])} << 8;
        out += table[(v >> 18) & 63];
        out += table[(v >> 12) & 63];
        out += i + 1 < data.size() ? table[(v >> 6) & 63] : '=';
        out += '=';
    }
    return out;
}

ConfigNode ModelAssets::describe() const {
    ConfigNode doc = ConfigNode::map();
    doc.set("id", id);
    doc.set("model", model);
    ConfigNode list = ConfigNode::map();
    for (const File& f : files) {
        ConfigNode meta = ConfigNode::map();
        meta.set("sha256", f.sha256);
        meta.set("size", static_cast<double>(f.size));
        list.set(f.name, std::move(meta));
    }
    doc.set("files", std::move(list));
    ConfigNode miss = ConfigNode::list();
    for (const std::string& m : missing) miss.append(m);
    doc.set("missing", std::move(miss));
    return doc;
}

Status ModelAssets::read(const std::string& name, std::uint64_t offset, std::size_t max_bytes,
                         std::string& out, std::uint64_t& total) const {
    const auto it = std::find_if(files.begin(), files.end(), [&](const File& f) { return f.name == name; });
    if (it == files.end()) return make_error(ErrorCode::not_found, "no asset '" + name + "'");
    total = it->size;
    if (offset > total) return make_error(ErrorCode::out_of_range, "offset past the end of '" + name + "'");
    const auto n = static_cast<std::size_t>(std::min<std::uint64_t>(max_bytes, total - offset));
    if (it->path.empty()) {
        out = document.substr(static_cast<std::size_t>(offset), n);
        return {};
    }
    std::error_code ec;
    if (fs::file_size(it->path, ec) != total || ec)
        return make_error(ErrorCode::io_error, "asset '" + name + "' changed on the server since start");
    std::ifstream in(it->path, std::ios::binary);
    out.assign(n, '\0');
    if (!in || !in.seekg(static_cast<std::streamoff>(offset)) ||
        !in.read(out.data(), static_cast<std::streamsize>(n)))
        return make_error(ErrorCode::io_error, "cannot read asset '" + name + "'");
    return {};
}

Result<ModelAssets> build_model_assets(const ModelSource& source, const ResourceResolver& resolver) {
    if (source.format != "urdf")
        return make_error(ErrorCode::unsupported, "assets are only built for URDF models");
    std::string text;
    fs::path base = source.base_dir;
    if (source.kind == ModelSource::Kind::file) {
        auto loaded = read_text_file(source.location);
        if (!loaded) return std::move(loaded).error();
        text = std::move(*loaded);
        if (base.empty()) base = fs::path(source.location).parent_path();
    } else {
        text = source.content;
    }

    ModelAssets assets;
    std::map<std::string, ModelAssets::File> files;
    std::map<fs::path, std::string> folders;
    std::string error;

    auto add = [&](const fs::path& path, const std::string& name) {
        if (files.count(name) != 0) return true;
        ModelAssets::File f{name, path, {}, 0};
        if (Status s = hash_file(path, f.sha256, f.size); !s) {
            error = s.error().message;
            return false;
        }
        files.emplace(name, std::move(f));
        return true;
    };

    // Each source folder stays together so meshes still find their materials/textures by relative path.
    auto add_asset = [&](const fs::path& path) -> std::string {
        auto [it, fresh] = folders.try_emplace(path.parent_path(), std::string{});
        if (fresh) {
            char buf[32];
            std::snprintf(buf, sizeof buf, "assets/%03zu", folders.size() - 1);
            it->second = buf;
        }
        const std::string& folder = it->second;
        const std::string name = folder + "/" + path.filename().string();
        if (!add(path, name)) return {};
        std::vector<fs::path> pending{path};
        while (!pending.empty()) {
            const fs::path next = pending.back();
            pending.pop_back();
            for (const fs::path& dep : mesh_dependencies(next)) {
                const fs::path rel = dep.lexically_relative(path.parent_path());
                if (rel.empty() || *rel.begin() == "..") continue;  // the client would not find it either
                const std::string dep_name = folder + "/" + rel.generic_string();
                if (files.count(dep_name) != 0) continue;
                if (!add(dep, dep_name)) return {};
                pending.push_back(dep);
            }
        }
        return name;
    };

    static const std::regex ref(R"((\bfilename\s*=\s*)(["'])([^"']*)\2)");
    std::string rewritten;
    auto last = text.cbegin();
    for (std::sregex_iterator it(text.begin(), text.end(), ref), end; it != end; ++it) {
        const std::smatch& m = *it;
        rewritten.append(last, m[0].first);
        last = m[0].second;
        const std::string reference = xml_unescape(m[3]);
        auto resolved = resolver.resolve(reference, base);
        std::error_code ec;
        if (!resolved || !fs::is_regular_file(*resolved, ec)) {
            assets.missing.push_back(reference);
            rewritten.append(m[0].first, m[0].second);
            continue;
        }
        const fs::path canonical = fs::weakly_canonical(*resolved, ec);
        const std::string name = add_asset(ec ? *resolved : canonical);
        if (name.empty()) return make_error(ErrorCode::io_error, error);
        rewritten += m[1].str() + m[2].str() + xml_escape(name) + m[2].str();
    }
    rewritten.append(last, text.cend());
    std::sort(assets.missing.begin(), assets.missing.end());
    assets.missing.erase(std::unique(assets.missing.begin(), assets.missing.end()), assets.missing.end());

    assets.document = std::move(rewritten);
    files.emplace(assets.model,
                  ModelAssets::File{assets.model, {}, sha256_hex(assets.document), assets.document.size()});
    std::string manifest;
    for (auto& [name, f] : files) {
        manifest += name + '\n' + f.sha256 + '\n' + std::to_string(f.size) + '\n';
        assets.files.push_back(std::move(f));
    }
    assets.id = sha256_hex(manifest).substr(0, 32);
    return assets;
}

}  // namespace kstone::detail
