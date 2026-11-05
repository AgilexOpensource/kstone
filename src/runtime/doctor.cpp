#include "kstone/runtime/doctor.hpp"

#include "kstone/collision/model.hpp"
#include "kstone/collision/query.hpp"
#include "kstone/collision/world.hpp"
#include "kstone/control/solver.hpp"
#include "kstone/control/tasks.hpp"
#include "kstone/core/context.hpp"
#include "kstone/core/version.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"

#include <Eigen/Core>
#include <coal/config.hh>
#include <pinocchio/config.hpp>

#if defined(__unix__) || defined(__APPLE__)
#include <dlfcn.h>
#endif

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

namespace kstone {
namespace {

#if defined(__SANITIZE_ADDRESS__)
constexpr const char* kSanitizer = "+asan";
#elif defined(__SANITIZE_THREAD__)
constexpr const char* kSanitizer = "+tsan";
#elif defined(__has_feature)
#if __has_feature(address_sanitizer)
constexpr const char* kSanitizer = "+asan";
#elif __has_feature(thread_sanitizer)
constexpr const char* kSanitizer = "+tsan";
#else
constexpr const char* kSanitizer = "";
#endif
#else
constexpr const char* kSanitizer = "";
#endif

const char* kSliderUrdf = R"(<robot name="doctor">
  <link name="base"/>
  <link name="mid"/>
  <link name="tip"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <joint name="slide" type="prismatic"><parent link="base"/><child link="mid"/><axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
  <joint name="lift" type="prismatic"><parent link="mid"/><child link="tip"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
</robot>)";

std::string basename(const std::string& path) {
    const std::size_t slash = path.rfind('/');
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

// libfoo.so.1.2 -> libfoo.so
std::string stem(const std::string& name) {
    const std::size_t so = name.find(".so");
    return so == std::string::npos ? name : name.substr(0, so + 3);
}

bool starts_with(const std::string& s, const char* prefix) { return s.rfind(prefix, 0) == 0; }

bool tracked(const std::string& name) {
    static const char* const prefixes[] = {"libkstone", "libpinocchio", "libcoal",    "libhpp-fcl",
                                           "libboost_", "libeigenpy",   "liburdfdom", "libconsole_bridge",
                                           "libassimp", "libqhull",     "liboctomap", "libtinyxml",
                                           "kstone."};
    return std::any_of(std::begin(prefixes), std::end(prefixes),
                       [&](const char* p) { return starts_with(name, p); });
}

bool pinocchio_family(const std::string& name) {
    return starts_with(name, "libpinocchio") || starts_with(name, "libcoal") ||
           starts_with(name, "libhpp-fcl");
}

std::vector<LoadedLibrary> scan_libraries() {
    std::vector<LoadedLibrary> out;
#ifdef __linux__
    std::ifstream maps("/proc/self/maps");
    std::set<std::string> seen;
    std::string line;
    while (std::getline(maps, line)) {
        const std::size_t slash = line.find('/');
        if (slash == std::string::npos) continue;
        std::string path = line.substr(slash);
        const std::string deleted = " (deleted)";
        if (path.size() > deleted.size() &&
            path.compare(path.size() - deleted.size(), deleted.size(), deleted) == 0)
            path.resize(path.size() - deleted.size());
        std::string name = basename(path);
        if (!tracked(name) || !seen.insert(path).second) continue;
        out.push_back({std::move(name), path, library_provider(path)});
    }
#endif
    return out;
}

std::string location() {
#if defined(__unix__) || defined(__APPLE__)
    Dl_info info{};
    if (dladdr(reinterpret_cast<const void*>(&library_version_string), &info) != 0 &&
        info.dli_fname != nullptr)
        return info.dli_fname;
#endif
    return {};
}

DoctorCheck smoke_solve() {
    DoctorCheck check{"solve", false, {}};
    const Context context = Context::defaults();
    auto model = Model::load(ModelSource::text(kSliderUrdf, "urdf"), ModelOptions{}, context);
    if (!model) return check.detail = "model: " + model.error().message, check;
    auto solver = Solver::create(*model);
    if (!solver) return check.detail = "solver: " + solver.error().message, check;
    const Eigen::Vector2d target(0.5, -0.5);
    if (auto s = solver->add_task("posture", std::make_shared<PostureTask>(target)); !s)
        return check.detail = "task: " + s.error().message, check;
    SolveResult result;
    if (auto s = solver->solve(Eigen::Vector2d::Zero(), 0.01, result); !s)
        return check.detail = "solve: " + s.error().message, check;
    if (result.v.size() != 2 || !result.v.allFinite() || result.v(0) <= 0.0 || result.v(1) >= 0.0)
        return check.detail = "unexpected velocity", check;
    check.ok = true;
    check.detail = std::string("DAQP, ") + to_string(result.quality);
    return check;
}

DoctorCheck smoke_collision() {
    DoctorCheck check{"collision", false, {}};
    const Context context = Context::defaults();
    const ModelSource urdf = ModelSource::text(kSliderUrdf, "urdf");
    auto model = Model::load(urdf, ModelOptions{}, context);
    if (!model) return check.detail = "model: " + model.error().message, check;
    CollisionModelOptions options;
    options.self_collision = false;
    auto collision = CollisionModel::from_urdf(*model, urdf, options, context);
    if (!collision) return check.detail = "collision model: " + collision.error().message, check;
    auto world = std::make_shared<CollisionWorld>();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
    if (auto id = world->add("ball", Shape::sphere(0.2), pose); !id)
        return check.detail = "world: " + id.error().message, check;
    Workspace workspace(*model);
    if (auto s = workspace.update(Eigen::Vector2d::Zero()); !s)
        return check.detail = "workspace: " + s.error().message, check;
    CollisionQuery query(*collision, world);
    if (auto s = query.update(workspace, 1.0); !s) return check.detail = "query: " + s.error().message, check;
    const double d = query.nearest().distance;
    if (std::abs(d - 0.7) > 1e-6) {
        std::ostringstream text;
        text << "distance " << d << ", expected 0.7";
        return check.detail = text.str(), check;
    }
    check.ok = true;
    check.detail = "coal";
    return check;
}

ConfigNode text_list(const std::vector<std::pair<std::string, std::string>>& pairs) {
    ConfigNode out = ConfigNode::map();
    for (const auto& [k, v] : pairs) out.set(k, v);
    return out;
}

}  // namespace

std::string library_provider(const std::string& path) {
    if (path.find("/cmeel.prefix/") != std::string::npos ||
        path.find("/site-packages/") != std::string::npos ||
        path.find("/dist-packages/") != std::string::npos)
        return "pypi";
    if (starts_with(path, "/opt/ros/")) return "ros";
    if (starts_with(path, "/usr/") || starts_with(path, "/lib")) return "system";
    return "other";
}

DoctorCheck check_libraries(const std::vector<LoadedLibrary>& libraries) {
    DoctorCheck check{"libraries", true, {}};
    std::map<std::string, std::vector<const LoadedLibrary*>> by_stem;
    std::set<std::string> providers;
    for (const auto& lib : libraries) {
        by_stem[stem(lib.name)].push_back(&lib);
        if (pinocchio_family(lib.name)) providers.insert(lib.provider);
    }
    std::ostringstream problems;
    for (const auto& [name, libs] : by_stem) {
        if (libs.size() < 2) continue;
        problems << (problems.tellp() > 0 ? "; " : "") << name << " loaded twice:";
        for (const auto* lib : libs) problems << ' ' << lib->path;
    }
    if (providers.size() > 1) {
        problems << (problems.tellp() > 0 ? "; " : "") << "Pinocchio libraries from mixed providers:";
        for (const auto& lib : libraries)
            if (pinocchio_family(lib.name)) problems << ' ' << lib.name << '=' << lib.provider;
    }
    if (problems.tellp() > 0) {
        check.ok = false;
        check.detail = problems.str();
    } else if (libraries.empty()) {
        check.detail = "scan unsupported on this platform";
    } else if (!providers.empty()) {
        check.detail = "Pinocchio from " + *providers.begin();
    }
    return check;
}

DoctorReport doctor() {
    DoctorReport report;
    report.version = library_version_string();
#ifdef NDEBUG
    report.build = std::string("release") + kSanitizer;
#else
    report.build = std::string("debug") + kSanitizer;
#endif
#ifdef KSTONE_STATIC_DEFINE
    report.shared = false;
#else
    report.shared = true;
#endif
    report.location = location();
    report.compiled = {
        {"eigen", std::to_string(EIGEN_WORLD_VERSION) + "." + std::to_string(EIGEN_MAJOR_VERSION) + "." +
                      std::to_string(EIGEN_MINOR_VERSION)},
        {"pinocchio", PINOCCHIO_VERSION},
        {"coal", COAL_VERSION},
    };
    report.libraries = scan_libraries();
    report.checks.push_back(check_libraries(report.libraries));
    report.checks.push_back(smoke_solve());
    report.checks.push_back(smoke_collision());
    return report;
}

ConfigNode to_config_node(const DoctorReport& report) {
    ConfigNode libraries = ConfigNode::list();
    for (const auto& lib : report.libraries)
        libraries.append(
            ConfigNode::map({{"name", lib.name}, {"path", lib.path}, {"provider", lib.provider}}));
    ConfigNode checks = ConfigNode::list();
    for (const auto& c : report.checks)
        checks.append(ConfigNode::map({{"name", c.name}, {"ok", c.ok}, {"detail", c.detail}}));
    return ConfigNode::map({{"ok", report.ok()},
                            {"version", report.version},
                            {"build", report.build},
                            {"shared", report.shared},
                            {"location", report.location},
                            {"compiled", text_list(report.compiled)},
                            {"libraries", std::move(libraries)},
                            {"checks", std::move(checks)}});
}

std::string to_text(const DoctorReport& report) {
    std::ostringstream out;
    out << "kstone " << report.version << " (" << report.build << ", "
        << (report.shared ? "shared" : "static") << ")\n";
    if (!report.location.empty()) out << "  location  " << report.location << '\n';
    out << "  compiled ";
    for (const auto& [k, v] : report.compiled) out << ' ' << k << ' ' << v;
    out << '\n';
    if (!report.libraries.empty()) out << "loaded libraries:\n";
    for (const auto& lib : report.libraries) {
        out << "  " << lib.provider << std::string(lib.provider.size() < 8 ? 8 - lib.provider.size() : 1, ' ')
            << lib.path << '\n';
    }
    out << "checks:\n";
    for (const auto& c : report.checks)
        out << "  " << (c.ok ? "ok  " : "FAIL") << "  " << c.name << (c.detail.empty() ? "" : ": ")
            << c.detail << '\n';
    out << (report.ok() ? "ok\n" : "FAILED\n");
    return out.str();
}

}  // namespace kstone
