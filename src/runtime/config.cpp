#include "kstone/runtime/config.hpp"

#include "kstone/module/yaml.hpp"
#include "kstone/runtime/robot_plugin.hpp"

#include "../module/builtin_schemas.hpp"
#include <algorithm>
#include <cstring>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace kstone {
namespace {

enum class RobotType { loopback, plugin };

struct ModelSection {
    std::string file;
    std::string format;
    std::vector<std::string> package_dirs;
    ModelOptions options;
};

struct RobotSection {
    RobotType type{RobotType::loopback};
    Eigen::VectorXd initial;
    std::optional<Eigen::Isometry3d> initial_root;
    std::string library;
};

struct EngineConfig {
    ModelSection model;
    EngineOptions engine;
    SolverOptions solver;
    RobotSection robot;
};

ParamSchema<EngineConfig> engine_schema() {
    ParamSchema<ModelOptions> options;
    options.field("root", &ModelOptions::root,
                  {{"fixed", RootJoint::fixed}, {"free_flyer", RootJoint::free_flyer}});
    options.field("root_joint_name", &ModelOptions::root_joint_name).non_empty();
    options.field("mimic", &ModelOptions::mimic,
                  {{"independent", MimicMode::independent}, {"coupled", MimicMode::coupled}});

    ParamSchema<ModelSection> model("Robot model.");
    model.field("file", &ModelSection::file).required().non_empty().doc("Path or URI.");
    model.field("format", &ModelSection::format).doc("Empty = from the file extension.");
    model.field("package_dirs", &ModelSection::package_dirs)
        .doc(
            "Roots for package://<name>/<path>, tried in order; relative entries resolve against the config "
            "file.");
    model.merge(&ModelSection::options, options);

    ParamSchema<CommandGuardOptions> safety("Command guard before every robot write.");
    safety.field("mode", &CommandGuardOptions::mode,
                 {{"reject", GuardMode::reject}, {"clamp", GuardMode::clamp}, {"off", GuardMode::off}});
    safety.field("velocity_scale", &CommandGuardOptions::velocity_scale)
        .above(0)
        .doc(
            "Multiplies model velocity limits for the guard only; modules still plan at model limits, so to "
            "slow the robot lower model.joint_limits.velocity.");
    safety.field("tolerance", &CommandGuardOptions::tolerance).at_least(0);

    ParamSchema<ThreadOptions> thread("Real-time setup when run starts (Linux); refused settings only warn.");
    thread.field("priority", &ThreadOptions::priority)
        .range(0, 99)
        .doc(
            "0 = keep the current policy; 1..99 = SCHED_FIFO (higher wins), needs CAP_SYS_NICE or `ulimit "
            "-r` >= priority.");
    thread.field("cpus", &ThreadOptions::cpus)
        .at_least(0)
        .doc("CPU indices to pin to; empty = keep the affinity.");
    thread.field("lock_memory", &ThreadOptions::lock_memory)
        .doc("mlockall the whole process, never undone; needs CAP_IPC_LOCK or a large `ulimit -l`.");

    ParamSchema<EngineOptions> engine;
    engine.field("rate_hz", &EngineOptions::rate_hz).above(0);
    engine.field("realtime", &EngineOptions::realtime);
    engine.field("feedback_timeout_s", &EngineOptions::feedback_timeout_s)
        .at_least(0)
        .doc("Default 0.25; 0 = feedback never stale.");
    engine.field("max_failed_cycles", &EngineOptions::max_failed_cycles)
        .at_least(1)
        .doc("Default 5; run stops after this many consecutive failed cycles (1 = at the first).");
    engine.field("feedback_joints", &EngineOptions::feedback_joints)
        .doc("Names or fnmatch patterns, each must select a joint; empty = every scalar joint.");
    engine.field("safety", &EngineOptions::safety, std::move(safety));
    engine.field("thread", &EngineOptions::thread, std::move(thread));

    ParamSchema<RobotSection> robot("Robot interface.");
    robot.field("type", &RobotSection::type,
                {{"loopback", RobotType::loopback}, {"plugin", RobotType::plugin}});
    robot.field("initial", &RobotSection::initial)
        .doc(
            "Initial q: a list of size nq, or a map {joint: value} over the model neutral; empty = neutral. "
            "Also the modules' initial posture.");
    robot.field("initial_root", &RobotSection::initial_root)
        .doc("Initial root pose {translation, rpy|quaternion}. Not with a list initial.");
    robot.field("library", &RobotSection::library)
        .doc(
            "Plugin shared library; paths with '/' resolve against the config file, bare names use the "
            "loader's search path.");

    ParamSchema<EngineConfig> s("Engine configuration.");
    s.field("model", &EngineConfig::model, std::move(model)).required();
    s.merge(&EngineConfig::engine, engine);
    s.field("solver", &EngineConfig::solver, builtin_schemas::solver());
    s.field("robot", &EngineConfig::robot, std::move(robot));
    return s;
}

Status initial_error(const ConfigNode& node, const std::string& path, std::string message) {
    return config_error(ErrorCode::invalid_argument, ConfigDiag::invalid_value, node.location(), path,
                        std::move(message));
}

// Empty when the joint has no position limit or the value is within it.
std::string limit_violation(const Model& model, JointId joint, double value) {
    if (!model.has_position_limit(joint)) return {};
    const Index i = model.joints()[static_cast<std::size_t>(joint)].idx_q;
    const double lo = model.lower_position_limit()[i], hi = model.upper_position_limit()[i];
    if (value >= lo - 1e-6 && value <= hi + 1e-6) return {};
    std::ostringstream out;
    out << value << " outside [" << lo << ", " << hi << ']';
    return out.str();
}

// Map form of robot.initial: named scalar joints over neutral; unset mimic followers track their leader.
Status apply_initial_map(const Model& model, const ConfigNode& map, Eigen::VectorXd& q) {
    std::vector<bool> set(model.joints().size(), false);
    for (const auto& [name, node] : map.entries()) {
        const std::string path = "robot.initial." + name;
        if (node.kind() != ConfigNode::Kind::number)
            return config_error(ErrorCode::invalid_argument, ConfigDiag::wrong_type, node.location(), path,
                                "expected a number, got " + std::string(to_string(node.kind())));
        const double value = node.as_number();
        auto joint = model.joint_id(name);
        if (!joint) return initial_error(node, path, "unknown joint");
        if (auto v = limit_violation(model, *joint, value); !v.empty()) return initial_error(node, path, v);
        if (auto s = model.set_joint_position(q, *joint, value); !s)
            return initial_error(node, path, s.error().message);
        set[static_cast<std::size_t>(*joint)] = true;
    }
    for (const auto& m : model.mimics()) {
        auto leader = model.joint_id(m.leader);
        auto follower = model.joint_id(m.follower);
        if (!leader || !follower || !set[static_cast<std::size_t>(*leader)] ||
            set[static_cast<std::size_t>(*follower)])
            continue;
        if (model.joints()[static_cast<std::size_t>(*follower)].nv != 1)
            continue;  // coupled: no own coordinate
        const ConfigNode& node = *map.find(m.leader);
        const std::string path = "robot.initial." + m.leader;
        const double value = m.multiplier * *model.joint_position(q, *leader) + m.offset;
        if (auto v = limit_violation(model, *follower, value); !v.empty())
            return initial_error(node, path, "mimic follower " + m.follower + ": " + v);
        if (auto s = model.set_joint_position(q, *follower, value); !s)
            return initial_error(node, path, s.error().message);
    }
    return {};
}

Status limits_error(const ConfigNode& node, const std::string& path, std::string message,
                    ConfigDiag diag = ConfigDiag::invalid_value) {
    return config_error(ErrorCode::invalid_argument, diag, node.location(), path, std::move(message));
}

Status expect_map(const ConfigNode& node, const std::string& path) {
    if (node.kind() == ConfigNode::Kind::map) return {};
    return limits_error(node, path, "expected a map, got " + std::string(to_string(node.kind())),
                        ConfigDiag::wrong_type);
}

Result<double> limit_number(const ConfigNode& node, const std::string& path) {
    if (node.kind() != ConfigNode::Kind::number)
        return limits_error(node, path, "expected a number, got " + std::string(to_string(node.kind())),
                            ConfigDiag::wrong_type)
            .error();
    return node.as_number();
}

// model.joint_limits: {velocity: {<joint>}, position: {lower: {<joint>}, upper: {<joint>}}}; replaces the model limits.
Result<ModelPtr> apply_joint_limits(ModelPtr model, const ConfigNode& limits) {
    const std::string root = "model.joint_limits";
    if (auto s = expect_map(limits, root); !s) return std::move(s).error();
    struct Position {
        JointLimitOverride o;
        const ConfigNode* node;
        std::string path;
    };
    std::vector<Position> positions;
    std::vector<std::pair<JointLimitOverride, std::pair<const ConfigNode*, std::string>>> velocities;
    for (const auto& [key, section] : limits.entries()) {
        const std::string spath = root + "." + key;
        if (auto s = expect_map(section, spath); !s) return std::move(s).error();
        if (key == "velocity") {
            for (const auto& [name, node] : section.entries()) {
                const std::string path = spath + "." + name;
                auto v = limit_number(node, path);
                if (!v) return std::move(v).error();
                velocities.push_back({JointLimitOverride{name, {}, {}, *v}, {&node, path}});
            }
        } else if (key == "position") {
            for (const auto& [side, joints] : section.entries()) {
                const std::string sidepath = spath + "." + side;
                if (side != "lower" && side != "upper")
                    return limits_error(section, sidepath, "unknown key; expected lower or upper",
                                        ConfigDiag::unknown_key)
                        .error();
                if (auto s = expect_map(joints, sidepath); !s) return std::move(s).error();
                for (const auto& [name, node] : joints.entries()) {
                    const std::string path = sidepath + "." + name;
                    auto v = limit_number(node, path);
                    if (!v) return std::move(v).error();
                    auto it = std::find_if(positions.begin(), positions.end(),
                                           [&](const Position& p) { return p.o.joint == name; });
                    if (it == positions.end())
                        it = positions.insert(positions.end(), {{name, {}, {}, {}}, &node, path});
                    (side == "lower" ? it->o.lower : it->o.upper) = *v;
                }
            }
        } else {
            return limits_error(section, spath, "unknown key; expected velocity or position",
                                ConfigDiag::unknown_key)
                .error();
        }
    }
    auto replace = [&](const JointLimitOverride& o, const ConfigNode& node,
                       const std::string& path) -> Status {
        auto next = model->with_limits({o});
        if (!next) return limits_error(node, path, next.error().message);
        model = std::move(*next);
        return {};
    };
    for (const auto& p : positions)
        if (auto s = replace(p.o, *p.node, p.path); !s) return std::move(s).error();
    for (const auto& [o, where] : velocities)
        if (auto s = replace(o, *where.first, where.second); !s) return std::move(s).error();
    return model;
}

}  // namespace

Result<Engine> build_engine(const ConfigNode& config, const std::filesystem::path& base_dir,
                            const EngineLoadOptions& options, const Context& context) {
    if (config.kind() != ConfigNode::Kind::map)
        return config_error(ErrorCode::invalid_argument, ConfigDiag::wrong_type, config.location(), {},
                            "expected a map, got " + std::string(to_string(config.kind())));
    // `modules` is read by the registry; everything else by the engine schema.
    ConfigNode rest = ConfigNode::map();
    rest.set_location(config.location());
    ConfigNode modules;
    ConfigNode robot_params;
    ConfigNode initial_map;
    ConfigNode joint_limits;
    for (const auto& [key, value] : config.entries()) {
        if (key == "modules") {
            modules = value;
        } else if (key == "model" && value.kind() == ConfigNode::Kind::string) {
            return config_error(
                ErrorCode::invalid_argument, ConfigDiag::wrong_type, value.location(), "model",
                "expected a map, got a string; write model: {file: " + value.as_string() + "}");
        } else if (key == "model" && value.kind() == ConfigNode::Kind::map) {
            ConfigNode model = ConfigNode::map();
            model.set_location(value.location());
            for (const auto& [k, v] : value.entries()) {
                if (k == "joint_limits")
                    joint_limits = v;  // name-keyed; applied after the model loads
                else
                    model.set(k, v);
            }
            rest.set(key, std::move(model));
        } else if (key == "robot" && value.kind() == ConfigNode::Kind::map) {
            // robot.params is free-form and belongs to the plugin.
            ConfigNode robot = ConfigNode::map();
            robot.set_location(value.location());
            for (const auto& [k, v] : value.entries()) {
                if (k == "params")
                    robot_params = v;
                else if (k == "initial" && v.kind() == ConfigNode::Kind::map)
                    initial_map = v;  // name form; applied after the model loads
                else
                    robot.set(k, v);
            }
            rest.set(key, std::move(robot));
        } else {
            rest.set(key, value);
        }
    }
    EngineConfig cfg;
    if (auto s = engine_schema().read(rest, cfg); !s) {
        // Keys split off above are not in the schema; list them so the hint is complete.
        Error e = std::move(s).error();
        for (const auto& [tail, extra] : {std::pair<const char*, const char*>{"solver, robot)", ", modules"},
                                          {"root_joint_name, mimic)", ", joint_limits"},
                                          {"initial_root, library)", ", params"}})
            if (const std::size_t at = e.message.find(tail); at != std::string::npos)
                e.message.insert(at + std::strlen(tail) - 1, extra);
        return e;
    }

    const ConfigNode& model_node = *config.find("model");
    Context ctx = context;
    if (!cfg.model.package_dirs.empty()) {
        std::vector<std::filesystem::path> roots;
        for (const auto& dir : cfg.model.package_dirs) {
            std::filesystem::path root{dir};
            roots.push_back(root.is_relative() ? (base_dir / root).lexically_normal() : root);
        }
        // package_dirs first; a host-registered package handler stays the fallback.
        SchemeHandler own = make_directory_scheme(std::move(roots));
        SchemeHandler prev;
        if (context.resources.has_scheme("package"))
            prev = [resources = context.resources](std::string_view ref) {
                return resources.resolve("package://" + std::string(ref));
            };
        ctx.resources.add_scheme("package",
                                 [own, prev](std::string_view ref) -> Result<std::filesystem::path> {
                                     auto found = own(ref);
                                     if (found || !prev || found.error().code != ErrorCode::not_found)
                                         return found;
                                     return prev(ref);
                                 });
    }
    ModelSource source = ModelSource::file(cfg.model.file, cfg.model.format);
    source.base_dir = base_dir;
    auto model = Model::load(source, cfg.model.options, ctx);
    if (!model) {
        const ConfigNode* file = model_node.find("file");
        return config_error(model.error().code, ConfigDiag::model_failed, file->location(), "model.file",
                            model.error().message);
    }
    if (!joint_limits.is_null()) {
        auto limited = apply_joint_limits(*model, joint_limits);
        if (!limited) return std::move(limited).error();
        model = std::move(limited);
    }

    const ConfigNode* robot_node = config.find("robot");
    const auto robot_location = [&](const char* key) {
        const ConfigNode* n = robot_node ? robot_node->find(key) : nullptr;
        return n ? n->location() : robot_node ? robot_node->location() : config.location();
    };
    Eigen::VectorXd q0 = cfg.robot.initial.size() == 0 ? (*model)->neutral() : cfg.robot.initial;
    if (!initial_map.is_null())
        if (auto s = apply_initial_map(**model, initial_map, q0); !s) return std::move(s).error();
    if (cfg.robot.initial_root) {
        if (cfg.robot.initial.size() != 0)
            return config_error(ErrorCode::invalid_argument, ConfigDiag::invalid_value,
                                robot_location("initial_root"), "robot.initial_root",
                                "not combinable with a list robot.initial; use the map form");
        if ((*model)->root() == RootJoint::fixed)
            return config_error(ErrorCode::invalid_argument, ConfigDiag::invalid_value,
                                robot_location("initial_root"), "robot.initial_root", "model root is fixed");
        if (auto s = (*model)->set_root_pose(q0, *cfg.robot.initial_root); !s)
            return config_error(s.error().code, ConfigDiag::invalid_value, robot_location("initial_root"),
                                "robot.initial_root", s.error().message);
    }
    if (q0.size() != (*model)->nq())
        return config_error(
            ErrorCode::invalid_argument, ConfigDiag::robot_mismatch, robot_location("initial"),
            "robot.initial",
            "expected " + std::to_string((*model)->nq()) + " values, got " + std::to_string(q0.size()));

    auto set = ModuleSet::create(*model, cfg.solver, ctx);
    if (!set) return std::move(set).error();
    if (auto s = set->set_initial(q0); !s)
        return config_error(s.error().code, ConfigDiag::robot_mismatch, robot_location("initial"),
                            "robot.initial", s.error().message);
    const ModuleRegistry builtins = options.registry ? ModuleRegistry{} : ModuleRegistry::with_builtins();
    const ModuleRegistry& registry = options.registry ? *options.registry : builtins;
    if (auto s = registry.populate(*set, modules, ctx, "modules", base_dir); !s) return std::move(s).error();

    std::shared_ptr<RobotInterface> robot = options.robot;
    if (!robot) {
        if (cfg.robot.type == RobotType::plugin) {
            if (cfg.robot.library.empty())
                return config_error(ErrorCode::invalid_argument, ConfigDiag::missing_key,
                                    robot_location("type"), "robot.library", "required for type plugin");
            if (!robot_params.is_null() && robot_params.kind() != ConfigNode::Kind::map)
                return config_error(ErrorCode::invalid_argument, ConfigDiag::wrong_type,
                                    robot_params.location(), "robot.params",
                                    "expected a map, got " + std::string(to_string(robot_params.kind())));
            RobotPluginOptions plugin;
            plugin.library = cfg.robot.library;
            if (cfg.robot.library.find('/') != std::string::npos && plugin.library.is_relative()) {
                // Absolute, so normalising "./x.so" can never turn it into a bare name dlopen would search for.
                std::error_code ec;
                auto abs = std::filesystem::absolute(base_dir / plugin.library, ec);
                plugin.library = (ec ? base_dir / plugin.library : abs).lexically_normal();
            }
            plugin.params_json = robot_params.is_null() ? "{}" : to_json(robot_params, 0);
            plugin.base_dir = base_dir;
            plugin.rate_hz = cfg.engine.rate_hz;
            plugin.initial = std::move(q0);
            auto loaded = load_robot_plugin(*model, plugin);
            if (!loaded)
                return config_error(loaded.error().code, ConfigDiag::robot_failed, robot_location("library"),
                                    "robot.library", loaded.error().message);
            robot = std::move(*loaded);
        } else {
            robot = std::make_shared<LoopbackRobot>(std::move(q0), (*model)->nv(), ctx.clock);
        }
    }
    auto engine = Engine::create(std::move(*set), std::move(robot), cfg.engine, ctx);
    if (!engine && modules.kind() == ConfigNode::Kind::map) {
        // Module configure() errors carry only "module 'id': "; point them at the module's entry.
        const std::string& text = engine.error().message;
        for (const auto& [id, node] : modules.entries()) {
            const std::string prefix = "module '" + id + "': ";
            if (text.rfind(prefix, 0) == 0)
                return config_error(engine.error().code, ConfigDiag::module_failed, node.location(),
                                    config_path("modules", id), text.substr(prefix.size()));
        }
    }
    return engine;
}

Result<Engine> load_engine(const std::filesystem::path& path, const EngineLoadOptions& options,
                           const Context& context) {
    YamlOptions yaml = options.yaml;
    if (yaml.resources == nullptr) yaml.resources = &context.resources;
    auto config = load_yaml(path, yaml);
    if (!config) return std::move(config).error();
    return build_engine(*config, path.parent_path(), options, context);
}

}  // namespace kstone
