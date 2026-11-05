#include "kstone/runtime/debug_server.hpp"

#include "kstone/model/workspace.hpp"
#include "kstone/module/config.hpp"
#include "kstone/module/yaml.hpp"

#include "model_assets.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <optional>
#include <poll.h>
#include <string_view>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <utility>
#include <vector>

namespace kstone {

namespace {

constexpr std::size_t kRingSize = 64;
constexpr std::size_t kMaxClients = 8;
constexpr std::uint32_t kMaxFrame = 1u << 20;
constexpr std::size_t kMaxPending = 8u << 20;  // Output bytes before a slow client is dropped.
constexpr std::size_t kStateBacklog =
    256u << 10;  // Above this a client skips state ticks instead of queueing them.
constexpr std::size_t kAssetChunk = 256u << 10;

// Single-producer single-consumer ring over preallocated slots.
template <typename T, std::size_t N>
class Ring {
  public:
    T* claim() noexcept {
        const std::size_t h = head_.load(std::memory_order_relaxed);
        return h - tail_.load(std::memory_order_acquire) == N ? nullptr : &slots_[h % N];
    }
    void push() noexcept {
        head_.store(head_.load(std::memory_order_relaxed) + 1, std::memory_order_release);
    }
    T* front() noexcept {
        const std::size_t t = tail_.load(std::memory_order_relaxed);
        return t == head_.load(std::memory_order_acquire) ? nullptr : &slots_[t % N];
    }
    void pop() noexcept { tail_.store(tail_.load(std::memory_order_relaxed) + 1, std::memory_order_release); }
    /// Slot of the i-th push.
    T& at(std::size_t i) noexcept { return slots_[i % N]; }

  private:
    std::array<T, N> slots_{};
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
};

using SteadyClock = std::chrono::steady_clock;

constexpr int kProtocolVersion = 1;
constexpr const char* kCommandOps[] = {"set_port", "set_enabled", "home",     "tune",       "tune_solver",
                                       "pause",    "step",        "set_rate", "tune_limits"};
constexpr double kMaxTimeoutS = 60.0;
constexpr auto kAuthGrace = std::chrono::seconds(5);  // Unauthenticated connections are dropped after this.
constexpr std::size_t kMinToken = 16;
constexpr std::size_t kMaxToken = 1024;

enum class Op { set_port, set_enabled, home, tune, tune_solver, pause, step, set_rate, tune_limits };
constexpr double kMaxStepCycles = 1e6;
constexpr double kMaxRateHz = 1e4;

// Ownership of a queued command: whichever thread moves it out of `pending` decides its fate.
enum : int { kPending, kTaken, kExpired };

struct Command {
    Op op{Op::home};
    std::size_t module{0};
    std::size_t port{0};
    PortValue value;
    bool enabled{false};
    std::uint64_t cycles{0};
    double rate_hz{0.0};
    ConfigNode params;
    std::atomic<int> state{kPending};
};

struct Reply {
    bool applied{true};  ///< False when the IO thread already answered with a timeout.
    bool ok{true};
    ErrorCode code{ErrorCode::internal};
    std::array<char, 256> message{};
    std::size_t module{0};
    Op op{Op::tune};    ///< values are module tunables (tune), solver options (tune_solver) or joint limits.
    ConfigNode values;  ///< Tunables after a successful tune; empty otherwise.
};

// IO-thread bookkeeping for one queued command.
struct Pending {
    ConfigNode id;
    std::uint64_t client{0};
    SteadyClock::time_point deadline;
    bool answered{false};
};

// Module outputs and world poses of its visuals, recomputed at the display configuration.
struct ModuleDisplay {
    bool valid{false};
    bool measured{false};  ///< At the measured state (fresh feedback), else at the command.
    std::vector<OutputValue> outputs;
    std::vector<Eigen::Isometry3d> poses;
    std::vector<char> visible;
};

struct ModuleFrame {
    ModuleDisplay display;
    bool enabled{false};
    bool running{false};
    int held_by{-1};
    std::uint64_t preempted{0};
    const char* home_state{nullptr};
    std::vector<PortValue> ports;
};

struct Snapshot {
    Eigen::VectorXd q;
    Eigen::VectorXd q_command;
    bool paused{false};
    bool fresh{true};
    double rate_hz{0.0};
    std::vector<ModuleFrame> modules;
    std::vector<TaskResidual> residuals;
    std::vector<StageReport> stages;
    std::string quality;
    EngineStats stats;
};

struct Client {
    int fd{-1};
    std::uint64_t id{0};
    std::string in;
    std::string out;
    bool dead{false};
    bool closing{false};  ///< Dead once `out` is flushed.
    bool authed{false};
    bool subscribed{true};
    SteadyClock::duration period{};
    SteadyClock::time_point next_state{};
    std::uint64_t last_seq{0};
    SteadyClock::time_point connected{};
};

// Compares in time independent of where the first mismatch is.
bool same_token(const std::string& expected, const std::string& given) noexcept {
    std::size_t diff = expected.size() ^ given.size();
    for (std::size_t i = 0; i < expected.size(); ++i) {
        const unsigned char g = i < given.size() ? static_cast<unsigned char>(given[i]) : 0;
        diff |= static_cast<std::size_t>(static_cast<unsigned char>(expected[i]) ^ g);
    }
    return diff == 0;
}

ConfigNode numbers(const Eigen::VectorXd& v) {
    ConfigNode list = ConfigNode::list();
    for (Eigen::Index i = 0; i < v.size(); ++i) list.append(v[i]);
    return list;
}

// Engine statistics in "state.stats", declared once in "model.stats".
struct StatInfo {
    const char* name;
    const char* kind;  // "counter" or "duration"
    double (*read)(const EngineStats&);
};

constexpr StatInfo kStats[] = {
    {"cycles", "counter", [](const EngineStats& s) { return static_cast<double>(s.cycles); }},
    {"overruns", "counter", [](const EngineStats& s) { return static_cast<double>(s.overruns); }},
    {"repeated_reads", "counter", [](const EngineStats& s) { return static_cast<double>(s.repeated_reads); }},
    {"guard_rejects", "counter", [](const EngineStats& s) { return static_cast<double>(s.guard_rejects); }},
    {"guard_clamps", "counter", [](const EngineStats& s) { return static_cast<double>(s.guard_clamps); }},
    {"held_cycles", "counter", [](const EngineStats& s) { return static_cast<double>(s.held_cycles); }},
    {"stale_cycles", "counter", [](const EngineStats& s) { return static_cast<double>(s.stale_cycles); }},
    {"resyncs", "counter", [](const EngineStats& s) { return static_cast<double>(s.resyncs); }},
    {"failed_cycles", "counter", [](const EngineStats& s) { return static_cast<double>(s.failed_cycles); }},
    {"last_cycle", "duration", [](const EngineStats& s) { return s.last_cycle_s; }},
    {"cycle_p50", "duration", [](const EngineStats& s) { return s.cycle_hist.quantile(0.5); }},
    {"cycle_p99", "duration", [](const EngineStats& s) { return s.cycle_hist.quantile(0.99); }},
    {"cycle_p999", "duration", [](const EngineStats& s) { return s.cycle_hist.quantile(0.999); }},
    {"max_cycle", "duration", [](const EngineStats& s) { return s.max_cycle_s; }},
    {"max_lateness", "duration", [](const EngineStats& s) { return s.max_lateness_s; }},
};

const char* shape_name(ShapeType type) noexcept {
    switch (type) {
        case ShapeType::sphere:
            return "sphere";
        case ShapeType::box:
            return "box";
        case ShapeType::cylinder:
            return "cylinder";
        case ShapeType::capsule:
            return "capsule";
        case ShapeType::convex:
            return "convex";
    }
    return "unknown";
}

ConfigNode pose_node(const Eigen::Isometry3d& pose) {
    const Eigen::Quaterniond r(pose.linear());
    const Eigen::Vector3d t = pose.translation();
    ConfigNode node = ConfigNode::map();
    node.set("translation", numbers(t));
    node.set("rotation", ConfigNode::list({r.w(), r.x(), r.y(), r.z()}));
    return node;
}

ConfigNode encode(const OutputInfo& info, const OutputValue& value) {
    switch (info.type) {
        case OutputType::number:
            return value.number;
        case OutputType::flag:
            return value.flag;
        case OutputType::text:
            return value.text;
        case OutputType::point:
            return numbers(value.points.col(0));
        case OutputType::segment:
        case OutputType::polygon: {
            ConfigNode list = ConfigNode::list();
            for (Eigen::Index i = 0; i < value.points.cols(); ++i) list.append(numbers(value.points.col(i)));
            return list;
        }
        case OutputType::visuals: {
            ConfigNode list = ConfigNode::list();
            for (const int v : value.visuals) list.append(static_cast<double>(v));
            return list;
        }
    }
    return {};
}

ConfigNode encode(const PortInfo& port, const PortValue& value) {
    switch (port.type) {
        case PortType::number:
            return value.number;
        case PortType::flag:
            return value.flag;
        case PortType::vector:
            return numbers(value.vector);
        case PortType::pose:
            return pose_node(value.pose);
    }
    return {};
}

bool read_numbers(const ConfigNode* node, Eigen::Index n, Eigen::VectorXd& out) {
    if (node == nullptr || node->kind() != ConfigNode::Kind::list ||
        node->size() != static_cast<std::size_t>(n))
        return false;
    out.resize(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        const ConfigNode& item = node->items()[static_cast<std::size_t>(i)];
        if (item.kind() != ConfigNode::Kind::number || !std::isfinite(item.as_number())) return false;
        out[i] = item.as_number();
    }
    return true;
}

Status decode(const PortInfo& port, const ConfigNode* node, PortValue& out) {
    std::string expected;
    switch (port.type) {
        case PortType::number:
            expected = "a finite number";
            break;
        case PortType::flag:
            expected = "a bool";
            break;
        case PortType::vector:
            expected = std::to_string(port.size()) + " finite numbers";
            break;
        case PortType::pose:
            expected = R"({"translation": [x, y, z], "rotation": [w, x, y, z]})";
            break;
    }
    const Error bad =
        make_error(ErrorCode::invalid_argument, "bad value for " + std::string(to_string(port.type)) +
                                                    " port '" + port.name + "', expected " + expected);
    if (node == nullptr) return bad;
    switch (port.type) {
        case PortType::number:
            if (node->kind() != ConfigNode::Kind::number || !std::isfinite(node->as_number())) return bad;
            out.number = node->as_number();
            return {};
        case PortType::flag:
            if (node->kind() != ConfigNode::Kind::boolean) return bad;
            out.flag = node->as_bool();
            return {};
        case PortType::vector:
            if (!read_numbers(node, port.size(), out.vector)) return bad;
            return {};
        case PortType::pose: {
            Eigen::VectorXd t, r;
            if (node->kind() != ConfigNode::Kind::map || node->size() != 2) return bad;
            if (!read_numbers(node->find("translation"), 3, t) || !read_numbers(node->find("rotation"), 4, r))
                return bad;
            Eigen::Quaterniond quat(r[0], r[1], r[2], r[3]);
            if (quat.norm() < 1e-9) return bad;
            quat.normalize();
            out.pose.setIdentity();
            out.pose.linear() = quat.toRotationMatrix();
            out.pose.translation() = t.head<3>();
            return {};
        }
    }
    return bad;
}

void frame(std::string& out, const std::string& payload) {
    const auto n = static_cast<std::uint32_t>(payload.size());
    const char header[4] = {static_cast<char>((n >> 24) & 0xff), static_cast<char>((n >> 16) & 0xff),
                            static_cast<char>((n >> 8) & 0xff), static_cast<char>(n & 0xff)};
    out.append(header, 4);
    out += payload;
}

Error sys_error(const std::string& what) {
    return make_error(ErrorCode::io_error, "debug server: " + what + ": " + std::strerror(errno));
}

}  // namespace

struct DebugServer::Impl {
    DebugServerOptions options;
    int listen_fd{-1};
    int wake[2]{-1, -1};
    std::uint16_t bound_port{0};
    std::thread io;
    std::atomic<bool> stopping{false};

    // Static description, fixed at start; sent as "model" once a client is authenticated.
    std::string model;
    std::optional<detail::ModelAssets> assets;
    ConfigNode capabilities;
    SteadyClock::duration publish_period{};
    std::vector<std::string> module_ids;
    // Solver task name -> (module index, local task name), for residual ownership.
    std::unordered_map<std::string, std::pair<std::size_t, std::string>> task_owners;
    std::vector<std::vector<PortInfo>> ports;
    std::vector<std::vector<OutputInfo>> outputs;
    // Per visual: frame id, or nullopt for world.
    std::vector<std::vector<std::optional<FrameId>>> visual_frames;
    std::vector<std::size_t> visual_counts;
    // Control thread: display workspace and the last recomputed displays, copied into every snapshot.
    std::optional<Workspace> display;
    std::vector<ModuleDisplay> displays;
    SteadyClock::time_point next_display{};
    // IO thread: last "params" message per module, replayed after "model" since the model's params are start values.
    std::vector<std::string> latest_params;
    std::string latest_solver_params;
    std::string latest_joint_limits;

    // Control -> IO: triple buffer; bit 2 of `shared` marks an unread publish.
    std::array<Snapshot, 3> snapshots;
    std::atomic<int> shared{0};
    int write_index{1};
    int read_index{2};

    Ring<Command, kRingSize> commands;
    Ring<Reply, kRingSize> replies;
    // Replies come back in command order; at most kRingSize are outstanding, so neither ring overflows.
    std::array<Pending, kRingSize> pending;
    std::size_t sent{0};
    std::size_t replied{0};

    std::vector<Client> clients;
    std::uint64_t next_id{1};
    std::uint64_t lease{0};  ///< Client id holding the control lease; 0 = none.
    std::uint64_t snapshot_seq{0};

    ~Impl() { close_all(); }

    void close_all() {
        for (Client& c : clients)
            if (c.fd >= 0) ::close(c.fd);
        clients.clear();
        for (int* fd : {&listen_fd, &wake[0], &wake[1]})
            if (*fd >= 0) {
                ::close(*fd);
                *fd = -1;
            }
    }

    void prepare_snapshots(const Engine& engine) {
        const auto& mods = engine.modules().modules();
        for (Snapshot& s : snapshots) {
            s.q.setZero(engine.modules().model()->nq());
            s.q_command.setZero(engine.modules().model()->nq());
            s.modules.resize(mods.size());
            for (std::size_t m = 0; m < mods.size(); ++m) {
                s.modules[m].ports.resize(ports[m].size());
                for (std::size_t p = 0; p < ports[m].size(); ++p)
                    if (ports[m][p].type == PortType::vector)
                        s.modules[m].ports[p].vector.setZero(ports[m][p].size());
            }
        }
        displays.resize(mods.size());
        for (std::size_t m = 0; m < mods.size(); ++m) {
            ModuleDisplay& d = displays[m];
            d.outputs.resize(outputs[m].size());
            for (std::size_t o = 0; o < outputs[m].size(); ++o) {
                const OutputInfo& info = outputs[m][o];
                OutputValue& v = d.outputs[o];
                if (info.type == OutputType::point) v.points.setZero(3, 1);
                if (info.type == OutputType::segment) v.points.setZero(3, 2);
                if (info.type == OutputType::polygon) v.points.setZero(3, info.size);
                if (info.type == OutputType::visuals) v.visuals.reserve(static_cast<std::size_t>(info.size));
                if (info.type == OutputType::text) v.text.reserve(64);
            }
            d.poses.assign(visual_counts[m], Eigen::Isometry3d::Identity());
            d.visible.assign(visual_counts[m], 1);
        }
        for (Snapshot& s : snapshots)
            for (std::size_t m = 0; m < mods.size(); ++m) {
                s.modules[m].display = displays[m];
                for (std::size_t o = 0; o < outputs[m].size(); ++o) {
                    s.modules[m].display.outputs[o].visuals.reserve(
                        static_cast<std::size_t>(outputs[m][o].size));
                    if (outputs[m][o].type == OutputType::text)
                        s.modules[m].display.outputs[o].text.reserve(64);
                }
            }
        display.emplace(engine.modules().model());
    }

    // Recomputes module displays at the measured state when fresh, else at the command (what clients draw).
    void update_displays(const Engine& engine) {
        const Eigen::VectorXd* measured = engine.measured();
        const bool ok = display->update(measured ? *measured : engine.command().q).ok();
        const auto& mods = engine.modules().modules();
        Eigen::Isometry3d placement;
        for (std::size_t m = 0; m < mods.size(); ++m) {
            ModuleDisplay& d = displays[m];
            if (outputs[m].empty() && visual_counts[m] == 0) continue;
            d.measured = measured != nullptr;
            d.valid = ok && mods[m]->update_outputs(*display).ok();
            for (std::size_t o = 0; d.valid && o < d.outputs.size(); ++o)
                d.valid = mods[m]->read_output(o, d.outputs[o]).ok();
            for (std::size_t v = 0; d.valid && v < d.poses.size(); ++v) {
                d.visible[v] = mods[m]->visual_placement(v, placement);
                if (!d.visible[v] || !visual_frames[m][v]) {
                    d.poses[v] = placement;
                    continue;
                }
                d.valid = display->frame_pose(*visual_frames[m][v], d.poses[v]).ok();
                d.poses[v] = d.poses[v] * placement;
            }
        }
    }

    void build_model(const Engine& engine) {
        const Model& robot = *engine.modules().model();
        ConfigNode doc = ConfigNode::map();
        doc.set("type", "model");
        doc.set("urdf", options.urdf.string());
        doc.set("assets", assets ? assets->describe() : ConfigNode{});
        doc.set("rate_hz", engine.options().rate_hz);
        doc.set("publish_hz", options.publish_hz);
        doc.set("nq", static_cast<double>(robot.nq()));
        doc.set("nv", static_cast<double>(robot.nv()));
        ConfigNode joints = ConfigNode::list();
        for (const JointInfo& j : robot.joints()) {
            if (j.type == JointType::universe) continue;
            ConfigNode node = ConfigNode::map();
            node.set("name", j.name);
            node.set("type", to_string(j.type));
            node.set("parent", robot.joints()[static_cast<std::size_t>(j.parent)].name);
            node.set("idx_q", static_cast<double>(j.idx_q));
            node.set("nq", static_cast<double>(j.nq));
            node.set("idx_v", static_cast<double>(j.idx_v));
            node.set("nv", static_cast<double>(j.nv));
            const auto segment = [](const Eigen::VectorXd& v, Index start, Index n) {
                ConfigNode list = ConfigNode::list();
                for (Index i = 0; i < n; ++i) list.append(ConfigNode(v[start + i]));
                return list;
            };
            ConfigNode limits = ConfigNode::map();
            limits.set("position", robot.has_position_limit(j.id));
            limits.set("lower", segment(robot.lower_position_limit(), j.idx_q, j.nq));
            limits.set("upper", segment(robot.upper_position_limit(), j.idx_q, j.nq));
            limits.set("velocity", segment(robot.velocity_limit(), j.idx_v, j.nv));
            limits.set("effort", segment(robot.effort_limit(), j.idx_v, j.nv));
            node.set("limits", std::move(limits));
            joints.append(std::move(node));
        }
        doc.set("joints", std::move(joints));
        ConfigNode mimics = ConfigNode::list();
        for (const MimicRelation& r : robot.mimics()) {
            ConfigNode node = ConfigNode::map();
            node.set("follower", r.follower);
            node.set("leader", r.leader);
            node.set("multiplier", r.multiplier);
            node.set("offset", r.offset);
            mimics.append(std::move(node));
        }
        doc.set("mimics", std::move(mimics));
        ConfigNode modules = ConfigNode::list();
        for (const auto& m : engine.modules().modules()) {
            ConfigNode node = ConfigNode::map();
            node.set("id", m->id());
            node.set("type", m->type());
            node.set("homeable", m->homeable());
            ConfigNode deps = ConfigNode::list();
            for (const std::string& d : m->dependencies()) deps.append(d);
            node.set("dependencies", std::move(deps));
            const Mask& driven = engine.modules().driven(m->id());
            ConfigNode driven_joints = ConfigNode::list();
            for (const JointInfo& j : robot.joints())
                if (j.nv > 0 && driven.test(static_cast<std::size_t>(j.idx_v))) driven_joints.append(j.name);
            node.set("driven", std::move(driven_joints));
            ConfigNode levels = ConfigNode::list();
            for (int level : engine.modules().task_levels(m->id())) levels.append(level);
            node.set("levels", std::move(levels));
            ConfigNode schema = m->tune_schema();
            node.set("tunable", !schema.is_null());
            if (!schema.is_null()) {
                node.set("params", m->tunables());
                node.set("tune_schema", std::move(schema));
            }
            ConfigNode list = ConfigNode::list();
            for (const PortInfo& p : m->ports()) {
                ConfigNode port = ConfigNode::map();
                port.set("name", p.name);
                port.set("type", to_string(p.type));
                if (p.type == PortType::vector) {
                    ConfigNode labels = ConfigNode::list();
                    for (const std::string& l : p.labels) labels.append(l);
                    port.set("labels", std::move(labels));
                }
                if (p.lower.size() != 0) port.set("lower", numbers(p.lower));
                if (p.upper.size() != 0) port.set("upper", numbers(p.upper));
                if (p.type == PortType::pose) {
                    port.set("frame", p.frame);
                    port.set("reference", p.reference);
                }
                if (p.timeout > 0.0) port.set("timeout_s", p.timeout);
                list.append(std::move(port));
            }
            node.set("ports", std::move(list));
            ConfigNode visuals = ConfigNode::list();
            for (const VisualInfo& v : m->visuals()) {
                ConfigNode visual = ConfigNode::map();
                visual.set("name", v.name);
                visual.set("frame", v.frame);
                ConfigNode shape = ConfigNode::map();
                shape.set("type", shape_name(v.shape.type));
                shape.set("size", numbers(v.shape.size));
                if (v.shape.type == ShapeType::convex) {
                    ConfigNode vertices = ConfigNode::list();
                    for (Eigen::Index i = 0; i < v.shape.vertices.cols(); ++i)
                        vertices.append(numbers(v.shape.vertices.col(i)));
                    shape.set("vertices", std::move(vertices));
                }
                visual.set("shape", std::move(shape));
                visuals.append(std::move(visual));
            }
            node.set("visuals", std::move(visuals));
            ConfigNode outs = ConfigNode::list();
            for (const OutputInfo& o : m->outputs()) {
                ConfigNode out = ConfigNode::map();
                out.set("name", o.name);
                out.set("type", to_string(o.type));
                if (o.type == OutputType::polygon || o.type == OutputType::visuals)
                    out.set("size", static_cast<double>(o.size));
                out.set("role", to_string(o.role));
                if (!o.unit.empty()) out.set("unit", o.unit);
                outs.append(std::move(out));
            }
            node.set("outputs", std::move(outs));
            modules.append(std::move(node));
        }
        doc.set("modules", std::move(modules));
        ConfigNode solver = ConfigNode::map();
        solver.set("params", engine.modules().solver_params());
        solver.set("tune_schema", engine.modules().solver_tune_schema());
        doc.set("solver", std::move(solver));
        doc.set("joint_limits", engine.modules().joint_limits());
        ConfigNode stats = ConfigNode::list();
        for (const StatInfo& info : kStats) {
            ConfigNode stat = ConfigNode::map();
            stat.set("name", info.name);
            stat.set("kind", info.kind);
            if (std::string_view(info.kind) == "duration") stat.set("unit", "s");
            stats.append(std::move(stat));
        }
        doc.set("stats", std::move(stats));
        model = to_json(doc, 0);
    }

    // ---- control thread ----

    void apply(Engine& engine) {
        if (stopping.load(std::memory_order_relaxed)) return;
        const auto& mods = engine.modules().modules();
        while (Command* c = commands.front()) {
            int expected = kPending;
            const bool take = c->state.compare_exchange_strong(expected, kTaken, std::memory_order_acq_rel);
            Status s;
            if (take) {
                switch (c->op) {
                    case Op::set_port:
                        s = mods[c->module]->write_port(c->port, c->value);
                        break;
                    case Op::set_enabled:
                        mods[c->module]->set_enabled(c->enabled);
                        break;
                    case Op::home:
                        s = mods[c->module]->request_home();
                        break;
                    case Op::tune:
                        s = mods[c->module]->tune(c->params);
                        break;
                    case Op::tune_solver:
                        s = engine.modules().tune_solver(c->params);
                        break;
                    case Op::pause:
                        engine.set_paused(c->enabled);
                        break;
                    case Op::step:
                        if (engine.paused())
                            engine.step_paused(c->cycles);
                        else
                            s = make_error(ErrorCode::invalid_argument, "step needs a paused engine");
                        break;
                    case Op::set_rate:
                        s = engine.set_rate_hz(c->rate_hz);
                        break;
                    case Op::tune_limits:
                        s = engine.modules().tune_joint_limits(c->params);
                        break;
                }
            }
            if (Reply* r = replies.claim()) {
                r->module = c->module;
                r->op = c->op;
                r->values = !(take && s)               ? ConfigNode{}
                            : c->op == Op::tune        ? mods[c->module]->tunables()
                            : c->op == Op::tune_solver ? engine.modules().solver_params()
                            : c->op == Op::tune_limits ? engine.modules().joint_limits()
                                                       : ConfigNode{};
                r->applied = take;
                r->ok = s.ok();
                r->code = s ? ErrorCode::internal : s.error().code;
                r->message[0] = '\0';
                if (!s) std::snprintf(r->message.data(), r->message.size(), "%s", s.error().message.c_str());
                replies.push();
            }
            commands.pop();
        }
    }

    void publish(const Engine& engine) {
        Snapshot& s = snapshots[static_cast<std::size_t>(write_index)];
        s.q = engine.state().q;
        s.q_command = engine.command().q;
        s.paused = engine.paused();
        s.rate_hz = engine.options().rate_hz;
        s.fresh = engine.fresh();
        s.stats = engine.stats();
        if (const auto now = SteadyClock::now(); now >= next_display) {
            next_display = now + publish_period;
            update_displays(engine);
        }
        const auto& mods = engine.modules().modules();
        for (std::size_t i = 0; i < mods.size(); ++i) {
            const Module& m = *mods[i];
            ModuleFrame& ms = s.modules[i];
            ms.display = displays[i];
            ms.enabled = m.enabled();
            ms.running = m.running();
            ms.preempted = m.preempted();
            ms.home_state = m.home_state();
            ms.held_by = -1;
            if (const std::string* h = m.held_by())
                for (std::size_t k = 0; k < mods.size(); ++k)
                    if (&mods[k]->id() == h || mods[k]->id() == *h) ms.held_by = static_cast<int>(k);
            for (std::size_t p = 0; p < ms.ports.size(); ++p) (void)m.read_port(p, ms.ports[p]);
        }
        const auto& res = engine.result().residuals;
        if (s.residuals.size() != res.size())
            s.residuals.resize(res.size());  // Settles after the first cycle.
        for (std::size_t i = 0; i < res.size(); ++i) {
            s.residuals[i].name = res[i].name;
            s.residuals[i].level = res[i].level;
            s.residuals[i].residual = res[i].residual;
        }
        s.stages = engine.result().stages;  // Capacity settles after the first cycles.
        s.quality = to_string(engine.result().quality);
        write_index = shared.exchange(write_index | 4, std::memory_order_acq_rel) & 3;
    }

    // ---- IO thread ----

    Client* find_client(std::uint64_t id) {
        for (Client& c : clients)
            if (c.id == id && !c.dead) return &c;
        return nullptr;
    }

    /// `code` is empty on success, otherwise an ErrorCode name or a protocol code
    /// (unauthorized, lease_held, busy, timeout).
    void reply(Client& c, const ConfigNode& id, const char* code, const std::string& message) {
        ConfigNode doc = ConfigNode::map();
        doc.set("type", "reply");
        doc.set("id", id);
        doc.set("ok", code == nullptr);
        if (code != nullptr) {
            doc.set("code", code);
            doc.set("error", message);
        }
        frame(c.out, to_json(doc, 0));
    }

    // Every client learns the new values, including the one that tuned.
    void send_params(std::size_t module, const ConfigNode& values) {
        ConfigNode doc = ConfigNode::map();
        doc.set("type", "params");
        doc.set("module", module_ids[module]);
        doc.set("values", values);
        std::string text = to_json(doc, 0);
        for (Client& c : clients)
            if (!c.dead && c.authed) frame(c.out, text);
        latest_params[module] = std::move(text);
    }

    void broadcast(const char* type, const ConfigNode& values, std::string& latest) {
        ConfigNode doc = ConfigNode::map();
        doc.set("type", type);
        doc.set("values", values);
        std::string text = to_json(doc, 0);
        for (Client& c : clients)
            if (!c.dead && c.authed) frame(c.out, text);
        latest = std::move(text);
    }

    void send_lease() {
        for (Client& c : clients) {
            if (c.dead || !c.authed) continue;
            ConfigNode doc = ConfigNode::map();
            doc.set("type", "lease");
            doc.set("holder", lease != 0 ? ConfigNode(static_cast<double>(lease)) : ConfigNode{});
            doc.set("mine", lease != 0 && lease == c.id);
            frame(c.out, to_json(doc, 0));
        }
    }

    void on_authed(Client& c) {
        c.authed = true;
        frame(c.out, model);
        for (const std::string& text : latest_params)
            if (!text.empty()) frame(c.out, text);
        if (!latest_solver_params.empty()) frame(c.out, latest_solver_params);
        if (!latest_joint_limits.empty()) frame(c.out, latest_joint_limits);
        ConfigNode doc = ConfigNode::map();
        doc.set("type", "lease");
        doc.set("holder", lease != 0 ? ConfigNode(static_cast<double>(lease)) : ConfigNode{});
        doc.set("mine", false);
        frame(c.out, to_json(doc, 0));
    }

    Status parse_command(const ConfigNode& doc, const std::string& name, Command& cmd) {
        cmd.params = ConfigNode{};
        cmd.module = 0;
        if (name == "pause") {
            const ConfigNode* paused = doc.find("paused");
            if (paused == nullptr || paused->kind() != ConfigNode::Kind::boolean)
                return make_error(ErrorCode::invalid_argument, "pause needs a boolean 'paused'");
            cmd.op = Op::pause;
            cmd.enabled = paused->as_bool();
            return {};
        }
        if (name == "step") {
            const ConfigNode* cycles = doc.find("cycles");
            double n = 1.0;
            if (cycles != nullptr) {
                if (cycles->kind() != ConfigNode::Kind::number)
                    n = 0.0;
                else
                    n = cycles->as_number();
            }
            if (!(n >= 1.0 && n <= kMaxStepCycles) || n != std::floor(n))
                return make_error(ErrorCode::invalid_argument,
                                  "step 'cycles' must be an integer in [1, 1000000]");
            cmd.op = Op::step;
            cmd.cycles = static_cast<std::uint64_t>(n);
            return {};
        }
        if (name == "set_rate") {
            const ConfigNode* rate = doc.find("rate_hz");
            if (rate == nullptr || rate->kind() != ConfigNode::Kind::number || !(rate->as_number() >= 1.0) ||
                !(rate->as_number() <= kMaxRateHz))
                return make_error(ErrorCode::invalid_argument, "set_rate needs 'rate_hz' in [1, 10000]");
            cmd.op = Op::set_rate;
            cmd.rate_hz = rate->as_number();
            return {};
        }
        if (name == "tune_solver" || name == "tune_limits") {
            const ConfigNode* params = doc.find("params");
            if (params == nullptr || params->kind() != ConfigNode::Kind::map)
                return make_error(ErrorCode::invalid_argument, name + " needs a map 'params'");
            cmd.op = name == "tune_solver" ? Op::tune_solver : Op::tune_limits;
            cmd.params = *params;
            return {};
        }
        if (name != "home" && name != "set_enabled" && name != "set_port" && name != "tune")
            return make_error(ErrorCode::unsupported, "unknown op '" + name + "'");
        const ConfigNode* module = doc.find("module");
        if (module == nullptr || module->kind() != ConfigNode::Kind::string)
            return make_error(ErrorCode::invalid_argument, "missing module");
        std::size_t m = 0;
        while (m < module_ids.size() && module_ids[m] != module->as_string()) ++m;
        if (m == module_ids.size())
            return make_error(ErrorCode::not_found, "unknown module '" + module->as_string() + "'");
        cmd.module = m;
        if (name == "home") {
            cmd.op = Op::home;
            return {};
        }
        if (name == "tune") {
            const ConfigNode* params = doc.find("params");
            if (params == nullptr || params->kind() != ConfigNode::Kind::map)
                return make_error(ErrorCode::invalid_argument, "tune needs a map 'params'");
            cmd.op = Op::tune;
            cmd.params = *params;
            return {};
        }
        if (name == "set_enabled") {
            const ConfigNode* enabled = doc.find("enabled");
            if (enabled == nullptr || enabled->kind() != ConfigNode::Kind::boolean)
                return make_error(ErrorCode::invalid_argument, "set_enabled needs a boolean 'enabled'");
            cmd.op = Op::set_enabled;
            cmd.enabled = enabled->as_bool();
            return {};
        }
        if (name == "set_port") {
            const ConfigNode* port = doc.find("port");
            if (port == nullptr || port->kind() != ConfigNode::Kind::string)
                return make_error(ErrorCode::invalid_argument, "set_port needs 'port'");
            const auto& list = ports[m];
            std::size_t p = 0;
            while (p < list.size() && list[p].name != port->as_string()) ++p;
            if (p == list.size())
                return make_error(ErrorCode::not_found,
                                  "module '" + module_ids[m] + "' has no port '" + port->as_string() + "'");
            cmd.op = Op::set_port;
            cmd.port = p;
            return decode(list[p], doc.find("value"), cmd.value);
        }
        return make_error(ErrorCode::unsupported, "unknown op '" + name + "'");
    }

    void handle_auth(Client& c, const ConfigNode& doc, const ConfigNode& id) {
        const ConfigNode* token = doc.find("token");
        const bool good =
            options.token.empty() || (token != nullptr && token->kind() == ConfigNode::Kind::string &&
                                      same_token(options.token, token->as_string()));
        if (c.authed) {  // Already in: answer truthfully but keep the session.
            reply(c, id, good ? nullptr : "unauthorized", good ? std::string() : "bad token");
            return;
        }
        if (!good) {
            reply(c, id, "unauthorized", "bad token");
            c.closing = true;
            return;
        }
        reply(c, id, nullptr, {});
        on_authed(c);
    }

    void handle_subscribe(Client& c, const ConfigNode& doc, const ConfigNode& id) {
        const ConfigNode* rate = doc.find("rate_hz");
        if (rate == nullptr || rate->kind() != ConfigNode::Kind::number ||
            !std::isfinite(rate->as_number()) || rate->as_number() < 0.0 ||
            rate->as_number() > options.publish_hz) {
            char msg[96];
            std::snprintf(msg, sizeof msg, "subscribe needs rate_hz in [0, %g] (publish_hz)",
                          options.publish_hz);
            reply(c, id, to_string(ErrorCode::invalid_argument), msg);
            return;
        }
        const double hz = rate->as_number();
        c.subscribed = hz > 0.0;
        if (c.subscribed)
            c.period =
                std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(1.0 / hz));
        c.next_state = SteadyClock::now();
        reply(c, id, nullptr, {});
    }

    void handle_acquire(Client& c, const ConfigNode& doc, const ConfigNode& id) {
        const ConfigNode* force = doc.find("force");
        const bool forced =
            force != nullptr && force->kind() == ConfigNode::Kind::boolean && force->as_bool();
        if (lease != 0 && lease != c.id && !forced) {
            reply(c, id, "lease_held", "control lease held by client " + std::to_string(lease));
            return;
        }
        const bool changed = lease != c.id;
        lease = c.id;
        reply(c, id, nullptr, {});
        if (changed) send_lease();
    }

    void handle_asset(Client& c, const ConfigNode& doc, const ConfigNode& id) {
        const ConfigNode* name = doc.find("name");
        const ConfigNode* off = doc.find("offset");
        if (name == nullptr || name->kind() != ConfigNode::Kind::string)
            return reply(c, id, to_string(ErrorCode::invalid_argument), "missing name");
        double offset = 0.0;
        if (off != nullptr) {
            if (off->kind() != ConfigNode::Kind::number || !(off->as_number() >= 0.0) ||
                off->as_number() != std::floor(off->as_number()) || off->as_number() > 9.0e15)
                return reply(c, id, to_string(ErrorCode::invalid_argument),
                             "offset must be a non-negative integer");
            offset = off->as_number();
        }
        if (!assets)
            return reply(c, id, to_string(ErrorCode::not_found), "this server does not serve model assets");
        std::string data;
        std::uint64_t total = 0;
        const auto at = static_cast<std::uint64_t>(offset);
        if (Status s = assets->read(name->as_string(), at, kAssetChunk, data, total); !s)
            return reply(c, id, to_string(s.error().code), s.error().message);
        ConfigNode r = ConfigNode::map();
        r.set("type", "reply");
        r.set("id", id);
        r.set("ok", true);
        r.set("offset", offset);
        r.set("total", static_cast<double>(total));
        r.set("data", detail::base64_encode(data));
        frame(c.out, to_json(r, 0));
    }

    static bool is_command(const std::string& name) {
        for (const char* op : kCommandOps)
            if (name == op) return true;
        return false;
    }

    void handle(Client& c, const std::string& text) {
        ConfigNode id;
        Result<ConfigNode> doc = parse_yaml(text, "command");
        if (!doc) {
            reply(c, id, to_string(doc.error().code), doc.error().message);
            return;
        }
        if (doc->kind() != ConfigNode::Kind::map) {
            reply(c, id, to_string(ErrorCode::invalid_argument), "command must be an object");
            return;
        }
        if (const ConfigNode* i = doc->find("id")) id = *i;
        const ConfigNode* op = doc->find("op");
        if (op == nullptr || op->kind() != ConfigNode::Kind::string) {
            reply(c, id, to_string(ErrorCode::invalid_argument), "missing op");
            return;
        }
        const std::string& name = op->as_string();
        if (name == "auth") return handle_auth(c, *doc, id);
        if (!c.authed) {
            reply(c, id, "unauthorized", "authenticate first");
            c.closing = true;
            return;
        }
        if (name == "ping") return reply(c, id, nullptr, {});
        if (name == "subscribe") return handle_subscribe(c, *doc, id);
        if (name == "acquire") return handle_acquire(c, *doc, id);
        if (name == "release") {
            if (lease == c.id) {
                lease = 0;
                send_lease();
            }
            return reply(c, id, nullptr, {});
        }
        if (name == "asset") return handle_asset(c, *doc, id);
        if (!is_command(name)) {
            reply(c, id, to_string(ErrorCode::unsupported), "unknown op '" + name + "'");
            return;
        }
        if (lease != 0 && lease != c.id) {
            reply(c, id, "lease_held", "control lease held by client " + std::to_string(lease));
            return;
        }
        double timeout = options.command_timeout_s;
        if (const ConfigNode* t = doc->find("timeout_s")) {
            if (t->kind() != ConfigNode::Kind::number || !std::isfinite(t->as_number()) ||
                t->as_number() <= 0.0 || t->as_number() > kMaxTimeoutS) {
                reply(c, id, to_string(ErrorCode::invalid_argument), "timeout_s must be in (0, 60]");
                return;
            }
            timeout = t->as_number();
        }
        Command* slot = sent - replied < kRingSize ? commands.claim() : nullptr;
        if (slot == nullptr) {
            reply(c, id, "busy", "command queue full");
            return;
        }
        if (Status s = parse_command(*doc, name, *slot); !s) {
            reply(c, id, to_string(s.error().code), s.error().message);
            return;
        }
        slot->state.store(kPending, std::memory_order_relaxed);
        Pending& p = pending[sent % kRingSize];
        p.id = std::move(id);
        p.client = c.id;
        p.deadline = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
                                              std::chrono::duration<double>(timeout));
        p.answered = false;
        ++sent;
        commands.push();
    }

    // Answers queued commands the control thread has not picked up in time (e.g. the engine is not stepping).
    void expire_commands() {
        const auto now = SteadyClock::now();
        for (std::size_t i = replied; i != sent; ++i) {
            Pending& p = pending[i % kRingSize];
            if (p.answered || now < p.deadline) continue;
            int expected = kPending;
            if (!commands.at(i).state.compare_exchange_strong(expected, kExpired, std::memory_order_acq_rel))
                continue;
            p.answered = true;
            if (Client* c = find_client(p.client)) reply(*c, p.id, "timeout", "command not applied in time");
        }
    }

    void drain_replies() {
        while (Reply* r = replies.front()) {
            Pending& p = pending[replied % kRingSize];
            if (!p.answered && r->applied)
                if (Client* c = find_client(p.client))
                    reply(*c, p.id, r->ok ? nullptr : to_string(r->code), r->message.data());
            if (r->applied && r->values.kind() == ConfigNode::Kind::map) {
                if (r->op == Op::tune_solver)
                    broadcast("solver_params", r->values, latest_solver_params);
                else if (r->op == Op::tune_limits)
                    broadcast("joint_limits", r->values, latest_joint_limits);
                else
                    send_params(r->module, r->values);
            }
            r->values = ConfigNode{};
            p.id = ConfigNode{};
            ++replied;
            replies.pop();
        }
    }

    void read_client(Client& c) {
        char buf[16384];
        for (;;) {
            const ssize_t n = ::recv(c.fd, buf, sizeof buf, 0);
            if (n > 0) {
                c.in.append(buf, static_cast<std::size_t>(n));
                continue;
            }
            if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) c.dead = true;
            if (n < 0 && errno == EINTR) continue;
            break;
        }
        std::size_t at = 0;
        while (!c.dead && !c.closing && c.in.size() - at >= 4) {
            const auto* h = reinterpret_cast<const unsigned char*>(c.in.data() + at);
            const std::uint32_t len = (std::uint32_t{h[0]} << 24) | (std::uint32_t{h[1]} << 16) |
                                      (std::uint32_t{h[2]} << 8) | std::uint32_t{h[3]};
            if (len > kMaxFrame) {
                c.dead = true;
                break;
            }
            if (c.in.size() - at - 4 < len) break;
            handle(c, c.in.substr(at + 4, len));
            at += 4 + len;
        }
        c.in.erase(0, at);
    }

    void flush(Client& c) {
        while (!c.dead && !c.out.empty()) {
            const ssize_t n = ::send(c.fd, c.out.data(), c.out.size(), MSG_NOSIGNAL);
            if (n > 0) {
                c.out.erase(0, static_cast<std::size_t>(n));
            } else if (n < 0 && errno == EINTR) {
                continue;
            } else {
                if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) c.dead = true;
                break;
            }
        }
        if (c.out.size() > kMaxPending) c.dead = true;
    }

    void accept_clients() {
        for (;;) {
            const int fd = ::accept4(listen_fd, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (fd < 0) return;
            if (clients.size() >= kMaxClients) {
                std::string out;
                frame(out, R"({"type":"reply","id":null,"ok":false,"code":"busy","error":"server full ()" +
                               std::to_string(kMaxClients) + " clients)\"}");
                (void)::send(fd, out.data(), out.size(), MSG_NOSIGNAL);
                ::close(fd);
                continue;
            }
            const int one = 1;
            (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
            Client c;
            c.fd = fd;
            c.id = next_id++;
            c.connected = SteadyClock::now();
            c.period = publish_period;
            c.next_state = c.connected;
            ConfigNode hello = ConfigNode::map();
            hello.set("type", "hello");
            hello.set("version", kProtocolVersion);
            hello.set("client", static_cast<double>(c.id));
            hello.set("auth_required", !options.token.empty());
            hello.set("capabilities", capabilities);
            frame(c.out, to_json(hello, 0));
            if (options.token.empty()) on_authed(c);
            clients.push_back(std::move(c));
        }
    }

    std::string state_json(const Snapshot& s) const {
        ConfigNode doc = ConfigNode::map();
        doc.set("type", "state");
        doc.set("time", s.stats.time);
        doc.set("paused", s.paused);
        doc.set("rate_hz", s.rate_hz);
        doc.set("fresh", s.fresh);
        ConfigNode stats = ConfigNode::map();
        for (const StatInfo& info : kStats) stats.set(info.name, info.read(s.stats));
        doc.set("stats", std::move(stats));
        doc.set("q", numbers(s.q));
        doc.set("q_command", numbers(s.q_command));
        ConfigNode modules = ConfigNode::list();
        for (std::size_t m = 0; m < s.modules.size(); ++m) {
            const ModuleFrame& ms = s.modules[m];
            ConfigNode node = ConfigNode::map();
            node.set("id", module_ids[m]);
            node.set("enabled", ms.enabled);
            node.set("running", ms.running);
            node.set("held_by", ms.held_by >= 0 ? ConfigNode(module_ids[static_cast<std::size_t>(ms.held_by)])
                                                : ConfigNode{});
            node.set("preempted", static_cast<double>(ms.preempted));
            if (ms.home_state) node.set("home", std::string(ms.home_state));
            ConfigNode values = ConfigNode::map();
            for (std::size_t p = 0; p < ms.ports.size(); ++p)
                values.set(ports[m][p].name, encode(ports[m][p], ms.ports[p]));
            node.set("ports", std::move(values));
            const ModuleDisplay& d = ms.display;
            if (d.valid) {
                ConfigNode display_node = ConfigNode::map();
                display_node.set("at", d.measured ? "measured" : "command");
                ConfigNode outs = ConfigNode::map();
                for (std::size_t o = 0; o < d.outputs.size(); ++o)
                    outs.set(outputs[m][o].name, encode(outputs[m][o], d.outputs[o]));
                display_node.set("outputs", std::move(outs));
                ConfigNode poses = ConfigNode::list();
                for (std::size_t v = 0; v < d.poses.size(); ++v)
                    poses.append(d.visible[v] ? pose_node(d.poses[v]) : ConfigNode{});
                display_node.set("visual_poses", std::move(poses));
                node.set("display", std::move(display_node));
            } else {
                node.set("display", ConfigNode{});
            }
            modules.append(std::move(node));
        }
        doc.set("modules", std::move(modules));
        ConfigNode residuals = ConfigNode::list();
        for (const TaskResidual& r : s.residuals) {
            ConfigNode node = ConfigNode::map();
            node.set("name", r.name);
            const auto owner = task_owners.find(r.name);
            if (owner != task_owners.end()) {
                node.set("module", module_ids[owner->second.first]);
                node.set("task", owner->second.second);
            } else {
                node.set("module", ConfigNode{});
                node.set("task", r.name);
            }
            node.set("level", r.level);
            node.set("residual", r.residual);
            residuals.append(std::move(node));
        }
        doc.set("residuals", std::move(residuals));
        ConfigNode stages = ConfigNode::list();
        for (const StageReport& r : s.stages) {
            ConfigNode node = ConfigNode::map();
            node.set("level", r.level);
            node.set("status", qp::to_string(r.status));
            node.set("iterations", r.iterations);
            node.set("held", r.held);
            node.set("solve_s", r.solve_s);
            stages.append(std::move(node));
        }
        doc.set("stages", std::move(stages));
        doc.set("quality", s.quality);
        return to_json(doc, 0);
    }

    void broadcast_state() {
        if ((shared.load(std::memory_order_acquire) & 4) != 0) {
            read_index = shared.exchange(read_index, std::memory_order_acq_rel) & 3;
            ++snapshot_seq;
        }
        const auto now = SteadyClock::now();
        // Half a publish period of slack so jitter does not make a client skip a tick.
        const auto due = now + publish_period / 2;
        std::string text;
        for (Client& c : clients) {
            if (c.dead || c.closing || !c.authed || !c.subscribed || c.last_seq == snapshot_seq) continue;
            if (due < c.next_state || c.out.size() > kStateBacklog) continue;
            if (text.empty()) text = state_json(snapshots[static_cast<std::size_t>(read_index)]);
            frame(c.out, text);
            c.last_seq = snapshot_seq;
            c.next_state += c.period;
            if (c.next_state < now) c.next_state = now + c.period;
        }
    }

    void run() {
        using clock = std::chrono::steady_clock;
        const auto period = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(1.0 / options.publish_hz));
        auto next = clock::now() + period;
        std::vector<pollfd> fds;
        while (!stopping.load(std::memory_order_acquire)) {
            fds.clear();
            fds.push_back({wake[0], POLLIN, 0});
            fds.push_back({listen_fd, POLLIN, 0});
            for (const Client& c : clients)
                fds.push_back({c.fd, static_cast<short>(POLLIN | (c.out.empty() ? 0 : POLLOUT)), 0});
            const auto wait =
                std::chrono::duration_cast<std::chrono::milliseconds>(next - clock::now()).count();
            if (::poll(fds.data(), fds.size(), wait > 0 ? static_cast<int>(wait) : 0) < 0 && errno != EINTR)
                break;
            if ((fds[0].revents & POLLIN) != 0) {
                char buf[64];
                while (::read(wake[0], buf, sizeof buf) > 0) {
                }
            }
            for (std::size_t i = 0; i < clients.size(); ++i) {
                const short ev = fds[i + 2].revents;
                if ((ev & (POLLIN | POLLHUP | POLLERR)) != 0) read_client(clients[i]);
            }
            if ((fds[1].revents & POLLIN) != 0) accept_clients();
            drain_replies();
            expire_commands();
            if (clock::now() >= next) {
                next += period;
                if (next < clock::now()) next = clock::now() + period;
                broadcast_state();
            }
            const auto now = clock::now();
            for (Client& c : clients) {
                if (!c.authed && now - c.connected > kAuthGrace) c.dead = true;
                flush(c);
                if (c.closing && c.out.empty()) c.dead = true;
            }
            bool lease_lost = false;
            for (auto it = clients.begin(); it != clients.end();) {
                if (it->dead) {
                    if (it->id == lease) {
                        lease = 0;
                        lease_lost = true;
                    }
                    ::close(it->fd);
                    it = clients.erase(it);
                } else {
                    ++it;
                }
            }
            if (lease_lost) send_lease();
        }
    }
};

DebugServer::DebugServer(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}

DebugServer::~DebugServer() { stop(); }

std::uint16_t DebugServer::port() const noexcept { return impl_->bound_port; }

void DebugServer::stop() {
    Impl& d = *impl_;
    if (d.stopping.exchange(true)) return;
    if (d.wake[1] >= 0) {
        const char c = 0;
        (void)!::write(d.wake[1], &c, 1);
    }
    if (d.io.joinable()) d.io.join();
    d.close_all();
}

void DebugServer::before_step(Engine& engine) { impl_->apply(engine); }

void DebugServer::after_step(const Engine& engine) { impl_->publish(engine); }

Result<std::shared_ptr<DebugServer>> DebugServer::start(Engine& engine, const DebugServerOptions& options) {
    if (!std::isfinite(options.publish_hz) || options.publish_hz <= 0.0 || options.publish_hz > 1000.0)
        return make_error(ErrorCode::invalid_argument, "debug server: publish_hz must be in (0, 1000]");
    if (!std::isfinite(options.command_timeout_s) || options.command_timeout_s <= 0.0 ||
        options.command_timeout_s > kMaxTimeoutS)
        return make_error(ErrorCode::invalid_argument, "debug server: command_timeout_s must be in (0, 60]");
    if (!options.token.empty() && (options.token.size() < kMinToken || options.token.size() > kMaxToken))
        return make_error(ErrorCode::invalid_argument, "debug server: token must be 16..1024 bytes");
    in_addr addr{};
    const std::string host = options.host == "localhost" ? "127.0.0.1" : options.host;
    if (::inet_pton(AF_INET, host.c_str(), &addr) != 1)
        return make_error(ErrorCode::invalid_argument,
                          "debug server: host '" + options.host + "' is not an IPv4 address");
    if ((ntohl(addr.s_addr) >> 24) != 127 && options.token.empty())
        return make_error(
            ErrorCode::invalid_argument,
            "debug server: host '" + options.host + "' is not a loopback address; a token is required");

    auto d = std::make_unique<Impl>();
    d->options = options;
    d->publish_period = std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(1.0 / options.publish_hz));
    for (const auto& m : engine.modules().modules()) {
        d->module_ids.push_back(m->id());
        for (const std::string& task : engine.modules().tasks(m->id()))
            d->task_owners.emplace(task,
                                   std::make_pair(d->module_ids.size() - 1, task.substr(m->id().size() + 1)));
        d->ports.push_back(m->ports());
        d->outputs.push_back(m->outputs());
        auto& frames = d->visual_frames.emplace_back();
        d->visual_counts.push_back(m->visuals().size());
        for (const VisualInfo& v : m->visuals()) {
            if (v.frame.empty()) {
                frames.emplace_back();
                continue;
            }
            auto id = engine.modules().model()->frame_id(v.frame);
            if (!id)
                return make_error(
                    ErrorCode::invalid_argument,
                    "debug server: module '" + m->id() + "' visual '" + v.name + "': " + id.error().message);
            frames.emplace_back(*id);
        }
    }
    d->latest_params.resize(d->module_ids.size());
    const ModelSource& source = engine.modules().model()->source();
    if (d->options.urdf.empty() && source.kind == ModelSource::Kind::file && source.format == "urdf")
        d->options.urdf = source.location;
    if (options.serve_assets) {
        auto built = detail::build_model_assets(source, engine.context().resources);
        const Logger& log = engine.context().log;
        if (!built) {
            if (source.format == "urdf")
                log.log(LogLevel::warn, "debug_server", "not serving model assets: " + built.error().message);
        } else {
            if (!built->missing.empty()) {
                std::string list;
                for (const std::string& m : built->missing) list += (list.empty() ? "" : ", ") + m;
                log.log(LogLevel::warn, "debug_server",
                        "model files not found, clients render without them: " + list);
            }
            d->assets = std::move(*built);
        }
    }
    std::vector<ConfigNode> caps;
    for (const char* op : {"ping", "subscribe", "acquire", "release"}) caps.emplace_back(op);
    for (const char* op : kCommandOps) caps.emplace_back(op);
    if (d->assets) caps.emplace_back("asset");
    d->capabilities = ConfigNode::list(std::move(caps));
    d->build_model(engine);
    d->prepare_snapshots(engine);

    if (::pipe2(d->wake, O_NONBLOCK | O_CLOEXEC) != 0) return sys_error("pipe");
    d->listen_fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (d->listen_fd < 0) return sys_error("socket");
    const int one = 1;
    (void)::setsockopt(d->listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port = htons(options.port);
    sa.sin_addr = addr;
    if (::bind(d->listen_fd, reinterpret_cast<const sockaddr*>(&sa), sizeof sa) != 0)
        return sys_error("bind " + host + ":" + std::to_string(options.port));
    if (::listen(d->listen_fd, 8) != 0) return sys_error("listen");
    socklen_t len = sizeof sa;
    if (::getsockname(d->listen_fd, reinterpret_cast<sockaddr*>(&sa), &len) != 0)
        return sys_error("getsockname");
    d->bound_port = ntohs(sa.sin_port);

    Impl* raw = d.get();
    d->io = std::thread([raw] { raw->run(); });
    std::shared_ptr<DebugServer> server(new DebugServer(std::move(d)));
    engine.add_observer(server);
    return server;
}

}  // namespace kstone
