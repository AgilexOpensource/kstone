// Python binding: load, step and run engines, poke module ports, attach the debug server, query kinematics.
#include "kstone/core/version.hpp"
#include "kstone/model/model.hpp"
#include "kstone/model/workspace.hpp"
#include "kstone/module/module.hpp"
#include "kstone/runtime/config.hpp"
#include "kstone/runtime/debug_server.hpp"
#include "kstone/runtime/doctor.hpp"
#include "kstone/runtime/snapshot.hpp"

#include "cli.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace py = pybind11;
using namespace pybind11::literals;

namespace {

using kstone::ErrorCode;
using kstone::PortType;

// Translated to kstone.Error with a `code` attribute.
struct KstoneError {
    kstone::Error error;
};

[[noreturn]] void raise(kstone::Error e) { throw KstoneError{std::move(e)}; }
void check(const kstone::Status& s) {
    if (!s) raise(s.error());
}

// Hands one snapshot from the control thread to a Python reader; costs one atomic load per cycle otherwise.
struct SnapshotRequest {
    std::mutex reader;
    std::atomic<bool> requested{false};
    std::atomic<bool> done{false};
    kstone::EngineSnapshot buffer;
};

class SnapshotObserver final : public kstone::EngineObserver {
  public:
    explicit SnapshotObserver(std::shared_ptr<SnapshotRequest> request) : request_(std::move(request)) {}
    void after_step(const kstone::Engine& engine) override {
        if (!request_->requested.load(std::memory_order_acquire)) return;
        kstone::take_snapshot(engine, request_->buffer);
        request_->requested.store(false, std::memory_order_relaxed);
        request_->done.store(true, std::memory_order_release);
    }

  private:
    std::shared_ptr<SnapshotRequest> request_;
};

struct PyEngine {
    explicit PyEngine(kstone::Engine e) : engine(std::move(e)) {
        engine.add_observer(std::make_shared<SnapshotObserver>(snapshot));
    }
    kstone::Engine engine;
    std::atomic<bool> running{false};
    std::atomic<bool> stop{false};
    std::shared_ptr<SnapshotRequest> snapshot = std::make_shared<SnapshotRequest>();

    // Engine state is owned by the run() thread while running.
    void idle() const {
        if (running.load()) throw std::runtime_error("kstone: engine is running; stop() it first");
    }
};

struct PyModule {
    std::shared_ptr<PyEngine> owner;
    std::shared_ptr<kstone::Module> module;

    std::size_t port(const std::string& name) const {
        const int i = module->port_index(name);
        if (i >= 0) return static_cast<std::size_t>(i);
        std::string known;
        for (const auto& p : module->ports()) known += (known.empty() ? "" : ", ") + p.name;
        raise(kstone::make_error(ErrorCode::not_found, module->id() + ": no port '" + name +
                                                           "'; ports: " + (known.empty() ? "none" : known)));
    }
};

struct PyServer {
    std::shared_ptr<PyEngine> owner;
    std::shared_ptr<kstone::DebugServer> server;
};

py::object encode(const kstone::PortInfo& info, const kstone::PortValue& value) {
    switch (info.type) {
        case PortType::number:
            return py::float_(value.number);
        case PortType::flag:
            return py::bool_(value.flag);
        case PortType::vector:
            return py::cast(value.vector);
        case PortType::pose: {
            const Eigen::Quaterniond r(value.pose.linear());
            const Eigen::Vector3d t = value.pose.translation();
            const Eigen::Vector4d wxyz(r.w(), r.x(), r.y(), r.z());
            return py::dict("translation"_a = t, "rotation"_a = wxyz);
        }
    }
    return py::none();
}

py::object encode_output(const kstone::OutputInfo& info, const kstone::OutputValue& value) {
    switch (info.type) {
        case kstone::OutputType::number:
            return py::float_(value.number);
        case kstone::OutputType::flag:
            return py::bool_(value.flag);
        case kstone::OutputType::text:
            return py::str(value.text);
        case kstone::OutputType::point:
            return py::cast(Eigen::Vector3d(value.points.col(0)));
        case kstone::OutputType::segment:
        case kstone::OutputType::polygon:
            return py::cast(Eigen::MatrixX3d(value.points.transpose()));
        case kstone::OutputType::visuals:
            return py::cast(value.visuals);
    }
    return py::none();
}

const char* shape_name(kstone::ShapeType type) noexcept {
    switch (type) {
        case kstone::ShapeType::sphere:
            return "sphere";
        case kstone::ShapeType::box:
            return "box";
        case kstone::ShapeType::cylinder:
            return "cylinder";
        case kstone::ShapeType::capsule:
            return "capsule";
        case kstone::ShapeType::convex:
            return "convex";
    }
    return "unknown";
}

bool has_bool(py::handle obj) {
    if (py::isinstance<py::array>(obj)) return py::reinterpret_borrow<py::array>(obj).dtype().kind() == 'b';
    if (py::isinstance<py::list>(obj) || py::isinstance<py::tuple>(obj))
        for (const auto& item : obj)
            if (py::isinstance<py::bool_>(item)) return true;
    return false;
}

void decode(const kstone::PortInfo& info, py::handle obj, kstone::PortValue& out) {
    const auto bad = [&info]() {
        std::string expected;
        switch (info.type) {
            case PortType::number:
                expected = "a finite number";
                break;
            case PortType::flag:
                expected = "a bool";
                break;
            case PortType::vector:
                expected = std::to_string(info.size()) + " finite numbers";
                break;
            case PortType::pose:
                expected = "{'translation': [x, y, z], 'rotation': [w, x, y, z]} or a 4x4 rigid transform";
                break;
        }
        raise(kstone::make_error(ErrorCode::invalid_argument,
                                 "bad value for " + std::string(kstone::to_string(info.type)) + " port '" +
                                     info.name + "', expected " + expected));
    };
    try {
        switch (info.type) {
            case PortType::number:
                if (py::isinstance<py::bool_>(obj)) bad();
                out.number = obj.cast<double>();
                if (!std::isfinite(out.number)) bad();
                return;
            case PortType::flag:
                if (!py::isinstance<py::bool_>(obj)) bad();
                out.flag = obj.cast<bool>();
                return;
            case PortType::vector:
                if (has_bool(obj)) bad();
                out.vector = obj.cast<Eigen::VectorXd>();
                if (out.vector.size() != info.size() || !out.vector.allFinite()) bad();
                return;
            case PortType::pose: {
                if (!py::isinstance<py::dict>(obj)) {
                    const Eigen::MatrixXd m = obj.cast<Eigen::MatrixXd>();
                    if (m.rows() != 4 || m.cols() != 4 || !m.allFinite()) bad();
                    const Eigen::Matrix3d rot = m.topLeftCorner<3, 3>();
                    if (!(m.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).isZero(1e-9) ||
                        !(rot.transpose() * rot - Eigen::Matrix3d::Identity()).isZero(1e-6) ||
                        rot.determinant() < 0)
                        bad();
                    out.pose.setIdentity();
                    out.pose.linear() = Eigen::Quaterniond(rot).normalized().toRotationMatrix();
                    out.pose.translation() = m.topRightCorner<3, 1>();
                    return;
                }
                const py::dict d = py::reinterpret_borrow<py::dict>(obj);
                if (d.size() != 2 || !d.contains("translation") || !d.contains("rotation")) bad();
                const Eigen::VectorXd t = d["translation"].cast<Eigen::VectorXd>();
                const Eigen::VectorXd r = d["rotation"].cast<Eigen::VectorXd>();
                if (t.size() != 3 || r.size() != 4 || !t.allFinite() || !r.allFinite()) bad();
                Eigen::Quaterniond q(r[0], r[1], r[2], r[3]);
                if (q.norm() < 1e-9) bad();
                q.normalize();
                out.pose.setIdentity();
                out.pose.linear() = q.toRotationMatrix();
                out.pose.translation() = t;
                return;
            }
        }
    } catch (const py::cast_error&) {
        bad();
    }
    bad();
}

// bool is checked before int because Python's bool subclasses int.
kstone::ConfigNode to_node(py::handle obj, const std::string& path) {
    if (obj.is_none()) return {};
    if (py::isinstance<py::bool_>(obj)) return obj.cast<bool>();
    if (py::isinstance<py::int_>(obj) || py::isinstance<py::float_>(obj)) return obj.cast<double>();
    if (py::isinstance<py::str>(obj)) return obj.cast<std::string>();
    if (py::isinstance<py::dict>(obj)) {
        kstone::ConfigNode out = kstone::ConfigNode::map();
        for (const auto& kv : obj.cast<py::dict>()) {
            if (!py::isinstance<py::str>(kv.first))
                raise(kstone::make_error(ErrorCode::invalid_argument, path + ": keys must be strings"));
            const std::string key = kv.first.cast<std::string>();
            out.set(key, to_node(kv.second, path + "." + key));
        }
        return out;
    }
    if (py::isinstance<py::sequence>(obj) || py::hasattr(obj, "tolist")) {
        py::object seq =
            py::hasattr(obj, "tolist") ? obj.attr("tolist")() : py::reinterpret_borrow<py::object>(obj);
        if (!py::isinstance<py::sequence>(seq)) return to_node(seq, path);
        kstone::ConfigNode out = kstone::ConfigNode::list();
        std::size_t i = 0;
        for (const auto& item : seq) out.append(to_node(item, path + "[" + std::to_string(i++) + "]"));
        return out;
    }
    raise(kstone::make_error(ErrorCode::invalid_argument,
                             path + ": unsupported type " + py::str(py::type::of(obj)).cast<std::string>()));
}

py::object to_py(const kstone::ConfigNode& n) {
    using K = kstone::ConfigNode::Kind;
    switch (n.kind()) {
        case K::null:
            return py::none();
        case K::boolean:
            return py::bool_(n.as_bool());
        case K::number:
            return py::float_(n.as_number());
        case K::string:
            return py::str(n.as_string());
        case K::list: {
            py::list out;
            for (const auto& item : n.items()) out.append(to_py(item));
            return py::object(std::move(out));
        }
        case K::map: {
            py::dict out;
            for (const auto& [k, v] : n.entries()) out[py::str(k)] = to_py(v);
            return py::object(std::move(out));
        }
    }
    return py::none();
}

py::dict port_info(const kstone::PortInfo& p) {
    py::dict d("name"_a = p.name, "type"_a = kstone::to_string(p.type));
    if (p.type == PortType::vector) d["labels"] = p.labels;
    if (p.lower.size() > 0) d["lower"] = p.lower;
    if (p.upper.size() > 0) d["upper"] = p.upper;
    if (p.type == PortType::pose) {
        d["frame"] = p.frame;
        d["reference"] = p.reference;
    }
    if (p.timeout > 0.0) d["timeout_s"] = p.timeout;
    return d;
}

std::shared_ptr<PyEngine> load(const std::filesystem::path& path,
                               const std::map<std::string, std::string>& vars) {
    kstone::EngineLoadOptions options;
    options.yaml.vars = vars;
    auto engine = kstone::load_engine(path, options);
    if (!engine) raise(std::move(engine).error());
    return std::make_shared<PyEngine>(std::move(*engine));
}

// Same preprocessing as load_engine; includes and the model path resolve against base_dir.
std::shared_ptr<PyEngine> load_string(const std::string& text, const std::filesystem::path& base_dir,
                                      const std::optional<std::map<std::string, std::string>>& vars) {
    const kstone::Context context = kstone::Context::defaults();
    kstone::EngineLoadOptions options;
    if (vars) options.yaml.vars = *vars;
    options.yaml.base_dir = base_dir;
    options.yaml.resources = &context.resources;
    auto config = kstone::parse_yaml(text, "<string>", options.yaml);
    if (!config) raise(std::move(config).error());
    auto engine = kstone::build_engine(*config, base_dir, options, context);
    if (!engine) raise(std::move(engine).error());
    return std::make_shared<PyEngine>(std::move(*engine));
}

// Offline: steps the engine's ModuleSet directly; the robot stays an unused loopback.
struct PySolver {
    std::shared_ptr<PyEngine> owner;
    Eigen::VectorXd initial;
    double time{0.0};
};

// The robot section keeps only the initial posture, so a plugin robot is never loaded.
kstone::ConfigNode offline(const kstone::ConfigNode& config) {
    if (config.kind() != kstone::ConfigNode::Kind::map) return config;
    kstone::ConfigNode out = kstone::ConfigNode::map();
    out.set_location(config.location());
    for (const auto& [key, value] : config.entries()) {
        if (key != "robot" || value.kind() != kstone::ConfigNode::Kind::map) {
            out.set(key, value);
            continue;
        }
        kstone::ConfigNode robot = kstone::ConfigNode::map();
        robot.set_location(value.location());
        for (const auto& [k, v] : value.entries())
            if (k == "initial" || k == "initial_root") robot.set(k, v);
        out.set(key, std::move(robot));
    }
    return out;
}

PySolver make_solver(const kstone::ConfigNode& config, const std::filesystem::path& base_dir,
                     const kstone::Context& context) {
    auto engine = kstone::build_engine(offline(config), base_dir, {}, context);
    if (!engine) raise(std::move(engine).error());
    auto owner = std::make_shared<PyEngine>(std::move(*engine));
    Eigen::VectorXd q0 = owner->engine.state().q;
    return PySolver{std::move(owner), std::move(q0)};
}

PySolver load_solver(const std::filesystem::path& path, const std::map<std::string, std::string>& vars) {
    const kstone::Context context = kstone::Context::defaults();
    kstone::YamlOptions yaml = kstone::YamlOptions::files();
    yaml.vars = vars;
    yaml.resources = &context.resources;
    auto config = kstone::load_yaml(path, yaml);
    if (!config) raise(std::move(config).error());
    return make_solver(*config, path.parent_path(), context);
}

PySolver load_solver_string(const std::string& text, const std::filesystem::path& base_dir,
                            const std::optional<std::map<std::string, std::string>>& vars) {
    const kstone::Context context = kstone::Context::defaults();
    kstone::YamlOptions yaml = kstone::YamlOptions::files();
    if (vars) yaml.vars = *vars;
    yaml.base_dir = base_dir;
    yaml.resources = &context.resources;
    auto config = kstone::parse_yaml(text, "<string>", yaml);
    if (!config) raise(std::move(config).error());
    return make_solver(*config, base_dir, context);
}

py::list stages(const kstone::SolveResult& r) {
    py::list out;
    for (const kstone::StageReport& st : r.stages)
        out.append(py::dict("level"_a = st.level, "status"_a = kstone::qp::to_string(st.status),
                            "exitflag"_a = st.exitflag, "iterations"_a = st.iterations, "held"_a = st.held));
    return out;
}

py::list residuals(const kstone::SolveResult& r) {
    py::list out;
    for (const kstone::TaskResidual& t : r.residuals)
        out.append(py::dict("name"_a = t.name, "level"_a = t.level, "residual"_a = t.residual));
    return out;
}

struct PyModel {
    kstone::ModelPtr model;

    kstone::JointId joint(const std::string& name) const {
        auto id = model->joint_id(name);
        if (!id) raise(std::move(id).error());
        return *id;
    }
    kstone::JointId joint(std::int64_t id) const {
        if (id < 0 || static_cast<std::size_t>(id) >= model->joints().size())
            throw py::index_error("kstone: joint id " + std::to_string(id) + " out of range");
        return static_cast<kstone::JointId>(id);
    }
    kstone::FrameId frame(const std::string& name) const {
        auto id = model->frame_id(name);
        if (!id) raise(std::move(id).error());
        return *id;
    }
    kstone::FrameId frame(std::int64_t id) const {
        if (id < 0 || static_cast<std::size_t>(id) >= model->frames().size())
            throw py::index_error("kstone: frame id " + std::to_string(id) + " out of range");
        return static_cast<kstone::FrameId>(id);
    }
    void check_vector(const Eigen::VectorXd& x, kstone::Index size, const char* what) const {
        if (x.size() != size)
            throw py::value_error("kstone: " + std::string(what) + " has size " + std::to_string(x.size()) +
                                  ", expected " + std::to_string(size));
        if (!x.allFinite()) throw py::value_error("kstone: " + std::string(what) + " is not finite");
    }
};

struct PyWorkspace {
    explicit PyWorkspace(const PyModel& m) : model(m), workspace(m.model) {}
    PyModel model;
    kstone::Workspace workspace;
};

template <typename E>
E parse_enum(const std::string& value, const char* what,
             std::initializer_list<std::pair<const char*, E>> names) {
    for (const auto& [name, e] : names)
        if (value == name) return e;
    throw py::value_error("kstone: unknown " + std::string(what) + " '" + value + "'");
}

kstone::ModelOptions model_options(const std::string& root, const std::string& mimic) {
    kstone::ModelOptions o;
    o.root = parse_enum<kstone::RootJoint>(
        root, "root", {{"fixed", kstone::RootJoint::fixed}, {"free_flyer", kstone::RootJoint::free_flyer}});
    o.mimic = parse_enum<kstone::MimicMode>(
        mimic, "mimic",
        {{"independent", kstone::MimicMode::independent}, {"coupled", kstone::MimicMode::coupled}});
    return o;
}

PyModel load_model(const kstone::ModelSource& source, const std::string& root, const std::string& mimic) {
    const kstone::ModelOptions options = model_options(root, mimic);
    auto model = kstone::Model::load(source, options, kstone::Context::defaults());
    if (!model) raise(std::move(model).error());
    return PyModel{std::move(*model)};
}

py::dict joint_info(const kstone::JointInfo& j) {
    return py::dict("name"_a = j.name, "type"_a = kstone::to_string(j.type), "id"_a = j.id,
                    "parent"_a = j.parent, "idx_q"_a = j.idx_q, "nq"_a = j.nq, "idx_v"_a = j.idx_v,
                    "nv"_a = j.nv);
}

Eigen::Matrix4d frame_pose(const PyWorkspace& w, kstone::FrameId frame) {
    Eigen::Isometry3d pose;
    check(w.workspace.frame_pose(frame, pose));
    return pose.matrix();
}

Eigen::Matrix4d joint_pose(const PyWorkspace& w, kstone::JointId joint) {
    Eigen::Isometry3d pose;
    check(w.workspace.joint_pose(joint, pose));
    return pose.matrix();
}

Eigen::Vector3d center_of_mass(PyWorkspace& w, kstone::JointId root) {
    Eigen::Vector3d out;
    check(w.workspace.subtree_center_of_mass(root, out));
    return out;
}

Eigen::MatrixXd center_of_mass_jacobian(PyWorkspace& w, kstone::JointId root) {
    Eigen::MatrixXd out(3, w.model.model->nv());
    check(w.workspace.subtree_center_of_mass_jacobian(root, out));
    return out;
}

Eigen::MatrixXd frame_jacobian(PyWorkspace& w, kstone::FrameId frame, const std::string& reference) {
    const auto ref = parse_enum<kstone::ReferenceFrame>(
        reference, "reference",
        {{"local", kstone::ReferenceFrame::local},
         {"local_world_aligned", kstone::ReferenceFrame::local_world_aligned},
         {"world", kstone::ReferenceFrame::world}});
    Eigen::MatrixXd jac(6, w.model.model->nv());
    check(w.workspace.frame_jacobian(frame, ref, jac));
    return jac;
}

constexpr const char* kJacobianDoc = R"doc(6 x nv Jacobian of the frame: rows 0-2 linear, rows 3-5 angular.

Args:
    frame: Frame name or index.
    reference: Axes the rows are expressed in: 'local_world_aligned' (world axes
        at the frame origin), 'local' (frame axes) or 'world' (world axes at the
        world origin).)doc";

py::dict stats_dict(const kstone::EngineStats& s) {
    return py::dict("cycles"_a = s.cycles, "overruns"_a = s.overruns, "repeated_reads"_a = s.repeated_reads,
                    "guard_rejects"_a = s.guard_rejects, "guard_clamps"_a = s.guard_clamps,
                    "held_cycles"_a = s.held_cycles, "stale_cycles"_a = s.stale_cycles,
                    "resyncs"_a = s.resyncs, "failed_cycles"_a = s.failed_cycles, "time"_a = s.time,
                    "last_cycle_s"_a = s.last_cycle_s, "max_cycle_s"_a = s.max_cycle_s,
                    "max_lateness_s"_a = s.max_lateness_s, "cycle_p50_s"_a = s.cycle_hist.quantile(0.5),
                    "cycle_p99_s"_a = s.cycle_hist.quantile(0.99),
                    "cycle_p999_s"_a = s.cycle_hist.quantile(0.999));
}

py::dict snapshot_dict(const PyEngine& e, const kstone::EngineSnapshot& s) {
    const auto& mods = e.engine.modules().modules();
    py::list modules;
    for (std::size_t i = 0; i < s.modules.size() && i < mods.size(); ++i) {
        const kstone::ModuleSnapshot& m = s.modules[i];
        const auto& infos = mods[i]->ports();
        py::dict ports;
        for (std::size_t p = 0; p < m.ports.size() && p < infos.size(); ++p)
            ports[py::str(infos[p].name)] = encode(infos[p], m.ports[p]);
        const py::object held_by =
            m.held_by >= 0 && static_cast<std::size_t>(m.held_by) < s.modules.size()
                ? py::object(py::str(s.modules[static_cast<std::size_t>(m.held_by)].id))
                : py::object(py::none());
        modules.append(
            py::dict("id"_a = m.id, "type"_a = m.type, "enabled"_a = m.enabled, "running"_a = m.running,
                     "held_by"_a = held_by, "preempted"_a = m.preempted,
                     "home"_a = m.home_state ? py::object(py::str(m.home_state)) : py::object(py::none()),
                     "ports"_a = ports));
    }
    py::list res;
    for (const kstone::TaskResidual& t : s.residuals)
        res.append(py::dict("name"_a = t.name, "level"_a = t.level, "residual"_a = t.residual));
    return py::dict(
        "schema_version"_a = s.schema_version, "stats"_a = stats_dict(s.stats), "paused"_a = s.paused,
        "fresh"_a = s.fresh, "written"_a = s.written,
        "state"_a = py::dict("q"_a = s.state.q, "v"_a = s.state.v, "stamp_ns"_a = s.state.stamp_ns),
        "command"_a = py::dict("q"_a = s.command.q, "v"_a = s.command.v, "fresh"_a = s.command.fresh),
        "ideal"_a = s.ideal, "measured"_a = s.has_measured ? py::cast(s.measured) : py::none(),
        "quality"_a = kstone::to_string(s.quality), "held"_a = s.held, "step_scale"_a = s.step_scale,
        "residuals"_a = res, "modules"_a = modules);
}

std::shared_ptr<kstone::Module> find_module(kstone::Engine& engine, const std::string& id) {
    auto mod = engine.modules().module(id);
    if (mod) return mod;
    std::string known;
    for (const auto& m : engine.modules().modules()) known += (known.empty() ? "" : ", ") + m->id();
    raise(kstone::make_error(ErrorCode::not_found, "no module '" + id + "'; modules: " + known));
}

// While running, the control thread fills the buffer after its next cycle; otherwise it is filled here.
py::dict snapshot(PyEngine& e) {
    SnapshotRequest& r = *e.snapshot;
    {
        py::gil_scoped_release release;
        r.reader.lock();
    }
    std::lock_guard<std::mutex> hold(r.reader, std::adopt_lock);
    for (;;) {
        // With the GIL held and running false, neither step() nor run() can touch the engine.
        if (!e.running.load()) {
            r.requested = false;
            kstone::take_snapshot(e.engine, r.buffer);
            break;
        }
        r.done = false;
        r.requested.store(true, std::memory_order_release);
        {
            py::gil_scoped_release release;
            while (!r.done.load(std::memory_order_acquire) && e.running.load())
                std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        if (r.done.load(std::memory_order_acquire)) break;
    }
    return snapshot_dict(e, r.buffer);
}

// Runs on a worker thread so Ctrl-C (checked here, on the caller) can stop it.
void run(PyEngine& e, double duration_s) {
    e.idle();
    e.stop = false;  // Before `running` turns true, so a stop() seen after that is never lost.
    bool expected = false;
    if (!e.running.compare_exchange_strong(expected, true))
        throw std::runtime_error("kstone: engine is running");
    bool interrupted = false;
    kstone::Status status;
    {
        py::gil_scoped_release release;
        auto done =
            std::async(std::launch::async, [&e, duration_s] { return e.engine.run(duration_s, &e.stop); });
        while (done.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
            py::gil_scoped_acquire acquire;
            if (!interrupted && PyErr_CheckSignals() != 0) {
                interrupted = true;
                e.stop = true;
            }
        }
        status = done.get();
    }
    e.running = false;
    if (interrupted) throw py::error_already_set();
    check(status);
}

}  // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() = "Native core of kstone; import kstone instead";
    m.attr("__version__") = kstone::library_version_string();
    m.attr("SNAPSHOT_SCHEMA_VERSION") = kstone::snapshot_schema_version;

    // Owned by the module attribute; a borrowed pointer avoids a static destructor after interpreter shutdown.
    static PyObject* error_type = py::exception<KstoneError>(m, "Error", PyExc_RuntimeError).ptr();
    py::setattr(error_type, "__doc__",
                py::str("Raised by kstone; `code` names the kind of failure, e.g. 'invalid_argument'."));
    py::register_exception_translator([](std::exception_ptr p) {
        try {
            if (p) std::rethrow_exception(p);
        } catch (const KstoneError& e) {
            py::object exc = py::reinterpret_borrow<py::object>(error_type)(e.error.message);
            exc.attr("code") = kstone::to_string(e.error.code);
            PyErr_SetObject(error_type, exc.ptr());
        }
    });

    py::class_<PyModule>(m, "Module", "One module of an engine or solver; get it with engine.module(id).")
        .def_property_readonly(
            "id", [](const PyModule& s) { return s.module->id(); }, "Module id from the config.")
        .def_property_readonly(
            "type", [](const PyModule& s) { return std::string(s.module->type()); },
            "Module type, e.g. 'frame_task'.")
        .def("__repr__",
             [](const PyModule& s) {
                 return "<kstone.Module '" + s.module->id() + "' type=" + std::string(s.module->type()) + ">";
             })
        .def_property(
            "enabled",
            [](const PyModule& s) {
                s.owner->idle();
                return s.module->enabled();
            },
            [](const PyModule& s, bool enabled) {
                s.owner->idle();
                s.module->set_enabled(enabled);
            },
            "Whether the module takes part in the solve.")
        .def_property_readonly(
            "running",
            [](const PyModule& s) {
                s.owner->idle();
                return s.module->running();
            },
            "True if the module was active in the last step.")
        .def_property_readonly(
            "held_by",
            [](const PyModule& s) -> py::object {
                s.owner->idle();
                const std::string* by = s.module->held_by();
                if (by == nullptr) return py::none();
                return py::str(*by);
            },
            "Id of the higher-priority module that blocked this one in the last step, or None.")
        .def_property_readonly(
            "preempted",
            [](const PyModule& s) {
                s.owner->idle();
                return s.module->preempted();
            },
            "How many times another module has blocked this one.")
        .def_property_readonly(
            "home_state",
            [](const PyModule& s) -> py::object {
                s.owner->idle();
                const char* state = s.module->home_state();
                return state ? py::object(py::str(state)) : py::object(py::none());
            },
            "Homing progress: idle, waiting, executing, succeeded, timed_out or aborted; None if not "
            "homeable.")
        .def_property_readonly(
            "ports",
            [](const PyModule& s) {
                py::list out;
                for (const auto& p : s.module->ports()) out.append(port_info(p));
                return out;
            },
            "Input ports as dicts with name, type, labels, lower and upper.")
        .def(
            "read_port",
            [](const PyModule& s, const std::string& name) {
                s.owner->idle();
                const std::size_t i = s.port(name);
                kstone::PortValue value;
                check(s.module->read_port(i, value));
                return encode(s.module->ports()[i], value);
            },
            "name"_a, R"doc(Current value of an input port.

Args:
    name: Port name; see ports.

Returns:
    A float, bool or array, or for a pose {'translation': [x, y, z],
    'rotation': [w, x, y, z]}.)doc")
        .def(
            "write_port",
            [](const PyModule& s, const std::string& name, py::handle value) {
                s.owner->idle();
                const std::size_t i = s.port(name);
                kstone::PortValue decoded;
                decode(s.module->ports()[i], value, decoded);
                check(s.module->write_port(i, decoded));
            },
            "name"_a, "value"_a,
            R"doc(Set an input port; the module uses it from the next step.

Args:
    name: Port name; see ports.
    value: A float, bool or sequence of the port's size. A pose takes a 4x4
        matrix or {'translation': [x, y, z], 'rotation': [w, x, y, z]}.

Example:
    engine.module("arm").write_port("target", [0.5, 0.0, 0.0]))doc")
        .def(
            "home",
            [](const PyModule& s) {
                s.owner->idle();
                check(s.module->request_home());
            },
            "Start homing on the next step; watch home_state.")
        .def_property_readonly(
            "homeable", [](const PyModule& s) { return s.module->homeable(); },
            "Whether home() is supported.")
        .def_property_readonly(
            "tunable", [](const PyModule& s) { return !s.module->tune_schema().is_null(); },
            "Whether tune() is supported.")
        .def_property_readonly(
            "tune_schema", [](const PyModule& s) { return to_py(s.module->tune_schema()); },
            "JSON Schema of the tune() parameters, or None.")
        .def(
            "tune",
            [](const PyModule& s, const py::dict& params) {
                s.owner->idle();
                check(s.module->tune(to_node(params, s.module->id())));
            },
            "params"_a, R"doc(Change some parameters; on error nothing changes.

Args:
    params: Names and new values, e.g. {'gain': 2.0}; see tune_schema.)doc")
        .def(
            "params",
            [](const PyModule& s) {
                s.owner->idle();
                return to_py(s.module->tunables());
            },
            "Current tunable parameters, or None.")
        .def(
            "outputs",
            [](const PyModule& s, const Eigen::VectorXd& q) {
                s.owner->idle();
                const PyModel model{s.owner->engine.modules().model()};
                model.check_vector(q, model.model->nq(), "q");
                check(model.model->validate(q));
                kstone::Workspace at(model.model);
                check(at.update(q));
                check(s.module->update_outputs(at));
                py::dict out;
                const auto& infos = s.module->outputs();
                for (std::size_t i = 0; i < infos.size(); ++i) {
                    kstone::OutputValue v;
                    check(s.module->read_output(i, v));
                    out[py::str(infos[i].name)] = encode_output(infos[i], v);
                }
                return out;
            },
            "q"_a,
            R"doc(Display values computed at q, keyed by output name.

Args:
    q: Configuration, size nq.)doc")
        .def(
            "visuals",
            [](const PyModule& s) {
                s.owner->idle();
                py::list out;
                const auto& infos = s.module->visuals();
                for (std::size_t i = 0; i < infos.size(); ++i) {
                    const kstone::VisualInfo& v = infos[i];
                    py::dict shape("type"_a = shape_name(v.shape.type), "size"_a = v.shape.size);
                    if (v.shape.type == kstone::ShapeType::convex)
                        shape["vertices"] = Eigen::MatrixX3d(v.shape.vertices.transpose());
                    Eigen::Isometry3d placement = v.placement;
                    const bool shown = s.module->visual_placement(i, placement);
                    out.append(py::dict("name"_a = v.name, "frame"_a = v.frame, "shape"_a = std::move(shape),
                                        "placement"_a = shown ? py::cast(Eigen::Matrix4d(placement.matrix()))
                                                              : py::object(py::none())));
                }
                return out;
            },
            "Shapes to draw: dicts with name, frame, shape and placement (4x4, or None when hidden).");

    py::class_<PyServer>(m, "DebugServer", "Debug server started by engine.serve().")
        .def_property_readonly(
            "port", [](const PyServer& s) { return s.server->port(); }, "Port the server listens on.")
        .def(
            "stop",
            [](const PyServer& s) {
                py::gil_scoped_release release;
                s.server->stop();
            },
            "Stop the server; safe to call twice.");

    py::class_<kstone::SolveResult>(m, "SolveResult", "Result of one solve.")
        .def_property_readonly(
            "v", [](const kstone::SolveResult& r) { return Eigen::VectorXd(r.v); },
            "Joint velocity, size nv.")
        .def_property_readonly(
            "q_next", [](const kstone::SolveResult& r) { return Eigen::VectorXd(r.q_next); },
            "Configuration after dt, size nq; pass it as the next q.")
        .def_property_readonly("stages", &stages,
                               "Per priority level: level, status, exitflag, iterations, held.")
        .def_property_readonly("residuals", &residuals, "Per task: name, level and remaining error.")
        .def_readonly("held", &kstone::SolveResult::held,
                      "True if some level could not be met and was held back.")
        .def_readonly("step_scale", &kstone::SolveResult::step_scale,
                      "1.0, or less when the step was shortened to stay within constraints.")
        .def_property_readonly(
            "quality", [](const kstone::SolveResult& r) { return std::string(kstone::to_string(r.quality)); },
            "'nominal' when every task was solved as asked, else 'degraded'.")
        .def("__repr__", [](const kstone::SolveResult& r) {
            return "<kstone.SolveResult quality=" + std::string(kstone::to_string(r.quality)) +
                   " stages=" + std::to_string(r.stages.size()) + ">";
        });

    py::class_<PyEngine, std::shared_ptr<PyEngine>>(m, "Engine",
                                                    "Control loop built from a YAML config; see load_engine.")
        .def_property_readonly(
            "dt", [](const PyEngine& e) { return e.engine.dt(); }, "Cycle period in seconds.")
        .def_property_readonly(
            "rate_hz", [](const PyEngine& e) { return e.engine.options().rate_hz; }, "Cycle rate in Hz.")
        .def_property_readonly(
            "realtime", [](const PyEngine& e) { return e.engine.options().realtime; },
            "Whether run() paces cycles to wall-clock time.")
        .def_property_readonly(
            "feedback_timeout_s", [](const PyEngine& e) { return e.engine.options().feedback_timeout_s; },
            "Feedback older than this counts as stale.")
        .def_property_readonly(
            "max_failed_cycles", [](const PyEngine& e) { return e.engine.options().max_failed_cycles; },
            "Consecutive failed cycles that stop the engine.")
        .def_property_readonly(
            "feedback_joints", [](const PyEngine& e) { return e.engine.options().feedback_joints; },
            "Joints whose measured position feeds back into the solve.")
        .def_property_readonly(
            "safety",
            [](const PyEngine& e) {
                const kstone::CommandGuardOptions& g = e.engine.options().safety;
                const char* mode = g.mode == kstone::GuardMode::off     ? "off"
                                   : g.mode == kstone::GuardMode::clamp ? "clamp"
                                                                        : "reject";
                return py::dict("mode"_a = mode, "velocity_scale"_a = g.velocity_scale,
                                "tolerance"_a = g.tolerance);
            },
            "Command check settings: mode ('reject', 'clamp' or 'off'), velocity_scale, tolerance.")
        .def_property_readonly(
            "quality",
            [](const PyEngine& e) {
                e.idle();
                return std::string(kstone::to_string(e.engine.result().quality));
            },
            "Quality of the last solve: 'nominal' or 'degraded'.")
        .def_property_readonly(
            "solver_tune_schema",
            [](const PyEngine& e) { return to_py(e.engine.modules().solver_tune_schema()); },
            "JSON Schema of the tune_solver() parameters.")
        .def(
            "tune_solver",
            [](PyEngine& e, const py::dict& params) {
                e.idle();
                check(e.engine.modules().tune_solver(to_node(params, "solver")));
            },
            "params"_a, R"doc(Change some solver options; on error nothing changes.

Args:
    params: Names and new values, e.g. {'damping': 1e-4}; see solver_tune_schema.)doc")
        .def(
            "solver_params",
            [](PyEngine& e) {
                e.idle();
                return to_py(e.engine.modules().solver_params());
            },
            "Current solver options.")
        .def(
            "tune_joint_limits",
            [](PyEngine& e, const py::dict& params) {
                e.idle();
                check(e.engine.modules().tune_joint_limits(to_node(params, "joint_limits")));
            },
            "params"_a, R"doc(Narrow joint limits at run time; on error nothing changes.

Args:
    params: {'velocity': {joint: limit}, 'position': {'lower': {joint: limit},
        'upper': {joint: limit}}}, any part optional. Limits can't exceed the
        model's. {'reset': True} removes all overrides.

Example:
    engine.tune_joint_limits({"velocity": {"j1": 0.5}}))doc")
        .def(
            "joint_limits",
            [](PyEngine& e) {
                e.idle();
                return to_py(e.engine.modules().joint_limits());
            },
            "Joint limit overrides currently in force.")
        .def_property_readonly(
            "result",
            [](const PyEngine& e) {
                e.idle();
                return e.engine.result();
            },
            "Result of the last successful solve.")
        .def_property_readonly(
            "running", [](const PyEngine& e) { return e.running.load(); }, "True while run() is active.")
        .def_property_readonly(
            "time",
            [](const PyEngine& e) {
                e.idle();
                return e.engine.stats().time;
            },
            "Engine time in seconds.")
        .def_property_readonly(
            "stats",
            [](const PyEngine& e) {
                e.idle();
                return stats_dict(e.engine.stats());
            },
            "Counters and cycle times, e.g. cycles, overruns, failed_cycles, cycle_p99_s.")
        .def_property_readonly(
            "q",
            [](const PyEngine& e) {
                e.idle();
                return Eigen::VectorXd(e.engine.state().q);
            },
            "Robot joint positions from the last read, size nq.")
        .def_property_readonly(
            "v",
            [](const PyEngine& e) {
                e.idle();
                return Eigen::VectorXd(e.engine.state().v);
            },
            "Robot joint velocities from the last read, size nv.")
        .def_property_readonly(
            "ideal",
            [](const PyEngine& e) {
                e.idle();
                return Eigen::VectorXd(e.engine.ideal());
            },
            "Commanded positions the next solve starts from, size nq.")
        .def_property_readonly(
            "fresh",
            [](const PyEngine& e) {
                e.idle();
                return e.engine.fresh();
            },
            "False when robot feedback is older than feedback_timeout_s.")
        .def_property_readonly(
            "written",
            [](const PyEngine& e) {
                e.idle();
                return e.engine.written();
            },
            "Whether the last cycle sent a command (False while paused).")
        .def_property_readonly(
            "command",
            [](const PyEngine& e) {
                e.idle();
                const kstone::RobotCommand& c = e.engine.command();
                return py::dict("q"_a = Eigen::VectorXd(c.q), "v"_a = Eigen::VectorXd(c.v),
                                "fresh"_a = c.fresh);
            },
            "Last command sent to the robot: {q, v, fresh}.")
        .def_property_readonly(
            "measured",
            [](const PyEngine& e) -> py::object {
                e.idle();
                const Eigen::VectorXd* est = e.engine.measured();
                return est ? py::cast(Eigen::VectorXd(*est)) : py::none();
            },
            "Measured positions used by feedback, or None while feedback is stale.")
        .def_property_readonly(
            "model", [](const PyEngine& e) { return PyModel{e.engine.modules().model()}; }, "Robot model.")
        .def_property_readonly(
            "modules",
            [](const PyEngine& e) {
                py::list out;
                for (const auto& mod : e.engine.modules().modules()) out.append(mod->id());
                return out;
            },
            "Module ids in config order.")
        .def(
            "module",
            [](const std::shared_ptr<PyEngine>& e, const std::string& id) {
                auto mod = find_module(e->engine, id);
                return PyModule{e, std::move(mod)};
            },
            "id"_a, "Module by id.")
        .def(
            "step",
            [](PyEngine& e) {
                e.idle();
                check(e.engine.step(e.engine.stats().time + e.engine.dt()));
            },
            "Run one cycle (read, solve, write).")
        .def("__repr__",
             [](const PyEngine& e) {
                 return "<kstone.Engine " + std::to_string(e.engine.modules().modules().size()) +
                        " modules, " + py::str(py::float_(e.engine.options().rate_hz)).cast<std::string>() +
                        " Hz>";
             })
        .def_property(
            "paused", [](const PyEngine& e) { return e.engine.paused(); },
            [](PyEngine& e, bool paused) { e.engine.set_paused(paused); },
            "While True, cycles keep reading but do not solve or send commands.")
        .def(
            "step_paused", [](PyEngine& e, std::uint64_t cycles) { e.engine.step_paused(cycles); },
            "cycles"_a = 1,
            R"doc(While paused, let some cycles run normally.

Args:
    cycles: How many cycles.)doc")
        .def(
            "set_rate_hz", [](PyEngine& e, double hz) { check(e.engine.set_rate_hz(hz)); }, "rate_hz"_a,
            R"doc(Change the cycle rate from the next cycle on.

Args:
    rate_hz: New rate in Hz, > 0.)doc")
        .def(
            "reset_stats",
            [](PyEngine& e) {
                e.idle();
                e.engine.reset_stats();
            },
            "Reset stats; time is kept.")
        .def("snapshot", &snapshot, "State of the last cycle as one dict; works while run() is active.")
        .def("run", &run, "duration_s"_a = 0.0,
             R"doc(Run cycles, paced to wall-clock time if realtime is set.

Args:
    duration_s: Engine time to run in seconds; 0 runs until stop() or Ctrl-C.)doc")
        .def(
            "stop", [](PyEngine& e) { e.stop = true; }, "Make run() return; call it from another thread.")
        .def(
            "serve",
            [](const std::shared_ptr<PyEngine>& e, const std::string& host, std::uint16_t port,
               const std::optional<std::filesystem::path>& urdf, double publish_hz, const std::string& token,
               double command_timeout_s, bool assets) {
                e->idle();
                kstone::DebugServerOptions o;
                o.host = host;
                o.port = port;
                o.urdf = urdf.value_or(std::filesystem::path{});
                o.publish_hz = publish_hz;
                o.token = token;
                o.command_timeout_s = command_timeout_s;
                o.serve_assets = assets;
                auto server = kstone::DebugServer::start(e->engine, o);
                if (!server) raise(std::move(server).error());
                return PyServer{e, std::move(*server)};
            },
            "host"_a = "127.0.0.1", "port"_a = 7311, "urdf"_a = py::none(), "publish_hz"_a = 30.0,
            "token"_a = "", "command_timeout_s"_a = 1.0, "assets"_a = true,
            R"doc(Start the debug server used by the viewer and remote tools.

Args:
    host: Address to listen on. Any address other than loopback requires a token.
    port: TCP port; 0 picks a free one (see server.port).
    urdf: URDF sent to clients; defaults to the model file if it is a URDF.
    publish_hz: Highest rate of state updates sent to clients.
    token: Secret clients must send, 16 to 1024 bytes; '' disables
        authentication. It travels in clear text.
    command_timeout_s: How long a remote command stays in effect when the
        client sets no timeout.
    assets: Let clients download the URDF and the files it references.

Returns:
    The server; it runs until server.stop().)doc");

    m.def("load_engine", &load, "path"_a, "vars"_a = std::map<std::string, std::string>{},
          R"doc(Load an engine from a YAML config file.

Args:
    path: Config file. Relative paths in it resolve against its folder.
    vars: Values for ${name} placeholders. Unset names fall back to
        environment variables, then to the ${name:-default} default.

Raises:
    kstone.Error: The file can't be read or the config is invalid.

Example:
    engine = kstone.load_engine("robot.yaml", vars={"ip": "192.168.1.10"}))doc");
    m.def("load_engine_from_string", &load_string, "text"_a, "base_dir"_a = std::filesystem::path{"."},
          "vars"_a = py::none(),
          R"doc(Load an engine from YAML text.

Args:
    text: The config.
    base_dir: Folder that relative paths in the config resolve against.
    vars: Values for ${name} placeholders, as in load_engine.

Raises:
    kstone.Error: The config is invalid.)doc");

    py::class_<PySolver>(m, "Solver",
                         "Solver without a robot or loop; you pass q and get the next q. See load_solver.")
        .def_property_readonly(
            "model", [](const PySolver& s) { return PyModel{s.owner->engine.modules().model()}; },
            "Robot model.")
        .def_property_readonly(
            "initial", [](const PySolver& s) { return Eigen::VectorXd(s.initial); },
            "Start configuration: robot.initial from the config, or the neutral pose.")
        .def_property_readonly(
            "time", [](const PySolver& s) { return s.time; }, "Sum of dt over all successful solves.")
        .def_property_readonly(
            "modules",
            [](const PySolver& s) {
                py::list out;
                for (const auto& mod : s.owner->engine.modules().modules()) out.append(mod->id());
                return out;
            },
            "Module ids in config order.")
        .def(
            "module",
            [](const PySolver& s, const std::string& id) {
                auto mod = find_module(s.owner->engine, id);
                return PyModule{s.owner, std::move(mod)};
            },
            "id"_a, "Module by id.")
        .def(
            "solve",
            [](PySolver& s, const Eigen::VectorXd& q, double dt, const std::optional<Eigen::VectorXd>& v,
               const std::optional<Eigen::VectorXd>& measured) {
                kstone::ModuleSet& set = s.owner->engine.modules();
                const PyModel model{set.model()};
                model.check_vector(q, model.model->nq(), "q");
                const Eigen::VectorXd vel = v.value_or(Eigen::VectorXd::Zero(model.model->nv()));
                model.check_vector(vel, model.model->nv(), "v");
                if (measured) model.check_vector(*measured, model.model->nq(), "measured");
                if (!(dt > 0.0) || !std::isfinite(dt))
                    throw py::value_error("kstone: dt must be finite and > 0");
                kstone::SolveResult out;
                check(set.step(q, vel, measured ? &*measured : nullptr, s.time + dt, dt, out));
                s.time += dt;
                return out;
            },
            "q"_a, "dt"_a, "v"_a = py::none(), "measured"_a = py::none(),
            R"doc(Solve one step and return where the robot should go next.

Args:
    q: Current configuration, size nq.
    dt: Step length in seconds, > 0.
    v: Current velocity, size nv; pass the previous result.v. Defaults to zeros.
    measured: Measured configuration, size nq, if you have one; modules and
        closed-loop constraints use it.

Returns:
    The result; pass result.q_next back as the next q.

Raises:
    ValueError: A size is wrong or dt <= 0.

Example:
    q = solver.initial
    for _ in range(100):
        q = solver.solve(q, 0.01).q_next)doc")
        .def(
            "resynchronize",
            [](PySolver& s, bool rebase) { s.owner->engine.modules().resynchronize(rebase); },
            "rebase"_a = false, R"doc(Call after q jumps, e.g. after you move the robot by hand.

Args:
    rebase: Also drop the targets the modules hold, so they hold the new q.)doc")
        .def_property_readonly(
            "solver_tune_schema",
            [](const PySolver& s) { return to_py(s.owner->engine.modules().solver_tune_schema()); },
            "JSON Schema of the tune_solver() parameters.")
        .def(
            "tune_solver",
            [](PySolver& s, const py::dict& params) {
                check(s.owner->engine.modules().tune_solver(to_node(params, "solver")));
            },
            "params"_a, R"doc(Change some solver options; on error nothing changes.

Args:
    params: Names and new values, e.g. {'damping': 1e-4}; see solver_tune_schema.)doc")
        .def(
            "solver_params",
            [](const PySolver& s) { return to_py(s.owner->engine.modules().solver_params()); },
            "Current solver options.")
        .def(
            "tune_joint_limits",
            [](PySolver& s, const py::dict& params) {
                check(s.owner->engine.modules().tune_joint_limits(to_node(params, "joint_limits")));
            },
            "params"_a, R"doc(Narrow joint limits at run time; on error nothing changes.

Args:
    params: {'velocity': {joint: limit}, 'position': {'lower': {joint: limit},
        'upper': {joint: limit}}}, any part optional. Limits can't exceed the
        model's. {'reset': True} removes all overrides.

Example:
    solver.tune_joint_limits({"velocity": {"j1": 0.5}}))doc")
        .def(
            "joint_limits", [](const PySolver& s) { return to_py(s.owner->engine.modules().joint_limits()); },
            "Joint limit overrides currently in force.");

    m.def("load_solver", &load_solver, "path"_a, "vars"_a = std::map<std::string, std::string>{},
          R"doc(Load only the solver part of a config file: model and modules, no robot or loop.

Args:
    path: Config file; its robot section is ignored.
    vars: Values for ${name} placeholders, as in load_engine.

Raises:
    kstone.Error: The file can't be read or the config is invalid.)doc");
    m.def("load_solver_from_string", &load_solver_string, "text"_a, "base_dir"_a = std::filesystem::path{"."},
          "vars"_a = py::none(), R"doc(Load a Solver from YAML text.

Args:
    text: The config; its robot section is ignored.
    base_dir: Folder that relative paths in the config resolve against.
    vars: Values for ${name} placeholders, as in load_engine.)doc");

    m.def(
        "doctor",
        [](bool print) {
            const kstone::DoctorReport report = kstone::doctor();
            py::dict out(to_py(kstone::to_config_node(report)));
            py::dict python("version"_a = py::module_::import("platform").attr("python_version")());
            try {
                python["numpy"] = py::module_::import("numpy").attr("__version__");
            } catch (const py::error_already_set&) {
                python["numpy"] = py::none();
            }
            out["python"] = python;
            if (print) py::print(kstone::to_text(report), "end"_a = "");
            return out;
        },
        "print"_a = false, R"doc(Check the installation.

Args:
    print: Also print a readable report.

Returns:
    A dict; result["ok"] is True when everything works.)doc");

    m.def(
        "_main",
        []() {
            std::vector<std::string> args =
                py::module_::import("sys").attr("argv").cast<std::vector<std::string>>();
            if (args.empty()) args.emplace_back("kstone");
            args[0] = "kstone";
            std::vector<char*> argv;
            for (std::string& a : args) argv.push_back(a.data());
            argv.push_back(nullptr);
            py::gil_scoped_release release;
            const int code = kstone_cli_main(static_cast<int>(args.size()), argv.data());
            std::fflush(stdout);
            return code;
        },
        "Run the kstone command line with sys.argv.");

    py::class_<PyModel>(m, "Model", "Robot model loaded from URDF or MJCF; immutable.")
        .def_static(
            "from_file",
            [](const std::filesystem::path& path, const std::string& format, const std::string& root,
               const std::string& mimic) {
                return load_model(kstone::ModelSource::file(path.string(), format), root, mimic);
            },
            "path"_a, "format"_a = "", "root"_a = "fixed", "mimic"_a = "independent",
            R"doc(Load a robot model from a URDF or MJCF file.

Args:
    path: The file.
    format: 'urdf' or 'mjcf'; '' picks it from the extension (.urdf, .xml, .mjcf).
    root: 'fixed' fixes the root link to the world; 'free_flyer' adds a floating
        base with 7 entries in q (x, y, z, qx, qy, qz, qw) and 6 in v.
    mimic: 'independent' keeps mimic joints as ordinary joints; 'coupled' makes
        them follow their leader, adding no entries to q.

Raises:
    kstone.Error: The file can't be read or parsed.
    ValueError: Unknown root or mimic.)doc")
        .def_static(
            "from_string",
            [](const std::string& text, const std::string& format, const std::filesystem::path& base_dir,
               const std::string& root, const std::string& mimic) {
                return load_model(kstone::ModelSource::text(text, format, base_dir), root, mimic);
            },
            "text"_a, "format"_a = "urdf", "base_dir"_a = std::filesystem::path{}, "root"_a = "fixed",
            "mimic"_a = "independent", R"doc(Load a robot model from URDF or MJCF text.

Args:
    text: The model.
    format: 'urdf' or 'mjcf'.
    base_dir: Folder that mesh paths in the text resolve against.
    root: 'fixed' or 'free_flyer', as in from_file.
    mimic: 'independent' or 'coupled', as in from_file.)doc")
        .def_property_readonly(
            "name", [](const PyModel& s) { return s.model->name(); }, "Robot name.")
        .def("__repr__",
             [](const PyModel& s) {
                 return "<kstone.Model '" + s.model->name() + "' nq=" + std::to_string(s.model->nq()) +
                        " nv=" + std::to_string(s.model->nv()) + ">";
             })
        .def_property_readonly(
            "nq", [](const PyModel& s) { return s.model->nq(); }, "Size of a configuration q.")
        .def_property_readonly(
            "nv", [](const PyModel& s) { return s.model->nv(); }, "Size of a velocity v.")
        .def_property_readonly(
            "joint_names",
            [](const PyModel& s) {
                std::vector<std::string> out;
                for (const auto& j : s.model->joints()) out.push_back(j.name);
                return out;
            },
            "Joint names by id; index 0 is the fixed 'universe' joint.")
        .def_property_readonly(
            "joints",
            [](const PyModel& s) {
                py::list out;
                for (const auto& j : s.model->joints()) out.append(joint_info(j));
                return out;
            },
            "Joint details: name, type, id, parent, idx_q, nq, idx_v, nv.")
        .def_property_readonly(
            "frame_names",
            [](const PyModel& s) {
                std::vector<std::string> out;
                for (const auto& f : s.model->frames()) out.push_back(f.name);
                return out;
            },
            "Frame names by index.")
        .def(
            "joint_id", [](const PyModel& s, const std::string& name) { return s.joint(name); }, "name"_a,
            "Joint id by name; raises kstone.Error (code 'not_found') if there is none.")
        .def(
            "frame_id", [](const PyModel& s, const std::string& name) { return s.frame(name); }, "name"_a,
            "Frame index by name; raises kstone.Error (code 'not_found') if there is none.")
        .def(
            "q_index",
            [](const PyModel& s, const std::string& joint) {
                return s.model->joints()[s.joint(joint)].idx_q;
            },
            "joint"_a)
        .def(
            "q_index",
            [](const PyModel& s, std::int64_t joint) { return s.model->joints()[s.joint(joint)].idx_q; },
            "joint"_a, R"doc(Position of the joint's first entry in q.

Args:
    joint: Joint name or id.

Example:
    q[model.q_index("j1")] = 0.5)doc")
        .def(
            "v_index",
            [](const PyModel& s, const std::string& joint) {
                return s.model->joints()[s.joint(joint)].idx_v;
            },
            "joint"_a)
        .def(
            "v_index",
            [](const PyModel& s, std::int64_t joint) { return s.model->joints()[s.joint(joint)].idx_v; },
            "joint"_a, R"doc(Position of the joint's first entry in v.

Args:
    joint: Joint name or id.)doc")
        .def(
            "neutral", [](const PyModel& s) { return s.model->neutral(); },
            "Default configuration (zeros, unit quaternions), size nq.")
        .def(
            "integrate",
            [](const PyModel& s, const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
                s.check_vector(q, s.model->nq(), "q");
                s.check_vector(v, s.model->nv(), "v");
                check(s.model->validate(q));
                Eigen::VectorXd out(s.model->nq());
                check(s.model->integrate(q, v, out));
                if (!out.allFinite()) throw py::value_error("kstone: integrate overflowed; v is too large");
                return out;
            },
            "q"_a, "v"_a, R"doc(Configuration reached from q by moving with velocity v for 1 s.

Args:
    q: Configuration, size nq.
    v: Velocity, size nv; scale it by dt for a step of dt seconds.)doc")
        .def(
            "normalize",
            [](const PyModel& s, const Eigen::VectorXd& q, double tolerance) {
                s.check_vector(q, s.model->nq(), "q");
                Eigen::VectorXd out = q;
                check(s.model->normalize(out, tolerance));
                return out;
            },
            "q"_a, "tolerance"_a = 1e-3,
            R"doc(Copy of q with quaternions rescaled to unit length.

Args:
    q: Configuration, size nq.
    tolerance: Largest accepted deviation of a norm from 1.

Raises:
    kstone.Error: A norm is off by more than tolerance.)doc")
        .def_property_readonly(
            "lower_position_limit",
            [](const PyModel& s) { return Eigen::VectorXd(s.model->lower_position_limit()); },
            "Lower joint limits, size nq; see has_position_limit.")
        .def_property_readonly(
            "upper_position_limit",
            [](const PyModel& s) { return Eigen::VectorXd(s.model->upper_position_limit()); },
            "Upper joint limits, size nq; see has_position_limit.")
        .def_property_readonly(
            "velocity_limit", [](const PyModel& s) { return Eigen::VectorXd(s.model->velocity_limit()); },
            "Velocity limits, size nv.")
        .def_property_readonly(
            "effort_limit", [](const PyModel& s) { return Eigen::VectorXd(s.model->effort_limit()); },
            "Effort limits, size nv.")
        .def(
            "has_position_limit",
            [](const PyModel& s, const std::string& joint) {
                return s.model->has_position_limit(s.joint(joint));
            },
            "joint"_a)
        .def(
            "has_position_limit",
            [](const PyModel& s, std::int64_t joint) { return s.model->has_position_limit(s.joint(joint)); },
            "joint"_a, R"doc(Whether the joint has position limits.

Args:
    joint: Joint name or id.)doc")
        .def(
            "check_position_limits",
            [](const PyModel& s, const Eigen::VectorXd& q, double tolerance) {
                s.check_vector(q, s.model->nq(), "q");
                check(s.model->check_position_limits(q, tolerance));
            },
            "q"_a, "tolerance"_a = 1e-6, R"doc(Raise kstone.Error if q is outside the position limits.

Args:
    q: Configuration, size nq.
    tolerance: Allowed overshoot, in rad or m.)doc")
        .def_property_readonly(
            "total_mass", [](const PyModel& s) { return s.model->total_mass(); },
            "Mass of the whole robot in kg.")
        .def(
            "subtree_mass",
            [](const PyModel& s, const std::string& root) { return s.model->subtree_mass(s.joint(root)); },
            "root"_a)
        .def(
            "subtree_mass",
            [](const PyModel& s, std::int64_t root) { return s.model->subtree_mass(s.joint(root)); },
            "root"_a, R"doc(Mass in kg of the joint and everything after it.

Args:
    root: Joint name or id.)doc");

    py::class_<PyWorkspace>(m, "Workspace",
                            "Forward kinematics for a model: call update(q), then query poses and Jacobians.")
        .def(py::init<const PyModel&>(), "model"_a, "Create a workspace for the model.")
        .def_property_readonly(
            "model", [](const PyWorkspace& w) { return w.model; }, "The model.")
        .def_property_readonly(
            "q", [](const PyWorkspace& w) { return Eigen::VectorXd(w.workspace.q()); },
            "Configuration of the last update().")
        .def(
            "update",
            [](PyWorkspace& w, const Eigen::VectorXd& q) {
                w.model.check_vector(q, w.model.model->nq(), "q");
                check(w.model.model->validate(q));
                check(w.workspace.update(q));
            },
            "q"_a, R"doc(Compute kinematics at q; call it before the queries below.

Args:
    q: Configuration, size nq.)doc")
        .def(
            "frame_pose",
            [](const PyWorkspace& w, const std::string& frame) {
                return frame_pose(w, w.model.frame(frame));
            },
            "frame"_a)
        .def(
            "frame_pose",
            [](const PyWorkspace& w, std::int64_t frame) { return frame_pose(w, w.model.frame(frame)); },
            "frame"_a, R"doc(4x4 pose of the frame in the world, as of the last update().

Args:
    frame: Frame name or index.)doc")
        .def(
            "joint_pose",
            [](const PyWorkspace& w, const std::string& joint) {
                return joint_pose(w, w.model.joint(joint));
            },
            "joint"_a)
        .def(
            "joint_pose",
            [](const PyWorkspace& w, std::int64_t joint) { return joint_pose(w, w.model.joint(joint)); },
            "joint"_a, R"doc(4x4 pose of the joint in the world, as of the last update().

Args:
    joint: Joint name or id.)doc")
        .def(
            "center_of_mass",
            [](PyWorkspace& w, const std::string& root) { return center_of_mass(w, w.model.joint(root)); },
            "root"_a)
        .def(
            "center_of_mass",
            [](PyWorkspace& w, std::int64_t root) { return center_of_mass(w, w.model.joint(root)); },
            "root"_a = 0, R"doc(Centre of mass in the world, as of the last update().

Args:
    root: Joint name or id; only it and the links after it count.
        The default 0 is the whole robot.)doc")
        .def(
            "center_of_mass_jacobian",
            [](PyWorkspace& w, const std::string& root) {
                return center_of_mass_jacobian(w, w.model.joint(root));
            },
            "root"_a)
        .def(
            "center_of_mass_jacobian",
            [](PyWorkspace& w, std::int64_t root) { return center_of_mass_jacobian(w, w.model.joint(root)); },
            "root"_a = 0, R"doc(3 x nv Jacobian of the centre of mass.

Args:
    root: Joint name or id, as in center_of_mass.)doc")
        .def(
            "frame_jacobian",
            [](PyWorkspace& w, const std::string& frame, const std::string& reference) {
                return frame_jacobian(w, w.model.frame(frame), reference);
            },
            "frame"_a, "reference"_a = "local_world_aligned")
        .def(
            "frame_jacobian",
            [](PyWorkspace& w, std::int64_t frame, const std::string& reference) {
                return frame_jacobian(w, w.model.frame(frame), reference);
            },
            "frame"_a, "reference"_a = "local_world_aligned", kJacobianDoc);
}
