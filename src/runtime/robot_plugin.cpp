#include "kstone/runtime/robot_plugin.hpp"

#include "kstone/runtime/robot_plugin.h"

#include <array>
#include <cstddef>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <utility>
#include <vector>

namespace kstone {
namespace {

constexpr std::size_t kMinPluginSize =
    offsetof(kstone_robot_plugin, write) + sizeof(kstone_robot_plugin::write);

ErrorCode to_error_code(int rc) {
    if (rc >= static_cast<int>(ErrorCode::invalid_argument) && rc <= static_cast<int>(ErrorCode::internal))
        return static_cast<ErrorCode>(rc);
    return ErrorCode::internal;
}

class PluginRobot final : public RobotInterface {
  public:
    PluginRobot(std::shared_ptr<void> library, const kstone_robot_plugin& plugin, std::string label, Index nq,
                Index nv)
        : library_(std::move(library)), plugin_(plugin), label_(std::move(label)), nq_(nq), nv_(nv) {}

    ~PluginRobot() override {
        if (handle_) plugin_.destroy(handle_);
    }

    PluginRobot(const PluginRobot&) = delete;
    PluginRobot& operator=(const PluginRobot&) = delete;

    Status create(const kstone_robot_config& config) {
        error_[0] = '\0';
        void* handle = nullptr;
        const int rc = plugin_.create(&config, &handle, error_.data(), error_.size());
        if (rc != KSTONE_ROBOT_OK) return failure("create", rc);
        if (!handle) return make_error(ErrorCode::internal, label_ + ": create returned a null handle");
        handle_ = handle;
        return {};
    }

    Status read(RobotState& state) override {
        if (state.q.size() != nq_) state.q.resize(nq_);
        if (state.v.size() != nv_) state.v.resize(nv_);
        error_[0] = '\0';
        std::int64_t stamp = -1;
        const int rc = plugin_.read(handle_, state.q.data(), size(nq_), state.v.data(), size(nv_), &stamp,
                                    error_.data(), error_.size());
        state.stamp_ns = stamp;
        return rc == KSTONE_ROBOT_OK ? Status{} : failure("read", rc);
    }

    Status write(const RobotCommand& command) override {
        if (command.q.size() != nq_ || command.v.size() != nv_)
            return make_error(ErrorCode::invalid_argument, label_ + ": command size mismatch");
        error_[0] = '\0';
        const int rc = plugin_.write(handle_, command.q.data(), size(nq_), command.v.data(), size(nv_),
                                     command.fresh ? 1 : 0, error_.data(), error_.size());
        return rc == KSTONE_ROBOT_OK ? Status{} : failure("write", rc);
    }

  private:
    static std::size_t size(Index n) { return static_cast<std::size_t>(n); }

    // Error path only: building the message allocates.
    Error failure(const char* call, int rc) {
        error_.back() = '\0';
        std::string text = label_ + ": " + call + " failed";
        if (error_[0] != '\0') text += ": " + std::string(error_.data());
        return make_error(to_error_code(rc), std::move(text));
    }

    std::shared_ptr<void> library_;  // Declared first: closed after destroy().
    kstone_robot_plugin plugin_;
    std::string label_;
    Index nq_;
    Index nv_;
    void* handle_{nullptr};
    std::array<char, 512> error_{};
};

}  // namespace

Result<std::shared_ptr<RobotInterface>> load_robot_plugin(const ModelPtr& model,
                                                          const RobotPluginOptions& options) {
    if (!model) return make_error(ErrorCode::invalid_argument, "robot plugin: model is null");
    const std::string path = options.library.string();
    if (path.empty()) return make_error(ErrorCode::invalid_argument, "robot plugin: library is empty");

    dlerror();
    void* raw = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!raw) {
        const char* why = dlerror();
        return make_error(ErrorCode::not_found,
                          "robot plugin: cannot load '" + path + "': " + (why ? why : "?"));
    }
    std::shared_ptr<void> library(raw, [](void* h) { dlclose(h); });

    void* sym = dlsym(raw, KSTONE_ROBOT_PLUGIN_SYMBOL);
    if (!sym)
        return make_error(ErrorCode::not_found,
                          "robot plugin: '" + path + "' does not export " KSTONE_ROBOT_PLUGIN_SYMBOL "()");
    kstone_robot_plugin_fn entry = nullptr;
    static_assert(sizeof entry == sizeof sym, "function and object pointers differ in size");
    std::memcpy(&entry, &sym, sizeof entry);
    const kstone_robot_plugin* desc = entry();
    if (!desc) return make_error(ErrorCode::internal, "robot plugin: '" + path + "' returned no descriptor");
    if (desc->abi_version != KSTONE_ROBOT_ABI_VERSION)
        return make_error(ErrorCode::unsupported, "robot plugin: '" + path + "' uses ABI version " +
                                                      std::to_string(desc->abi_version) + ", expected " +
                                                      std::to_string(KSTONE_ROBOT_ABI_VERSION));
    if (desc->struct_size < kMinPluginSize || !desc->create || !desc->destroy || !desc->read || !desc->write)
        return make_error(ErrorCode::invalid_argument,
                          "robot plugin: '" + path + "' has an incomplete descriptor");

    kstone_robot_plugin plugin{};
    std::memcpy(&plugin, desc, kMinPluginSize);
    const std::string label = "robot plugin '" + std::string(desc->name ? desc->name : path) + "'";

    const Model& m = *model;
    Eigen::VectorXd q0 = options.initial.size() == 0 ? m.neutral() : options.initial;
    if (q0.size() != m.nq())
        return make_error(ErrorCode::invalid_argument,
                          label + ": initial has " + std::to_string(q0.size()) +
                              " values, the model has nq=" + std::to_string(m.nq()));

    std::vector<kstone_robot_joint> joints;
    for (const JointInfo& j : m.joints()) {
        if (j.type == JointType::universe) continue;
        joints.push_back({j.name.c_str(), static_cast<std::size_t>(j.idx_q), static_cast<std::size_t>(j.nq),
                          static_cast<std::size_t>(j.idx_v), static_cast<std::size_t>(j.nv)});
    }
    const std::string base_dir = options.base_dir.string();

    kstone_robot_config config{};
    config.struct_size = sizeof config;
    config.nq = static_cast<std::size_t>(m.nq());
    config.nv = static_cast<std::size_t>(m.nv());
    config.rate_hz = options.rate_hz;
    config.joints = joints.data();
    config.joint_count = joints.size();
    config.q0 = q0.data();
    config.params_json = options.params_json.c_str();
    config.base_dir = base_dir.c_str();

    auto robot = std::make_shared<PluginRobot>(std::move(library), plugin, label, m.nq(), m.nv());
    if (auto s = robot->create(config); !s) return std::move(s).error();
    return std::shared_ptr<RobotInterface>(std::move(robot));
}

}  // namespace kstone
