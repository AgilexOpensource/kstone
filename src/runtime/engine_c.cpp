#include "kstone/runtime/engine_c.h"

#include "kstone/control/solver.hpp"
#include "kstone/core/context.hpp"
#include "kstone/core/error.hpp"
#include "kstone/model/model.hpp"
#include "kstone/module/module.hpp"
#include "kstone/module/yaml.hpp"
#include "kstone/runtime/config.hpp"
#include "kstone/runtime/engine.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <exception>
#include <filesystem>
#include <memory>
#include <new>
#include <string>
#include <utility>

struct kstone_engine {
    explicit kstone_engine(kstone::Engine e) : engine(std::move(e)) {}
    kstone::Engine engine;
    std::atomic<bool> stop{false};
    mutable kstone::PortValue scratch;  // Reused by set_port/read_port so steady-state calls do not allocate.
};

namespace {

using kstone::ErrorCode;
using kstone::PortType;

void set_error(char* error, std::size_t size, const char* message) noexcept {
    if (!error || size == 0) return;
    const std::size_t n = std::min(std::strlen(message), size - 1);
    std::memcpy(error, message, n);
    error[n] = '\0';
}

int fail(char* error, std::size_t size, const kstone::Error& e) noexcept {
    set_error(error, size, e.message.c_str());
    return static_cast<int>(e.code);
}

int fail(char* error, std::size_t size, ErrorCode code, const char* message) noexcept {
    set_error(error, size, message);
    return static_cast<int>(code);
}

// Runs body at the boundary; C++ exceptions never cross into the host.
template <typename F>
int guarded(char* error, std::size_t size, F&& body) noexcept {
    try {
        return body();
    } catch (const std::bad_alloc&) {
        return fail(error, size, ErrorCode::internal, "out of memory");
    } catch (const std::exception& ex) {
        return fail(error, size, ErrorCode::internal, ex.what());
    } catch (...) {
        return fail(error, size, ErrorCode::internal, "unknown exception");
    }
}

std::size_t flat_size(const kstone::PortInfo& info) noexcept {
    switch (info.type) {
        case PortType::vector:
            return info.labels.size();
        case PortType::pose:
            return 7;
        case PortType::number:
        case PortType::flag:
            return 1;
    }
    return 1;
}

void copy_out(const Eigen::VectorXd& from, double* to, std::size_t n) noexcept {
    if (!to) return;
    const std::size_t m = std::min(n, static_cast<std::size_t>(from.size()));
    std::copy(from.data(), from.data() + m, to);
    std::fill(to + m, to + n, 0.0);
}

struct Port {
    kstone::Module* module{nullptr};
    std::size_t index{0};
};

int find_port(const kstone_engine* e, const char* module, const char* port, Port& out, char* error,
              std::size_t size) {
    if (!e || !module || !port) return fail(error, size, ErrorCode::invalid_argument, "null argument");
    for (const auto& m : e->engine.modules().modules()) {
        if (m->id() != module) continue;
        const int i = m->port_index(port);
        if (i < 0)
            return fail(error, size, ErrorCode::not_found, (m->id() + ": no port '" + port + "'").c_str());
        out = {m.get(), static_cast<std::size_t>(i)};
        return KSTONE_ENGINE_OK;
    }
    return fail(error, size, ErrorCode::not_found, (std::string("no module '") + module + "'").c_str());
}

int finish(std::unique_ptr<kstone_engine> e, kstone_engine** out) noexcept {
    *out = e.release();
    return KSTONE_ENGINE_OK;
}

}  // namespace

extern "C" {

uint32_t kstone_engine_abi_version(void) { return KSTONE_ENGINE_ABI_VERSION; }

int kstone_engine_load(const char* path, const kstone_engine_options* options, kstone_engine** out,
                       char* error, std::size_t error_size) {
    (void)options;
    return guarded(error, error_size, [&]() -> int {
        if (!path || !out) return fail(error, error_size, ErrorCode::invalid_argument, "null argument");
        *out = nullptr;
        auto engine = kstone::load_engine(path);
        if (!engine) return fail(error, error_size, engine.error());
        return finish(std::make_unique<kstone_engine>(std::move(*engine)), out);
    });
}

int kstone_engine_create(const char* config_yaml, const char* base_dir, const kstone_engine_options* options,
                         kstone_engine** out, char* error, std::size_t error_size) {
    (void)options;
    return guarded(error, error_size, [&]() -> int {
        if (!config_yaml || !out)
            return fail(error, error_size, ErrorCode::invalid_argument, "null argument");
        *out = nullptr;
        const std::filesystem::path dir = base_dir ? base_dir : ".";
        const kstone::Context context = kstone::Context::defaults();
        kstone::EngineLoadOptions load;
        load.yaml.base_dir = dir;
        load.yaml.resources = &context.resources;
        auto config = kstone::parse_yaml(config_yaml, "<string>", load.yaml);
        if (!config) return fail(error, error_size, config.error());
        auto engine = kstone::build_engine(*config, dir, load, context);
        if (!engine) return fail(error, error_size, engine.error());
        return finish(std::make_unique<kstone_engine>(std::move(*engine)), out);
    });
}

void kstone_engine_destroy(kstone_engine* engine) { delete engine; }

size_t kstone_engine_nq(const kstone_engine* engine) {
    return engine ? static_cast<std::size_t>(engine->engine.modules().model()->nq()) : 0;
}

size_t kstone_engine_nv(const kstone_engine* engine) {
    return engine ? static_cast<std::size_t>(engine->engine.modules().model()->nv()) : 0;
}

double kstone_engine_dt(const kstone_engine* engine) { return engine ? engine->engine.dt() : 0.0; }

int kstone_engine_step(kstone_engine* engine, double now, char* error, std::size_t error_size) {
    return guarded(error, error_size, [&]() -> int {
        if (!engine) return fail(error, error_size, ErrorCode::invalid_argument, "null engine");
        const kstone::Status s = engine->engine.step(now);
        return s ? KSTONE_ENGINE_OK : fail(error, error_size, s.error());
    });
}

int kstone_engine_run(kstone_engine* engine, double duration_s, char* error, std::size_t error_size) {
    return guarded(error, error_size, [&]() -> int {
        if (!engine) return fail(error, error_size, ErrorCode::invalid_argument, "null engine");
        const kstone::Status s = engine->engine.run(duration_s, &engine->stop);
        engine->stop.store(false, std::memory_order_relaxed);
        return s ? KSTONE_ENGINE_OK : fail(error, error_size, s.error());
    });
}

void kstone_engine_request_stop(kstone_engine* engine) {
    if (engine) engine->stop.store(true, std::memory_order_relaxed);
}

void kstone_engine_set_paused(kstone_engine* engine, int paused) {
    if (engine) engine->engine.set_paused(paused != 0);
}

int kstone_engine_snapshot(const kstone_engine* engine, kstone_engine_status* out, char* error,
                           std::size_t error_size) {
    return guarded(error, error_size, [&]() -> int {
        if (!engine || !out) return fail(error, error_size, ErrorCode::invalid_argument, "null argument");
        if (out->struct_size < sizeof(kstone_engine_status))
            return fail(error, error_size, ErrorCode::invalid_argument, "snapshot struct_size too small");
        const kstone::Engine& e = engine->engine;
        const kstone::EngineStats& st = e.stats();
        out->cycles = st.cycles;
        out->overruns = st.overruns;
        out->repeated_reads = st.repeated_reads;
        out->guard_rejects = st.guard_rejects;
        out->guard_clamps = st.guard_clamps;
        out->held_cycles = st.held_cycles;
        out->stale_cycles = st.stale_cycles;
        out->resyncs = st.resyncs;
        out->failed_cycles = st.failed_cycles;
        out->time = st.time;
        out->last_cycle_s = st.last_cycle_s;
        out->max_cycle_s = st.max_cycle_s;
        out->max_lateness_s = st.max_lateness_s;
        out->cycle_p50_s = st.cycle_hist.quantile(0.5);
        out->cycle_p99_s = st.cycle_hist.quantile(0.99);
        out->cycle_p999_s = st.cycle_hist.quantile(0.999);
        out->paused = e.paused();
        out->fresh = e.fresh();
        out->written = e.written();
        const kstone::SolveResult& r = e.result();
        out->degraded = r.quality == kstone::SolveQuality::degraded;
        out->held = r.held;
        out->step_scale = r.step_scale;
        out->state_stamp_ns = e.state().stamp_ns;
        out->command_fresh = e.command().fresh;
        const Eigen::VectorXd* measured = e.measured();
        out->has_measured = measured != nullptr;
        const std::size_t nq = kstone_engine_nq(engine), nv = kstone_engine_nv(engine);
        copy_out(e.state().q, out->state_q, nq);
        copy_out(e.state().v, out->state_v, nv);
        copy_out(e.command().q, out->command_q, nq);
        copy_out(e.command().v, out->command_v, nv);
        copy_out(e.ideal(), out->ideal, nq);
        if (measured)
            copy_out(*measured, out->measured, nq);
        else if (out->measured)
            std::fill(out->measured, out->measured + nq, 0.0);
        return KSTONE_ENGINE_OK;
    });
}

int kstone_engine_set_port(kstone_engine* engine, const char* module, const char* port, const double* values,
                           size_t n, char* error, std::size_t error_size) {
    return guarded(error, error_size, [&]() -> int {
        Port p;
        if (const int rc = find_port(engine, module, port, p, error, error_size)) return rc;
        const kstone::PortInfo& info = p.module->ports()[p.index];
        const std::size_t want = flat_size(info);
        if (n > 0 && !values) return fail(error, error_size, ErrorCode::invalid_argument, "null values");
        if (n != want)
            return fail(
                error, error_size, ErrorCode::invalid_argument,
                (info.name + ": expected " + std::to_string(want) + " values, got " + std::to_string(n))
                    .c_str());
        for (std::size_t i = 0; i < n; ++i)
            if (!std::isfinite(values[i]))
                return fail(error, error_size, ErrorCode::invalid_argument,
                            (info.name + ": non-finite value").c_str());
        kstone::PortValue& v = engine->scratch;
        switch (info.type) {
            case PortType::number:
                v.number = values[0];
                break;
            case PortType::flag:
                v.flag = values[0] != 0.0;
                break;
            case PortType::vector:
                v.vector = Eigen::Map<const Eigen::VectorXd>(values, static_cast<Eigen::Index>(n));
                break;
            case PortType::pose: {
                Eigen::Quaterniond q(values[3], values[4], values[5], values[6]);
                if (q.norm() < 1e-9)
                    return fail(error, error_size, ErrorCode::invalid_argument,
                                (info.name + ": zero quaternion").c_str());
                q.normalize();
                v.pose.setIdentity();
                v.pose.linear() = q.toRotationMatrix();
                v.pose.translation() = Eigen::Vector3d(values[0], values[1], values[2]);
                break;
            }
        }
        const kstone::Status s = p.module->write_port(p.index, v);
        return s ? KSTONE_ENGINE_OK : fail(error, error_size, s.error());
    });
}

int kstone_engine_read_port(const kstone_engine* engine, const char* module, const char* port, double* values,
                            size_t capacity, size_t* n, char* error, std::size_t error_size) {
    return guarded(error, error_size, [&]() -> int {
        Port p;
        if (const int rc = find_port(engine, module, port, p, error, error_size)) return rc;
        if (!n) return fail(error, error_size, ErrorCode::invalid_argument, "null argument");
        const kstone::PortInfo& info = p.module->ports()[p.index];
        *n = flat_size(info);
        if (!values) return KSTONE_ENGINE_OK;
        if (capacity < *n)
            return fail(error, error_size, ErrorCode::out_of_range,
                        (info.name + ": needs " + std::to_string(*n) + " values").c_str());
        kstone::PortValue& v = engine->scratch;
        const kstone::Status s = p.module->read_port(p.index, v);
        if (!s) return fail(error, error_size, s.error());
        switch (info.type) {
            case PortType::number:
                values[0] = v.number;
                break;
            case PortType::flag:
                values[0] = v.flag ? 1.0 : 0.0;
                break;
            case PortType::vector:
                copy_out(v.vector, values, *n);
                break;
            case PortType::pose: {
                const Eigen::Quaterniond q(v.pose.linear());
                const Eigen::Vector3d t = v.pose.translation();
                const double flat[7] = {t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z()};
                std::copy(flat, flat + 7, values);
                break;
            }
        }
        return KSTONE_ENGINE_OK;
    });
}

}  // extern "C"
