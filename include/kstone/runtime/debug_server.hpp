#pragma once

#include "kstone/core/error.hpp"
#include "kstone/core/export.hpp"
#include "kstone/module/config.hpp"
#include "kstone/runtime/engine.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace kstone {

struct DebugServerOptions {
    /// IPv4 address or "localhost". Non-loopback hosts (including 0.0.0.0) require a token.
    std::string host{"127.0.0.1"};
    std::uint16_t port{7311};  ///< 0 = ephemeral, see DebugServer::port().
    /// Reported to clients so they can render the model; empty = the engine model's file when it is a URDF.
    std::filesystem::path urdf;
    /// Lets authenticated clients download the URDF model and the files it references (nothing else).
    bool serve_assets{true};
    double publish_hz{30.0};
    /// Shared secret (16..1024 bytes); empty = no authentication. Sent in clear text: use a trusted network or tunnel.
    std::string token;
    double command_timeout_s{1.0};  ///< Default for commands without "timeout_s".
};

/// Remote control/inspection channel for one engine, protocol version 1.
/// Frames: 4-byte big-endian length + JSON object (at most 1 MiB; a longer one closes the connection).
/// Non-finite numbers go out as null; requests may spell infinity as Infinity / -Infinity.
/// Server -> client:
///   hello  {version, client, auth_required, capabilities}  on connect; capabilities lists the
///          optional ops this server accepts (asset only when it serves assets);
///   model  {urdf, assets, rate_hz, publish_hz, nq, nv, joints, mimics, modules, solver, stats}  once authenticated; each module has
///          {id, type, ports, homeable, tunable, visuals, outputs} plus {params, tune_schema} when tunable;
///          visuals [{name, frame ("" = world), shape {type, size, vertices? (convex)}}];
///          outputs [{name, type (number|flag|point|segment|polygon|visuals), size? (polygon, visuals),
///          role (value|margin|boundary|warning|alert, see OutputRole), unit? (SI symbol)}];
///          each joint has {name, type, parent, idx_q, nq, idx_v, nv, limits} (parent "universe" for the root);
///          limits {position (declared range), lower[nq], upper[nq], velocity[nv], effort[nv]} from Model;
///          mimics [{follower, leader, multiplier, offset}]: q_follower = multiplier * q_leader + offset;
///          solver {params, tune_schema} (see ModuleSet::tune_solver);
///          joint_limits {velocity, position {lower, upper}}: run-time overrides (see ModuleSet::joint_limits);
///          stats [{name, kind (counter|duration), unit? ("s" for durations)}]: the keys of state.stats (EngineStats);
///          assets is null or {id, model, files: {name: {sha256, size}}, missing}: a URDF whose references point
///          into the bundle, so a client on another host can fetch each file with the asset op;
///   params {module, values}  to every client after a successful tune, and the latest one per module right after model;
///   solver_params {values}  likewise after a successful tune_solver;
///   joint_limits {values}  likewise after a successful tune_limits;
///   lease  {holder, mine}  after auth and whenever the control lease changes;
///   state  {time, paused, fresh, rate_hz, stats {name: value}, q (measured), q_command, modules, residuals}  per subscription
///          (default publish_hz); each module's display is null (no outputs or visuals, or not computed yet) or
///          {at ("measured" when fresh, else "command"), outputs {name: value}, visual_poses [{translation, rotation (wxyz)} or null (hidden)]},
///          recomputed at most at publish_hz; point = [x,y,z], segment/polygon = list of points, visuals = indices;
///          residuals [{name (solver task), module (owner id or null), task (name within the module), level, residual}];
///          a client that reads slower than its rate skips state ticks; one with 8 MiB of other output queued is closed.
///   reply  {id, ok, code?, error?}  for every request, matched by id (order is not guaranteed).
/// Client -> server, all {"op": ..., "id": any}:
///   auth {token}; ping; subscribe {rate_hz in [0, publish_hz], 0 = off};
///   acquire {force?}; release;
///   set_port {module, port, value}; set_enabled {module, enabled}; home {module};
///   tune {module, params (partial map, all-or-nothing)}  with optional timeout_s; tune_solver {params} likewise;
///          tune_limits {params} (ModuleSet::tune_joint_limits, tighten only) likewise.
///   pause {paused}: hold the robot (see Engine::set_paused); step {cycles in [1, 1e6], default 1}: step the modules
///          while paused (invalid_argument when running).
///   set_rate {rate_hz in [1, 10000]}: Engine::set_rate_hz, applied at the next cycle.
///   asset {name, offset?}  -> reply {data (base64, <= 256 KiB), offset, total}; no lease needed.
/// Unknown ops are answered with code "unsupported".
/// At most 8 clients; a further connection gets reply {id: null, code "busy"} instead of hello and is closed.
/// Before auth only "auth" is accepted; a bad token or an unauthenticated connection after 5 s is closed.
/// Commands are rejected ("lease_held") while another client holds the lease; with no holder anyone may command.
/// Leases are released on disconnect. Codes: ErrorCode names, unauthorized, lease_held, busy, timeout.
/// Socket IO runs on its own thread; the control thread only touches preallocated, lock-free buffers,
/// except tune, tune_solver and tune_limits, which allocate on the control thread during that cycle.
class KSTONE_EXPORT DebugServer final : public EngineObserver {
  public:
    /// Binds, starts the IO thread and registers itself as an observer (so not while the engine runs).
    static Result<std::shared_ptr<DebugServer>> start(Engine& engine, const DebugServerOptions& options = {});

    ~DebugServer() override;
    DebugServer(const DebugServer&) = delete;
    DebugServer& operator=(const DebugServer&) = delete;

    /// Bound port.
    std::uint16_t port() const noexcept;
    /// Closes every connection and joins the IO thread; commands are no longer applied.
    void stop();

    void before_step(Engine& engine) override;
    void after_step(const Engine& engine) override;

    struct Impl;

  private:
    explicit DebugServer(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

}  // namespace kstone
