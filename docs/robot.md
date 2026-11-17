# Robot integration

[中文](zh-CN/robot.md) · [README](../README.md)

## Running a config

```bash
kstone check <config.yaml>       # build the engine without running it
kstone run <config.yaml> [--duration s] [--realtime] [--serve [host:port|port]] [--token-file path]
```

- `--set name=value` (repeatable) sets a config variable, see [variables and includes](config.md#variables-and-includes).
- Relative file paths in the config, such as `model.file` or a module's `urdf`, are resolved against the directory of the config file first.
- `kstone schema [type]` lists the module types or prints the JSON Schema of one type. The config keys are in [Configuration](config.md), the module pages in [Modules](modules/README.md).

## Driver plugins

A driver is a shared library implementing the C ABI in [robot_plugin.h](../include/kstone/runtime/robot_plugin.h). Start from [example_robot.c](../examples/robot_plugin/example_robot.c). `read` must return a CLOCK_MONOTONIC timestamp, otherwise the feedback counts as stale. With `feedback_groups` configured, each joint carries its group index, `read` also stamps each group in `group_stamps_ns` (-1 = stale; only `default` falls back to the state stamp), and `write` receives each group's attached flag in `group_attached`.

```bash
cc -shared -fPIC -Iinclude examples/robot_plugin/example_robot.c -o libexample_robot.so
```

```yaml
robot:
  type: plugin
  library: ./libexample_robot.so  # a path with '/' is relative to the config file; a bare name uses the loader search path
  params: {}                      # passed unchanged to the plugin's create()
```

C++ hosts can implement `RobotInterface` directly instead, see [C++ API](api.md) and [engine.cpp](../examples/cpp/engine.cpp).

## Real-time thread

On Linux the control thread can use real-time scheduling. The settings apply when `run()` starts with `realtime: true`, and priority and affinity are restored when it returns. With `realtime: false` the `thread` block is ignored.

```yaml
thread:
  priority: 50       # SCHED_FIFO 1..99, higher runs first, 0 = unchanged; needs CAP_SYS_NICE or ulimit -r >= value
  cpus: [3]          # CPUs to pin to, empty = unchanged; works best with isolcpus
  lock_memory: true  # mlockall for the whole process, never undone; needs CAP_IPC_LOCK or a large enough ulimit -l
```

- If the system refuses a setting (missing CAP_SYS_NICE, RLIMIT_RTPRIO or CAP_IPC_LOCK), the engine logs a warning and keeps running, as ros2_control_node does. Invalid values still fail at creation.
- `lock_memory` calls `mlockall(MCL_CURRENT | MCL_FUTURE)`.
- When realtime is in effect (`--realtime` or `realtime: true`), `kstone run` calls `elevate_thread_limits`: if `ulimit -r` is below `priority` (or `ulimit -l` is limited with `lock_memory`) and the capability is missing, it runs `sudo prlimit --pid <self>` once at startup, prompting for a password on the terminal. Only the limits are raised; the process does not run as root. Without a terminal, or if sudo fails, it warns and continues.
- Calling `step()` directly is not affected.

## Run contract

- **Failed cycle.** If reading, solving or writing fails, the engine writes no command and does not synthesize a stop command. The cycle counts in `failed_cycles`. `run()` keeps going and returns the last error after `max_failed_cycles` (default 5) consecutive failures. Drivers must implement their own command timeout and safe state.
- **Stale feedback.** Feedback older than `feedback_timeout_s` is not a failure. The command is still written, with `RobotCommand::fresh = false`, and the driver or caller decides whether to send it.
- **Attach.** `Engine::attached()` becomes true after `feedback_timeout_s` of continuously fresh feedback. Samples must keep arriving across that window: a short burst followed by silence stays fresh for one timeout but does not attach. A stale read detaches; the first fresh, unpaused cycle rebases modules and synchronizes to raw feedback. Both in-range and out-of-limit synchronization track raw feedback throughout the settling window, including its attachment boundary. Unattached internal writes do not consume the correction: the first attached command is raw with zero velocity, then the profile progresses without fresh-sample resets. Attachment is a freshness flag, not a guarantee that correction has completed or that an external relay forwarded earlier commands. Writing is not gated; callers must enforce their forwarding lifecycle. With `feedback_timeout_s = 0` the engine is always attached.
- **Feedback groups.** With `feedback_groups` (for example an arm on joint encoders and a base on odometry), each group has its own stamp (`RobotState::group_stamps_ns`), staleness, window and attached state (`Engine::group_attached()`, `RobotCommand::group_attached`). A stale group detaches only itself: its sync touches only its joints and resynchronizes only the modules driving them. `attached()` and `RobotCommand::fresh` are true only while every group is attached; a driver can instead forward each group on its own flag.
- **Measured positions outside software limits.** Finite feedback remains unchanged. At startup, reattachment or pause/resume synchronization, an out-of-limit independent scalar joint seeds a continuous `JointTrajectory` from raw feedback toward its model-limit projection. The first engine command for that feedback group equals raw positions (with valid derived mimics), with zero initial commanded velocity; subsequent successful writes advance actual Ruckig using the guard-scaled velocity limit, default acceleration factor 2 and jerk factor 10. Mimic velocity limits also constrain the leader. No model limits are relaxed. Modules driving/locking corrected joints remain enabled with constraints active, but their preparation, claims and tasks are held; targets/FK/ramps rebase from current commanded state, including coupled held modules. Unrelated modules/groups keep running. Completion rebases affected targets to the legal posture before normal control resumes. In-range synchronization preserves normal target semantics; correction deliberately drops targets on affected modules. Stale feedback discards the profile; fresh resynchronization starts from the latest measurement. Pause/resume and rate changes are supported. Solve/guard/write failures do not advance the profile; a guard clamp that would distort it rejects the cycle.
- **Automatic behavior after synchronization.** Normal IK/gaze, CoM/collision recovery and null-space behavior resume after correction without requiring a fresh target/home, subject to existing arbitration. This is not a new motion-intent gate or a collision/CoM feasibility guarantee. Solver, closed-loop protection, endpoint checks and command expiry remain active; ordinary module takeover is not covered by the startup profile's acceleration/jerk limits.
- **External forwarding boundary.** A closed relay can let the core finish correction without moving the robot. Opening/reopening it must synchronize again using fresh feedback at the actual forwarding boundary; cold-start correction alone is insufficient. The existing generic control is `Engine::set_paused(true)` followed by `set_paused(false)` (debug protocol `pause`, with `paused: true` then `false`, when a debug server is enabled). Coordinate forwarding with the first successful resynchronized command; do not open a gate merely because `attached()` is true. No relay/guardian checks are removed, and closed-to-open relay integration is not verified here.
- **Command guard.** `CommandGuardOptions` checks commands against limits. In reject mode, a guard violation makes the cycle return an error and counts in `guard_rejects`. In clamp mode, the command is clamped, written and counted in `guard_clamps`. Feedback acceptance does not relax model or hardware limits.
- **Telemetry.** Consumers read value-type snapshots (`EngineSnapshot`) and never hold a `const Engine&`.

## Debug server

`kstone run --serve` starts the debug server, default `127.0.0.1:7311`. It streams telemetry and supports tuning and remote commands. Clients are equal and the last write wins. A motion port your own code writes is held for `api_hold_s` (default 1 s), and server `set_port` on that module's motion ports gets `api_owned`; see [debug-protocol.md](debug-protocol.md).

- Binding to a non-loopback address requires a token of at least 16 bytes, from `--token-file` or `$KSTONE_TOKEN`.
- The protocol is not encrypted and the token is sent in clear text. Across untrusted networks, use an SSH tunnel or a VPN.
- [kstone-viewer](https://github.com/AgilexOpensource/kstone_viewer) connects through this protocol. It is a separate smoke-test package that does not need kstone, so it can run on another machine. C++ and ROS programs can serve the same protocol with `DebugServer`.
- To write a client of your own, see [Debug protocol](debug-protocol.md).

### Remote access

To reach the server from another machine on a trusted network, create a token file once, readable only by you:

```bash
(umask 077 && python3 -c "import secrets; print(secrets.token_urlsafe(24))" > ~/.kstone-token)
```

Start the engine on every interface with that token:

```bash
kstone run <config.yaml> --realtime --serve 0.0.0.0:7311 --token-file ~/.kstone-token
```

Copy the file to the viewer machine and connect with the same token:

```bash
kstone-viewer --connect <robot-ip>:7311 --token-file ~/.kstone-token
```

Setting `KSTONE_TOKEN` to the same value on both sides works instead of `--token-file`:

```bash
export KSTONE_TOKEN="$(cat ~/.kstone-token)"
```

Across an untrusted network, keep the server on loopback, which needs no token, and forward the port over SSH:

```bash
kstone run <config.yaml> --realtime --serve 7311          # on the robot
ssh -N -L 7311:127.0.0.1:7311 user@<robot-ip>             # on the viewer machine
kstone-viewer --connect 127.0.0.1:7311
```

In your own program, set the token when starting the server and read it from the environment or a file rather than the source.

Python:

```python
import os
import kstone

engine = kstone.load_engine("robot.yaml")
server = engine.serve("0.0.0.0", 7311, token=os.environ["KSTONE_TOKEN"])
```

C++:

```cpp
kstone::DebugServerOptions options;
options.host = "0.0.0.0";
const char* token = std::getenv("KSTONE_TOKEN");
options.token = token != nullptr ? token : "";
auto server = kstone::DebugServer::start(engine, options);
if (!server) std::fprintf(stderr, "%s\n", server.error().message.c_str());
```

`start` fails with `invalid_argument` when a non-loopback host has no token or the token is outside 16..1024 bytes.
