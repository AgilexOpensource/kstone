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
- **Attach.** `Engine::attached()` is true once feedback has stayed fresh for `feedback_timeout_s` and the commanded state has been synced to its model-limit projection on every cycle of that window. A stale read detaches the engine. The first fresh, unpaused cycle after that rebases the modules (held targets are dropped, so each module resets from the clipped robot state). From then on every fresh, unpaused cycle syncs the commanded state to the clipped latest measurement before solving; targets set during the window are kept. The cycle that completes the window attaches and counts in `attaches`, so attaching takes `feedback_timeout_s` of wall time and every command flagged fresh starts from a clipped current measurement. An engine created with stale or unstamped feedback starts detached; one created with fresh feedback still waits out the window. Writing is not gated: callers that drive hardware should forward commands only while `attached()`, and set targets after it turns true. With `feedback_timeout_s = 0` the engine is always attached.
- **Feedback groups.** With `feedback_groups` (for example an arm on joint encoders and a base on odometry), each group has its own stamp (`RobotState::group_stamps_ns`), staleness, window and attached state (`Engine::group_attached()`, `RobotCommand::group_attached`). A stale group detaches only itself: its sync touches only its joints and resynchronizes only the modules driving them. `attached()` and `RobotCommand::fresh` are true only while every group is attached; a driver can instead forward each group on its own flag.
- **Measured positions outside software limits.** Finite feedback remains unchanged. At startup, attachment, reattachment and pause/resume synchronization, commanded/ideal scalar positions are clipped to the model lower/upper limits; mimic followers retain the existing model derivation. Joint targets and frame FK/ramp resets use this clipped commanded state, not raw feedback. Synchronization refreshes FK before prepare, including frames held by coupled idle joint claims; rebases clear their old targets even while held. Reattachment drops detached targets; ordinary pause/resume retains still-valid targets and resets their trajectories from the clipped state. This follows the legacy `ik_teleop_ros` clipped initialization, not a no-target hold policy.
- **Automatic behavior after synchronization.** Normal IK runs immediately without a fresh target/home: gaze may aim or return home, CoM/collision may demand recovery (including collision cutting planes), and null-space preferences follow their existing primary-module rules. Closed-loop checks, endpoint protection, position/velocity limits and command leases are unchanged; target expiry does not introduce a new motion-intent lifecycle. The first position command can differ abruptly from raw feedback by the clipping correction, and automatic modules can move it further in the same solve. Clipping is not a slew-limited recovery or a guarantee that a posture is collision/CoM feasible; existing solver/guard failures still apply. Normal module priorities and idle claims still arbitrate control; enabled automatic modules need not move every posture.
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
