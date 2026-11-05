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

A driver is a shared library implementing the C ABI in [robot_plugin.h](../include/kstone/runtime/robot_plugin.h). Start from [example_robot.c](../examples/robot_plugin/example_robot.c). `read` must return a CLOCK_MONOTONIC timestamp, otherwise the feedback counts as stale.

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

On Linux the control thread can use real-time scheduling. The settings apply when `run()` starts, and priority and affinity are restored when it returns.

```yaml
thread:
  priority: 50       # SCHED_FIFO 1..99, higher runs first, 0 = unchanged; needs CAP_SYS_NICE or ulimit -r >= value
  cpus: [3]          # CPUs to pin to, empty = unchanged; works best with isolcpus
  lock_memory: true  # mlockall for the whole process, never undone; needs CAP_IPC_LOCK or a large enough ulimit -l
```

- If the system refuses a setting (missing CAP_SYS_NICE, RLIMIT_RTPRIO or CAP_IPC_LOCK), the engine logs a warning and keeps running, as ros2_control_node does. Invalid values still fail at creation.
- `lock_memory` calls `mlockall(MCL_CURRENT | MCL_FUTURE)`.
- Calling `step()` directly is not affected.

## Run contract

- **Failed cycle.** If reading, solving or writing fails, the engine writes no command and does not synthesize a stop command. The cycle counts in `failed_cycles`. `run()` keeps going and returns the last error after `max_failed_cycles` (default 5) consecutive failures. Drivers must implement their own command timeout and safe state.
- **Stale feedback.** Feedback older than `feedback_timeout_s` is not a failure. The command is still written, with `RobotCommand::fresh = false`, and the driver or caller decides whether to send it.
- **Command guard.** `CommandGuardOptions` checks commands against limits. In reject mode, an out-of-range command makes the cycle return an error and counts in `guard_rejects`. In clamp mode, the command is clamped, written and counted in `guard_clamps`.
- **Telemetry.** Consumers read value-type snapshots (`EngineSnapshot`) and never hold a `const Engine&`.

## Debug server

`kstone run --serve` starts the debug server, default `127.0.0.1:7311`. It streams telemetry and supports tuning parameters and taking a control lease.

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
