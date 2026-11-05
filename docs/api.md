# C++ API

[中文](zh-CN/api.md) · [README](../README.md)

Public headers fall into three layers by audience. Paths are relative to `include/kstone/` and are included as `#include "kstone/<path>"`.

Runnable programs are in [examples/cpp](../examples/cpp).

## Application layer: configure, run, connect a robot

| Header | Contents |
|---|---|
| [runtime/config.hpp](../include/kstone/runtime/config.hpp) | `load_engine` / `build_engine`: build an engine from a config |
| [runtime/engine.hpp](../include/kstone/runtime/engine.hpp) | `Engine`: step / run, state, last solve result, module access, statistics |
| [runtime/snapshot.hpp](../include/kstone/runtime/snapshot.hpp) | `EngineSnapshot`, `TelemetrySink`, `make_telemetry_observer`: read-only telemetry |
| [runtime/robot.hpp](../include/kstone/runtime/robot.hpp) | `RobotInterface` for C++ drivers or simulators; `LoopbackRobot` |
| [runtime/robot_plugin.hpp](../include/kstone/runtime/robot_plugin.hpp) | `load_robot_plugin`: load a C ABI driver library |
| [runtime/debug_server.hpp](../include/kstone/runtime/debug_server.hpp) | Debug server: telemetry, tuning, control lease |
| [runtime/doctor.hpp](../include/kstone/runtime/doctor.hpp) | Environment check |
| [module/module.hpp](../include/kstone/module/module.hpp) | `Module`: ports, enable/disable, tune; `ModuleSet`: offline solving without a robot. Built-in types are in [modules](modules/README.md) |
| [module/config.hpp](../include/kstone/module/config.hpp), [module/yaml.hpp](../include/kstone/module/yaml.hpp) | `ConfigNode`; YAML loading, `!include`, `${var}` expansion |
| [model/model.hpp](../include/kstone/model/model.hpp), [model/source.hpp](../include/kstone/model/source.hpp) | Model loading, joint and frame information, name-to-index lookup |
| [model/workspace.hpp](../include/kstone/model/workspace.hpp) | Forward kinematics, frame and joint poses, Jacobians, center of mass (one per thread) |
| [collision/world.hpp](../include/kstone/collision/world.hpp), [collision/geometry.hpp](../include/kstone/collision/geometry.hpp) | Add, remove and move obstacles at run time |
| [core/error.hpp](../include/kstone/core/error.hpp) | `Result` / `Status` / `ErrorCode` |
| [core/context.hpp](../include/kstone/core/context.hpp), [core/clock.hpp](../include/kstone/core/clock.hpp), [core/log.hpp](../include/kstone/core/log.hpp), [core/resources.hpp](../include/kstone/core/resources.hpp) | Injected clock, log sink, resource resolution such as `package://` |

## Extension layer: custom modules, tasks, model formats, drivers

| Header | Contents |
|---|---|
| [module/registry.hpp](../include/kstone/module/registry.hpp) | Register custom module types |
| [module/schema.hpp](../include/kstone/module/schema.hpp) | Schema of module config and tunables |
| [module/homing.hpp](../include/kstone/module/homing.hpp) | Homing state machine |
| [module/builtins.hpp](../include/kstone/module/builtins.hpp) | Built-in modules, to construct or compose in code |
| [control/task.hpp](../include/kstone/control/task.hpp), [control/tasks.hpp](../include/kstone/control/tasks.hpp) | Task base class and built-in tasks |
| [control/constraint.hpp](../include/kstone/control/constraint.hpp), [control/constraints.hpp](../include/kstone/control/constraints.hpp) | Constraint base class and built-in constraints |
| [control/solver.hpp](../include/kstone/control/solver.hpp) | `Solver`, `SolverOptions`, `SolveResult` |
| [core/mask.hpp](../include/kstone/core/mask.hpp) | Velocity dimension selection, used by tasks and modules |
| [motion/joint_trajectory.hpp](../include/kstone/motion/joint_trajectory.hpp), [motion/pose_trajectory.hpp](../include/kstone/motion/pose_trajectory.hpp) | Joint and pose trajectory shaping, usable on their own |
| [chassis/chassis.hpp](../include/kstone/chassis/chassis.hpp) | Omnidirectional and differential base pose tracking, usable on its own |
| [collision/model.hpp](../include/kstone/collision/model.hpp), [collision/query.hpp](../include/kstone/collision/query.hpp), [collision/constraint.hpp](../include/kstone/collision/constraint.hpp) | Robot collision geometry, distance queries, collision constraints |
| [model/loader.hpp](../include/kstone/model/loader.hpp), [model/pinocchio.hpp](../include/kstone/model/pinocchio.hpp) | New model formats; direct Pinocchio access (pulls in Pinocchio headers) |
| [runtime/robot_plugin.h](../include/kstone/runtime/robot_plugin.h) | C ABI for driver plugins, see [Robot integration](robot.md#driver-plugins) |
| [runtime/engine_c.h](../include/kstone/runtime/engine_c.h) | C ABI of the engine: load/create, step/run, snapshots, ports, for FFI hosts in other languages |

## Implementation layer: rarely used directly

| Header | Contents |
|---|---|
| [qp/problem.hpp](../include/kstone/qp/problem.hpp) | QP problem structure; `SolveStatus` appears in `SolveResult::stages` |
| [qp/backend.hpp](../include/kstone/qp/backend.hpp), [qp/daqp.hpp](../include/kstone/qp/daqp.hpp) | QP backend interface, DAQP implementation, hierarchical solve; `DaqpSettings` appears in `SolverOptions` |
| [core/histogram.hpp](../include/kstone/core/histogram.hpp) | Cycle latency histogram used by `Engine` statistics |

## Callback interfaces

These are implemented by users and called by kstone. Callbacks on the control thread must not allocate, lock or block.

| Interface | When it is called | Notes |
|---|---|---|
| `RobotInterface`, driver plugin C ABI | Every cycle: read state, write command | On failure no command is written; the driver owns timeouts and the safe state |
| Modules registered with `ModuleRegistry` | Every cycle; on tune | Compile-time registration only |
| `ModelLoader` | Model loading | Compile-time registration only, through `Context::loaders` |
| `ResourceResolver` scheme handler | Resource URI resolution | `package://` works with `model.package_dirs` in the config |
| `EngineObserver` | Before and after every cycle, on the control thread | Prefer `TelemetrySink` for read-only telemetry |
| `TelemetrySink` | Throttled to `publish_hz`, on the control thread | Cross-thread hand-off needs the implementation's own lock-free queue |
| `qp::BackendFactory` | Solver construction | Compile-time injection only |

The run contract (failure handling, stale feedback, command guard, real-time thread) is in [Robot integration](robot.md#run-contract).

## Versioning and compatibility

Structures that cross library or language boundaries carry `struct_size` and only grow at the end. Readers of the protocol and snapshots ignore unknown fields.

| Boundary | Version | Rule |
|---|---|---|
| Driver plugin C ABI | `KSTONE_ROBOT_ABI_VERSION` = 1 | Bumped only on incompatible changes; new fields rely on `struct_size` |
| Engine C ABI | `KSTONE_ENGINE_ABI_VERSION` = 1 | Same |
| Debug protocol | `hello.version` = 1 | New fields are optional; new ops must be listed in `capabilities` |
| Snapshot | `EngineSnapshot::schema_version` = 1 | Fields are only appended; bumped on removal or change of meaning |

The library version and the CMake package compatibility are described in [C++](cpp.md#using-kstone-from-your-cmake-project). The Python binding coverage is in [Python](python.md).
