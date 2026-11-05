# C++ API

[English](../api.md) · [README](../../README.zh-CN.md)

公开头文件按使用者分三层。路径相对 `include/kstone/`，统一通过 `#include "kstone/<路径>"` 引入。

可运行的程序在 [examples/cpp](../../examples/cpp)。

## 应用层：配置、运行、接机器人

| 头文件 | 内容 |
|---|---|
| [runtime/config.hpp](../../include/kstone/runtime/config.hpp) | `load_engine` / `build_engine`：从配置构建引擎 |
| [runtime/engine.hpp](../../include/kstone/runtime/engine.hpp) | `Engine`：step / run、状态、最近求解结果、模块访问、统计 |
| [runtime/snapshot.hpp](../../include/kstone/runtime/snapshot.hpp) | `EngineSnapshot`、`TelemetrySink`、`make_telemetry_observer`：只读遥测 |
| [runtime/robot.hpp](../../include/kstone/runtime/robot.hpp) | `RobotInterface`：C++ 驱动或仿真器；`LoopbackRobot` |
| [runtime/robot_plugin.hpp](../../include/kstone/runtime/robot_plugin.hpp) | `load_robot_plugin`：加载 C ABI 驱动库 |
| [runtime/debug_server.hpp](../../include/kstone/runtime/debug_server.hpp) | 调试服务（遥测、调参、控制租约） |
| [runtime/doctor.hpp](../../include/kstone/runtime/doctor.hpp) | 环境自检 |
| [module/module.hpp](../../include/kstone/module/module.hpp) | `Module`：端口读写、启停、tune；`ModuleSet`：不带机器人的离线求解；内置模块见 [modules](modules/README.md) |
| [module/config.hpp](../../include/kstone/module/config.hpp)、[module/yaml.hpp](../../include/kstone/module/yaml.hpp) | `ConfigNode`；YAML 加载、`!include`、`${var}` 展开 |
| [model/model.hpp](../../include/kstone/model/model.hpp)、[model/source.hpp](../../include/kstone/model/source.hpp) | 模型加载、关节与 frame 信息、名称到索引 |
| [model/workspace.hpp](../../include/kstone/model/workspace.hpp) | 正运动学、frame / 关节位姿、雅可比、质心（每线程一份） |
| [collision/world.hpp](../../include/kstone/collision/world.hpp)、[collision/geometry.hpp](../../include/kstone/collision/geometry.hpp) | 运行时增删、移动障碍物 |
| [core/error.hpp](../../include/kstone/core/error.hpp) | `Result` / `Status` / `ErrorCode` |
| [core/context.hpp](../../include/kstone/core/context.hpp)、[core/clock.hpp](../../include/kstone/core/clock.hpp)、[core/log.hpp](../../include/kstone/core/log.hpp)、[core/resources.hpp](../../include/kstone/core/resources.hpp) | 注入时钟、日志 sink、`package://` 等资源解析 |

## 扩展层：自定义模块、任务、模型格式、驱动

| 头文件 | 内容 |
|---|---|
| [module/registry.hpp](../../include/kstone/module/registry.hpp) | 注册自定义模块类型 |
| [module/schema.hpp](../../include/kstone/module/schema.hpp) | 模块配置与 tune 的 schema |
| [module/homing.hpp](../../include/kstone/module/homing.hpp) | 回零状态机 |
| [module/builtins.hpp](../../include/kstone/module/builtins.hpp) | 内置模块，可在代码里直接构造或组合 |
| [control/task.hpp](../../include/kstone/control/task.hpp)、[control/tasks.hpp](../../include/kstone/control/tasks.hpp) | 任务基类与内置任务 |
| [control/constraint.hpp](../../include/kstone/control/constraint.hpp)、[control/constraints.hpp](../../include/kstone/control/constraints.hpp) | 约束基类与内置约束 |
| [control/solver.hpp](../../include/kstone/control/solver.hpp) | `Solver`、`SolverOptions`、`SolveResult` |
| [core/mask.hpp](../../include/kstone/core/mask.hpp) | 速度维度选择，任务与模块接口使用 |
| [motion/joint_trajectory.hpp](../../include/kstone/motion/joint_trajectory.hpp)、[motion/pose_trajectory.hpp](../../include/kstone/motion/pose_trajectory.hpp) | 关节 / 位姿轨迹整形，可单独使用 |
| [chassis/chassis.hpp](../../include/kstone/chassis/chassis.hpp) | 全向 / 差速底盘位姿跟踪，可单独使用 |
| [collision/model.hpp](../../include/kstone/collision/model.hpp)、[collision/query.hpp](../../include/kstone/collision/query.hpp)、[collision/constraint.hpp](../../include/kstone/collision/constraint.hpp) | 机器人碰撞几何、距离查询、碰撞约束 |
| [model/loader.hpp](../../include/kstone/model/loader.hpp)、[model/pinocchio.hpp](../../include/kstone/model/pinocchio.hpp) | 新模型格式；直接访问 Pinocchio（引入后依赖 Pinocchio 头文件） |
| [runtime/robot_plugin.h](../../include/kstone/runtime/robot_plugin.h) | 驱动插件的 C ABI，见[接入机器人](robot.md#driver-plugins) |
| [runtime/engine_c.h](../../include/kstone/runtime/engine_c.h) | 引擎的 C ABI：load/create、step/run、快照、端口读写，供其他语言宿主经 FFI 调用 |

## 实现层：一般无需直接使用

| 头文件 | 内容 |
|---|---|
| [qp/problem.hpp](../../include/kstone/qp/problem.hpp) | QP 问题结构；`SolveStatus` 出现在 `SolveResult::stages` 中 |
| [qp/backend.hpp](../../include/kstone/qp/backend.hpp)、[qp/daqp.hpp](../../include/kstone/qp/daqp.hpp) | QP 后端接口与 DAQP 实现、分级求解；`DaqpSettings` 出现在 `SolverOptions` 中 |
| [core/histogram.hpp](../../include/kstone/core/histogram.hpp) | 周期延迟直方图，`Engine` 统计使用 |

## 回调接口

以下接口由用户实现、交给 kstone 调用。控制线程上的回调不得分配、加锁或阻塞。

| 接口 | 调用时机 | 限制 |
|---|---|---|
| `RobotInterface`、驱动插件 C ABI | 每周期读状态、写命令 | 失败时引擎不写命令，驱动自建超时与安全态 |
| `ModuleRegistry` 注册的模块 | 每周期更新，tune 时改参数 | 仅编译期注册 |
| `ModelLoader` | 加载模型 | 仅编译期注册，经 `Context::loaders` |
| `ResourceResolver` scheme handler | 解析资源 URI | `package://` 可直接用配置的 `model.package_dirs` |
| `EngineObserver` | 每周期前后，控制线程 | 只读遥测优先用 `TelemetrySink` |
| `TelemetrySink` | 按 `publish_hz` 节流，控制线程 | 跨线程由实现自备无锁队列 |
| `qp::BackendFactory` | 构建求解器 | 仅编译期注入 |

运行契约（失败处理、反馈过期、命令守卫、实时线程）见[接入机器人](robot.md#run-contract)。

<a id="versioning-and-compatibility"></a>
## 版本与兼容

跨库、跨语言的结构带 `struct_size`，只在尾部追加字段；协议和快照的读取方忽略未知字段。

| 边界 | 版本标识 | 兼容规则 |
|---|---|---|
| 驱动插件 C ABI | `KSTONE_ROBOT_ABI_VERSION` = 1 | 不兼容改动才递增；新增字段靠 `struct_size` |
| Engine C ABI | `KSTONE_ENGINE_ABI_VERSION` = 1 | 同上 |
| 调试协议 | `hello.version` = 1 | 新增字段可选；新增 op 必须出现在 `capabilities` |
| 快照结构 | `EngineSnapshot::schema_version` = 1 | 只追加字段；删除或改义才递增 |

库版本号与 CMake 包的兼容规则见 [C++](cpp.md#using-kstone-from-your-cmake-project)，Python 绑定范围见 [Python](python.md)。
