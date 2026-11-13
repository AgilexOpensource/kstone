# 配置

[English](../config.md) · [README](../../README.zh-CN.md)

引擎配置是一个 YAML 文件。`kstone check <config.yaml>` 只构建引擎不运行，报出第一个错误及其行列号。未知键会报错。最小示例，取自 [arm_loopback.yaml](../../tests/fixtures/arm_loopback.yaml)：

```yaml
model:
  file: arm.urdf
rate_hz: 100
robot:
  type: loopback
modules:
  arm:
    type: joint_target
    joints: [j1, j2, j3]
```

<a id="top-level-keys"></a>
## 顶层键

| 键 | 默认值 | 含义 |
|---|---|---|
| `model` | 必填 | 机器人模型，见 [model](#model) |
| `rate_hz` | 100 | 控制频率 |
| `realtime` | false | 在 `run()` 中应用 `thread` 设置（实时调度）；`kstone run --realtime` 强制开启。`run()` 总是按时钟节拍运行 |
| `feedback_timeout_s` | 0.25 | 反馈超过该时长或没有时间戳即视为过期；也是反馈转为新鲜后的同步窗口；0 = 永不过期且立即同步。见[运行契约](robot.md#run-contract) |
| `max_failed_cycles` | 5 | 连续失败这么多个周期后 `run()` 返回 |
| `feedback_joints` | 全部标量关节 | 实测位置参与闭环估计（碰撞、质心、回零）的关节。关节名或 fnmatch 模式 |
| `feedback_groups` | 无 | 各自有反馈时间戳、新鲜度和同步状态的关节组，例如 `[{name: base, joints: [root_joint]}]`。关节名或 fnmatch 模式；未分组的关节组成最后一组 `default`。见[运行契约](robot.md#run-contract) |
| `safety` | | 命令守卫，见 [safety](#safety) |
| `solver` | | QP 求解器，见 [solver](#solver) |
| `robot` | loopback | 机器人接口，见 [robot](#robot) |
| `thread` | | 实时线程设置，见[接入机器人](robot.md#real-time-thread) |
| `modules` | 无 | 模块 id 到 `{type, ...}` 的映射，见[模块](modules/README.md) |

## model

| 键 | 默认值 | 含义 |
|---|---|---|
| `file` | 必填 | 模型路径或 URI（URDF 或 MJCF） |
| `format` | 按扩展名 | `urdf` 或 `mjcf` |
| `package_dirs` | | `package://<name>/<path>` 的查找根目录，按顺序尝试 |
| `root` | `fixed` | `free_flyer` 加一个浮动基座，其 q 为 x, y, z, qx, qy, qz, qw |
| `root_joint_name` | `root_joint` | 浮动基座关节名 |
| `mimic` | `independent` | `independent` 把 mimic 从动关节当普通自由度；`coupled` 由主动关节带动 |
| `joint_limits` | | 按关节覆盖限位，见下文 |
| `locked_joints` | | 保持在初始值的关节，见下文 |

`joint_limits` 替换模型里对应关节的限位，所以也可以放宽：

```yaml
model:
  file: robot.urdf
  joint_limits:
    velocity: {gripper_joint: 1.0}
    position:
      lower: {elbow_joint: 0.0873}
      upper: {elbow_joint: 2.4}
```

`locked_joints` 列出求解器永远不动的关节。它们保持 `robot.initial` 的值（没给就用零位），仍留在模型里，也仍在状态和命令里，所以 `joint_states` 依旧完整。列了锁定关节的模块继续用其余关节工作。浮动基座和 coupled 模式下的 mimic 跟随关节不能列；前者用 `root: fixed`，后者锁它的主关节：

```yaml
model:
  file: robot.urdf
  locked_joints: [left_wheel_joint, right_wheel_joint]
```

相对的 `file` 路径先按配置文件所在目录解析，相对的 `package_dirs` 条目也按该目录解析。

## safety

命令守卫在每次写入机器人之前按关节限位检查命令。

| 键 | 默认值 | 含义 |
|---|---|---|
| `mode` | `reject` | `reject` 使该周期失败，`clamp` 钳位后写出，`off` 关闭守卫 |
| `velocity_scale` | 1 | 缩放守卫检查用的速度限位。模块仍按模型限位规划，要让机器人变慢应调低 `model.joint_limits.velocity` |
| `tolerance` | 1e-6 | 超出限位多少才算越界 |

## solver

| 键 | 默认值 | 含义 |
|---|---|---|
| `mode` | `weighted` | [任务层级](modules/README.md#task-levels)如何变成 QP 阶段，见下表 |
| `damping` | 1e-3 | 加到 QP Hessian 对角线上，> 0 |
| `limit_gain` | 0.5 | 每步可用的到位置限位距离的比例，取值 (0, 1] |
| `feasibility_tolerance` | 1e-6 | 可行性检查的容差 |
| `check_feasibility` | true | 按约束检查每个解 |
| `hold_retry` | true | 阶段不可行时放宽恢复要求重试，仍失败则保持不动 |

| 模式 | 行为 |
|---|---|
| `weighted` | 单个 QP，忽略层级，所有任务加权求和 |
| `lexicographic` | 严格优先级，低层级不会干扰高层级。每周期开销最大 |
| `layered_independent` | 每个层级一个 QP，只移动本层任务；后面的层级补偿前面层级带来的偏移 |
| `layered_coupled` | 每个层级一个 QP，包含该层级及之前的所有任务；冲突时折中，不是严格优先级 |

所有 solver 键都能在运行时用 `Engine.tune_solver` 修改。

## robot

| 键 | 默认值 | 含义 |
|---|---|---|
| `type` | `loopback` | `loopback` 是回报上一条命令的理想机器人；`plugin` 加载[驱动插件](robot.md#driver-plugins) |
| `library` | | 插件动态库。含 `/` 的路径按配置文件目录解析，裸名走动态库搜索路径 |
| `params` | | 任意映射，原样传给插件的 `create()` |
| `initial` | 模型零位 | 初始 q：长度为 nq 的列表，或在零位基础上覆盖的映射 `{关节: 值}`。也是各模块的初始姿态 |
| `initial_root` | | 浮动基座初始位姿 `{position, rpy}` 或 `{position, quaternion}`。不能与列表形式的 `initial` 同用 |

## modules

`modules:` 把 id 映射到模块。`type` 选择模块类型，其余字段归该类型解析。这一节可以省略。各类型见[模块](modules/README.md)，完整 JSON Schema 用 `kstone schema <type>` 查看。

<a id="variables-and-includes"></a>
## 变量与 include

磁盘上的配置文件支持占位符和 include：

```yaml
rate_hz: ${rate:-50}          # 50 unless rate is set
modules: !include modules.yaml  # path relative to this file
```

- `${name}` 先查传入的变量，再查环境变量；`${name:-default}` 在两者都未设置或为空时取默认值。`$${` 表示字面的 `${`。
- 整个标量只有一个占位符时会重新推断类型，所以 `${rate:-50}` 是数字。
- 用 `kstone run/check --set name=value`（可重复）传变量，Python 加载函数用 `vars={"name": "value"}`。传入的每个变量都必须在配置里被引用，否则加载失败，报 KST-E010。
- Python 的 `load_*_from_string` 也展开占位符；`base_dir` 指定相对路径和 include 的解析目录。

<a id="naming-conventions"></a>
## 命名约定

各模块的键遵循以下规则：

- 时长以 `_s` 结尾（秒）；其他单位见各模块文档。
- `max_` 表示上限。逐轴上限组内的字段不加 `max_`：`model.joint_limits`（`velocity`、`position.lower` / `upper`）和 frame_target 的 `trajectory.position` / `trajectory.orientation`（`velocity`、`acceleration` 等）。chassis 的 `pose_limits`（仅可调参）保留 `max_`，因为其字段与它所取初值的 chassis `max_*` 字段同名。
- `_scale` 是无量纲比例；`_factor` 是带单位的系数（如 `acceleration_factor`，1/s）。
- `speed` 是标量大小；`velocity` 是向量、逐轴速率或其限制。
- 位姿量用 `position` / `orientation`；速率和限制用 `linear` / `angular`。
- 位姿值在端口上是 `{position, orientation}`（wxyz 四元数），在配置里是 `{position, quaternion}` 或 `{position, rpy}`。
- `velocity_limit` 是 joint_target 规划时用的逐关节速度限制（求解器仍按模型限制约束）；`max_velocity` 是对单个模块自身运动的额外上限，如轨迹的巡航速度。
