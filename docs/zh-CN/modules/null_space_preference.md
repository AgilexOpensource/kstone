# null_space_preference

[English](../../modules/null_space_preference.md) · [模块总览](README.md)

在一个 frame_task（`primary`）的零空间里施加偏好，不影响该任务的末端跟踪。运行时可切换偏好，不分配内存。

## 关键参数

- `primary`：frame_task 的 id，同时是本模块的依赖。`joints` 为空时取 primary 驱动的标量关节。
- `preference`：
  - `posture`：靠近参考姿态。
  - `manipulability`：提高可操作度，`manipulability_axes` 选参与的方向（默认只看平移）。
  - `joint_limits`：远离关节限位，在距限位 `joint_limits_band` 内起作用；所有关节都无界时此偏好不可用。
- `reference`（posture 的基准）：
  - `initial`（默认）：初始 q。
  - `current`：每周期的当前 q，相当于只阻尼零空间运动。
  - `neutral`：关节范围中点，夹在有效限位内（模型限位 ∩ yaml 收窄 ∩ 运行时 `tune_joint_limits`）。
  - `custom`：`posture` 给出的姿态，空 = 初始 q。
- `posture_values`：`{关节名: 值}`，覆盖在参考姿态之上。`current` 不接受 `posture` 和 `posture_values`。
- 运行时切换：
  - 只把 `reference` 调成 `custom` 时，取配置的 `posture`；没配置就取当前 q；再叠加 `posture_values`。
  - 切到 `initial` / `neutral` / `current` 会清空 `posture_values`。
  - `reseed_current: true`（仅 tune）把当前 q 设为 custom 姿态并选中 posture 偏好。
- `posture_weight`、`manipulability_weight`、`joint_limits_weight`：各偏好的权重。`posture_velocity_scale` 限制回到姿态的速度，取值 (0, 1]。
- `projector.damping` / `threshold`：零空间投影的阻尼，threshold > 0 启用自适应阻尼。

## 示例

```yaml
left_null:
  type: null_space_preference
  primary: left
  reference: initial
  posture_weight: 0.05
  manipulability_weight: 0.05
```

```python
null = e.module("left_null")
null.tune({"reference": "custom", "posture_values": {"left_arm_joint_3": -1.0}})
null.tune({"reseed_current": True})  # 把当前姿态记为偏好
null.tune({"preference": "joint_limits"})
```

## 常见坑

- 只在没有其他模块占有其关节时运行。示例里 `left_arm`（joint_target，`claim_idle: true`）占着左臂，`left_null` 一直 held。
- 偏好在零空间里，不会牺牲 primary 的精度；但 primary 有轴被放开（权重 0）时，这些轴也属于零空间，偏好会推动它们。
- 参考姿态与目标位姿相差很远时，偏好只能在冗余自由度里尽量靠近，关节不会到达参考值。
