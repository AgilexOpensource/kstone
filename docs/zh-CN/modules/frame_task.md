# frame_task

[English](../../modules/frame_task.md) · [模块总览](README.md)

让 `frame` 跟踪位姿目标。目标先经过加加速度受限的轨迹整形再交给求解器；没有目标时保持 reset 时的位姿。

## 端口

| 名称 | 类型 | 说明 |
|---|---|---|
| `target` | pose | 目标位姿，在 `reference` 坐标系下 |

## 关键参数

- `frame`：被控 frame。`joints` 为空时驱动从根到该 frame 的全部关节；非空时只驱动列出的关节。
- `reference`：目标坐标系，空 = world。底盘移动时，以 base_link 为参考的目标会随底盘走。
- `position_weight` / `orientation_weight`：每轴权重（x, y, z），再乘 `weight`。某轴设 0 即放开该轴。轴向默认在 frame 坐标系，`world_axes: true` 改为世界坐标系。
- `lm_damping`：Levenberg-Marquardt 阻尼，接近奇异时更稳，代价是收敛变慢。
- `trajectory`：必填，`{}` 合法。`translation` / `rotation` 各有 velocity、acceleration、deceleration、reversal_deceleration、jerk；某轴 velocity 为 0 表示不整形。`sync` 选 `axis_phase` 或 `axis_time`。`lead_speed_scale` 限制整形参考领先实际 frame 的距离（`lead_speed_scale × translation.velocity × 0.02 s`），frame 被挡住时参考不会继续跑远，0 = 不限。
- `home`：回零位姿，在 reference 坐标系下；缺省取初始姿态下的位姿。
- `on_held`：默认 `abort`。
- `locked`：冻结驱动关节，要求显式给出 `joints`。解锁后保持当前位姿，丢弃锁定前和锁定期间写入的目标。回零会覆盖锁定。

## 示例

```yaml
left:
  type: frame_task
  frame: left_tool_frame
  joints: [left_arm_joint_1, left_arm_joint_2, left_arm_joint_3, left_arm_joint_4, left_arm_joint_5, left_arm_joint_6, left_arm_joint_7]
  reference: base_link
  weight: 5
  orientation_weight: [0.2, 0.2, 0.2]
  lm_damping: 1.5
  task: {level: 1}
  trajectory:
    translation: {velocity: 2.0, acceleration: 20, deceleration: 20, reversal_deceleration: 40, jerk: 200}
    rotation: {velocity: 5.0, acceleration: 10, deceleration: 10, reversal_deceleration: 20, jerk: 100}
```

```python
left = e.module("left")
left.write_port("target", {"translation": [0.4, 0.2, 0.3], "rotation": [1, 0, 0, 0]})
left.tune({"orientation_weight": [0, 0, 0]})  # 只控位置
```

## 输出

`planned`（整形后的参考点）、`lag`（目标到参考的距离）、`speed`、`angular_speed`、`error`、`angular_error`（目标到实际 frame）。

## 常见坑

- `trajectory` 不能省略；只想直接跟目标就写 `trajectory: {}`。
- 与 joint_target 共用关节时，joint_target 默认 `task.level: 0`，优先级更高；关节被它占有期间本任务 held，默认 abort 会丢掉目标。
- 自由度不足时（如 6 维目标配少于 6 个关节）用每轴权重放开不需要的轴，否则误差会在各轴间折中。
