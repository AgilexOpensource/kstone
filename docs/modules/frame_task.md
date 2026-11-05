# frame_task

[中文](../zh-CN/modules/frame_task.md) · [Module overview](README.md)

Makes `frame` track a pose target. The target passes through jerk-limited trajectory shaping before reaching the solver. Without a target, the frame holds its pose at reset.

## Ports

| Name | Type | Description |
|---|---|---|
| `target` | pose | Target pose in the `reference` frame |

## Key parameters

- `frame`: the controlled frame. With `joints` empty, every joint from the root to the frame is driven; otherwise only the listed joints.
- `reference`: target frame, empty = world. When the base moves, a target referenced to base_link moves with it.
- `position_weight` / `orientation_weight`: per-axis weights (x, y, z), multiplied by `weight`. 0 frees that axis. Axes are in the frame by default; `world_axes: true` uses world axes.
- `lm_damping`: Levenberg-Marquardt damping. More is steadier near singularities and converges more slowly.
- `trajectory`: required, `{}` is valid. `translation` / `rotation` each have velocity, acceleration, deceleration, reversal_deceleration and jerk; a velocity of 0 disables shaping on that part. `sync` is `axis_phase` or `axis_time`. `lead_speed_scale` caps how far the shaped reference may run ahead of the actual frame (`lead_speed_scale × translation.velocity × 0.02 s`), so a blocked frame does not leave the reference far ahead. 0 = no cap.
- `home`: homing pose in the reference frame; defaults to the pose at the initial posture.
- `on_held`: defaults to `abort`.
- `locked`: freezes the driven joints and requires explicit `joints`. On unlock the frame holds its current pose, and targets written before or during the lock are dropped. Homing overrides the lock.

## Example

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
left.tune({"orientation_weight": [0, 0, 0]})  # position only
```

## Outputs

`planned` (shaped reference), `lag` (target to reference distance), `speed`, `angular_speed`, `error`, `angular_error` (target to actual frame).

## Pitfalls

- `trajectory` cannot be omitted. To follow the target directly, write `trajectory: {}`.
- When sharing joints with a joint_target, the joint_target defaults to `task.level: 0` and wins. While it owns the joints this task is held, and the default `abort` drops the target.
- With too few degrees of freedom (for example a 6-D target on fewer than 6 joints), free the axes you don't need with per-axis weights. Otherwise the error is traded off across all axes.
