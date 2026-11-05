# chassis

[English](../../modules/chassis.md) · [模块总览](README.md)

平面底盘控制器，在 QP 之外运行。它锁住 free-flyer 根节点，在 prepare() 中把底盘位姿写进 q，z = 0 为地面。支持速度指令和位姿目标两种模式，带指令租约、优先级仲裁、Ruckig 轨迹整形、航位推算和里程计覆盖。

## 端口

| 端口 | 类型 | 说明 |
|---|---|---|
| `velocity` | vector `[vx, vy, wz]` | 车体系速度 |
| `pose_goal` | vector `[x, y, yaw]` | 世界系位姿目标 |

两个端口都在 `command_timeout_s` 后失效。最后写入的那个在下一次 step 以来源 `port` 提交。

## 关键参数

配置放在 `chassis:` 子对象里：

- `kinematics`：`omni` 或 `differential`。
- `base_frame`：控制中心所在 frame，默认 `base_link`。
- `max_linear`、`max_angular` 及对应的 `*_acceleration`、`*_deceleration`、`*_reversal_deceleration`、`*_jerk`：上限。速度模式直接用这些值，位姿模式的初始限制也取它们。
- `arrival_position_tolerance`、`arrival_rotation_tolerance`：到达判定，默认 0.01 m、0.02 rad。
- `target_translation_deadband`、`target_rotation_deadband`：差速底盘下，目标变化小于这个量时沿用当前路径。
- `command_timeout_s`（默认 0.25）：指令超过这么久没更新就清除运动，reason 为 `command_timeout`，不算 fault。
- `odom_timeout_s`（默认 0.25）：比这更旧的里程计被拒收；两次里程计之间用输出速度推算位姿。
- `max_dt_s`（0.1）、`allowed_missed_cycles`（2）：两次 step 间隔超过 `max(max_dt_s, 周期 × allowed_missed_cycles)` 时进入 fault（`scheduler_discontinuity`）。模块内只有跳过一步（禁用后再启用）才会重新同步并恢复。

顶层：`center_in_root`（控制中心在根节点系中的固定位姿）、`period`（初始周期，之后每周期取引擎 dt）。

### pose_limits（只能 tune）

位姿模式的运行时限制：`linear_`/`angular_` 加 `velocity`、`acceleration`、`deceleration`、`reversal_deceleration`、`jerk`，0 = 不限；全为 0 时直接输出未整形的速度。`planner_mode`：

- `axis_phase`（默认）：全向底盘 x/y/yaw 相位同步，不能同步时退回时间同步。
- `axis_time`：全向底盘时间同步。
- `straight`：只用于差速，原地对准、直线行驶、终点对准。

## 生命周期

第一次 step 时启动。模块被禁用的那些 step 会暂停控制器，重新启用后重新同步。C++ 里通过 `controller()` 提交指令和里程计，`output()` 是上一步要发给底盘的速度。

## 示例

```yaml
chassis:
  type: chassis
  chassis:
    kinematics: omni
    base_frame: base_link
    max_linear: 1.0
    max_angular: 3.0
    max_linear_acceleration: 2.5
    arrival_position_tolerance: 0.001
    arrival_rotation_tolerance: 0.002
```

```python
ch = e.module("chassis")
ch.write_port("velocity", [0.2, 0.0, 0.0])  # 需要在 command_timeout_s 内持续写
ch.write_port("pose_goal", [1.0, 0.0, 0.0])
ch.tune({"pose_limits": {"linear_velocity": 0.3, "planner_mode": "axis_time"}})
```

## 输出

- `vx`、`vy`、`wz`：整形后的速度。`x`、`y`、`yaw`：位姿。`nominal_vx/vy/wz`：未整形的目标速度。
- `planned`：轨迹当前位姿。`goal`：位姿目标，非位姿模式为 NaN。
- `active`、`fault`、`state`、`reason`：状态及原因。
- `command`（指令来源）、`command_mode`、`command_priority`、`command_fresh`。
- `source`：位姿来源（内部推算或里程计）。`phase`：差速位姿模式的阶段。

## 常见坑

- 速度指令是租约，停止写入后 `command_timeout_s` 内底盘停下。遥控要按固定频率持续写。
- `pose_limits` 只影响位姿模式，速度模式始终用 `chassis` 里的 `max_*`。tune 时不检查是否超过这些上限，要自己保证不超过底盘能力。
- `straight` 只能配 `differential`，`axis_*` 只能配 `omni`，其他组合会被拒。
- 里程计时间戳比当前时间早 `odom_timeout_s` 以上会被拒收，不报错，位姿只靠推算。实机要检查里程计和控制器时钟是否一致。
- 调试时暂停进程或单步过久会触发 `scheduler_discontinuity` fault，底盘停住，把模块禁用再启用即可恢复。
