# chassis

[English](../../modules/chassis.md) · [模块总览](README.md)

平面底盘控制器，在 QP 之外运行。它锁住 free-flyer 根节点，在 prepare() 中把底盘位姿写进 q，z = 0 为地面。支持速度指令和位姿目标两种模式，带指令租约、优先级仲裁、Ruckig 轨迹整形、航位推算和里程计覆盖。

## 端口

| 端口 | 类型 | 说明 |
|---|---|---|
| `velocity` | vector `[vx, vy, wz]` | 车体系速度 |
| `pose_target` | vector `[x, y, yaw]` | 世界系位姿目标 |

两个端口都在 `command_timeout_s` 后失效。自上次 step 以来写入的内容都以来源 `port` 提交。`velocity` 优先于 `pose_target`：速度流保持新鲜（在超时内持续重写）时位姿目标会被丢弃，速度流停止后再从底盘当前位姿重新接上。零速流和其他速度指令一样会占住底盘，所以不操控时遥控端必须停止发布，而不是发零速。

## 关键参数

配置放在 `chassis:` 子对象里：

- `kinematics`：`omni` 或 `differential`。不可调，可从 `Module.properties` 读取。
- `base_frame`：控制中心所在 frame，默认 `base_link`。
- `max_linear_velocity`、`max_angular_velocity` 及对应的 `*_acceleration`、`*_deceleration`：上限。速度方向与剩余位移相反时按减速度上限刹车。不限 jerk，加速度的平滑由底盘固件完成。速度模式只把指令限在速度上限内（加减速由底盘自己完成）；位姿模式的初始限制取全部这些值。
- `max_lateral_velocity`、`max_lateral_acceleration`、`max_lateral_deceleration`（默认 0，即取对应的 `max_linear_*`）：全向底盘车体 y 方向的上限，用于侧移比前后弱的底盘。此时 `max_linear_*` 只管车体 x，平移限制在椭圆 $(v_x/x_{max})^2 + (v_y/y_{max})^2 \le 1$ 内（两者相等时就是平面合速度）。
- `position_tolerance`、`orientation_tolerance`：到达判定，默认 0.01 m、0.02 rad。
- 位姿模式下的里程计：底盘执行 Ruckig 轨迹，规划基于自身参考位姿，运动中不被里程计覆盖，因此与底盘从指令到里程计的延迟无关（对延迟的里程计做反馈会过冲）。里程计与参考轨迹的偏离判断参照 ros2_control joint_trajectory_controller：运动中里程计须落在最近 1 s 内某一时刻参考位姿的 0.1 m / 0.2 rad 范围内（允许底盘最多滞后 1 s）；超出即说明有其他控制源（遥控器、外力），目标视为已完成，停在底盘所在位置（reason 为 `yielded`，到位后被挪离也是；持续发送目标的客户端应随之跟随底盘，直到发出新目标）。轨迹结束时若在容差内即锁存到位：输出保持为零、位姿跟随里程计，直到来了偏离锁存目标超过容差的新目标（同 Nav2 stateful goal checker），不会以底盘跟不上的低速反复微调，之后被挪动也不会开回去。若在容差外（打滑、底盘滞后或冲过），待里程计静止 0.2 s 后再规划剩余误差。非位姿模式下位姿直接跟随里程计。
- `command_timeout_s`（默认 0.25）：指令超过这么久没更新就清除运动，reason 为 `command_timeout`，不算 fault。
- 里程计超时：取引擎的 `feedback_timeout_s`（0 = 里程计永不过期）。比这更旧的里程计被拒收。非位姿模式（或位姿模式已锁存到位）下，最近一次里程计未过期时位姿保持该值，过期后用输出速度推算位姿。
- 底盘从第一个周期的根节点位姿开始（在引擎上即机器人初始状态）。引擎带 rebase 的重新同步覆盖根节点时（例如包含根节点的分组同步时），底盘位姿重置为测得的根节点位姿。
- `max_dt_s`（0.1）、`allowed_missed_cycles`（2）：两次 step 间隔超过 `max(max_dt_s, 周期 × allowed_missed_cycles)` 时进入 fault（`scheduler_discontinuity`）。只有跳过一步（禁用后再启用）或覆盖根节点的引擎重新同步才会重新同步并恢复。

顶层：`center_in_root`（控制中心在根节点系中的固定位姿）、`period_s`（初始周期，之后每周期取引擎 dt）。

### pose_limits（只能 tune）

位姿模式的运行时限制：`max_linear_velocity`、`max_linear_acceleration`、`max_linear_deceleration`、`max_lateral_*`（0 = 取对应的 linear 值）及对应的 `max_angular_*` 字段，0 = 不限；全为 0 时直接输出未整形的速度。`planner_mode`：

- `axis_phase`（默认）：各轴相位同步，不能同步时退回时间同步。
- `axis_time`：各轴时间同步。

全向底盘的轴是世界系 x/y/yaw。差速底盘的轴是沿当前朝向的距离和 yaw：边走边转，不横移，目标在朝向侧方的部分不追。

## 生命周期

第一次 step 时启动。模块被禁用的那些 step 会暂停控制器，重新启用后重新同步。C++ 里通过 `controller()` 提交指令和里程计，`output()` 是上一步要发给底盘的速度。

## 示例

```yaml
chassis:
  type: chassis
  chassis:
    kinematics: omni
    base_frame: base_link
    max_linear_velocity: 1.0
    max_angular_velocity: 3.0
    max_linear_acceleration: 2.5
    position_tolerance: 0.001
    orientation_tolerance: 0.002
```

```python
ch = e.module("chassis")
ch.write_port("velocity", [0.2, 0.0, 0.0])  # 需要在 command_timeout_s 内持续写
ch.write_port("pose_target", [1.0, 0.0, 0.0])
ch.tune({"pose_limits": {"max_linear_velocity": 0.3, "planner_mode": "axis_time"}})
```

## 输出

- `vx`、`vy`、`wz`：整形后的速度。`x`、`y`、`yaw`：位姿。`nominal_vx/vy/wz`：未整形的目标速度。
- `planned`：轨迹当前位姿。`target`：位姿目标，非位姿模式为 NaN。
- `active`、`fault`、`state`、`reason`：状态及原因。
- `command`（指令来源）、`command_mode`、`command_priority`、`command_fresh`。
- `source`：位姿来源（内部推算或里程计）。

## 常见坑

- 速度指令是租约，停止写入后 `command_timeout_s` 内底盘停下。遥控要按固定频率持续写。
- 位姿目标同样是租约，且会被新鲜的速度流压掉：两者同时持续重写，底盘会一直在速度模式下，永远到不了目标。
- `pose_limits` 只影响位姿模式，速度模式始终用 `chassis` 里的 `max_*_velocity`。tune 时不检查是否超过这些上限，要自己保证不超过底盘能力。
- 差速底盘位姿模式到不了正侧方的目标，调用方要先把底盘转向目标。
- 里程计时间戳比当前时间早引擎 `feedback_timeout_s` 以上会被拒收，不报错，位姿只靠推算。实机要检查里程计和控制器时钟是否一致。
- 调试时暂停进程或单步过久会触发 `scheduler_discontinuity` fault，底盘停住，把模块禁用再启用即可恢复。
