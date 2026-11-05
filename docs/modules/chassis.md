# chassis

[中文](../zh-CN/modules/chassis.md) · [Module overview](README.md)

Planar base controller running outside the QP. It locks the free-flyer root and writes the base pose into q in prepare(); z = 0 is the ground. It supports velocity commands and pose goals, with command leases, priority arbitration, Ruckig shaping, dead reckoning and odometry override.

## Ports

| Port | Type | Description |
|---|---|---|
| `velocity` | vector `[vx, vy, wz]` | Body-frame velocity |
| `pose_goal` | vector `[x, y, yaw]` | Pose goal in the world |

Both ports lapse after `command_timeout_s`. The last one written is submitted on the next step with source `port`.

## Key parameters

Config lives in the `chassis:` sub-object:

- `kinematics`: `omni` or `differential`.
- `base_frame`: frame of the control centre, default `base_link`.
- `max_linear`, `max_angular` and the matching `*_acceleration`, `*_deceleration`, `*_reversal_deceleration`, `*_jerk`: limits. Velocity mode uses them directly, and they seed the initial pose-mode limits.
- `arrival_position_tolerance`, `arrival_rotation_tolerance`: arrival check, default 0.01 m and 0.02 rad.
- `target_translation_deadband`, `target_rotation_deadband`: differential only; goal changes below these keep the current path.
- `command_timeout_s` (default 0.25): a command not refreshed for this long clears motion with reason `command_timeout`. This is not a fault.
- `odom_timeout_s` (default 0.25): odometry older than this is rejected. Between odometry updates the pose is dead-reckoned from the output velocity.
- `max_dt_s` (0.1), `allowed_missed_cycles` (2): a gap between steps longer than `max(max_dt_s, period × allowed_missed_cycles)` faults with `scheduler_discontinuity`. Inside the module, only a skipped step (disable, then enable) resynchronises and recovers.

Top level: `center_in_root` (fixed pose of the control centre in the root frame), `period` (initial period; afterwards the engine dt is used each cycle).

### pose_limits (tune only)

Runtime limits for pose mode: `linear_`/`angular_` plus `velocity`, `acceleration`, `deceleration`, `reversal_deceleration`, `jerk`; 0 = unbounded. When all are 0 the unshaped velocity is output directly. `planner_mode`:

- `axis_phase` (default): omni, x/y/yaw phase-synchronised, falling back to time sync.
- `axis_time`: omni, time-synchronised.
- `straight`: differential only; align in place, drive straight, align at the end.

## Lifecycle

Starts on the first step. Steps skipped while the module is disabled pause the controller, which resynchronises once re-enabled. In C++, submit commands and odometry through `controller()`; `output()` is the velocity of the last step to send to the base.

## Example

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
ch.write_port("velocity", [0.2, 0.0, 0.0])  # keep writing within command_timeout_s
ch.write_port("pose_goal", [1.0, 0.0, 0.0])
ch.tune({"pose_limits": {"linear_velocity": 0.3, "planner_mode": "axis_time"}})
```

## Outputs

- `vx`, `vy`, `wz`: shaped velocity. `x`, `y`, `yaw`: pose. `nominal_vx/vy/wz`: unshaped target velocity.
- `planned`: current trajectory pose. `goal`: pose goal, NaN outside pose mode.
- `active`, `fault`, `state`, `reason`: state and its cause.
- `command` (command source), `command_mode`, `command_priority`, `command_fresh`.
- `source`: pose source (internal estimate or odometry). `phase`: differential pose-mode phase.

## Pitfalls

- A velocity command is a lease: once writes stop, the base stops within `command_timeout_s`. Teleoperation must write at a fixed rate.
- `pose_limits` affects pose mode only; velocity mode always uses the `max_*` values in `chassis`. A tune does not check them against those maxima, so keep them within what the base can do.
- `straight` only goes with `differential`, `axis_*` only with `omni`. Other pairs are rejected.
- Odometry stamped more than `odom_timeout_s` before now is rejected silently and the pose relies on dead reckoning alone. On hardware, check that the odometry and controller clocks agree.
- Pausing the process in a debugger or stepping too slowly triggers a `scheduler_discontinuity` fault and the base stops. Disable and re-enable the module to recover.
