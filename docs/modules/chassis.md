# chassis

[中文](../zh-CN/modules/chassis.md) · [Module overview](README.md)

Planar base controller running outside the QP. It locks the free-flyer root and writes the base pose into q in prepare(); z = 0 is the ground. It supports velocity commands and pose targets, with command leases, priority arbitration, Ruckig shaping, dead reckoning and odometry override.

## Ports

| Port | Type | Description |
|---|---|---|
| `velocity` | vector `[vx, vy, wz]` | Body-frame velocity |
| `pose_target` | vector `[x, y, yaw]` | Pose target in the world |

Both ports lapse after `command_timeout_s`. The last one written is submitted on the next step with source `port`.

## Key parameters

Config lives in the `chassis:` sub-object:

- `kinematics`: `omni` or `differential`. Not tunable; read it from `Module.properties`.
- `base_frame`: frame of the control centre, default `base_link`.
- `max_linear_velocity`, `max_angular_velocity` and the matching `*_acceleration`, `*_deceleration`: limits. A velocity moving against the remaining displacement brakes at the deceleration limit. There is no jerk limit: the base firmware smooths its own acceleration. Velocity mode only clamps the command to the velocity limits (the base ramps it); all of them seed the initial pose-mode limits.
- `max_lateral_velocity`, `max_lateral_acceleration`, `max_lateral_deceleration` (default 0 = the matching `max_linear_*`): omni body-y limits for a base whose sideways motion is weaker than forward motion. `max_linear_*` then covers body x, and translation stays inside the ellipse $(v_x/x_{max})^2 + (v_y/y_{max})^2 \le 1$ (the planar norm when both are equal).
- `position_tolerance`, `orientation_tolerance`: arrival check, default 0.01 m and 0.02 rad.
- `odom_latency_s` (default 0 = off): omni pose mode only. The base's response delay: odometry reflects only the outputs sent up to this long before its stamp, so the outputs sent since then are replayed onto it before planning. Without it, a base whose commands take effect late overshoots and keeps moving back and forth around the goal. Measure it from a velocity step (command to odometry).
- `target_position_deadband`, `target_orientation_deadband`: differential only; target changes below these keep the current path.
- `command_timeout_s` (default 0.25): a command not refreshed for this long clears motion with reason `command_timeout`. This is not a fault.
- Odometry timeout: the engine `feedback_timeout_s` (0 = odometry never goes stale). Odometry older than this is rejected. While the last odometry is fresh the pose holds it; once it expires the pose is dead-reckoned from the output velocity.
- The chassis starts at the root pose of its first cycle (on an engine, the robot's initial state). An engine resynchronisation with rebase that covers the root (for example when the group holding the root attaches) resets the chassis to the measured root pose.
- `max_dt_s` (0.1), `allowed_missed_cycles` (2): a gap between steps longer than `max(max_dt_s, period × allowed_missed_cycles)` faults with `scheduler_discontinuity`. Only a skipped step (disable, then enable) or an engine resynchronisation covering the root resynchronises and recovers.

Top level: `center_in_root` (fixed pose of the control centre in the root frame), `period_s` (initial period; afterwards the engine dt is used each cycle).

### pose_limits (tune only)

Runtime limits for pose mode: `max_linear_velocity`, `max_linear_acceleration`, `max_linear_deceleration`, the `max_lateral_*` fields (0 = the matching linear bound) and the matching `max_angular_*` fields; 0 = unbounded. When all are 0 the unshaped velocity is output directly. `planner_mode`:

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
    max_linear_velocity: 1.0
    max_angular_velocity: 3.0
    max_linear_acceleration: 2.5
    position_tolerance: 0.001
    orientation_tolerance: 0.002
```

```python
ch = e.module("chassis")
ch.write_port("velocity", [0.2, 0.0, 0.0])  # keep writing within command_timeout_s
ch.write_port("pose_target", [1.0, 0.0, 0.0])
ch.tune({"pose_limits": {"max_linear_velocity": 0.3, "planner_mode": "axis_time"}})
```

## Outputs

- `vx`, `vy`, `wz`: shaped velocity. `x`, `y`, `yaw`: pose. `nominal_vx/vy/wz`: unshaped target velocity.
- `planned`: current trajectory pose. `target`: pose target, NaN outside pose mode.
- `active`, `fault`, `state`, `reason`: state and its cause.
- `command` (command source), `command_mode`, `command_priority`, `command_fresh`.
- `source`: pose source (internal estimate or odometry). `phase`: differential pose-mode phase.

## Pitfalls

- A velocity command is a lease: once writes stop, the base stops within `command_timeout_s`. Teleoperation must write at a fixed rate.
- `pose_limits` affects pose mode only; velocity mode always uses the `max_*_velocity` values in `chassis`. A tune does not check them against those maxima, so keep them within what the base can do.
- `straight` only goes with `differential`, `axis_*` only with `omni`. Other pairs are rejected.
- Odometry stamped more than the engine `feedback_timeout_s` before now is rejected silently and the pose relies on dead reckoning alone. On hardware, check that the odometry and controller clocks agree.
- Pausing the process in a debugger or stepping too slowly triggers a `scheduler_discontinuity` fault and the base stops. Disable and re-enable the module to recover.
