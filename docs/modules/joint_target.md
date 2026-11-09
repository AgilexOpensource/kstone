# joint_target

[中文](../zh-CN/modules/joint_target.md) · [Module overview](README.md)

A joint-space target, fed to the solver through a jerk-limited joint trajectory. Used for direct joint control and for parts that only need joint targets, such as grippers and pan-tilt heads.

## Ports

| Name | Type | Description |
|---|---|---|
| `target` | vector | One value per joint in `joints` order; components outside the position limits are clamped |

C++ `JointTargetModule::set_target(names, values)` sets only the named joints, in any order; the others keep the current target, or hold the measured position if the module is disabled or held. Unknown or duplicate names are an error. A labeled port write does the same, e.g. Python `write_port("target", [0.2], labels=["elbow"])`; see [Ports](README.md#ports).

## Key parameters

- `joints`: scalar joints, at most one per mimic group.
- `task.level`: 0 when unset, so joint control wins over pose tasks sharing these joints.
- `trajectory.trigger`: `continuous` (default) always shapes; `threshold` shapes only moves that take longer than `threshold` seconds at the velocity limit and follows small moves directly; `direct` does no shaping, the reference is the target.
- `trajectory.sync`: `phase` (default) moves all joints in phase along a straight line, falling back to `time` when not collinear; `time` makes them arrive together; `none` moves each independently.
- `trajectory.velocity_scale` / `max_velocity`: overall scale and per-joint cruise cap. With `velocity_limit` empty the model limits are used, and each joint's limit must then be finite and > 0.
- Ownership:
  - The module claims its joints when `target` is written or homing finishes, and keeps them after arrival until another module has new input and `release_after_s` has passed since this module's last input (default 0.25 s, 0 = yield at once).
  - `priority`: claim priority.
  - `on_held`: defaults to `wait`, keeping the target while held and resuming afterwards.
  - `on_idle`: what happens without new input. `hold` (default) keeps the last target until another module gets new input. `release` drops the task so other modules can move the joints. `claim` also claims the joints, so conflicting modules stay held; this makes it a standing joint controller.
- `home`: per-joint homing positions, empty = initial posture.

## Example

```yaml
torso:
  type: joint_target
  joints: [torso_joint_1, torso_joint_2, torso_joint_3, torso_joint_4]
  task: {level: 0}
  home_motion: {velocity_scale: 0.1667, timeout_s: 2.0}
  on_idle: claim
  release_after_s: 0.25
```

```python
torso = e.module("torso")
torso.write_port("target", [0.0, 1.2, 0.0, 0.0])
torso.tune({"trajectory": {"velocity_scale": 0.5}})
```

## Pitfalls

- With `on_idle: claim`, a frame_target, null_space or gaze sharing the joints stays held until another module has new input and `release_after_s` has passed. In the example `left_arm` owns the left arm, so `left_null` has no effect. To use the null-space preference, disable that joint_target or set `on_idle: hold`.
- A large `release_after_s` gives a visible pause when switching to pose control. 0 lets other modules take the joints at any time.
