# null_space_preference

[中文](../zh-CN/modules/null_space_preference.md) · [Module overview](README.md)

Applies a preference in the null space of a frame_task (`primary`) without affecting that task's end-effector tracking. The preference can be switched at runtime without allocating.

## Key parameters

- `primary`: id of the frame_task, which also becomes a dependency. With `joints` empty, the scalar joints driven by primary are used.
- `preference`:
  - `posture`: stay close to a reference posture.
  - `manipulability`: raise manipulability; `manipulability_axes` selects the directions (translation only by default).
  - `joint_limits`: stay away from joint limits, active within `joint_limits_band` of a limit. Unavailable when every joint is unbounded.
- `reference` (the posture baseline):
  - `initial` (default): the initial q.
  - `current`: the current q each cycle, which only damps null-space motion.
  - `neutral`: mid-range of each joint, clamped to the effective limits (model limits ∩ yaml narrowing ∩ runtime `tune_joint_limits`).
  - `custom`: the posture given by `posture`, empty = initial q.
- `posture_values`: `{joint name: value}`, applied on top of the reference posture. `current` rejects both `posture` and `posture_values`.
- Runtime switching:
  - Tuning only `reference` to `custom` takes the configured `posture`, or the current q when none is configured, then applies `posture_values`.
  - Switching to `initial` / `neutral` / `current` clears `posture_values`.
  - `reseed_current: true` (tune only) stores the current q as the custom posture and selects the posture preference.
- `posture_weight`, `manipulability_weight`, `joint_limits_weight`: weights of each preference. `posture_velocity_scale` caps the speed of returning to the posture, in (0, 1].
- `projector.damping` / `threshold`: null-space projector damping; threshold > 0 enables adaptive damping.

## Example

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
null.tune({"reseed_current": True})  # store the current posture as the preference
null.tune({"preference": "joint_limits"})
```

## Pitfalls

- Runs only while no other module owns its joints. In the example `left_arm` (joint_target, `claim_idle: true`) owns the left arm, so `left_null` stays held.
- The preference lives in the null space and does not cost primary any accuracy. But axes freed in primary (weight 0) are part of the null space too, and the preference will move them.
- When the reference posture is far from what the target pose allows, the preference only gets as close as the redundant degrees of freedom permit. The joints will not reach the reference values.
