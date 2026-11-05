# self_collision

[中文](../zh-CN/modules/self_collision.md) · [Module overview](README.md)

Adds distance constraints between the robot's own geometries, keeping every pair at least `d_min` apart. Constraint only; it neither drives nor claims joints.

## Key parameters

- `urdf`: URDF with collision geometry; relative paths are tried against the config folder first. `srdf`: link pairs listed in its `disable_collisions` are skipped.
- `groups`: bodies within one group are not checked against each other. A joint matches when it is in `joint_names` or starts with one of `joint_prefixes`, and is not in `exclude_joints`; the body it carries belongs to the group.
- `exclude_pairs`: frame or geometry name pairs; excludes every geometry pair between the two bodies. `exclude_subtrees`: excludes every pair between the two bodies' kinematic subtrees (two hands, for example).
- `exclude_initial_collisions`: true by default. Drops pairs already penetrating at `robot.initial` (neutral when unset), with one warning per pair.
- `closed_loop`: also bound the measured state (with fresh feedback), on top of the commanded one.
- `constraint`: shared with world_collision, see below.

### constraint

- `d_min`: minimum distance kept (m), default 0.03.
- `gain`: allowed approach speed per metre above `d_min` (1/s), default 50. Each row is $\dot d \ge -\text{gain}\,(d - d_{min})$.
- `activation_margin`: a pair gets a row once closer than `d_min + activation_margin`, default 0.05. `hysteresis`: an active pair stays active up to this much further, default 0.005.
- `approach_deceleration`: braking deceleration (m/s²). The approach speed stays below $\sqrt{2a(d - d_{min})}$ and below half the remaining gap per step. Default 5, 0 = off.
- `max_rows`: fixed row count, nearest pairs first, default 16. Pairs beyond it are dropped, farthest first.
- `max_cuts`: extra rows for cutting planes, default 8, 0 = off. When a step ends with a pair below its floor, the step is re-solved with that pair relinearized at the step end. This covers the error of start-linearized rows on curved motion such as sliding or rotating.
- `clip_slack`: how far below the floor the endpoint clip allows (m), default 0. A slack lets small steps skip the check and can deepen a penetration.
- `exempt_penetrating`: true by default. A pair already penetrating when it enters the band gets no constraint until it is back beyond `d_min`; it may move out or deeper. With false it is pushed out like any other pair, which a deep start may not allow.

## Example

```yaml
self_collision:
  type: self_collision
  urdf: omni_dual_arm.urdf
  groups:
    - {name: left_arm, joint_prefixes: [left_arm_, left_gripper_]}
    - {name: right_arm, joint_prefixes: [right_arm_, right_gripper_]}
  exclude_pairs:
    - [head_base_link, head_yaw_link]
    - [left_gripper_base, right_gripper_base]  # let the grippers close in for bimanual work
  closed_loop: true
  constraint: {d_min: 0.002, activation_margin: 0.01, hysteresis: 0.005, gain: 1.5, approach_deceleration: 20}
```

```python
sc = e.module("self_collision")
sc.tune({"constraint": {"d_min": 0.01}})
print(sc.outputs(e.q)["min_distance"])
```

## Outputs

- `min_distance`, `closest` (the two witness points of the nearest pair).
- `closest_pair`: visuals of the nearest pair, empty outside the band. `band`: visuals within the activation band.
- `pair`: nearest pair as `a/b`. `pairs`: number of pairs checked. `active_pairs`: pairs within the band at the latest solve.

## Pitfalls

- Geometries of adjacent links often touch by design. Exclude them with `exclude_pairs` or an SRDF, or the constraint stays saturated. The `exclude_initial_collisions` warnings in the startup log are a hint, but cover the initial posture only.
- When `active_pairs` often equals `max_rows`, there are too few rows and farther pairs are dropped. Raise `max_rows` or narrow `activation_margin`.
- A larger `gain` lets pairs approach the boundary more boldly. In the example, `gain` 1.5 at 50 Hz uses at most about 3% of the remaining slack per step.
- Pairs already penetrating when the module is re-enabled start exempt and are not pushed apart at once.
