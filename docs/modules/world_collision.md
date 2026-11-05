# world_collision

[中文](../zh-CN/modules/world_collision.md) · [Module overview](README.md)

Adds distance constraints between the robot geometries and a fixed number of obstacles. Constraint only; it neither drives nor claims joints. Robot self pairs are not checked; use [self_collision](self_collision.md) for those.

## Ports

| Port | Type | Description |
|---|---|---|
| `<name>` | pose | Obstacle pose in the world |
| `<name>/enabled` | flag | When off, the obstacle is not checked and its visual is hidden |

## Key parameters

- `urdf`: robot collision geometry.
- `obstacles`: at least one obstacle. Names are unique and may not contain `/` (the name is also the port name). The set cannot change after startup.
  - `shape.type`: `sphere`, `box`, `cylinder` or `capsule`.
  - `shape.size`: sphere `[r, 0, 0]`, box `[x, y, z]`, cylinder and capsule `[r, length, 0]` along z.
  - `pose`: `translation` plus either a quaternion `rotation` (wxyz) or `rpy` (extrinsic XYZ), not both.
  - `enabled`: initial state.
- `velocity_timeout`: obstacle velocity is estimated from successive pose writes and zeroed this long (s) after the last write, default 0.2. 0 treats every obstacle as static.
- `closed_loop`: also bound the measured state.
- `constraint`: same as self_collision, see [self_collision](self_collision.md#constraint).

An obstacle moving towards the robot raises its rows by the closing speed at the witness point. Cutting planes and the endpoint clip treat obstacles as static within a step.

## Example

```yaml
obstacles:
  type: world_collision
  velocity_timeout: 0.2
  urdf: omni_dual_arm.urdf
  obstacles:
    - name: table
      shape: {type: box, size: [0.4, 0.8, 0.04]}
      pose: {translation: [0.615, 0, 0.427]}
    - name: box2
      shape: {type: box, size: [0.2, 0.2, 0.2]}
      pose: {translation: [0.655, 0.1, 0.469]}
  constraint: {d_min: 0.002, activation_margin: 0.02, hysteresis: 0.005, gain: 1.5}
```

```python
obs = e.module("obstacles")
obs.write_port("box2", {"translation": [0.655, 0.1, 0.6], "rotation": [1, 0, 0, 0]})
obs.write_port("table/enabled", False)
```

## Outputs

Same as self_collision: `min_distance`, `closest`, `closest_pair`, `band`, `pair`, `pairs`, `active_pairs`. Visuals list the robot geometries first, then the obstacles.

## Pitfalls

- The obstacle count is fixed. When the number of objects in a scene varies, configure the maximum up front and switch them with `<name>/enabled`.
- Discontinuous pose writes (a jump after dropped perception frames, for example) produce a large velocity estimate and suddenly tighten the constraint. When perception runs slower than `1 / velocity_timeout`, the velocity drops to zero between writes.
- A newly enabled obstacle already penetrating the robot is exempt by default and will not push the arm out. Move it away first.
