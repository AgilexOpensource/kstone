# center_of_mass

[中文](../zh-CN/modules/center_of_mass.md) · [Module overview](README.md)

Keeps the projection of the centre of mass (payloads included) on the support plane inside a support polygon. Joins the solve as an inequality constraint and neither drives nor claims joints.

## Key parameters

- `frame`: frame of the support polygon, which lies in its xy plane. `polygon`: strictly convex polygon with at least 3 `[x, y]` vertices.
- `subtree`: use the CoM of this frame's subtree only, empty = whole model.
- `margin`: distance kept from the polygon edges (m); a negative value allows that much overshoot. A positive value may not exceed the polygon's inscribed-circle radius.
- `gain`: fraction of the slack usable per step, in (0, 1]. `recovery_speed`: speed of pulling back once past the margin (m/s); recovery need not happen in one step.
- `payloads`: point masses on frames (`frame`, `mass`, `offset`) for end-effector loads. With `subtree` set, only payloads inside the subtree count.
- `recovery_tasks`: only the joints driven by these modules pull the CoM back, and these modules become dependencies. Other joints cannot push further out once past the margin, and are not used to pull back. Empty = all joints.
- `closed_loop`: also bound the measured state (with fresh feedback), on top of the commanded one.

## Example

```yaml
com:
  type: center_of_mass
  frame: base_link
  polygon: [[0.216, 0.216], [-0.216, 0.211], [-0.216, -0.211], [0.216, -0.216]]
  margin: 0.05
  recovery_tasks: [waist]
  closed_loop: true
  payloads:
    - {frame: left_tool_frame, mass: 3.0}
    - {frame: right_tool_frame, mass: 3.0}
```

```python
com = e.module("com")
com.tune({"payloads": [{"frame": "left_tool_frame", "mass": 5.0}]})
print(com.outputs(e.q)["min_slack"])
```

## Outputs

`com`, `ground` (CoM projected on the polygon plane), `polygon`, `margin_polygon`, `min_slack` (m), `inside` (slack >= margin).

## Pitfalls

- The polygon must be strictly convex. Either winding is accepted.
- Non-recovery joints may use the slack plus what the recovery joints can still pull back in one step. When the recovery joints hit their velocity or position limits, the non-recovery joints stop too. With few recovery joints or tight limits, the arms stop earlier.
- `payloads` is replaced as a whole list, so a tune must give the complete list.
