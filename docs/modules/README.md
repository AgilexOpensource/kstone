# Modules

[中文](../zh-CN/modules/README.md) · [README](../../README.md)

Modules are the entries under `modules:` in a [config](../config.md). The key is the id, `type` selects the type, and the remaining fields are parsed by that type. These pages describe each type's purpose, ports, key parameter semantics and common pitfalls. For the full field list, defaults and ranges, see the JSON Schema printed by `kstone schema <type>`.

| Type | Purpose |
|---|---|
| [frame_task](frame_task.md) | Make a frame track a pose target |
| [joint_target](joint_target.md) | Joint-space target, direct joint control |
| [null_space_preference](null_space_preference.md) | Posture preference in a frame_task's null space |
| [gaze](gaze.md) | Point a camera at a set of frames |
| [center_of_mass](center_of_mass.md) | Keep the CoM inside a support polygon |
| [self_collision](self_collision.md) | Self-collision avoidance |
| [world_collision](world_collision.md) | Collision avoidance against obstacles |
| [chassis](chassis.md) | Base velocity and pose control (outside the QP) |

The example config [omni_dual_arm.yaml](../../examples/omni_dual_arm/omni_dual_arm.yaml) uses every type, and the snippets on these pages come from it.

## Ports

Ports are a module's runtime inputs. `Module.ports` lists their names, types and ranges. `write_port` takes effect on the next step. The debug protocol's `set_port` is equivalent.

| Type | Value |
|---|---|
| `vector` | Fixed-length array with `labels`, `lower`, `upper` |
| `pose` | `{translation, rotation}` (rotation is a wxyz quaternion) or a 4×4 matrix; `frame` / `reference` give the target's frames |
| `flag` | bool |
| `number` | scalar |

A port with `timeout_s` lapses when it is not written within that time.

## Runtime tuning

Fields marked `tunable: true` in the schema can be changed at runtime:

- `Module.tune(dict)` changes only the given keys, and nested objects may give only some sub-keys. A call is atomic: if any item fails validation, nothing changes.
- `Module.params()` returns the current tunable values. `Module.tune_schema` returns the schema of the tunable part.
- The debug protocol's `tune` is equivalent.
- The solver and joint limits are tuned on the engine: `Engine.tune_solver`, `Engine.tune_joint_limits`.

```python
waist = e.module("waist")
waist.tune({"trajectory": {"translation": {"velocity": 0.5}}})
print(waist.params()["trajectory"]["translation"]["velocity"])  # 0.5
```

## Enabling and joint ownership

- `Module.enabled` is read/write (debug protocol `set_enabled`). Disabled modules take no part in the solve.
- Modules that drive joints compete for them. A joint_target claims its joints once a target is written, and conflicting modules are held meanwhile. `Module.held_by` names the owner.
- `on_held` sets the behaviour while held: `wait` keeps the target and resumes tracking afterwards; `abort` drops the target and holds the pose at that moment.
- A joint_target's `release_after_s` is an input watchdog: for this long after its last input, requests from other modules are deferred.

## Task levels

Modules with a `task` field (frame_task, joint_target, null_space_preference, gaze) register a task with the solver:

- `task.level`: priority in the layered solve; lower is more important.
- `task.enabled`: whether the task takes part in the solve.
- `task.locked`: contributes no rows and holds the task's joints still.

center_of_mass, self_collision and world_collision join the solve as inequality constraints. They neither drive nor claim joints.

## Homing

Modules with `Module.homeable` true (frame_task, joint_target) accept `home()`. `home_state` is `idle` / `waiting` / `executing` / `succeeded` / `timed_out` / `aborted`. `home_motion.velocity_scale` sets the speed. `home_motion.timeout_s` is a no-progress timeout, so a move that keeps approaching is not cut off.

## Outputs and visuals

`Module.outputs(q)` returns display values at q (errors, CoM slack, closest distance and so on). `Module.visuals()` returns drawable shapes. Output keys are listed on each module page and are meant for display and diagnostics only.
