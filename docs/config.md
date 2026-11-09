# Configuration

[中文](zh-CN/config.md) · [README](../README.md)

An engine config is one YAML file. `kstone check <config.yaml>` builds the engine without running it and reports the first error with its line and column. Unknown keys are errors. Minimal example, from [arm_loopback.yaml](../tests/fixtures/arm_loopback.yaml):

```yaml
model:
  file: arm.urdf
rate_hz: 100
robot:
  type: loopback
modules:
  arm:
    type: joint_target
    joints: [j1, j2, j3]
```

## Top-level keys

| Key | Default | Meaning |
|---|---|---|
| `model` | required | Robot model, see [model](#model) |
| `rate_hz` | 100 | Control rate |
| `realtime` | false | Apply the `thread` settings (real-time scheduling) in `run()`; `kstone run --realtime` forces it on. `run()` always paces cycles to the clock |
| `feedback_timeout_s` | 0.25 | Feedback that is older or has no timestamp is stale; also the attach window after feedback turns fresh; 0 = never stale and attached at once. See [run contract](robot.md#run-contract) |
| `max_failed_cycles` | 5 | `run()` returns after this many consecutive failed cycles |
| `feedback_joints` | all scalar joints | Joints whose measured position feeds the closed-loop estimate (collision, CoM, homing). Names or fnmatch patterns |
| `feedback_groups` | none | Joints with their own feedback stamp, freshness and attach, e.g. `[{name: base, joints: [root_joint]}]`. Names or fnmatch patterns; ungrouped joints form the last group, `default`. See [run contract](robot.md#run-contract) |
| `safety` | | Command guard, see [safety](#safety) |
| `solver` | | QP solver, see [solver](#solver) |
| `robot` | loopback | Robot interface, see [robot](#robot) |
| `thread` | | Real-time thread settings, see [Robot integration](robot.md#real-time-thread) |
| `modules` | none | Map of module id to `{type, ...}`, see [Modules](modules/README.md) |

## model

| Key | Default | Meaning |
|---|---|---|
| `file` | required | Model path or URI (URDF or MJCF) |
| `format` | from extension | `urdf` or `mjcf` |
| `package_dirs` | | Roots for `package://<name>/<path>`, tried in order |
| `root` | `fixed` | `free_flyer` adds a floating base; its q is x, y, z, qx, qy, qz, qw |
| `root_joint_name` | `root_joint` | Name of the floating base joint |
| `mimic` | `independent` | `independent` keeps mimic followers as ordinary DoFs; `coupled` drives them from the leader |
| `joint_limits` | | Per-joint overrides, see below |
| `locked_joints` | | Joints held at their initial value, see below |

`joint_limits` replaces the model's limits for the named joints, so it can also widen them:

```yaml
model:
  file: omni_dual_arm.urdf
  joint_limits:
    velocity: {left_gripper_joint: 1.0}
    position:
      lower: {torso_joint_2: 0.2618}
      upper: {torso_joint_2: 2.2}
```

`locked_joints` lists joints the solver never moves. They keep their `robot.initial` value (or the neutral value), stay in the model, and are still part of the state and the command, so `joint_states` stays complete. Modules that list a locked joint keep working on the remaining joints. The floating base and coupled mimic followers cannot be listed; use `root: fixed`, or lock the leader:

```yaml
model:
  file: robot.urdf
  locked_joints: [left_wheel_joint, right_wheel_joint]
```

Relative `file` paths resolve against the config file's directory first, relative `package_dirs` entries against that directory.

## safety

The command guard checks every command against the joint limits before it is written to the robot.

| Key | Default | Meaning |
|---|---|---|
| `mode` | `reject` | `reject` fails the cycle, `clamp` clamps and writes, `off` disables the guard |
| `velocity_scale` | 1 | Scales the velocity limits the guard checks against. Modules still plan at the model limits, so to slow the robot lower `model.joint_limits.velocity` instead |
| `tolerance` | 1e-6 | Allowed overshoot before a limit counts as violated |

## solver

| Key | Default | Meaning |
|---|---|---|
| `mode` | `weighted` | How [task levels](modules/README.md#task-levels) become QP stages, see below |
| `damping` | 1e-3 | Added to the QP Hessian diagonal, > 0 |
| `limit_gain` | 0.5 | Fraction of the distance to a position limit usable per step, in (0, 1] |
| `feasibility_tolerance` | 1e-6 | Tolerance of the feasibility check |
| `check_feasibility` | true | Check each solution against the constraints |
| `hold_retry` | true | Retry an infeasible stage with relaxed recovery demands, then hold |

| Mode | Behavior |
|---|---|
| `weighted` | One QP; levels are ignored and all tasks are summed |
| `lexicographic` | Strict priority: lower levels never disturb higher ones. Most expensive per cycle |
| `layered_independent` | One QP per level, each moving only its own tasks; later levels compensate earlier ones |
| `layered_coupled` | One QP per level with every task up to that level; conflicts are traded off, not strict |

All solver keys can be changed at run time with `Engine.tune_solver`.

## robot

| Key | Default | Meaning |
|---|---|---|
| `type` | `loopback` | `loopback` is an ideal robot that reports the last command; `plugin` loads a [driver plugin](robot.md#driver-plugins) |
| `library` | | Plugin shared library. A path with `/` resolves against the config file, a bare name uses the loader search path |
| `params` | | Free-form map passed to the plugin's `create()` |
| `initial` | model neutral | Initial q: a list of size nq, or a map `{joint: value}` over the neutral. Also the modules' initial posture |
| `initial_root` | | Initial floating base pose `{position, rpy}` or `{position, quaternion}`. Not allowed with a list `initial` |

## modules

`modules:` maps an id to a module. `type` selects the module type and the other fields belong to that type. The section is optional. See [Modules](modules/README.md) for each type, and `kstone schema <type>` for its full JSON Schema.

## Variables and includes

Config files on disk support placeholders and includes:

```yaml
rate_hz: ${rate:-50}          # 50 unless rate is set
modules: !include modules.yaml  # path relative to this file
```

- `${name}` is looked up in the variables passed in, then the environment; `${name:-default}` falls back to the default when both are unset or empty. `$${` writes a literal `${`.
- A scalar that is exactly one placeholder is re-typed, so `${rate:-50}` becomes a number.
- Pass variables with `kstone run/check --set name=value` (repeatable), or `vars={"name": "value"}` in the Python loaders. Every variable passed must be referenced in the config, otherwise loading fails with KST-E010.
- Python's `load_*_from_string` loaders also expand placeholders; `base_dir` sets where relative paths and includes resolve.

## Naming conventions

Keys across modules follow these rules:

- Durations end in `_s` (seconds); other units are given in each module's docs.
- `max_` marks an upper bound, except inside `*_limits` groups, where every field is already a bound.
- `_scale` is a dimensionless ratio; `_factor` is a coefficient with units (e.g. `acceleration_factor`, 1/s).
- `speed` is a scalar magnitude; `velocity` is a vector, a per-axis rate or a limit on one.
- Pose quantities are `position` / `orientation`; rates and limits are `linear` / `angular`.
- Pose values are `{position, orientation}` (wxyz quaternion) on ports, and `{position, quaternion}` or `{position, rpy}` in config.
- `velocity_limit` is the per-joint limit a joint_target plans within (the solver still enforces the model limit); `max_velocity` is an extra cap on one module's own motion, such as a trajectory's cruise speed.
