# Configuration

[中文](zh-CN/config.md) · [README](../README.md)

An engine config is one YAML file. `kstone check <config.yaml>` builds the engine without running it and reports the first error with its line and column. Unknown keys are errors. Minimal example, from [arm_loopback.yaml](../tests/fixtures/arm_loopback.yaml):

```yaml
model:
  file: arm.urdf
rate_hz: 100
realtime: false
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
| `realtime` | true | Pace cycles with the clock; `kstone run --realtime` forces it on |
| `feedback_timeout_s` | 0.25 | Feedback that is older or has no timestamp is stale; 0 = never stale. See [run contract](robot.md#run-contract) |
| `max_failed_cycles` | 5 | `run()` returns after this many consecutive failed cycles |
| `feedback_joints` | all scalar joints | Joints whose measured position feeds the closed-loop estimate (collision, CoM, homing). Names or fnmatch patterns |
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
| `feasibility_tol` | 1e-6 | Tolerance of the feasibility check |
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
| `initial_root` | | Initial floating base pose `{translation, rpy}` or `{translation, quaternion}`. Not allowed with a list `initial` |

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
