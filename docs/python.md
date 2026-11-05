# Python guide

[中文](zh-CN/python.md) · [README](../README.md)

Install steps are in the [README](../README.md#install). The snippets below run from the repository root and use [arm_loopback.yaml](../tests/fixtures/arm_loopback.yaml), a small arm with a two-finger gripper.

Every class and function has a docstring and type hints, so `help(kstone.Engine)` and IDE completion work.

## Concepts

- **Config**: a YAML file with the robot model (`model:`), the robot to drive (`robot:`), and a list of **modules** (`modules:`). See [Configuration](config.md).
- **Module**: one goal or limit, such as "move this frame to a pose" (`frame_task`) or "move these joints to angles" (`joint_target`). You command a module by writing to its **ports**.
- **Solver**: computes the next joint configuration from the current one and the module targets. No robot, no timing.
- **Engine**: a Solver plus a robot and a fixed-rate loop. Each cycle it reads the robot state, solves and sends the command.
- **q** and **v**: joint positions (size `model.nq`) and joint velocities (size `model.nv`). They differ when the robot has continuous joints or a floating base, so look indices up by joint name with `model.q_index(name)` and `model.v_index(name)`.

## Solve IK offline

```python
import kstone

solver = kstone.load_solver("tests/fixtures/arm_loopback.yaml")
print(solver.modules)          # ['tool', 'arm', 'fingers']
print(solver.module("arm").ports)

solver.module("arm").write_port("target", [0.5, -0.3, 0.2])
q = solver.initial
for _ in range(300):
    result = solver.solve(q, 0.01)
    q = result.q_next
```

`solve(q, dt)` moves one step of `dt` seconds. Modules approach their targets over several steps, at the speed set by their `gain` and `trajectory` parameters. The returned `SolveResult` has:

| Field | Meaning |
|---|---|
| `q_next`, `v` | Next configuration and the joint velocity that leads there |
| `quality` | `"nominal"`, `"relaxed"`, `"degraded"` or `"failed"` |
| `residuals` | Remaining error of each task |
| `held` | `True` if the step failed and the robot is held in place |

A pose target is a 4×4 matrix or `{"translation": [x, y, z], "rotation": [w, x, y, z]}`:

```python
ws = kstone.Workspace(solver.model)
ws.update(q)
target = ws.frame_pose("tool")    # 4x4, in the world frame
target[:3, 3] += [0.0, 0.05, 0.0]
solver.module("tool").write_port("target", target)
```

[examples/python/ik.py](../examples/python/ik.py) puts this together with an inline config.

## Run an engine

```python
engine = kstone.load_engine("tests/fixtures/arm_loopback.yaml")
engine.module("arm").write_port("target", [0.5, -0.3, 0.2])

engine.run(duration_s=3.0)        # or call engine.step() once per cycle
print(engine.q, engine.quality)
print(engine.stats["cycles"], engine.stats["failed_cycles"])
```

- `run()` blocks until `duration_s` passes or `stop()` is called from another thread. Without a duration it runs until stopped.
- With `realtime: false` in the config, `run()` advances simulated time as fast as it can; with `realtime: true` it follows the wall clock.
- `engine.q` is the commanded configuration and `engine.measured` the last one read from the robot.
- `engine.snapshot()` returns everything at once as a dict.

To drive real hardware, change the `robot:` section of the config. See [Robot integration](robot.md).

## Load a config from a string

`load_solver_from_string` and `load_engine_from_string` take YAML text. Relative paths inside it, such as the URDF, resolve against `base_dir`:

```python
config = """
model: {file: arm.urdf}
modules:
  arm: {type: joint_target, joints: [j1, j2]}
"""
solver = kstone.load_solver_from_string(config, base_dir="tests/fixtures")
```

All four loaders accept `vars={"name": "value"}` to fill `${name}` placeholders in the config, see [variables and includes](config.md#variables-and-includes).

## Query the model

```python
model = kstone.Model.from_file("tests/fixtures/arm.urdf")
print(model.nq, model.nv, model.joint_names)

ws = kstone.Workspace(model)
ws.update(model.neutral())
ws.frame_pose("tool")             # 4x4 pose in the world frame
ws.frame_jacobian("tool")         # 6 x nv, linear rows first
ws.center_of_mass()               # whole robot
```

Frames and joints are given by name or by id (`model.frame_id(name)`, `model.joint_id(name)`). Call `update(q)` again after `q` changes.

## Tune at run time

```python
tool = engine.module("tool")
print(tool.tune_schema["properties"].keys())
tool.tune({"gain": 0.8})
print(tool.params()["gain"])

engine.tune_solver({"damping": 1e-2})
engine.tune_joint_limits({"velocity": {"j1": 0.5}})
engine.tune_joint_limits({"reset": True})
```

Modules can also be switched off with `module.enabled = False`.

## Watch it in the browser

`engine.serve()` or `kstone run --serve` starts the debug server, and [kstone-viewer](https://github.com/AgilexOpensource/kstone_viewer) connects to it. The viewer is a separate package for smoke tests:

```bash
pip install "git+https://github.com/AgilexOpensource/kstone_viewer.git"
kstone run tests/fixtures/arm_loopback.yaml --realtime --serve 7311
# in a second terminal
kstone-viewer --connect 127.0.0.1:7311
```

The viewer prints the URL to open. The [debug server](robot.md#debug-server) section covers tokens and remote access.

## Errors

kstone raises `kstone.Error`. Its `code` says what went wrong:

```python
try:
    engine.module("nope")
except kstone.Error as e:
    print(e.code, e)              # not_found ...
```

Common codes are `not_found`, `invalid_argument`, `out_of_range`, `parse_error` and `io_error`. A numeric id that is too large raises `IndexError`.

## Not available in Python

Robot drivers are written in C or C++ (see [Robot integration](robot.md#driver-plugins)). The collision world and telemetry sinks are C++ only. `kstone.doctor(print=True)` checks the installation, see [Installation](install.md#environment-check).
