# kstone

[中文](README.zh-CN.md)

kstone solves inverse kinematics for robots and runs it as a control loop. You describe the robot and what it should do in a YAML file, and kstone turns targets (a hand pose, joint angles, a gaze point) into joint commands while respecting joint limits and avoiding collisions.

It is a C++17 library with a Python package, a `kstone` command-line tool and a separate web viewer, [kstone-viewer](https://github.com/AgilexOpensource/kstone_viewer), for testing.

## Install

kstone is installed from source and needs a C++17 compiler. CI tests Ubuntu 22.04 on x86_64 and aarch64 (such as Jetson with JetPack 6) with Python 3.10 and ROS 2 Humble; other Linux setups are not tested.

### Python

Python 3.10 or later. kstone is not on PyPI; pip builds it from GitHub:

```bash
python3 -m pip install "git+https://github.com/AgilexOpensource/kstone.git"
kstone doctor
```

pip clones the repository with its submodules, builds the package (if CMake ≥ 3.22 is missing, pip downloads it) and pulls Pinocchio and Coal as pip wheels.

- Virtual environment: create and activate one first (`python3 -m venv .venv && source .venv/bin/activate`).
- System Python: run the command as is. On Python installs marked externally managed (Debian 12, Ubuntu 23.04 and later), pip refuses this; use a virtual environment there.

### ROS 2

kstone is a plain CMake package with a `package.xml`, so colcon builds it in a workspace and Pinocchio comes from the ROS apt repository:

```bash
mkdir -p ~/kstone_ws/src && cd ~/kstone_ws/src
git clone --recursive https://github.com/AgilexOpensource/kstone.git
cd ~/kstone_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

A package in the same workspace adds `<depend>kstone</depend>` and uses `find_package(kstone)`, see [Installation](docs/install.md#ros).

### C++

Build Coal and Pinocchio from source, then kstone, into one prefix with the commands in [Building from source](docs/install.md#source). With ROS 2 Humble, `ros-humble-pinocchio` from apt replaces the source builds, see [ROS (apt)](docs/install.md#ros). Linking from your project, CMake options and the C++ API are in [C++](docs/cpp.md).

### Running the examples

The examples below use files from the repository. Clone it to run them:

```bash
git clone --recursive https://github.com/AgilexOpensource/kstone.git
cd kstone
python3 -m pip install .   # or skip this if kstone is already installed
```

## A first look

Solve IK without a robot. The config file names a URDF and the modules to use; here `arm` is a `joint_target` module and `tool` a `frame_target`:

```python
import kstone

solver = kstone.load_solver("tests/fixtures/arm_loopback.yaml")
solver.module("arm").write_port("target", [0.5, -0.3, 0.2])  # angles for j1, j2, j3

q = solver.initial
for _ in range(300):
    q = solver.solve(q, 0.01).q_next  # one 10 ms step

print(q[solver.model.q_index("j1")])  # close to 0.5
```

Run the same config as a control loop. The config's `robot:` section says which robot to drive; `loopback` is a simulated robot that follows every command exactly:

```python
engine = kstone.load_engine("tests/fixtures/arm_loopback.yaml")
engine.module("arm").write_port("target", [0.5, -0.3, 0.2])
engine.run(duration_s=3.0)  # blocks for 3 s of wall-clock time
print(engine.stats["cycles"], engine.quality)
```

The [Python guide](docs/python.md) goes through these steps in more detail.

## Examples

| Example | Shows |
|---|---|
| [python/ik.py](examples/python/ik.py) | Move a tool frame 5 cm and check the result with `Workspace` |
| [python/engine.py](examples/python/engine.py) | Load an engine, command it, optionally serve it to the viewer |
| [omni_dual_arm](examples/omni_dual_arm) | A mobile dual-arm robot whose config uses every module type |

```bash
python examples/python/ik.py tests/fixtures/arm.urdf
python examples/python/engine.py tests/fixtures/arm_loopback.yaml
```

To watch a robot in the browser, see [Debug server and viewer](docs/python.md#watch-it-in-the-browser).

## Documentation

| Page | For |
|---|---|
| [Python guide](docs/python.md) | Solving, running an engine, model queries, tuning, errors |
| [Configuration](docs/config.md) | Config file layout: model, solver, safety, robot, variables and includes |
| [Modules](docs/modules/README.md) | Every module type with its ports and parameters |
| [Robot integration](docs/robot.md) | Connecting real hardware: driver plugins, real-time settings, debug server |
| [Debug protocol](docs/debug-protocol.md) | Writing a client for the debug server |
| [C++](docs/cpp.md) | Building with CMake, linking kstone |
| [C++ API](docs/api.md) | Headers by layer, callback interfaces, versioning |
| [Installation](docs/install.md) | Dependency sources (pip, ROS, source), `kstone doctor`, troubleshooting |
| [Development](docs/development.md) | Tests, sanitizers, benchmarks, CI and releases |
| [Changelog](CHANGELOG.md) | Changes in each release |

## License

[Apache License 2.0](LICENSE)
