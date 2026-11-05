# kstone

[中文](README.zh-CN.md)

kstone solves inverse kinematics for robots and runs it as a control loop. You describe the robot and what it should do in a YAML file, and kstone turns targets (a hand pose, joint angles, a gaze point) into joint commands while respecting joint limits and avoiding collisions.

It is a C++17 library with a Python package, a `kstone` command-line tool and a separate web viewer, [kstone-viewer](https://github.com/AgilexOpensource/kstone_viewer), for testing.

## Install

kstone is installed from source and needs a C++17 compiler. CI tests Ubuntu 22.04 on x86_64 and aarch64 (such as Jetson with JetPack 6) with Python 3.10 and ROS 2 Humble; other Linux setups are not tested yet.

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
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

A package in the same workspace adds `<depend>kstone</depend>` and uses `find_package(kstone)`, see [Installation](docs/install.md#ros).

### C++

Build Pinocchio and Coal from source, then kstone, all into one prefix:

```bash
sudo apt install build-essential cmake git libboost-all-dev libeigen3-dev liburdfdom-dev libassimp-dev
PREFIX=$HOME/opt/kstone

git clone --recursive --depth 1 --branch v3.0.3 https://github.com/coal-library/coal.git
cmake -S coal -B coal/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX \
      -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_TESTING=OFF -DCOAL_HAS_QHULL=OFF
cmake --build coal/build -j"$(nproc)" && cmake --install coal/build

git clone --recursive --depth 1 --branch v4.1.0 https://github.com/stack-of-tasks/pinocchio.git
cmake -S pinocchio -B pinocchio/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX \
      -DCMAKE_PREFIX_PATH=$PREFIX -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_TESTING=OFF \
      -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=ON
cmake --build pinocchio/build -j"$(nproc)" && cmake --install pinocchio/build

git clone --recursive https://github.com/AgilexOpensource/kstone.git
cmake -S kstone -B kstone/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$PREFIX -DCMAKE_INSTALL_PREFIX=$PREFIX
cmake --build kstone/build -j"$(nproc)" && cmake --install kstone/build
```

Shortcuts that skip building Pinocchio and Coal:

- ROS apt: `sudo apt install ros-humble-pinocchio`, `source /opt/ros/humble/setup.bash`, then build kstone with CMake as above.
- pip wheels: `pip install pin coal cmeel-eigen cmeel-urdfdom-headers` in a virtual environment, then build kstone with CMake from that venv; it falls back to the wheels when no ROS or source install is found. Full commands are in [C++](docs/cpp.md#build-from-source).

Details and tested versions are in [Installation](docs/install.md). Linking from your project, CMake options and the C++ API are in [C++](docs/cpp.md).

### Running the examples

The examples below use files from the repository. Clone it to run them:

```bash
git clone --recursive https://github.com/AgilexOpensource/kstone.git
cd kstone
python3 -m pip install .   # or skip this if kstone is already installed
```

## A first look

Solve IK without a robot. The config file names a URDF and the modules to use; here `arm` is a `joint_target` module and `tool` a `frame_task`:

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
engine.run(duration_s=3.0)  # 3 s of engine time; this config has realtime: false, so it returns at once
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
