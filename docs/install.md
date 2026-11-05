# Installation

[中文](zh-CN/install.md) · [README](../README.md)

## Requirements

| Item | Version |
|---|---|
| CMake | ≥ 3.22 |
| Compiler | C++17 (GCC 11 in CI) |
| Pinocchio | ≥ 4.0.0, with Coal collision support |
| Coal | ≥ 3.0.3 |
| Eigen | ≥ 3.4 |

DAQP, Ruckig, yaml-cpp and doctest are bundled as submodules, and convex hulls of collision meshes use Coal (qhull) when available, with a built-in fallback, so Coal does not need qhull. `pip install` (see the [README](../README.md#install)) takes Pinocchio and Coal from pip wheels; the sections below cover ROS and source builds.

CI tests Ubuntu 22.04 on x86_64 and aarch64 with Python 3.10 and ROS 2 Humble. Other Ubuntu versions, architectures and Python versions are not tested yet.

Jetson with JetPack 6 (Ubuntu 22.04, aarch64) uses the same steps; the pip wheels and ROS Humble apt packages exist for aarch64. CI runs on GitHub's generic arm64 runners, not on Jetson hardware. JetPack 5 is Ubuntu 20.04 and is not supported.

urdfdom-headers before 2.0, including Ubuntu 22.04 apt (1.0.5) and ROS Humble (1.0.6), has no capsule geometry. A kstone built against it rejects `<capsule>` in collision models; the pip wheels ship 3.0.

## Pip dependency versions

The extension links the Pinocchio, Coal, urdfdom, octomap, tinyxml2, console_bridge and Boost shared libraries from the pip wheels. Pinocchio minor releases are not ABI compatible with each other: a build against 4.1.0 crashes with 4.0.0 at run time, and the reverse also fails. The package therefore records the versions it was built against as runtime requirements, for example `libpinocchio>=4.1.0,==4.1.*` and `cmeel-boost==1.90.0.*`.

`pip install` builds in an isolated environment with the newest allowed versions (`libpinocchio>=4.0.0`, `libcoal>=3.0.3,<3.1`). To build against the Pinocchio already in an environment instead, install the build tools there and turn off isolation:

```bash
python3 -m pip install scikit-build-core==1.1.1 pybind11==3.1.0 "cmeel-eigen~=3.4.0" "cmeel-urdfdom-headers>=3.0"
python3 -m pip install --no-build-isolation "git+https://github.com/AgilexOpensource/kstone.git"
```

Tested combinations (C++ tests, build from the sdist and Python tests):

| Pinocchio | Coal | urdfdom | urdfdom-headers | tinyxml2 | Boost |
|---|---|---|---|---|---|
| 4.0.0 | 3.0.3 | 6.0 | 3.0 | 11 | 1.90 |

- Tested with Python 3.10, the minimum of `cmeel-boost` 1.90, whose wheels also require numpy 2. The isolated `pip install` takes the newest allowed versions, so it also covers Pinocchio 4.1.
- The `pin` Python package of the same version can be installed next to kstone and imported in the same process.

## Choosing the dependency source

| `KSTONE_PINOCCHIO_SOURCE` | Behavior |
|---|---|
| `AUTO` (default) | `SYSTEM` first; if it has no acceptable Pinocchio and Coal, `PIP` |
| `SYSTEM` | Plain `find_package`, for source or ROS installs: `CMAKE_PREFIX_PATH` (including a sourced ROS), then `/usr/local` and `/usr` |
| `PIP` | Only the `pin`/`coal` wheels of the active Python, ignoring a sourced ROS; fail if they are missing. `pip install` of kstone always uses this |

So a sourced ROS wins over a venv, and a venv is used when no usable ROS or source install is found.

- Every candidate CMake finds, including an explicit `-Dpinocchio_DIR`, is checked before it is loaded: Pinocchio must be 4.x and >= 4.0.0, Coal 3.x and >= 3.0.3. A source is used only if both pass; otherwise the next one is tried.
- Skipped candidates are printed with the reason. If no source works, configure fails with the full search list and the install commands for source, ROS and pip.
- AUTO remembers its pick in the build directory so a reconfigure does not mix the two. Delete `CMakeCache.txt` to re-detect.
- A CMake or colcon build that falls back to pip links against the venv by absolute path (DT_RPATH). Moving or deleting the venv, or changing its `pin`/`coal` versions, breaks the installed binaries; rebuild after such changes.

<a id="ros"></a>
## ROS (apt)

Tested with ROS 2 Humble, which has `ros-humble-pinocchio` 4.0.0 and `ros-humble-coal` 3.0.3.

```bash
sudo apt install ros-humble-pinocchio
source /opt/ros/humble/setup.bash

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

- The `No header defined for python310` warning at configure time comes from ROS eigenpy and can be ignored.

### In a colcon workspace

The [package.xml](../package.xml) declares kstone as a `cmake` build type package that depends on `pinocchio` and `eigen`, so `rosdep` installs them and colcon builds kstone like any other package. The library does not depend on ROS. The [README](../README.md#ros-2) has the commands. A package that uses kstone adds:

```xml
<depend>kstone</depend>
```

```cmake
find_package(kstone REQUIRED)
target_link_libraries(my_node PRIVATE kstone::kstone)
```

The `ros` job of the [ci](../.github/workflows/ci.yml) workflow runs `colcon build` and `colcon test` in the `ros:humble` image, then builds [tests/consumer](../tests/consumer) against the install space.

<a id="source"></a>
## Building from source

The `source` job of the [ci](../.github/workflows/ci.yml) workflow runs the [README](../README.md#c) recipe (Coal 3.0.3, Pinocchio 4.1.0) on Ubuntu 22.04 with apt Boost, Eigen and urdfdom.

Boost, Eigen, urdfdom and assimp (needed by Coal) are required, for example `libboost-all-dev libeigen3-dev liburdfdom-dev libassimp-dev`, or the urdfdom shipped with ROS.

- If urdfdom comes from ROS, pass `-DCMAKE_PREFIX_PATH="$PREFIX;/opt/ros/humble"` when configuring Pinocchio and kstone.

Linking kstone into your own CMake project is described in [C++](cpp.md#using-kstone-from-your-cmake-project).

## Environment check

`kstone doctor` (`--json` for JSON) or `kstone.doctor(print=True)` in Python reports:

- The kstone version and build type, and the Eigen, Pinocchio and Coal versions it was compiled against.
- The native libraries loaded in the process and their source (pypi, ros, system or other).
- The result of a solve and collision-distance smoke test.

The check fails if one library is loaded twice, or if Pinocchio and Coal come from different sources. The CLI then exits with 1, and the dict returned in Python has `ok` set to `False`. The loaded-library scan works on Linux only.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `Pinocchio/Coal not found` | Not found with the current source. Install one of the ways above, or point `CMAKE_PREFIX_PATH` at the install prefix |
| `kstone needs Pinocchio >= 4.0.0` | The Pinocchio found is older. Install 4.x one of the ways above |
| pip reports a conflict on `libpinocchio` after installing kstone | kstone requires the Pinocchio minor version it was built against. Rebuild kstone with `--no-build-isolation` in the target environment (see [Pip dependency versions](#pip-dependency-versions)) |
| Crashes or odd behavior, possibly from mixing pip and ROS libraries | Run `kstone doctor` and check the loaded libraries and the `libraries` check |
