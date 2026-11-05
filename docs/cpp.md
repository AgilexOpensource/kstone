# C++

[中文](zh-CN/cpp.md) · [README](../README.md)

## Build from source

By default CMake finds Pinocchio, Coal and Eigen with plain `find_package` (point `CMAKE_PREFIX_PATH` at their prefix, or source ROS), and falls back to the pip wheels of the active Python when those are missing or too old. Building them from source and ROS apt are covered in [Installation](install.md). The example below takes them from pip wheels in a virtual environment; `-DKSTONE_PINOCCHIO_SOURCE=PIP` makes that explicit.

```bash
git clone --recursive https://github.com/AgilexOpensource/kstone.git
cd kstone

python3 -m venv .venv
source .venv/bin/activate
pip install pin coal cmeel-eigen cmeel-urdfdom-headers ninja

cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DKSTONE_PINOCCHIO_SOURCE=PIP
cmake --build build
ctest --test-dir build
```

Configure prints where the dependencies came from, for example:

```
-- kstone: Eigen 3.4.1, Pinocchio 4.1.0, Coal 3.0.3 from pip wheels (...)
```

If the clone has no submodules, run `git submodule update --init --recursive`.

## CMake options

| Option | Default | Effect |
|---|---|---|
| `KSTONE_PINOCCHIO_SOURCE` | `AUTO` | Where to find Pinocchio and Coal, see [Installation](install.md#choosing-the-dependency-source) |
| `KSTONE_BUILD_TESTS` | `ON` as top-level project | Unit tests |
| `KSTONE_BUILD_TOOLS` | `ON` as top-level project | The `kstone` CLI |
| `KSTONE_BUILD_EXAMPLES` | `ON` as top-level project | The examples in [examples](../examples) |
| `KSTONE_BUILD_BENCH` | `OFF` | Benchmarks, see [Development](development.md#benchmarks) |
| `KSTONE_BUILD_PYTHON` | `OFF` | Python extension, needs pybind11 |
| `BUILD_SHARED_LIBS` | `OFF` | Build `libkstone` as a shared library |

## Examples

| Example | Shows | Run from the repository root |
|---|---|---|
| [cpp/ik.cpp](../examples/cpp/ik.cpp) | Offline IK with `Solver` and `FrameTask`, checked with `Workspace` | `build/examples/kstone_example_ik examples/omni_dual_arm/omni_dual_arm.urdf` |
| [cpp/engine.cpp](../examples/cpp/engine.cpp) | An engine in a host loop with a custom `RobotInterface` | `build/examples/kstone_example_engine tests/fixtures/arm.urdf` |
| [robot_plugin/example_robot.c](../examples/robot_plugin/example_robot.c) | A driver plugin implementing the C ABI | See [Robot integration](robot.md#driver-plugins) |

The `kstone` CLI runs a config without writing code: `build/tools/kstone run <config.yaml>`, see [Robot integration](robot.md#running-a-config).

## Using kstone from your CMake project

After `cmake --install`, `find_package(kstone)` also looks up Pinocchio, so `CMAKE_PREFIX_PATH` must contain both the kstone prefix and the Pinocchio prefix:

```cmake
find_package(kstone REQUIRED)
target_link_libraries(my_app PRIVATE kstone::kstone)
```

```bash
# pip dependencies: Pinocchio lives under cmeel.prefix of the Python environment
PIN_PREFIX=$(python -c "import cmeel; print(cmeel.__path__[0] + '.prefix')")
cmake -S . -B build -DCMAKE_PREFIX_PATH="$KSTONE_PREFIX;$PIN_PREFIX"
```

For ROS or source installs, replace `$PIN_PREFIX` with that prefix, such as `/opt/ros/humble` or `$HOME/opt/pin`.

- A version passed to `find_package(kstone <version>)` matches only the same major.minor version.
- The library is static by default. With `-DBUILD_SHARED_LIBS=ON` it exports only the `kstone::` namespace and the `kstone_engine_*` C functions, its SOVERSION is `major.minor` while the major version is 0 and `major` from 1.0 on, and a `kstone.pc` pkg-config file is installed as well.

[tests/consumer](../tests/consumer) is a minimal out-of-tree project built this way.

## API

[C++ API](api.md) lists the headers by layer, the callback interfaces and the versioning rules. [Configuration](config.md) and [Modules](modules/README.md) document the config format, which C++ and Python share.
