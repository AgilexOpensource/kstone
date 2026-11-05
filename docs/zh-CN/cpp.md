# C++

[English](../cpp.md) · [README](../../README.zh-CN.md)

<a id="build-from-source"></a>
## 从源码构建

默认情况下 CMake 用普通的 `find_package` 查找 Pinocchio、Coal 和 Eigen（用 `CMAKE_PREFIX_PATH` 指向安装前缀，或 source ROS），找不到或版本不合格时再用当前 Python 里的 pip wheel。源码编译和 ROS apt 的装法见[安装](install.md)。下面的例子用虚拟环境里的 pip wheel，`-DKSTONE_PINOCCHIO_SOURCE=PIP` 用来显式指定。

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

配置时会打印依赖来源，例如：

```
-- kstone: Eigen 3.4.1, Pinocchio 4.1.0, Coal 3.0.3 from pip wheels (...)
```

克隆时没带子模块的，执行 `git submodule update --init --recursive`。

## CMake 选项

| 选项 | 默认 | 作用 |
|---|---|---|
| `KSTONE_PINOCCHIO_SOURCE` | `AUTO` | Pinocchio 和 Coal 的来源，见[安装](install.md#choosing-the-dependency-source) |
| `KSTONE_BUILD_TESTS` | 作为顶层项目时 `ON` | 单元测试 |
| `KSTONE_BUILD_TOOLS` | 作为顶层项目时 `ON` | `kstone` 命令行工具 |
| `KSTONE_BUILD_EXAMPLES` | 作为顶层项目时 `ON` | [examples](../../examples) 中的示例 |
| `KSTONE_BUILD_BENCH` | `OFF` | 基准测试，见[开发](development.md#benchmarks) |
| `KSTONE_BUILD_PYTHON` | `OFF` | Python 扩展，需要 pybind11 |
| `BUILD_SHARED_LIBS` | `OFF` | 把 `libkstone` 编成共享库 |

## 示例

| 示例 | 内容 | 在仓库根目录运行 |
|---|---|---|
| [cpp/ik.cpp](../../examples/cpp/ik.cpp) | 用 `Solver` 和 `FrameTask` 离线求 IK，用 `Workspace` 校验 | `build/examples/kstone_example_ik examples/omni_dual_arm/omni_dual_arm.urdf` |
| [cpp/engine.cpp](../../examples/cpp/engine.cpp) | 在宿主循环里运行引擎，接自定义 `RobotInterface` | `build/examples/kstone_example_engine tests/fixtures/arm.urdf` |
| [robot_plugin/example_robot.c](../../examples/robot_plugin/example_robot.c) | 实现 C ABI 的驱动插件 | 见[接入机器人](robot.md#driver-plugins) |

不写代码也能用 `kstone` 命令行运行配置：`build/tools/kstone run <config.yaml>`，见[接入机器人](robot.md#running-a-config)。

<a id="using-kstone-from-your-cmake-project"></a>
## 在自己的 CMake 项目里使用

`cmake --install` 之后，`find_package(kstone)` 会连带查找 Pinocchio，所以 `CMAKE_PREFIX_PATH` 要同时包含 kstone 和 Pinocchio 的安装目录：

```cmake
find_package(kstone REQUIRED)
target_link_libraries(my_app PRIVATE kstone::kstone)
```

```bash
# pip 依赖：Pinocchio 在当前 Python 环境的 cmeel.prefix 下
PIN_PREFIX=$(python -c "import cmeel; print(cmeel.__path__[0] + '.prefix')")
cmake -S . -B build -DCMAKE_PREFIX_PATH="$KSTONE_PREFIX;$PIN_PREFIX"
```

ROS、源码安装时，把 `$PIN_PREFIX` 换成对应目录，如 `/opt/ros/humble`、`$HOME/opt/pin`。

- `find_package(kstone <版本>)` 只匹配相同的 major.minor 版本。
- 默认编成静态库。加 `-DBUILD_SHARED_LIBS=ON` 时只导出 `kstone::` 命名空间和 `kstone_engine_*` C 函数；major 为 0 时 SOVERSION 是 `major.minor`，从 1.0 起是 `major`；另外安装 pkg-config 文件 `kstone.pc`。

[tests/consumer](../../tests/consumer) 是按这种方式构建的最小外部项目。

## API

[C++ API](api.md) 按层列出头文件，并说明回调接口和版本规则。配置格式见[配置](config.md)和[模块](modules/README.md)，C++ 和 Python 共用。
