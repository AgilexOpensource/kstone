# 安装

[English](../install.md) · [README](../../README.zh-CN.md)

## 环境要求

| 项 | 版本 |
|---|---|
| CMake | ≥ 3.22 |
| 编译器 | C++17（CI 使用 GCC 11） |
| Pinocchio | ≥ 4.0.0，需带 Coal 碰撞支持 |
| Coal | ≥ 3.0.3 |
| Eigen | ≥ 3.4 |

DAQP、Ruckig、yaml-cpp、doctest 以子模块形式内置；碰撞网格的凸包优先用 Coal（qhull）计算，不可用时用内置实现兜底，所以 Coal 不需要 qhull。`pip install`（见 [README](../../README.zh-CN.md#安装)）从 pip wheel 获取 Pinocchio 和 Coal；下面几节介绍 ROS 和源码编译。

CI 只测试 Ubuntu 22.04 x86_64 和 aarch64、Python 3.10 和 ROS 2 Humble，其他 Ubuntu 版本、架构和 Python 版本暂未测试。

JetPack 6 的 Jetson（Ubuntu 22.04，aarch64）按同样步骤安装，pip wheel 和 ROS Humble 的 apt 包都有 aarch64 版本。CI 跑在 GitHub 的通用 arm64 机器上，不是 Jetson 实机。JetPack 5 是 Ubuntu 20.04，不支持。

2.0 之前的 urdfdom-headers（包括 Ubuntu 22.04 apt 的 1.0.5 和 ROS Humble 的 1.0.6）没有胶囊体几何，基于它构建的 kstone 不接受碰撞模型里的 `<capsule>`；pip wheel 带的是 3.0。

<a id="pip-dependency-versions"></a>
## pip 依赖版本

扩展模块直接链接 pip wheel 里的 Pinocchio、Coal、urdfdom、octomap、tinyxml2、console_bridge 和 Boost 共享库。Pinocchio 不同 minor 版本之间 ABI 不兼容：基于 4.1.0 构建的包在 4.0.0 下运行会崩溃，反过来也一样。因此包会把构建时的版本记为运行时依赖，例如 `libpinocchio>=4.1.0,==4.1.*` 和 `cmeel-boost==1.90.0.*`。

`pip install` 在隔离环境里构建，使用允许范围内的最新版本（`libpinocchio>=4.0.0`、`libcoal>=3.0.3,<3.1`）。如果要基于环境里已有的 Pinocchio 构建，先在该环境装好构建工具，再关闭隔离：

```bash
python3 -m pip install scikit-build-core==1.1.1 pybind11==3.1.0 "cmeel-eigen~=3.4.0" "cmeel-urdfdom-headers>=3.0"
python3 -m pip install --no-build-isolation "git+https://github.com/AgilexOpensource/kstone.git"
```

已测试的组合（C++ 测试、从 sdist 构建、Python 测试）：

| Pinocchio | Coal | urdfdom | urdfdom-headers | tinyxml2 | Boost |
|---|---|---|---|---|---|
| 4.0.0 | 3.0.3 | 6.0 | 3.0 | 11 | 1.90 |

- 用 Python 3.10 测试，这是 `cmeel-boost` 1.90 的下限，它的 wheel 还要求 numpy 2。隔离 `pip install` 会取允许范围内的最新版本，因此也覆盖了 Pinocchio 4.1。
- 同版本的 `pin` Python 包可以和 kstone 装在一起，并在同一进程里导入。

<a id="choosing-the-dependency-source"></a>
## 选择依赖来源

| `KSTONE_PINOCCHIO_SOURCE` | 行为 |
|---|---|
| `AUTO`（默认） | 先按 `SYSTEM` 找，没有合格的 Pinocchio 和 Coal 再用 `PIP` |
| `SYSTEM` | 普通 `find_package`，适用于源码、ROS 安装：先 `CMAKE_PREFIX_PATH`（含 source 过的 ROS），再 `/usr/local`、`/usr` |
| `PIP` | 只用当前 Python 里的 `pin`/`coal` wheel，忽略 source 过的 ROS，找不到直接报错。`pip install` kstone 时总是用这个 |

也就是说 source 了 ROS 时优先用 ROS，没有可用的 ROS 或源码安装时用 venv 里的 wheel。

- CMake 找到的每个候选（包括显式的 `-Dpinocchio_DIR`）都在加载前检查版本：Pinocchio 必须是 4.x 且 >= 4.0.0，Coal 必须是 3.x 且 >= 3.0.3。同一来源两者都合格才用，否则换下一个来源。
- 被跳过的候选会连同原因打印出来。所有来源都不行时，配置失败并列出完整查找过程，以及源码、ROS、pip 三种安装命令。
- AUTO 会把选中的来源记在构建目录里，重新配置时不会混用两种来源。要重新检测就删掉 `CMakeCache.txt`。
- CMake 或 colcon 构建回退到 pip 时，会按绝对路径（DT_RPATH）链接 venv 里的库。移动、删除 venv 或改动其中 `pin`/`coal` 的版本，都会让已安装的程序无法运行，改动后需要重新构建。

<a id="ros"></a>
## ROS（apt）

已测试 ROS 2 Humble： `ros-humble-pinocchio` 4.0.0、`ros-humble-coal` 3.0.3。

```bash
sudo apt install ros-humble-pinocchio
source /opt/ros/humble/setup.bash

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

- 配置时的 `No header defined for python310` 警告来自 ROS 的 eigenpy，可忽略。

### 在 colcon 工作区中

[package.xml](../../package.xml) 把 kstone 声明为 `cmake` 构建类型的包，依赖 `pinocchio` 和 `eigen`，因此 `rosdep` 会装好依赖，colcon 像构建其他包一样构建 kstone。库本身不依赖 ROS。命令见 [README](../../README.zh-CN.md#ros-2)。使用 kstone 的包加上：

```xml
<depend>kstone</depend>
```

```cmake
find_package(kstone REQUIRED)
target_link_libraries(my_node PRIVATE kstone::kstone)
```

[ci](../../.github/workflows/ci.yml) 工作流的 `ros` job 在 `ros:humble` 镜像里运行 `colcon build` 和 `colcon test`，再基于 install 空间构建 [tests/consumer](../../tests/consumer)。

<a id="source"></a>
## 源码编译

[ci](../../.github/workflows/ci.yml) 工作流的 `source` job 在 Ubuntu 22.04 上用 apt 的 Boost、Eigen、urdfdom 运行 [README](../../README.zh-CN.md#c) 里的步骤（Coal 3.0.3、Pinocchio 4.1.0）。

需要 Boost、Eigen、urdfdom 和 assimp（Coal 依赖），如 `libboost-all-dev libeigen3-dev liburdfdom-dev libassimp-dev`，或使用 ROS 自带的 urdfdom。

- urdfdom 来自 ROS 时，配置 Pinocchio 和 kstone 时传 `-DCMAKE_PREFIX_PATH="$PREFIX;/opt/ros/humble"`。

在自己的 CMake 项目里链接 kstone 见 [C++](cpp.md#using-kstone-from-your-cmake-project)。

<a id="environment-check"></a>
## 环境自检

`kstone doctor`（加 `--json` 输出 JSON）或 Python 里的 `kstone.doctor(print=True)` 报告：

- kstone 版本、构建类型，以及编译时的 Eigen、Pinocchio、Coal 版本。
- 进程里加载的原生库及其来源（pypi、ros、system、other）。
- 一次求解和碰撞距离冒烟测试的结果。

同一个库被加载两份、或 Pinocchio 与 Coal 来自不同来源时检查失败：CLI 退出码为 1，Python 返回的 dict 里 `ok` 为 `False`。加载库扫描只在 Linux 上可用。

## 常见问题

| 现象 | 原因与处理 |
|---|---|
| `Pinocchio/Coal not found` | 当前来源下没找到。按上文任选一种安装方式，或用 `CMAKE_PREFIX_PATH` 指向安装目录 |
| `kstone needs Pinocchio >= 4.0.0` | 找到的 Pinocchio 版本过旧，按上面任一方式安装 4.x |
| 装 kstone 后 pip 报 `libpinocchio` 冲突 | kstone 要求与构建时相同的 Pinocchio minor 版本。在目标环境里用 `--no-build-isolation` 重新构建（见 [pip 依赖版本](#pip-dependency-versions)） |
| 崩溃或行为异常，怀疑混用了 pip 和 ROS 的库 | 运行 `kstone doctor`，查看加载的库和 `libraries` 检查 |
