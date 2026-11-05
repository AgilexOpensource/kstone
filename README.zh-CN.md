# kstone

[English](README.md)

kstone 为机器人求解逆运动学，并把它作为控制循环运行。在一份 YAML 文件里描述机器人和要完成的动作，kstone 就会把目标（手的位姿、关节角、注视点）转成关节命令，同时遵守关节限位并避开碰撞。

它是一个 C++17 库，附带 Python 包、`kstone` 命令行工具，另有用于测试的网页 viewer [kstone-viewer](https://github.com/AgilexOpensource/kstone_viewer/blob/main/README.zh-CN.md)。

## 安装

kstone 从源码安装，需要 C++17 编译器。CI 只测试 Ubuntu 22.04 x86_64 和 aarch64（如 JetPack 6 的 Jetson）、Python 3.10 和 ROS 2 Humble，其他 Linux 环境暂未测试。

### Python

需要 Python 3.10 及以上。kstone 没有发布到 PyPI，用 pip 从 GitHub 构建安装：

```bash
python3 -m pip install "git+https://github.com/AgilexOpensource/kstone.git"
kstone doctor
```

pip 会连同子模块克隆仓库并编译（没有 CMake ≥ 3.22 时 pip 会自动下载），同时以 pip wheel 的形式装好 Pinocchio 和 Coal。

- 虚拟环境：先创建并激活（`python3 -m venv .venv && source .venv/bin/activate`）。
- 系统 Python：直接运行上面的命令。被标记为 externally managed 的系统（Debian 12、Ubuntu 23.04 及以后）上 pip 会拒绝安装，请改用虚拟环境。

### ROS 2

kstone 是带 `package.xml` 的普通 CMake 包，可以放进工作区用 colcon 构建，Pinocchio 来自 ROS 的 apt 源：

```bash
mkdir -p ~/kstone_ws/src && cd ~/kstone_ws/src
git clone --recursive https://github.com/AgilexOpensource/kstone.git
cd ~/kstone_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

同一工作区里的包在 `package.xml` 中加 `<depend>kstone</depend>`，CMake 里用 `find_package(kstone)`，详见[安装](docs/zh-CN/install.md#ros)。

### C++

从源码构建 Pinocchio 和 Coal，再构建 kstone，都装到同一个前缀：

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

不想编译 Pinocchio 和 Coal 时可以用这两种快捷方式：

- ROS apt：`sudo apt install ros-humble-pinocchio`，`source /opt/ros/humble/setup.bash`，再按上面的命令用 CMake 构建 kstone。
- pip wheel：在虚拟环境里 `pip install pin coal cmeel-eigen cmeel-urdfdom-headers`，在该 venv 里用 CMake 构建 kstone；没有 ROS 或源码安装时会自动用这些 wheel。完整命令见 [C++](docs/zh-CN/cpp.md#build-from-source)。

细节和测过的版本见[安装](docs/zh-CN/install.md)。在自己的项目里链接、CMake 选项和 C++ API 见 [C++](docs/zh-CN/cpp.md)。

### 运行示例

下面的示例用到仓库里的文件，运行前先克隆：

```bash
git clone --recursive https://github.com/AgilexOpensource/kstone.git
cd kstone
python3 -m pip install .   # 已经装过 kstone 可跳过
```

## 上手

不接机器人，直接求 IK。配置文件指定 URDF 和要用的模块，这里 `arm` 是 `joint_target` 模块，`tool` 是 `frame_task` 模块：

```python
import kstone

solver = kstone.load_solver("tests/fixtures/arm_loopback.yaml")
solver.module("arm").write_port("target", [0.5, -0.3, 0.2])  # j1、j2、j3 的目标角度

q = solver.initial
for _ in range(300):
    q = solver.solve(q, 0.01).q_next  # 每步 10 ms

print(q[solver.model.q_index("j1")])  # 接近 0.5
```

把同一份配置作为控制循环运行。配置里的 `robot:` 指定要驱动的机器人，`loopback` 是一个精确执行每条命令的模拟机器人：

```python
engine = kstone.load_engine("tests/fixtures/arm_loopback.yaml")
engine.module("arm").write_port("target", [0.5, -0.3, 0.2])
engine.run(duration_s=3.0)  # 引擎时间 3 s；这份配置是 realtime: false，所以会立即返回
print(engine.stats["cycles"], engine.quality)
```

[Python 指南](docs/zh-CN/python.md)会更详细地讲解这些步骤。

## 示例

| 示例 | 内容 |
|---|---|
| [python/ik.py](examples/python/ik.py) | 把工具坐标系移动 5 cm，并用 `Workspace` 检查结果 |
| [python/engine.py](examples/python/engine.py) | 加载引擎、下发命令，可选地开放给 viewer |
| [omni_dual_arm](examples/omni_dual_arm) | 移动双臂机器人，配置用到了全部模块类型 |

```bash
python examples/python/ik.py tests/fixtures/arm.urdf
python examples/python/engine.py tests/fixtures/arm_loopback.yaml
```

在浏览器里查看机器人，见[调试服务与 viewer](docs/zh-CN/python.md#watch-it-in-the-browser)。

## 文档

| 页面 | 内容 |
|---|---|
| [Python 指南](docs/zh-CN/python.md) | 求解、运行引擎、模型查询、调参、错误 |
| [配置](docs/zh-CN/config.md) | 配置文件结构：模型、求解器、安全、机器人、变量与 include |
| [模块](docs/zh-CN/modules/README.md) | 每种模块的端口和参数 |
| [接入机器人](docs/zh-CN/robot.md) | 接真机：驱动插件、实时设置、调试服务 |
| [调试协议](docs/zh-CN/debug-protocol.md) | 为调试服务编写客户端 |
| [C++](docs/zh-CN/cpp.md) | 用 CMake 构建、链接 kstone |
| [C++ API](docs/zh-CN/api.md) | 按层列出的头文件、回调接口、版本规则 |
| [安装](docs/zh-CN/install.md) | 依赖来源（pip、ROS、源码）、`kstone doctor`、常见问题 |
| [开发](docs/zh-CN/development.md) | 测试、sanitizer、基准测试、CI 和发布 |
| [更新日志](CHANGELOG.zh-CN.md) | 各版本的变更 |

## 许可证

[Apache License 2.0](LICENSE)
