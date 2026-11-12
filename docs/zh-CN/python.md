# Python 指南

[English](../python.md) · [README](../../README.zh-CN.md)

安装步骤见 [README](../../README.zh-CN.md#安装)。下面的代码都在仓库根目录运行，使用 [arm_loopback.yaml](../../tests/fixtures/arm_loopback.yaml)，这是一个带两指夹爪的小型机械臂。

所有类和函数都有文档字符串和类型标注，`help(kstone.Engine)` 和 IDE 补全都能用。

## 基本概念

- **配置**：一份 YAML 文件，包含机器人模型（`model:`）、要驱动的机器人（`robot:`）和一组**模块**（`modules:`）。见[配置](config.md)。
- **模块**：一个目标或限制，比如"把这个坐标系移到某个位姿"（`frame_target`）或"把这些关节转到某些角度"（`joint_target`）。通过写模块的**端口**来下发命令。
- **Solver**：根据当前关节位置和各模块的目标，算出下一步的关节位置。不涉及机器人，也不涉及时序。
- **Engine**：Solver 加上机器人和固定频率的循环。每个周期读取机器人状态、求解、发送命令。
- **q** 和 **v**：关节位置（长度 `model.nq`）和关节速度（长度 `model.nv`）。机器人有连续关节或浮动基座时两者长度不同，所以要用 `model.q_index(name)` 和 `model.v_index(name)` 按关节名查下标。

## 离线求 IK

```python
import kstone

solver = kstone.load_solver("tests/fixtures/arm_loopback.yaml")
print(solver.modules)          # ['tool', 'arm', 'fingers']
print(solver.module("arm").ports)

solver.module("arm").write_port("target", [0.5, -0.3, 0.2])  # labels=["j3"] 只写 j3
q = solver.initial
for _ in range(300):
    result = solver.solve(q, 0.01)
    q = result.q_next
```

`solve(q, dt)` 前进一步，步长 `dt` 秒。模块会经过多步逐渐到达目标，速度由模块的 `gain` 和 `trajectory` 参数决定。返回的 `SolveResult` 包含：

| 字段 | 含义 |
|---|---|
| `q_next`、`v` | 下一步的关节位置，以及到达它的关节速度 |
| `quality` | `"nominal"`、`"relaxed"`、`"degraded"` 或 `"failed"` |
| `residuals` | 每个任务剩余的误差 |
| `held` | 这一步求解失败、机器人保持原位时为 `True` |

位姿目标可以是 4×4 矩阵，也可以是 `{"position": [x, y, z], "orientation": [w, x, y, z]}`：

```python
ws = kstone.Workspace(solver.model)
ws.update(q)
target = ws.frame_pose("tool")    # 4x4，世界坐标系
target[:3, 3] += [0.0, 0.05, 0.0]
solver.module("tool").write_port("target", target)
solver.module("tool").tune({"orientation_weight": [0, 0, 0]})  # 只要位置
solver.module("arm").enabled = False  # 它的关节目标会和 tool 互相拉扯
for _ in range(300):
    q = solver.solve(q, 0.01).q_next
print(solver.module("tool").outputs(q)["position_error"])  # 0.0
```

测试用的机械臂只有四个关节，到不了任意 6D 位姿，所以这段只要位置。同一次求解里的任务会互相竞争：`arm` 不关的话，tool 会差大约 4 cm 到不了。

[examples/python/ik.py](../../examples/python/ik.py) 用内联配置完整演示了这个过程。

## 运行引擎

```python
engine = kstone.load_engine("tests/fixtures/arm_loopback.yaml")
engine.module("arm").write_port("target", [0.5, -0.3, 0.2])

engine.run(duration_s=3.0)        # 或者每个周期调用一次 engine.step()
print(engine.q, engine.quality)
print(engine.stats["cycles"], engine.stats["failed_cycles"])
```

- `run()` 会阻塞，直到过了 `duration_s` 或者另一个线程调用了 `stop()`。不传时长就一直运行到停止为止。
- `run()` 运行期间，其他 Python 线程可以照常使用引擎和模块（`write_port`、`tune`、`home`、`q`、`stats` 等）。每次调用都在控制线程的下一个周期之前执行，所以最多等一个周期。`step()` 和 `serve()` 会报错；`serve()` 要在 `run()` 之前调用。
- `run()` 按真实时钟运行。要比实时更快地推进（测试、离线仿真），循环调用 `step()`，并设 `feedback_timeout_s: 0`；否则引擎要等这么长的真实时间才同步，此前模块保持不动。
- `engine.q` 是命令的关节位置，`engine.measured` 是最近一次从机器人读到的关节位置。
- `engine.snapshot()` 把所有状态打包成一个 dict 返回。
- `engine.fresh` 表示最近一次读取是否新鲜；`engine.attached` 表示引擎是否已同步到新鲜反馈且此后一直新鲜。只在 attached 时向真机下发命令，见[运行契约](robot.md#run-contract)。
- `engine.groups` 以 `{name, fresh, attached}` 列出各反馈组（先是配置的组，最后是 `default`）；`engine.feedback_groups` 返回配置的组。`snapshot()["groups"]` 是同一列表。

要让循环一直跑、同时由你的代码发目标，就把它放到线程里：

```python
import threading
import time

runner = threading.Thread(target=engine.run)
runner.start()
for x in (0.3, 0.4, 0.5):
    engine.module("arm").write_port("target", [x, -0.3, 0.2])
    time.sleep(1.0)
print(engine.snapshot()["stats"]["cycles"])
engine.stop()
runner.join()
```

要驱动真机，修改配置的 `robot:` 部分即可，见[接入机器人](robot.md)。

## 从字符串加载配置

`load_solver_from_string` 和 `load_engine_from_string` 接受 YAML 文本。其中的相对路径（比如 URDF）相对于 `base_dir` 解析：

```python
config = """
model: {file: arm.urdf}
modules:
  arm: {type: joint_target, joints: [j1, j2]}
"""
solver = kstone.load_solver_from_string(config, base_dir="tests/fixtures")
```

四个加载函数都接受 `vars={"name": "value"}`，用来填充配置里的 `${name}` 占位符，见[变量与 include](config.md#variables-and-includes)。

## 查询模型

```python
model = kstone.Model.from_file("tests/fixtures/arm.urdf")
print(model.nq, model.nv, model.joint_names)

ws = kstone.Workspace(model)
ws.update(model.neutral())
ws.frame_pose("tool")             # 4x4 位姿，世界坐标系
ws.frame_jacobian("tool")         # 6 x nv，前三行是线速度
ws.center_of_mass()               # 整个机器人
```

坐标系和关节可以用名字，也可以用 id（`model.frame_id(name)`、`model.joint_id(name)`）。`q` 改变后要重新调用 `update(q)`。

## 运行时调参

```python
tool = engine.module("tool")
print(tool.tune_schema["properties"].keys())
tool.tune({"gain": 0.8})
print(tool.params()["gain"])

engine.tune_solver({"damping": 1e-2})
engine.tune_joint_limits({"velocity": {"j1": 0.5}})
engine.tune_joint_limits({"reset": True})
```

用 `module.enabled = False` 可以关闭模块。`engine.home_all()` 让当前控制着关节、且支持 home 的模块都回到 home（看 `home_state`）。回 home 只在引擎已同步（attached）时推进，而同步要求反馈连续新鲜 `feedback_timeout_s` 的真实时间。请用 `run()`，或者按真实时钟节拍调用 `step()`。

<a id="watch-it-in-the-browser"></a>
## 在浏览器里查看

`engine.serve()` 或 `kstone run --serve` 会启动调试服务，[kstone-viewer](https://github.com/AgilexOpensource/kstone_viewer/blob/main/README.zh-CN.md) 连接到它。viewer 是单独的包，用于冒烟测试：

```bash
pip install "git+https://github.com/AgilexOpensource/kstone_viewer.git"
kstone run tests/fixtures/arm_loopback.yaml --serve 7311
# 在第二个终端里
kstone-viewer --connect 127.0.0.1:7311
```

viewer 会打印要打开的网址。token 和远程访问见[调试服务](robot.md#debug-server)。

## 错误

kstone 抛出 `kstone.Error`，它的 `code` 说明出错的类型：

```python
try:
    engine.module("nope")
except kstone.Error as e:
    print(e.code, e)              # not_found ...
```

常见的 code 有 `not_found`、`invalid_argument`、`out_of_range`、`parse_error` 和 `io_error`。数字 id 超出范围时抛出 `IndexError`。

## Python 中不提供的功能

机器人驱动用 C 或 C++ 编写（见[接入机器人](robot.md#driver-plugins)）。碰撞世界和遥测输出只有 C++ 接口。`kstone.doctor(print=True)` 检查安装是否正常，见[安装](install.md#environment-check)。
