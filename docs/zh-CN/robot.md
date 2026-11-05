# 接入机器人

[English](../robot.md) · [README](../../README.zh-CN.md)

<a id="running-a-config"></a>
## 运行配置

```bash
kstone check <config.yaml>       # 只构建引擎，不运行
kstone run <config.yaml> [--duration s] [--realtime] [--serve [host:port|port]] [--token-file path]
```

- `--set name=value`（可重复）设置配置变量，见[变量与 include](config.md#variables-and-includes)。
- 配置里的相对文件路径（如 `model.file` 或模块的 `urdf`）先按配置文件所在目录解析。
- `kstone schema [类型]` 列出模块类型，或打印某个类型的 JSON Schema。配置键见[配置](config.md)，各模块说明见[模块](modules/README.md)。

<a id="driver-plugins"></a>
## 驱动插件

驱动写成实现 [robot_plugin.h](../../include/kstone/runtime/robot_plugin.h) C ABI 的共享库，以 [example_robot.c](../../examples/robot_plugin/example_robot.c) 为起点。`read` 必须带 CLOCK_MONOTONIC 时间戳，否则反馈视为过期。

```bash
cc -shared -fPIC -Iinclude examples/robot_plugin/example_robot.c -o libexample_robot.so
```

```yaml
robot:
  type: plugin
  library: ./libexample_robot.so  # 含 '/' 按配置文件目录解析，裸名走动态库搜索路径
  params: {}                      # 原样传给插件的 create()
```

C++ 宿主也可以直接实现 `RobotInterface`，见 [C++ API](api.md) 和 [engine.cpp](../../examples/cpp/engine.cpp)。

<a id="real-time-thread"></a>
## 实时线程

Linux 上控制线程可用实时调度。设置在 `run()` 开始时生效，返回时恢复优先级和亲和。

```yaml
thread:
  priority: 50       # SCHED_FIFO 1..99，值越大越优先，0 = 不改；需 CAP_SYS_NICE 或 ulimit -r >= 该值
  cpus: [3]          # 绑定的 CPU，空 = 不改；配合 isolcpus 效果最好
  lock_memory: true  # 整个进程 mlockall，不会解除；需 CAP_IPC_LOCK 或足够大的 ulimit -l
```

- 系统拒绝某项设置（如缺 CAP_SYS_NICE、RLIMIT_RTPRIO 或 CAP_IPC_LOCK）时记一条 warn 日志并照常运行，与 ros2_control_node 一致。配置值非法仍在创建时报错。
- `lock_memory` 调用 `mlockall(MCL_CURRENT | MCL_FUTURE)`。
- 直接调用 `step()` 不受影响。

<a id="run-contract"></a>
## 运行契约

- **周期失败。** 读、解或写出错时引擎不写命令，也不合成停止命令，计入 `failed_cycles`。`run()` 继续运行，连续 `max_failed_cycles`（默认 5）个失败周期后返回最后一个错误。驱动必须自建命令超时与安全态。
- **反馈过期。** 反馈超过 `feedback_timeout_s` 不算失败：命令照常写出，但带 `RobotCommand::fresh = false`，由驱动或调用方决定是否下发。
- **命令守卫。** `CommandGuardOptions` 按限位检查命令。reject 模式下越界命令使该周期返回错误并计入 `guard_rejects`；clamp 模式钳位后照常写出并计入 `guard_clamps`。
- **遥测。** 使用方只读值类型快照（`EngineSnapshot`），不持有 `const Engine&`。

<a id="debug-server"></a>
## 调试服务

`kstone run --serve` 启动调试服务，默认 `127.0.0.1:7311`，提供遥测流、调参和控制租约。

- 绑定非回环地址需要至少 16 字节的 token，来自 `--token-file` 或 `$KSTONE_TOKEN`。
- 协议不加密，token 明文传输。跨不可信网络时套 SSH 隧道或 VPN。
- [kstone-viewer](https://github.com/AgilexOpensource/kstone_viewer/blob/main/README.zh-CN.md) 经此协议连接。它是单独的冒烟测试包，不依赖 kstone，可以在另一台机器上运行。C++ 和 ROS 程序可以用 `DebugServer` 提供同一协议。
- 自己编写客户端见[调试协议](debug-protocol.md)。

<a id="remote-access"></a>
### 远程访问

要在可信网络里从另一台机器连接，先生成一次 token 文件，只允许自己读取：

```bash
(umask 077 && python3 -c "import secrets; print(secrets.token_urlsafe(24))" > ~/.kstone-token)
```

用这个 token 让引擎监听所有网卡：

```bash
kstone run <config.yaml> --realtime --serve 0.0.0.0:7311 --token-file ~/.kstone-token
```

把文件复制到运行 viewer 的机器上，用同一个 token 连接：

```bash
kstone-viewer --connect <机器人IP>:7311 --token-file ~/.kstone-token
```

也可以在两端把 `KSTONE_TOKEN` 设为同一个值，代替 `--token-file`：

```bash
export KSTONE_TOKEN="$(cat ~/.kstone-token)"
```

跨不可信网络时，服务只监听回环地址，这样不需要 token，再用 SSH 转发端口：

```bash
kstone run <config.yaml> --realtime --serve 7311          # 机器人上
ssh -N -L 7311:127.0.0.1:7311 user@<机器人IP>             # viewer 所在机器上
kstone-viewer --connect 127.0.0.1:7311
```

在自己的程序里，启动服务时设置 token，并从环境变量或文件读取，不要写进源码。

Python：

```python
import os
import kstone

engine = kstone.load_engine("robot.yaml")
server = engine.serve("0.0.0.0", 7311, token=os.environ["KSTONE_TOKEN"])
```

C++：

```cpp
kstone::DebugServerOptions options;
options.host = "0.0.0.0";
const char* token = std::getenv("KSTONE_TOKEN");
options.token = token != nullptr ? token : "";
auto server = kstone::DebugServer::start(engine, options);
if (!server) std::fprintf(stderr, "%s\n", server.error().message.c_str());
```

非回环地址没有 token，或 token 长度不在 16..1024 字节内时，`start` 返回 `invalid_argument` 错误。
