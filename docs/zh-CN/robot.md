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

驱动写成实现 [robot_plugin.h](../../include/kstone/runtime/robot_plugin.h) C ABI 的共享库，以 [example_robot.c](../../examples/robot_plugin/example_robot.c) 为起点。`read` 必须带 CLOCK_MONOTONIC 时间戳，否则反馈视为过期。配置了 `feedback_groups` 时，每个关节带有所属组的索引，`read` 还要在 `group_stamps_ns` 里给每组打时间戳（-1 表示过期；只有 `default` 组会回退到整体状态时间戳），`write` 通过 `group_attached` 收到每组的同步状态。

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

Linux 上控制线程可用实时调度。`realtime: true` 时设置在 `run()` 开始时生效，返回时恢复优先级和亲和；`realtime: false` 时忽略 `thread` 块。

```yaml
thread:
  priority: 50       # SCHED_FIFO 1..99，值越大越优先，0 = 不改；需 CAP_SYS_NICE 或 ulimit -r >= 该值
  cpus: [3]          # 绑定的 CPU，空 = 不改；配合 isolcpus 效果最好
  lock_memory: true  # 整个进程 mlockall，不会解除；需 CAP_IPC_LOCK 或足够大的 ulimit -l
```

- 系统拒绝某项设置（如缺 CAP_SYS_NICE、RLIMIT_RTPRIO 或 CAP_IPC_LOCK）时记一条 warn 日志并照常运行，与 ros2_control_node 一致。配置值非法仍在创建时报错。
- realtime 生效时（`--realtime` 或 `realtime: true`），`kstone run` 会调用 `elevate_thread_limits`：若 `ulimit -r` 低于 `priority`（或开了 `lock_memory` 而 `ulimit -l` 有限）且缺对应 capability，启动时执行一次 `sudo prlimit --pid <self>`，在终端提示输入密码。只提高这两个限额，进程本身不以 root 运行。无终端或 sudo 失败时记 warn 并照常运行。
- 直接调用 `step()` 不受影响。

<a id="run-contract"></a>
## 运行契约

- **周期失败。** 读、解或写出错时引擎不写命令，也不合成停止命令，计入 `failed_cycles`。`run()` 继续运行，连续 `max_failed_cycles`（默认 5）个失败周期后返回最后一个错误。驱动必须自建命令超时与安全态。
- **反馈过期。** 反馈超过 `feedback_timeout_s` 不算失败：命令照常写出，但带 `RobotCommand::fresh = false`，由驱动或调用方决定是否下发。
- **同步（attach）。** `Engine::attached()` 为 true 表示反馈已连续新鲜 `feedback_timeout_s`，且这段窗口内每个周期都把命令状态同步到了新鲜反馈的模型限位投影。一次过期读取即脱离同步。之后第一个新鲜且未暂停的周期让各模块重新基准（丢弃保持的目标，各模块从裁限后的当前状态重置）。此后每个新鲜且未暂停的周期都在求解前把命令状态同步到裁限后的最新测量状态；窗口内设的目标保留。窗口走完的那个周期完成同步，计入 `attaches`。因此同步需要 `feedback_timeout_s` 的真实时间，所有标为新鲜的命令都从裁限后的当前测量状态出发。创建时反馈过期或无时间戳的引擎从未同步状态开始；创建时反馈新鲜的引擎也要等满这段窗口。写出不受门控：驱动真机的调用方应只在 `attached()` 时下发命令，并在其变为 true 之后再设目标。`feedback_timeout_s = 0` 时引擎始终处于同步状态。
- **反馈分组。** 配置 `feedback_groups` 后（例如机械臂用关节编码器、底盘用里程计），每组有各自的时间戳（`RobotState::group_stamps_ns`）、过期判断、窗口和同步状态（`Engine::group_attached()`、`RobotCommand::group_attached`）。某组过期只让该组脱离同步：它的同步只改本组关节，只重新同步驱动这些关节的模块。只有所有组都已同步时 `attached()` 和 `RobotCommand::fresh` 才为 true；驱动也可以按每组的标志分别下发。
- **实测位置超出软件限位。** 合法有限的反馈保持原值。启动、附着、重附着及暂停恢复同步时，命令/ideal 的标量位置裁到模型 lower/upper 限位；mimic 从关节沿用既有模型派生。关节目标及位姿 FK/轨迹重置使用裁限后的命令状态，而不是原始反馈。同步在 prepare 前刷新 FK，包含被耦合 idle 关节占用挂起的位姿模块；rebase 即使在模块挂起时也清除旧目标。重附着丢弃失联目标；普通暂停恢复保留仍有效的目标，并从裁限状态重置轨迹。这采用旧 `ik_teleop_ros` 的裁限初始化，而不是无目标严格保持策略。
- **同步后的自动行为。** 无需 fresh target/home 即正常逐周期运行 IK：gaze 可以瞄准或回 home，CoM/碰撞可以要求恢复（包括碰撞切平面），null-space 偏好沿用原有主模块规则。闭环检查、终点保护、位置/速度限位及命令租期不变；目标过期不引入新的运动意图生命周期。第一条位置命令可能因裁限修正而与原始反馈产生突变，自动模块也可在同一次求解继续改变命令。裁限不是限速恢复，也不保证姿态满足碰撞/CoM 约束；原有求解器/守卫失败处理仍适用。 原有模块优先级及 idle claim 仍负责控制权仲裁；自动模块启用并不意味着所有姿态都会运动。
- **命令守卫。** `CommandGuardOptions` 按限位检查命令。reject 模式下守卫违规使该周期返回错误并计入 `guard_rejects`；clamp 模式钳位后照常写出并计入 `guard_clamps`。接受反馈不放宽模型或硬件限位。
- **遥测。** 使用方只读值类型快照（`EngineSnapshot`），不持有 `const Engine&`。

<a id="debug-server"></a>
## 调试服务

`kstone run --serve` 启动调试服务，默认 `127.0.0.1:7311`，提供遥测流、调参和远程命令。各客户端平等，以最后一次写入为准。自己的代码写入运动端口后保持 `api_hold_s`（默认 1 s），期间服务端对该模块运动端口的 `set_port` 返回 `api_owned`；见 [debug-protocol.md](debug-protocol.md)。

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
