# 调试协议

[English](../debug-protocol.md) · [README](../../README.zh-CN.md)

调试服务（`kstone run --serve`、`Engine.serve` 或 `DebugServer`）使用这个协议。启动服务、token 和远程访问见[接入机器人](robot.md#debug-server)。本页面向自己编写客户端，或接入其他可视化工具的桥接程序。完整字段列表见 [debug_server.hpp](../../include/kstone/runtime/debug_server.hpp) 中 `DebugServer` 上方的注释。协议版本为 1，见[版本](api.md#versioning-and-compatibility)。

<a id="connecting"></a>
## 连接流程

1. 建立 TCP 连接。双向的每一帧都是 4 字节大端长度，后面跟一个 JSON 对象，最大 1 MiB。
2. 连上后服务端发送 `hello`，其中有 `version`、`auth_required` 和 `capabilities`（服务端接受的可选 op）。
3. `auth_required` 为真时，要在 5 秒内发送 `{"op": "auth", "id": 1, "token": "..."}`。在此之前服务端只接受 `auth`。token 错误时会收到 `code` 为 `unauthorized` 的 reply，随后连接被关闭。
4. 认证通过后，或服务端没设 token 时连上即可，服务端发送 `model`，包含关节、模块、求解器参数和 URDF。
5. 发送带 `rate_hz` 的 `subscribe`，就会收到 `state` 消息，其中有 `q`、模块输出和统计量。
6. 每个请求都会收到 `id` 相同的 `reply`，顺序不保证。失败时 `ok` 为 false，并带有 `code` 和 `error`。

<a id="messages"></a>
## 消息

| 服务端到客户端 | 时机 |
|---|---|
| `hello` | 连上时 |
| `model` | 认证通过后 |
| `params`、`solver_params`、`joint_limits` | `model` 之后，以及调参成功后发给所有客户端 |
| `lease` | 认证后，以及控制租约变化时 |
| `state` | 按订阅频率 |
| `reply` | 每个请求一条 |

| 客户端到服务端 | 用途 |
|---|---|
| `auth`、`ping` | 认证、检查连接 |
| `subscribe` | 设置 `state` 频率；0 停止 |
| `acquire`、`release` | 取得或交还控制租约 |
| `set_port`、`set_enabled`、`home` | 控制模块 |
| `tune`、`tune_solver`、`tune_limits` | 修改模块、求解器或关节限位参数 |
| `pause`、`step`、`set_rate` | 暂停引擎、暂停时单步、修改控制频率 |
| `asset` | 获取 URDF 引用的文件（如网格）；仅当 `capabilities` 中列出时可用 |

其他客户端持有租约时，命令会被拒绝，返回 `lease_held`。无人持有时任何客户端都可以发命令。客户端断开时其租约自动释放。

<a id="errors-and-limits"></a>
## 错误与限制

- `code` 是错误码名（如 `invalid_argument`），或 `unauthorized`、`lease_held`、`busy`、`timeout`、`unsupported`（未知 op）之一。
- 最多 8 个客户端。再连入的连接收到 `code` 为 `busy` 的 reply 而不是 `hello`，随后被关闭。
- 非有限数值以 `null` 发送。
- 读取慢于订阅频率的客户端会跳过部分 `state`。其他待发输出积压到 8 MiB 的客户端会被断开。

<a id="minimal-client"></a>
## 最小客户端

下面是一个只用标准库的最小 Python 客户端，完成认证后打印三条 state：

```python
import json, os, socket, struct

def send(sock, msg):
    data = json.dumps(msg).encode()
    sock.sendall(struct.pack(">I", len(data)) + data)

def recv(stream):
    header = stream.read(4)
    if len(header) < 4:
        raise ConnectionError("server closed the connection")
    return json.loads(stream.read(struct.unpack(">I", header)[0]))

sock = socket.create_connection(("192.168.1.10", 7311))
stream = sock.makefile("rb")
hello = recv(stream)
if hello["auth_required"]:
    send(sock, {"op": "auth", "id": 1, "token": os.environ["KSTONE_TOKEN"]})
count = 0
while count < 3:
    msg = recv(stream)
    if msg["type"] == "reply" and not msg["ok"]:
        raise SystemExit(f"{msg['code']}: {msg['error']}")
    if msg["type"] == "model":
        print("joints:", [j["name"] for j in msg["joints"]])
        send(sock, {"op": "subscribe", "id": 2, "rate_hz": 10})
    elif msg["type"] == "state":
        print(msg["time"], msg["q"])
        count += 1
```

把地址换成你的机器人地址。其他语言按同样的步骤实现即可。
