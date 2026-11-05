# Debug protocol

[中文](zh-CN/debug-protocol.md) · [README](../README.md)

The debug server (`kstone run --serve`, `Engine.serve` or `DebugServer`) speaks this protocol. Starting the server, tokens and remote access are covered in [Robot integration](robot.md#debug-server). This page is for writing a client of your own, or a bridge to another visualization tool. The full field list is the comment above `DebugServer` in [debug_server.hpp](../include/kstone/runtime/debug_server.hpp). The protocol version is 1, see [versioning](api.md#versioning-and-compatibility).

## Connecting

1. Open a TCP connection. Each frame is a 4-byte big-endian length followed by a JSON object, in both directions, at most 1 MiB.
2. On connect the server sends `hello`, with `version`, `auth_required` and `capabilities` (the optional ops it accepts).
3. When `auth_required` is true, send `{"op": "auth", "id": 1, "token": "..."}` within 5 seconds. Before that the server accepts only `auth`. A wrong token gets a reply with `code` `unauthorized`, and then the server closes the connection.
4. After authentication, or right away when no token is set, the server sends `model` with the joints, modules, solver parameters and URDF.
5. Send `subscribe` with `rate_hz` to receive `state` messages, which carry `q`, the module outputs and the stats.
6. Every request gets a `reply` with the same `id`, not necessarily in order. `ok` is false on failure, with `code` and `error` set.

## Messages

| Server to client | When |
|---|---|
| `hello` | On connect |
| `model` | Once authenticated |
| `params`, `solver_params`, `joint_limits` | After `model`, and to every client after a successful tune |
| `lease` | After auth and whenever the control lease changes |
| `state` | Per subscription |
| `reply` | For every request |

| Client to server | Purpose |
|---|---|
| `auth`, `ping` | Authenticate, check the connection |
| `subscribe` | Set the `state` rate; 0 stops it |
| `acquire`, `release` | Take or give back the control lease |
| `set_port`, `set_enabled`, `home` | Command a module |
| `tune`, `tune_solver`, `tune_limits` | Change module, solver or joint limit parameters |
| `pause`, `step`, `set_rate` | Pause the engine, step while paused, change the control rate |
| `asset` | Fetch a file the URDF references, such as a mesh; only when listed in `capabilities` |

Commands are rejected with `lease_held` while another client holds the lease. With no holder, any client may command. A lease is released when its client disconnects.

## Errors and limits

- `code` is an error code name (such as `invalid_argument`), or one of `unauthorized`, `lease_held`, `busy`, `timeout` and `unsupported` (unknown op).
- At most 8 clients. A further connection gets a reply with `code` `busy` instead of `hello`, and is closed.
- Non-finite numbers are sent as `null`.
- A client that reads slower than its subscription skips `state` messages. One with 8 MiB of other output queued is closed.

## Minimal client

A minimal Python client, with only the standard library, that authenticates and prints three states:

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

Replace the address with your robot's. Other languages follow the same steps.
