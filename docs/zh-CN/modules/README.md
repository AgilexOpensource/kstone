# 模块

[English](../../modules/README.md) · [README](../../../README.zh-CN.md)

模块是[配置](../config.md)里 `modules:` 下的条目：键名是 id，`type` 选类型，其余字段按类型解析。本目录按类型说明用途、端口、关键参数语义和常见坑；完整字段、默认值和范围以 `kstone schema <类型>` 打印的 JSON Schema 为准。

| 类型 | 作用 |
|---|---|
| [frame_task](frame_task.md) | 让某个 frame 跟踪位姿目标 |
| [joint_target](joint_target.md) | 关节空间目标，直接关节控制 |
| [null_space_preference](null_space_preference.md) | 在 frame_task 的零空间里保持姿态偏好 |
| [gaze](gaze.md) | 让相机对准若干 frame |
| [center_of_mass](center_of_mass.md) | 把质心约束在支撑多边形内 |
| [self_collision](self_collision.md) | 自碰撞避免 |
| [world_collision](world_collision.md) | 与环境障碍物的碰撞避免 |
| [chassis](chassis.md) | 底盘速度和位姿控制（在 QP 之外） |

示例配置 [omni_dual_arm.yaml](../../../examples/omni_dual_arm/omni_dual_arm.yaml) 用到了全部类型，各文档的片段都取自它。

## 端口

端口是模块的运行时输入。`Module.ports` 列出名称、类型和范围，`write_port` 在下一次 step 生效，调试协议的 `set_port` 与之等价。

| 类型 | 值 |
|---|---|
| `vector` | 定长数组，带 `labels`、`lower`、`upper` |
| `pose` | `{translation, rotation}`（rotation 为 wxyz 四元数）或 4×4 矩阵；`frame` / `reference` 说明目标的坐标系 |
| `flag` | bool |
| `number` | 标量 |

带 `timeout_s` 的端口超时未写入即失效。

## 运行时调参

schema 里标 `tunable: true` 的字段可在运行时修改：

- `Module.tune(dict)` 只改给出的键，嵌套对象可只给部分子键；整次调用是原子的，任一项校验失败都不改动。
- `Module.params()` 返回当前可调参数，`Module.tune_schema` 返回可调部分的 schema。
- 调试协议的 `tune` 与之等价。
- 求解器和关节限位在引擎层调：`Engine.tune_solver`、`Engine.tune_joint_limits`。

```python
waist = e.module("waist")
waist.tune({"trajectory": {"translation": {"velocity": 0.5}}})
print(waist.params()["trajectory"]["translation"]["velocity"])  # 0.5
```

## 启用与控制权

- `Module.enabled` 可读写（调试协议 `set_enabled`），禁用的模块不参与求解。
- 驱动关节的模块会争夺关节：joint_target 写入目标后占有（claim）其关节，与之冲突的模块在此期间处于 held 状态，`Module.held_by` 给出占有者。
- 被 held 时的行为由 `on_held` 决定：`wait` 保留目标，恢复后继续追；`abort` 丢弃目标，恢复后保持当时的位姿。
- joint_target 的 `release_after_s` 是输入看门狗：最后一次输入后这段时间内，其他模块的请求被推迟。

<a id="task-levels"></a>
## 任务层级

带 `task` 字段的模块（frame_task、joint_target、null_space_preference、gaze）向求解器注册任务：

- `task.level`：分层求解的优先级，越小越优先。
- `task.enabled`：是否参与求解。
- `task.locked`：不贡献行，并保持该任务的关节不动。

center_of_mass、self_collision、world_collision 作为不等式约束加入求解，不驱动也不占有关节。

## 回零

`Module.homeable` 为真的模块（frame_task、joint_target）可调用 `home()`，`home_state` 为 `idle` / `waiting` / `executing` / `succeeded` / `timed_out` / `aborted`。`home_motion.velocity_scale` 控制速度，`home_motion.timeout_s` 是"没有进展"的超时，持续接近时不限时。

## 输出与可视化

`Module.outputs(q)` 返回模块在 q 处的显示量（误差、质心裕度、最近距离等），`Module.visuals()` 返回可绘制形状。输出键列在各模块文档里，只用于显示和诊断。
