# joint_target

[English](../../modules/joint_target.md) · [模块总览](README.md)

关节空间目标，经加加速度受限的关节轨迹送给求解器。用于直接关节控制，也用于夹爪、云台这类只需要关节目标的部件。

## 端口

| 名称 | 类型 | 说明 |
|---|---|---|
| `target` | vector | 每个关节一个值，按 `joints` 顺序；超出位置限位的分量会被夹紧 |

## 关键参数

- `joints`：标量关节，每个 mimic 组最多一个。
- `task.level`：未设时为 0，所以关节控制优先于共用这些关节的位姿任务。
- `trajectory.trigger`：`continuous`（默认）总是整形；`threshold` 只整形以速度上限走完超过 `threshold` 秒的大位移，小位移直接跟；`direct` 不整形，参考就是目标。
- `trajectory.sync`：`phase`（默认）各关节同相位、沿直线到达，不共线时退化为 `time`；`time` 同时到达；`none` 各自独立。
- `trajectory.velocity_scale` / `velocity_cap`：整体缩放和每关节巡航速度上限。`velocity_limit` 为空时用模型限位，此时每个关节的限位必须有限且大于 0。
- 控制权：
  - 写入 `target` 或回零完成后占有关节，到达后也保持，直到其他模块有新输入且距本模块最后一次输入已过 `release_after_s`（默认 0.25 s，0 = 立即让出）。
  - `priority`：占有优先级。
  - `on_held`：默认 `wait`，被占有期间保留目标，恢复后继续。
  - `hold_idle`：空闲时保持最后目标（默认 true）；false 时让出任务，其他模块可动这些关节。
  - `claim_idle`：空闲时也占有关节，冲突模块一直 held，相当于常驻关节控制。
- `home`：每关节回零位置，空 = 初始姿态。

## 示例

```yaml
torso:
  type: joint_target
  joints: [torso_joint_1, torso_joint_2, torso_joint_3, torso_joint_4]
  task: {level: 0}
  home_motion: {velocity_scale: 0.1667, timeout_s: 2.0}
  claim_idle: true
  release_after_s: 0.25
```

```python
torso = e.module("torso")
torso.write_port("target", [0.0, 1.2, 0.0, 0.0])
torso.tune({"trajectory": {"velocity_scale": 0.5}})
```

## 常见坑

- `claim_idle: true` 时，共用关节的 frame_task、null_space_preference、gaze 会一直 held，直到其他模块有新输入并过了 `release_after_s`。示例里 `left_arm` 占着左臂，所以 `left_null` 不起作用；要用零空间偏好，就先禁用对应的 joint_target，或者设 `claim_idle: false`。
- `release_after_s` 太大时，切换到位姿控制会有明显停顿；设为 0 则其他模块随时能抢走关节。
