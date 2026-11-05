# self_collision

[English](../../modules/self_collision.md) · [模块总览](README.md)

在机器人自身几何体之间加距离约束，让每对几何体至少相距 `d_min`。只加约束，不驱动、不占有关节。

## 关键参数

- `urdf`：带碰撞几何的 URDF，相对路径先按配置文件所在目录查找。`srdf`：其中 `disable_collisions` 列出的 link 对不检查。
- `groups`：同组内的物体之间不检查。关节在 `joint_names` 中或以 `joint_prefixes` 之一开头，且不在 `exclude_joints` 中，它所带的物体就属于该组。
- `exclude_pairs`：frame 或几何体名称对，排除这两个物体之间的所有几何对。`exclude_subtrees`：排除两个物体的运动学子树之间的所有几何对（比如两只手）。
- `exclude_initial_collisions`：默认 true，丢弃在 `robot.initial`（未配置时为 neutral）下已经穿透的几何对，每对打一条 warn。
- `closed_loop`：除了指令状态，同时约束测得状态（有新鲜反馈时）。
- `constraint`：与 world_collision 共用，见下。

### constraint

- `d_min`：保持的最小距离（m），默认 0.03。
- `gain`：每高出 `d_min` 1 m 允许的接近速度（1/s），默认 50。约束行为 $\dot d \ge -\text{gain}\,(d - d_{min})$。
- `activation_margin`：距离小于 `d_min + activation_margin` 的几何对才加约束行，默认 0.05。`hysteresis`：已激活的几何对再远这么多才退出，默认 0.005。
- `approach_deceleration`：制动减速度（m/s²），接近速度不超过 $\sqrt{2a(d - d_{min})}$，且每步不超过剩余间隙的一半。默认 5，0 = 关闭。
- `max_rows`：固定行数，近的几何对优先，默认 16。超出的几何对被丢弃，从最远的开始。
- `max_cuts`：切平面的额外行数，默认 8，0 = 关闭。步末有几何对低于下限时，在该步末端重新线性化再解一次，弥补滑动、转动这类弯曲运动在起点线性化下的误差。
- `clip_slack`：步末裁剪的下限放宽量（m），默认 0。放宽后小步可以跳过检查，但可能加深穿透。
- `exempt_penetrating`：默认 true，进入激活带时已经穿透的几何对在离开 `d_min` 之前不加约束，可以退出也可以更深。false 时按普通几何对往外推，起始穿透较深时可能推不出来。

## 示例

```yaml
self_collision:
  type: self_collision
  urdf: omni_dual_arm.urdf
  groups:
    - {name: left_arm, joint_prefixes: [left_arm_, left_gripper_]}
    - {name: right_arm, joint_prefixes: [right_arm_, right_gripper_]}
  exclude_pairs:
    - [head_base_link, head_yaw_link]
    - [left_gripper_base, right_gripper_base]  # 双臂协作时允许夹爪靠近
  closed_loop: true
  constraint: {d_min: 0.002, activation_margin: 0.01, hysteresis: 0.005, gain: 1.5, approach_deceleration: 20}
```

```python
sc = e.module("self_collision")
sc.tune({"constraint": {"d_min": 0.01}})
print(sc.outputs(e.q)["min_distance"])
```

## 输出

- `min_distance`、`closest`（最近点对的两个见证点）。
- `closest_pair`：最近几何对的 visual，不在激活带内时为空。`band`：在激活带内的 visual。
- `pair`：最近几何对名称，格式为 `a/b`。`pairs`：检查的几何对数。`active_pairs`：最近一次求解时在激活带内的几何对数。

## 常见坑

- 相邻 link 的几何体常常本来就贴在一起，要用 `exclude_pairs` 或 SRDF 排除，否则约束一直饱和。启动日志里 `exclude_initial_collisions` 打出的 warn 可以作为线索，但只覆盖初始姿态。
- `active_pairs` 经常等于 `max_rows` 时说明行数不够，较远的几何对被丢掉，可以加大 `max_rows` 或收窄 `activation_margin`。
- `gain` 越大越敢贴近边界；示例里 `gain` 1.5 配合 50 Hz，每步最多用掉约 3% 的余量。
- 模块重新启用时已经穿透的几何对一开始是豁免的，不会被立即推开。
