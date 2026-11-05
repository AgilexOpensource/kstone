# world_collision

[English](../../modules/world_collision.md) · [模块总览](README.md)

在机器人几何体和一组固定数量的障碍物之间加距离约束。只加约束，不驱动、不占有关节。不检查机器人自身的几何对，自碰撞用 [self_collision](self_collision.md)。

## 端口

| 端口 | 类型 | 说明 |
|---|---|---|
| `<name>` | pose | 障碍物在世界系中的位姿 |
| `<name>/enabled` | flag | 关闭后不检查该障碍物，visual 也隐藏 |

## 关键参数

- `urdf`：机器人碰撞几何。
- `obstacles`：障碍物列表，至少一个，名称唯一且不能含 `/`（名称同时是端口名）。启动后不能增删。
  - `shape.type`：`sphere`、`box`、`cylinder` 或 `capsule`。
  - `shape.size`：sphere 为 `[r, 0, 0]`，box 为 `[x, y, z]`，cylinder 和 capsule 为 `[r, length, 0]`，轴沿 z。
  - `pose`：`translation` 加四元数 `rotation`（wxyz）或 `rpy`（外旋 XYZ），两者只能给一个。
  - `enabled`：初始是否启用。
- `velocity_timeout`：根据相邻两次位姿写入估计障碍物速度，最后一次写入后过这么久（s）速度归零，默认 0.2。0 = 所有障碍物视为静止。
- `closed_loop`：同时约束测得状态。
- `constraint`：与 self_collision 相同，见 [self_collision](self_collision.md#constraint)。

障碍物朝机器人移动时，约束行会按见证点处的接近速度抬高；切平面和步末裁剪在一步内把障碍物当作静止。

## 示例

```yaml
obstacles:
  type: world_collision
  velocity_timeout: 0.2
  urdf: omni_dual_arm.urdf
  obstacles:
    - name: table
      shape: {type: box, size: [0.4, 0.8, 0.04]}
      pose: {translation: [0.615, 0, 0.427]}
    - name: box2
      shape: {type: box, size: [0.2, 0.2, 0.2]}
      pose: {translation: [0.655, 0.1, 0.469]}
  constraint: {d_min: 0.002, activation_margin: 0.02, hysteresis: 0.005, gain: 1.5}
```

```python
obs = e.module("obstacles")
obs.write_port("box2", {"translation": [0.655, 0.1, 0.6], "rotation": [1, 0, 0, 0]})
obs.write_port("table/enabled", False)
```

## 输出

与 self_collision 相同：`min_distance`、`closest`、`closest_pair`、`band`、`pair`、`pairs`、`active_pairs`。visual 先列机器人几何体，再列障碍物。

## 常见坑

- 障碍物数量固定。场景里物体数量会变时，按上限预先配置，用 `<name>/enabled` 开关。
- 障碍物位姿写入不连续（比如感知丢帧后跳变）会估出很大的速度，约束突然收紧。感知频率低于 `1 / velocity_timeout` 时速度会在两次写入之间归零。
- 刚启用的障碍物如果已经和机器人穿透，默认豁免，不会把手臂推出来，要先移开。
