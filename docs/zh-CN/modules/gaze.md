# gaze

[English](../../modules/gaze.md) · [模块总览](README.md)

让相机 `frame` 的光轴对准 `frames` 中活跃 frame 的平均位置；没有活跃 frame 时，相机关节回到初始位置。

## 关键参数

- `frame`：相机 frame。`joints`：用来瞄准的标量关节。
- `offset`：光心在相机 frame 中的位置。`optical_axis`：光轴方向，默认 x 轴。
- `frames`：关注的 frame，无论它们由位姿任务还是关节控制驱动。
- `target_idle_s`：frame 相对模型根移动后，在这段时间内算活跃（启动也算一次移动）；<= 0 = 一直活跃。运动在相对根的求解位姿上判断，所以只动底盘不算。
- `max_velocity`：关节速度上限，0 = 不限。

## 控制权

有 frame 时以 yielding 优先级占有关节，任何 joint_target 的占有都优先。每当 frame 移动，gaze 重新请求关节，joint_target 按自己的 `release_after_s` 让出。

## 示例

```yaml
gaze:
  type: gaze
  frame: head_yaw_link
  joints: [head_pitch_joint, head_yaw_joint]
  frames: [left_tool_frame, right_tool_frame]
  max_velocity: 3.0
  target_idle_s: 5.0
```

```python
e.module("gaze").tune({"target_idle_s": 0})  # 一直看着末端
```

## 输出

`focus`（关注点，回零时为 NaN）、`active_frames`（活跃 frame 数）。

## 常见坑

- 示例里 `head` 是 `claim_idle: true` 的 joint_target。手动控制头部后，要等末端再次移动且过了 `release_after_s`，gaze 才接管。
- `optical_axis` 与相机 frame 实际朝向不符时会看偏，先用 viewer 确认 frame 方向。
