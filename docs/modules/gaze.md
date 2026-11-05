# gaze

[中文](../zh-CN/modules/gaze.md) · [Module overview](README.md)

Points the optical axis of camera `frame` at the mean position of the active frames in `frames`. With no active frame, the camera joints return to their initial positions.

## Key parameters

- `frame`: camera frame. `joints`: scalar joints used for aiming.
- `offset`: optical centre in the camera frame. `optical_axis`: axis direction, x by default.
- `frames`: frames to watch, whether a pose task or joint control moves them.
- `target_idle_s`: a frame counts as active for this long after it moves relative to the model root (startup counts as a move); <= 0 = always active. Motion is judged on the solved pose relative to the root, so base motion alone does not count.
- `max_velocity`: joint velocity cap, 0 = none.

## Ownership

While it has frames, gaze claims its joints at yielding priority, so any joint_target claim wins. Each time a frame moves, gaze requests the joints again, and a joint_target yields according to its own `release_after_s`.

## Example

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
e.module("gaze").tune({"target_idle_s": 0})  # always watch the end effectors
```

## Outputs

`focus` (the watched point, NaN while homing), `active_frames` (number of active frames).

## Pitfalls

- In the example `head` is a joint_target with `claim_idle: true`. After manual head control, gaze takes over only once an end effector moves again and `release_after_s` has passed.
- If `optical_axis` does not match the camera frame's real orientation, the aim is off. Check the frame orientation in the viewer first.
