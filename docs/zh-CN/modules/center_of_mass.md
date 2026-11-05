# center_of_mass

[English](../../modules/center_of_mass.md) · [模块总览](README.md)

把质心（含负载）在支撑平面上的投影约束在支撑多边形内。作为不等式约束加入求解，不驱动也不占有关节。

## 关键参数

- `frame`：支撑多边形所在 frame，多边形在其 xy 平面上。`polygon`：严格凸多边形，至少 3 个 `[x, y]` 顶点。
- `subtree`：只算该 frame 子树的质心，空 = 整个模型。
- `margin`：与多边形边保持的距离（m）；负值允许超出这么多。正值不能超过多边形内切圆半径。
- `gain`：每步可用的裕量比例，(0, 1]。`recovery_speed`：越过 margin 后向内拉回的速度（m/s），不要求一步拉回。
- `payloads`：挂在 frame 上的点质量（`frame`、`mass`、`offset`），用于末端负载；设了 `subtree` 时只计入子树内的负载。
- `recovery_tasks`：只有这些模块驱动的关节负责把质心拉回，这些模块同时成为依赖；其余关节越过 margin 后不能再往外推，也不参与拉回。空 = 全部关节。
- `closed_loop`：除了指令状态，也约束测量状态（反馈新鲜时）。

## 示例

```yaml
com:
  type: center_of_mass
  frame: base_link
  polygon: [[0.216, 0.216], [-0.216, 0.211], [-0.216, -0.211], [0.216, -0.216]]
  margin: 0.05
  recovery_tasks: [waist]
  closed_loop: true
  payloads:
    - {frame: left_tool_frame, mass: 3.0}
    - {frame: right_tool_frame, mass: 3.0}
```

```python
com = e.module("com")
com.tune({"payloads": [{"frame": "left_tool_frame", "mass": 5.0}]})
print(com.outputs(e.q)["min_slack"])
```

## 输出

`com`、`ground`（质心在多边形平面上的投影）、`polygon`、`margin_polygon`、`min_slack`（m）、`inside`（slack >= margin）。

## 常见坑

- 多边形必须严格凸，顶点顺时针、逆时针都可以。
- 非 recovery 关节可以用掉裕量，再加上 recovery 关节一步内还能拉回的量；recovery 关节到达速度或位置限位时，非 recovery 关节也就停下。recovery 关节太少或限位太紧时，手臂会更早推不动。
- `payloads` 按整个列表替换，tune 时要给出完整列表。
