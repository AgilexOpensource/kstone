# 更新日志

[English](CHANGELOG.md)

## 0.0.1

首个版本。

- 用 YAML 配置的逆运动学，模块包括 `frame_task`、`joint_target`、`gaze`、`chassis`、`center_of_mass`、`null_space_preference`、`self_collision` 和 `world_collision`。
- 求解模式 `weighted`、`lexicographic`、`layered_independent` 和 `layered_coupled`。
- 以控制循环运行求解、通过机器人驱动插件收发数据的引擎。
- 调试服务器，以及单独仓库 [kstone-viewer](https://github.com/AgilexOpensource/kstone_viewer)中用于测试的网页 viewer。
- 带类型存根的 Python 包 `kstone`，以及 `kstone` 命令行工具。
- 只支持源码安装：`pip install git+...`、CMake 安装，或通过 `package.xml` 在 ROS 2 工作区用 colcon 构建。

接口版本：机器人插件 ABI 1，引擎 C ABI 1，调试协议 1。

内置依赖（git 子模块）：

| 库 | 版本 |
|---|---|
| DAQP | v0.11.0 |
| Ruckig | 99751e6 |
| yaml-cpp | 0.8.0 |
| doctest | v2.4.12 |
