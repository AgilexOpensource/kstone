# Changelog

[中文](CHANGELOG.zh-CN.md)

## 0.0.1

First release.

- Inverse kinematics configured from YAML, with the modules `frame_task`, `joint_target`, `gaze`, `chassis`, `center_of_mass`, `null_space_preference`, `self_collision` and `world_collision`.
- Solver modes `weighted`, `lexicographic`, `layered_independent` and `layered_coupled`.
- An engine that runs the solve as a control loop against a robot driver plugin.
- A debug server, and a web viewer for testing in the separate [kstone-viewer](https://github.com/AgilexOpensource/kstone_viewer) repository.
- The `kstone` Python package with type stubs, and the `kstone` command-line tool.
- Source installs only: `pip install git+...`, a CMake install, or a colcon build in a ROS 2 workspace through `package.xml`.

Interface versions: robot plugin ABI 1, engine C ABI 1, debug protocol 1.

Bundled dependencies (git submodules):

| Library | Version |
|---|---|
| DAQP | v0.11.0 |
| Ruckig | 99751e6 |
| yaml-cpp | 0.8.0 |
| doctest | v2.4.12 |
