# 开发

[English](../development.md) · [README](../../README.zh-CN.md)

## Preset 与完整检查

`CMakePresets.json` 提供 `debug`、`release`、`asan`、`tsan` 四个 preset，`scripts/check.sh` 依次配置、编译并测试：

```bash
./scripts/check.sh            # 全部 preset
./scripts/check.sh debug asan # 指定 preset
```

- `PATH` 里必须有 `ninja`。
- `KSTONE_BUILD_ROOT` 指定构建根目录（默认 `build`），`KSTONE_CMAKE_ARGS` 传入额外的 CMake 参数。依赖来自 venv 里的 pip wheel 且没有 source ROS 时，默认的 `AUTO` 会自动用它们，CI 就是这样跑的。
- 当前 `python3` 带 pybind11 时，在 `debug` 和 `release` 中构建并测试 Python 扩展；sanitizer preset 不构建。
- `release` 之后把构建结果安装到临时前缀，并基于它构建、运行 [tests/consumer](../../tests/consumer)。

较新内核（如 6.8）上 TSan 启动会失败，需关闭 ASLR：`setarch -R ./scripts/check.sh tsan`。手动运行 TSan 构建的可执行文件同样要加 `setarch -R`。

C 和 C++ 源码遵循 [.clang-format](../../.clang-format)，用 clang-format 21.1.8 检查（`pip install clang-format==21.1.8`）：

```bash
git ls-files -z -- '*.c' '*.h' '*.cpp' '*.hpp' | xargs -0 clang-format -i
```

`scripts/rm_tmp.sh` 清理构建和测试产物：构建根目录、仓库根目录的 `*.log`、`__pycache__`、`*.egg-info`、mypy 和 pytest 缓存，以及 `test_wheel.sh` 中断时留在 `/tmp` 下的临时目录。加 `-n` 只列出不删除。`.venv` 会保留。

<a id="benchmarks"></a>
## 基准测试

[bench](../../bench) 中的源码在 `-DKSTONE_BUILD_BENCH=ON` 时编译，不属于 `ctest`。请用 Release 构建。

| 程序 | 测量内容 |
|---|---|
| `kstone_bench [steps]` | 各模型、各模式下 `Solver::solve` 的中位数、p99 和最大值 |
| `kstone_cycle_bench [cycles]` | 完整控制周期：正运动学、`PoseTrajectory::step`、`FrameTask::set_target`、`Solver::solve` |
| `kstone_collision_bench [steps]` | 基本几何体与凸包的碰撞开销，包括单对查询和双臂自碰撞 |

```bash
cmake -S . -B build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release -DKSTONE_BUILD_BENCH=ON
cmake --build build/bench
build/bench/bench/kstone_bench
```

## CI

[ci](../../.github/workflows/ci.yml) 是唯一的工作流，在 Actions 页面手动触发，全部任务都在 Ubuntu 22.04 上运行。x86_64 上的 format、sdist、pip 和 ros 每次都跑；较慢的任务通过勾选框加入：`aarch64`（aarch64 上的 pip、isolated 和 ros）、`debug`、`asan`、`tsan`、`isolated` 和 `source`，勾选 `all` 则运行全部任务。sanitizer 预设和 source 只跑 x86_64。设置 `tag` 时所有勾选框都视为打开。[plan.py](../../.github/ci/plan.py) 生成任务列表并写入运行摘要，同时检查 [install.md](install.md) 和 README 里写的已测试版本与任务列表一致；本地运行 `ALL=true python3 .github/ci/plan.py`，或设置 `ASAN=true` 等单项，可以预览。

| 任务 | 内容 |
|---|---|
| format | clang-format 检查 |
| sdist | 只构建一次 sdist 并运行 `twine check`，pip 和 isolated 从它安装，发布时附上它 |
| presets | 跑 `check.sh` debug、asan、tsan |
| pip | 像 `pip install git+...` 一样从 sdist 构建一次（打开测试），在这次构建上跑 ctest 和 [tests/consumer](../../tests/consumer)，再运行 [test_wheel.sh](../../scripts/test_wheel.sh)（[test_binding.py](../../python/tests/test_binding.py)、`kstone` 命令、[Python 示例](../../examples/python)、类型存根检查），最后装上同版本的 `pin` Python 包，运行 `pip check` 并再测一遍。用 Python 3.10 和 Pinocchio 4.0，与 ROS 2 Humble 一致 |
| isolated | 像最终用户一样隔离 `pip install` sdist，取允许范围内最新的 Pinocchio；崩溃时打印调用栈 |
| source | 运行 README 的 C++ 步骤（由 `plan.py source-script` 提取），再跑 ctest 和 [tests/consumer](../../tests/consumer)。Coal 和 Pinocchio 的安装结果会缓存，缓存键取自它们的编译步骤，只有这些步骤改了才重新编译 |
| ros | 在 `ros:humble` 镜像里同时用 ROS apt 和 pip 安装 Pinocchio 与 Coal：`colcon build` 和 `colcon test` 必须用 ROS 的版本，再基于 install 空间构建 [tests/consumer](../../tests/consumer)；在 source 了 ROS 的环境里 `pip install` 必须用 pip 的 wheel |
| release | 仅在设置了 `tag` 时运行 |

kstone 只以源码形式发布，不构建也不发布 wheel。发布时把 `tag` 设为 `v` 加 `CMakeLists.txt` 中的版本号，它必须与 `package.xml` 一致；`CHANGELOG.md` 里要有标题正好为 `## ` 加该版本号（不带 `v`）的章节。设置 `tag` 或勾选 `all` 时，`plan.py` 会检查该章节存在且不为空。全部任务通过后，工作流在被测提交上推送该 tag，并创建 GitHub 草稿发行版，附上 sdist，说明取自 CHANGELOG 对应章节。草稿需人工检查后发布。

Dependabot 每周为 GitHub Actions 更新开一个 PR。

## 类型存根

[python/kstone/__init__.pyi](../../python/kstone/__init__.pyi) 是手写的，为 IDE 提供签名和文档字符串。修改 [python/src/kstone.cpp](../../python/src/kstone.cpp) 中的绑定时，要同步修改存根。`python/stubs` 这个 ctest 运行 [check_stubs.py](../../python/tests/check_stubs.py)，用 `mypy.stubtest` 对比存根和编译出的模块；没装 mypy 时跳过。

## 子模块

DAQP、Ruckig、yaml-cpp、doctest 位于 [third_party](../../third_party)，固定在指定提交。

[DAQP 后端](../../src/qp/daqp.cpp)在问题规模不变时热启动。它把对称化后的 Hessian 和约束矩阵与缓存逐元素精确比较：Hessian 不变时保留分解，约束矩阵不变时保留变换后的约束。线性项和上下界每次求解都更新；哪些行是等式约束发生变化时，更新约束类型。需要 DAQP v0.11.0 及以上，因为它的约束矩阵更新会恢复之前求解中全为零的行。热启动求解失败时，后端只从等式约束重新求解一次。字典序问题走[多 QP 求解器](../../src/qp/backend.cpp)，不用 DAQP 自带的分层求解。
