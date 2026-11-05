# Development

[中文](zh-CN/development.md) · [README](../README.md)

## Presets and full check

`CMakePresets.json` defines the presets `debug`, `release`, `asan` and `tsan`. `scripts/check.sh` configures, builds and tests each of them:

```bash
./scripts/check.sh            # all presets
./scripts/check.sh debug asan # selected presets
```

- `ninja` must be in `PATH`.
- `KSTONE_BUILD_ROOT` sets the build root (default `build`), and `KSTONE_CMAKE_ARGS` passes extra CMake arguments. With pip wheels in an active venv and no ROS sourced, the default `AUTO` source picks them up; CI relies on this.
- If the active `python3` has pybind11, the Python extension is built and tested in `debug` and `release`. It is never built under sanitizers.
- After `release`, the build is installed into a temporary prefix and [tests/consumer](../tests/consumer) is built and run against it.

On newer kernels (such as 6.8) TSan fails at startup unless ASLR is disabled: `setarch -R ./scripts/check.sh tsan`. Running a TSan-built executable by hand needs the same `setarch -R` prefix.

C and C++ sources follow [.clang-format](../.clang-format), checked with clang-format 21.1.8 (`pip install clang-format==21.1.8`):

```bash
git ls-files -z -- '*.c' '*.h' '*.cpp' '*.hpp' | xargs -0 clang-format -i
```

`scripts/rm_tmp.sh` removes build and test output: the build root, `*.log` in the repo root, `__pycache__`, `*.egg-info`, the mypy and pytest caches, and scratch dirs that `test_wheel.sh` leaves under `/tmp` if it is interrupted. `-n` only lists them. `.venv` is kept.

<a id="benchmarks"></a>
## Benchmarks

The sources in [bench](../bench) are built with `-DKSTONE_BUILD_BENCH=ON` and are not part of `ctest`. Use a Release build.

| Binary | Measures |
|---|---|
| `kstone_bench [steps]` | Median, p99 and max of `Solver::solve` per model and mode |
| `kstone_cycle_bench [cycles]` | A full control cycle: forward kinematics, `PoseTrajectory::step`, `FrameTask::set_target` and `Solver::solve` |
| `kstone_collision_bench [steps]` | Collision cost of primitives versus convex hulls, single pairs and dual-arm self collision |

```bash
cmake -S . -B build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release -DKSTONE_BUILD_BENCH=ON
cmake --build build/bench
build/bench/bench/kstone_bench
```

## CI

[ci](../.github/workflows/ci.yml) is the only workflow and is started by hand from the Actions page. Every job runs on Ubuntu 22.04. Format, sdist, pip and ros on x86_64 always run; checkboxes add the slower ones: `aarch64` (pip, isolated and ros on aarch64), `debug`, `asan`, `tsan`, `isolated` and `source`, or `all` for every job. The sanitizer presets and source run on x86_64 only. Setting `tag` turns every checkbox on. [plan.py](../.github/ci/plan.py) builds the job list, writes it to the run summary and checks that the tested versions in [install.md](install.md) and the README match the list; run `ALL=true python3 .github/ci/plan.py`, or set individual names such as `ASAN=true`, to preview it.

| Job | Runs |
|---|---|
| format | clang-format check |
| sdist | builds the sdist once and runs `twine check`; pip and isolated install from it and a release attaches it |
| presets | `check.sh` debug, asan and tsan |
| pip | builds once from the sdist as `pip install git+...` does, with tests on, runs ctest and [tests/consumer](../tests/consumer) on that build, then [test_wheel.sh](../scripts/test_wheel.sh) ([test_binding.py](../python/tests/test_binding.py), the `kstone` command, the [Python examples](../examples/python), the type-stub check), then installs the `pin` Python package of the same version and runs `pip check` and the tests again. Python 3.10 and Pinocchio 4.0, as in ROS 2 Humble |
| isolated | an isolated `pip install` of the sdist, as an end user would do it, with the newest allowed Pinocchio; prints a backtrace on a crash |
| source | runs the README C++ recipe, extracted by `plan.py source-script`, then ctest and [tests/consumer](../tests/consumer). The Coal and Pinocchio install is cached, keyed on their recipe lines, so only a change to those lines rebuilds them |
| ros | Pinocchio and Coal from both ROS apt and pip in the `ros:humble` image: `colcon build` and `colcon test` must use the ROS ones, then [tests/consumer](../tests/consumer) against the install space; `pip install` with ROS sourced must use the pip wheels |
| release | only when `tag` is set |

kstone ships as source only: no wheels are built or published. To release, set `tag` to `v` plus the version in `CMakeLists.txt`, which must match `package.xml`, and `CHANGELOG.md` needs a `## ` heading with exactly that version (no `v`). With `tag` or `all` set, `plan.py` checks the section exists and is not empty. When every job passes, the workflow pushes the tag at the tested commit and creates a draft GitHub release with the sdist and the changelog section as notes. Review and publish the draft by hand.

Dependabot opens a weekly pull request for GitHub Actions updates.

## Type stubs

[python/kstone/__init__.pyi](../python/kstone/__init__.pyi) is written by hand and gives IDEs the signatures and docstrings. When the binding in [python/src/kstone.cpp](../python/src/kstone.cpp) changes, update the stub too. The `python/stubs` ctest runs [check_stubs.py](../python/tests/check_stubs.py), which compares the stub with the built module using `mypy.stubtest`; it is skipped when mypy is not installed.

## Submodules

DAQP, Ruckig, yaml-cpp and doctest live in [third_party](../third_party) and are pinned to fixed commits.

[The DAQP backend](../src/qp/daqp.cpp) warm-starts when the problem size is unchanged. It compares the symmetrized Hessian and the constraint matrix exactly with the buffered copies: an unchanged Hessian keeps the factorization, and an unchanged constraint matrix keeps the transformed constraints. The linear term and bounds are updated on every solve, and a change in which rows are equalities updates the constraint senses. DAQP v0.11.0 or later is required, because its constraint-matrix update restores rows that were all zero in an earlier solve. If a warm solve fails, the backend retries once from the equality constraints alone. Lexicographic problems go through [the multi-QP solver](../src/qp/backend.cpp), not DAQP's native hierarchical mode.
