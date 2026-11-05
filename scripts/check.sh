#!/usr/bin/env bash
# Configure, build and test every preset: ./scripts/check.sh [preset...]  (default: debug release asan tsan)
# Run it inside whatever environment provides the dependencies: venv, sourced ROS, or plain system.
# KSTONE_BUILD_ROOT: build root (default: build); KSTONE_CMAKE_ARGS: extra configure arguments.
set -euo pipefail
cd "$(dirname "$0")/.."
command -v ninja > /dev/null || { echo "check.sh: ninja not in PATH; activate the build environment first" >&2; exit 1; }

presets=("$@")
[[ ${#presets[@]} -eq 0 ]] && presets=(debug release asan tsan)
root=${KSTONE_BUILD_ROOT:-build}
read -ra extra <<< "${KSTONE_CMAKE_ARGS:-}"

# The Python extension is built and tested when the active python has pybind11 (not under sanitizers).
python=()
pin_py=()
if py=$(command -v python3); then
    # Pin the probe to PATH's python3; FindPython3 would otherwise prefer Python3_ROOT_DIR (set by setup-python).
    pin_py=(-DPython3_EXECUTABLE="$py")
    if pybind11_dir=$("$py" -m pybind11 --cmakedir 2> /dev/null); then
        python=(-DKSTONE_BUILD_PYTHON=ON -DPython_EXECUTABLE="$py" -Dpybind11_DIR="$pybind11_dir")
    fi
fi

for p in "${presets[@]}"; do
    echo "== $p"
    case "$p" in
        debug|release) py_args=("${python[@]}") ;;
        *) py_args=(-DKSTONE_BUILD_PYTHON=OFF) ;;
    esac
    cmake --preset "$p" -B "$root/$p" "${pin_py[@]}" "${py_args[@]}" "${extra[@]}" > /dev/null
    cmake --build "$root/$p"
    if [[ "$root" == build ]]; then
        ctest --preset "$p"
    else
        # cmake 3.22 ignores --test-dir under --preset, so mirror the test preset by hand.
        case "$p" in
            asan) env=(ASAN_OPTIONS=detect_leaks=1:abort_on_error=1 UBSAN_OPTIONS=print_stacktrace=1) ;;
            tsan) env=(TSAN_OPTIONS=halt_on_error=1) ;;
            *) env=() ;;
        esac
        env "${env[@]}" ctest --test-dir "$root/$p" --output-on-failure --no-tests=error
    fi
done

# Install the release build and build an out-of-tree consumer against it.
if [[ " ${presets[*]} " == *" release "* ]]; then
    scripts/consumer.sh "$root/release"
fi
