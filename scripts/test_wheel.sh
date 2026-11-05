#!/usr/bin/env bash
# Test an installed kstone wheel: ./scripts/test_wheel.sh [python]
# The python (default: python3) must have kstone and mypy installed.
# Runs from a mktemp dir under /tmp, which is removed on exit.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
py=$(command -v "${1:-python3}")
bin=$(dirname "$py")
export PYTHONDONTWRITEBYTECODE=1
unset LD_LIBRARY_PATH

tmp=$(mktemp -d /tmp/kstone-wheeltest.XXXXXX)
trap 'rm -rf "$tmp"' EXIT
cd "$tmp"

ext=$("$py" -c 'import kstone._core; print(kstone._core.__file__)')
echo "== $ext"
if command -v readelf > /dev/null; then readelf -d "$ext" | grep -E "RPATH|RUNPATH"; fi
ldd "$ext" | grep -E "pinocchio|coal|urdfdom|not found" || true

echo "== test installed package"
KSTONE_TOKEN= "$py" "$repo/python/tests/test_binding.py" "$repo/tests/fixtures"
"$py" -c "import kstone; kstone.doctor(print=True)"
"$bin/kstone" check "$repo/examples/omni_dual_arm/omni_dual_arm.yaml"
"$py" "$repo/examples/python/ik.py" "$repo/tests/fixtures/arm.urdf"
"$py" "$repo/examples/python/engine.py" "$repo/tests/fixtures/arm_loopback.yaml"

echo "== check type stubs"
"$py" "$repo/python/tests/check_stubs.py"
