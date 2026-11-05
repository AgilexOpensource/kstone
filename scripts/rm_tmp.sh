#!/usr/bin/env bash
# Remove build and test output: ./scripts/rm_tmp.sh [-n]  (-n: only list what would be removed)
# Leaves .venv alone. KSTONE_BUILD_ROOT: build root (default: build).
set -euo pipefail
cd "$(dirname "$0")/.."
dry=0
[[ ${1:-} == -n ]] && dry=1
[[ $# -le $dry ]] || { echo "usage: $0 [-n]" >&2; exit 2; }

root=${KSTONE_BUILD_ROOT:-build}
targets=()
shopt -s nullglob
for p in "$root" ./*.log .mypy_cache .pytest_cache; do
    [[ -e $p ]] && targets+=("$p")
done
while IFS= read -r -d '' p; do targets+=("$p"); done < <(
    find . \( -path ./.git -o -path ./.venv -o -path ./third_party -o -path "./${root#./}" \) -prune -o \
        \( -name __pycache__ -o -name '*.egg-info' \) -type d -print0)
# Scratch dirs that test_wheel.sh leaves behind if it is killed.
while IFS= read -r -d '' p; do targets+=("$p"); done < <(
    find /tmp -maxdepth 1 -name 'kstone-wheeltest.*' -type d -user "$(id -u)" -print0)

[[ ${#targets[@]} -eq 0 ]] && { echo "nothing to remove"; exit 0; }
printf '%s\n' "${targets[@]}"
[[ $dry -eq 1 ]] || rm -rf -- "${targets[@]}"
