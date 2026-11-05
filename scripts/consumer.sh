#!/usr/bin/env bash
# Install a kstone build tree and build an out-of-tree consumer against it: ./scripts/consumer.sh <build-dir>
# Uses a mktemp dir, which is removed on exit.
set -euo pipefail
cd "$(dirname "$0")/.."
build=${1:?usage: consumer.sh <build-dir>}

echo "== consumer"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cmake --install "$build" --prefix "$tmp/prefix" > /dev/null
# Reuse the exact dependency packages the build resolved, whichever environment they came from.
deps=()
while IFS= read -r d; do deps+=("-D$d"); done < <(
    sed -n -E 's/^([A-Za-z0-9_.+-]+_DIR):PATH=(.+)$/\1=\2/p' "$build/CMakeCache.txt" | grep -v NOTFOUND)
cmake -S tests/consumer -B "$tmp/build" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$tmp/prefix" "${deps[@]}" > /dev/null
cmake --build "$tmp/build"
"$tmp/build/consumer" tests/fixtures/arm.urdf
