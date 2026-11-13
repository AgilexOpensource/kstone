#!/usr/bin/env bash
# Check C/C++ formatting with the pinned clang-format: ./scripts/format.sh [-i] [--probe] [file...]
# -i rewrites files in place; --probe only tests that the pinned version is available; no files means all tracked sources.
set -euo pipefail
cd "$(dirname "$0")/.."
version=21.1.8
bin=${CLANG_FORMAT:-clang-format}

mode=(--dry-run --Werror)
probe=0
files=()
for a in "$@"; do
    case "$a" in
        -i) mode=(-i) ;;
        --probe) probe=1 ;;
        *) files+=("$a") ;;
    esac
done

have=$("$bin" --version 2> /dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n 1 || true)
if [[ "$have" != "$version" ]]; then
    [[ $probe -eq 1 ]] && exit 1
    echo "format.sh: need clang-format $version, found '${have:-none}'; run: pip install clang-format==$version" >&2
    exit 1
fi
[[ $probe -eq 1 ]] && exit 0

if [[ ${#files[@]} -eq 0 ]]; then
    git ls-files -z -- '*.c' '*.h' '*.cpp' '*.hpp' | xargs -0 "$bin" "${mode[@]}"
else
    "$bin" "${mode[@]}" "${files[@]}"
fi
