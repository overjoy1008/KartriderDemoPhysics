#!/bin/sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_dir="$repo_dir/build-macos"
dist_dir="$repo_dir/dist-macos"

cmake -S "$repo_dir" -B "$build_dir" -G Xcode
cmake --build "$build_dir" --config Release
ctest --test-dir "$build_dir" -C Release --output-on-failure

mkdir -p "$dist_dir"
rm -f "$dist_dir/kart_topdown-macos.zip" "$dist_dir/kart_3d-macos.zip"
ditto -c -k --sequesterRsrc --keepParent \
    "$build_dir/Release/kart_topdown.app" "$dist_dir/kart_topdown-macos.zip"
ditto -c -k --sequesterRsrc --keepParent \
    "$build_dir/Release/kart_3d.app" "$dist_dir/kart_3d-macos.zip"

printf 'Created:\n  %s\n  %s\n' \
    "$dist_dir/kart_topdown-macos.zip" "$dist_dir/kart_3d-macos.zip"
