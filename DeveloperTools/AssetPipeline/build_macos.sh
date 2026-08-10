#!/bin/sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir="$repo_dir/.build-macos"
dist_dir="$repo_dir/build-macos"

# Do not require the full Xcode application. CMake's default generator also
# creates native .app bundles and works with the standalone Command Line Tools.
cmake -S "$repo_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" --config Release
ctest --test-dir "$build_dir" -C Release --output-on-failure

topdown_app="$build_dir/kart_topdown.app"
three_d_app="$build_dir/kart_3d.app"
if [ -d "$build_dir/Release/kart_topdown.app" ]; then
    topdown_app="$build_dir/Release/kart_topdown.app"
    three_d_app="$build_dir/Release/kart_3d.app"
fi

if [ ! -d "$topdown_app" ] || [ ! -d "$three_d_app" ]; then
    printf 'Could not find the built macOS application bundles.\n' >&2
    exit 1
fi

# CMake may leave only the linker's executable signature when using the
# standalone Command Line Tools. Re-sign each complete bundle so Gatekeeper's
# structural validation sees the generated Info.plist as part of the app.
codesign --force --deep --sign - "$topdown_app"
codesign --force --deep --sign - "$three_d_app"
codesign --verify --deep --strict "$topdown_app"
codesign --verify --deep --strict "$three_d_app"

mkdir -p "$dist_dir"
dist_topdown_app="$dist_dir/Kart Physics Top Down.app"
dist_three_d_app="$dist_dir/Kart Physics 3D.app"

rm -rf "$dist_topdown_app" "$dist_three_d_app"
ditto "$topdown_app" "$dist_topdown_app"
ditto "$three_d_app" "$dist_three_d_app"

codesign --verify --deep --strict "$dist_topdown_app"
codesign --verify --deep --strict "$dist_three_d_app"

rm -f "$dist_dir/kart_topdown-macos.zip" "$dist_dir/kart_3d-macos.zip"
ditto -c -k --sequesterRsrc --keepParent \
    "$dist_topdown_app" "$dist_dir/kart_topdown-macos.zip"
ditto -c -k --sequesterRsrc --keepParent \
    "$dist_three_d_app" "$dist_dir/kart_3d-macos.zip"

printf 'Created:\n  %s\n  %s\n  %s\n  %s\n' \
    "$dist_topdown_app" "$dist_three_d_app" \
    "$dist_dir/kart_topdown-macos.zip" "$dist_dir/kart_3d-macos.zip"
