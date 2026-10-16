#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/native-tests"

cmake_args=(
    -S "$project_dir" -B "$build_dir" -G Ninja
    -DUMBRA_BUILD_APP=OFF -DUMBRA_BUILD_TESTS=ON
    -DCMAKE_BUILD_TYPE=RelWithDebInfo
)

# Arch currently packages OpenCV 5 under opencv5, while the Pi sysroot uses 4.10.
# Pass OpenCV_DIR explicitly if your host package is installed elsewhere.
if [[ -z "${OpenCV_DIR:-}" && -f /usr/lib/cmake/opencv5/OpenCVConfig.cmake ]]; then
    cmake_args+=(-DOpenCV_DIR=/usr/lib/cmake/opencv5)
elif [[ -n "${OpenCV_DIR:-}" ]]; then
    cmake_args+=(-DOpenCV_DIR="$OpenCV_DIR")
fi

cmake "${cmake_args[@]}"
cmake --build "$build_dir" --parallel 4
ctest --test-dir "$build_dir" --output-on-failure "$@"
