#!/usr/bin/env bash
set -euo pipefail

# Directories
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
QE_DIR="$(dirname "$PROJECT_DIR")"

# Orange Pi SSH address
PI="dietpi@10.90.15.11"

echo "=== Configuring Umbra ==="

podman run --rm \
    -v "$QE_DIR:/work" \
    -w /work/Umbra \
    umbra-cross \
    cmake -S . -B build/arm64 \
        -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE=/work/Umbra/cmake/aarch64-dietpi.cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DOpenCV_DIR=/work/umbra-sysroot/usr/lib/aarch64-linux-gnu/cmake/opencv4 \
        -Dnlohmann_json_DIR=/work/umbra-sysroot/usr/share/cmake/nlohmann_json

echo "=== Building Umbra ==="

podman run --rm \
    -v "$QE_DIR:/work" \
    -w /work/Umbra \
    umbra-cross \
    cmake --build build/arm64 --parallel 4

echo "=== Deploying test executable ==="

rsync -av --progress \
    "$PROJECT_DIR/build/arm64/Umbra" \
    "$PI:/home/dietpi/Umbra/Umbra-test"

echo "=== Deployment complete ==="
echo "Test executable: /home/dietpi/Umbra/Umbra-test"
echo "The production umbra.service was not changed."
