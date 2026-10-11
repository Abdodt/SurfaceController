#!/usr/bin/env bash
set -euo pipefail

BUILD_TYPE="${1:-Release}"
BUILD_DIR="build/${BUILD_TYPE}"

echo "=========================================================="
echo " Building Flight Surface Controller (${BUILD_TYPE}) "
echo "=========================================================="

/opt/homebrew/bin/cmake -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DBUILD_TESTING=ON

/opt/homebrew/bin/cmake --build "${BUILD_DIR}" -j"$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)"

echo ""
echo "Build succeeded. Binaries located in ${BUILD_DIR}/"
