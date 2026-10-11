#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="build/Release"

if [ ! -f "${BUILD_DIR}/tests/unit_tests" ]; then
    echo "Tests binary not found. Building project first..."
    ./scripts/build.sh Release
fi

echo "=========================================================="
echo " Running GoogleTest Unit Test Suite "
echo "=========================================================="

"${BUILD_DIR}/tests/unit_tests" --gtest_color=yes
