#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="build/Release"

if [ ! -f "${BUILD_DIR}/flight_controller_sim" ]; then
    echo "Simulation binary not found. Building project first..."
    ./scripts/build.sh Release
fi

echo "=========================================================="
echo " Launching Flight Controller FreeRTOS Real-Time Simulator "
echo "=========================================================="

"${BUILD_DIR}/flight_controller_sim"
