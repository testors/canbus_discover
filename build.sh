#!/bin/bash
# Build script for CAN Bus Discover

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

echo "========================================"
echo "  CAN Bus Discover Build"
echo "========================================"
echo ""

# Check platform
if [[ "$(uname)" != "Darwin" ]]; then
    echo "Error: This program requires macOS (CoreBluetooth)."
    exit 1
fi

# Create build directory
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Configure and build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(sysctl -n hw.ncpu 2>/dev/null || echo 2)

echo ""
echo "Build complete: ${BUILD_DIR}/canbus_discover"
echo ""
echo "Usage:"
echo "  ./build/canbus_discover [device_prefix]"
echo ""
echo "  device_prefix: BLE name prefix (e.g., vLinker, OBDLink)"
echo "                 If omitted, connects to first discovered device."
echo ""
echo "Note: Requires Bluetooth permission in System Settings > Privacy & Security."
echo ""
