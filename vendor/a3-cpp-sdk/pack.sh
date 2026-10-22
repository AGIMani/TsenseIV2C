#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

if [[ ! -f build-linux/CMakeCache.txt ]]; then
    echo "Run bash build.sh first."
    exit 1
fi

cmake --build build-linux -j"$(nproc)"
ctest --test-dir build-linux --output-on-failure --no-tests=error
cmake --install build-linux --prefix dist/a3-cpp-sdk-linux
tar -czf dist/a3-cpp-sdk-linux.tar.gz -C dist a3-cpp-sdk-linux

echo
echo "SDK pack: dist/a3-cpp-sdk-linux"
echo "  include/   public headers"
echo "  lib/       liba3_sdk.a"
echo "  bin/       a3_example a3_web_bridge"
echo "  99-a3-h7-cdc.rules"
echo "Ship that folder, not the H7 firmware tree."
