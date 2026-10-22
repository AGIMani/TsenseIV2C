#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

if [[ "$(uname -s)" != "Linux" ]]; then
    echo "build.sh is for Linux. On Windows use build.bat."
    exit 1
fi

if [[ -f build-linux/CMakeCache.txt ]] && ! grep -q 'CMAKE_GENERATOR:INTERNAL=Unix Makefiles' build-linux/CMakeCache.txt; then
    echo "build-linux/ was configured with another generator; removing it."
    rm -rf build-linux
fi

cmake -S . -B build-linux -DCMAKE_BUILD_TYPE=Release
cmake --build build-linux -j"$(nproc)"

echo
echo "HTML teach pendant: ./build-linux/a3_web_bridge"
echo "Example:            ./build-linux/a3_example --list"
echo "Browser:            http://127.0.0.1:8080/"
echo "Open the H7 CDC node (/dev/ttyACM*, VID 0483 PID 5740). Do not use the ST-Link VCP."
