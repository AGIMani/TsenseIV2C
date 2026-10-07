#!/usr/bin/env bash
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build="${A32_NATIVE_BUILD_DIR:-$here/build-linux}"
cmake -S "$here/native_linux" -B "$build" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "$build" --parallel
ctest --test-dir "$build" --output-on-failure
printf 'Built and tested: %s/bin/liba32_native.so\n' "$here"
