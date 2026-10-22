#!/usr/bin/env bash
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build="${A32_NATIVE_BUILD_DIR:-$here/build-linux}"
mkdir -p "$build"
build="$(cd -- "$build" && pwd)"
staged="$build/artifacts"
cmake -S "$here/native_linux" -B "$build" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DA32_NATIVE_OUTPUT="$staged"
cmake --build "$build" --parallel
ctest --test-dir "$build" --output-on-failure
mkdir -p "$here/bin"
# Replace the inode only after tests pass. Processes already using the library
# retain their existing mapping instead of seeing a truncated shared object.
published="$(mktemp "$here/bin/.liba32_native.so.XXXXXX")"
trap 'rm -f -- "$published"' EXIT
install -m 755 "$staged/liba32_native.so" "$published"
mv -f -- "$published" "$here/bin/liba32_native.so"
printf 'Built and tested: %s/bin/liba32_native.so\n' "$here"
