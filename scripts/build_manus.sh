#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
MANUS_PYTHON="${1:-$ROOT/.venvs/track/bin/python}"
if [[ ! -x "$MANUS_PYTHON" ]]; then
  printf '%s\n' '用法：bash scripts/build_manus.sh /绝对路径/python（使用运行网页服务的解释器）' >&2
  exit 1
fi
if [[ "$(uname -sm)" != 'Linux x86_64' ]]; then
  printf '%s\n' '随仓库提供的 Manus 3.1.1 运行库仅支持 Linux x86_64。' >&2
  exit 1
fi
"$MANUS_PYTHON" - <<'PY'
from pathlib import Path
import sysconfig
if not (Path(sysconfig.get_path('include')) / 'Python.h').is_file():
    raise SystemExit('缺少 Python 开发头文件。Ubuntu Python 3.12 请安装 python3.12-dev，或传入带开发头文件的 Conda Python。')
PY
MANUS_PYBIND_DIR="$("$MANUS_PYTHON" -m pybind11 --cmakedir)"
cmake -S "$ROOT/vendor/manussdk/bindings/python" -B "$ROOT/vendor/manussdk/build/python" \
  -DCMAKE_BUILD_TYPE=Release -Dpybind11_DIR="$MANUS_PYBIND_DIR" -DPython_EXECUTABLE="$MANUS_PYTHON"
cmake --build "$ROOT/vendor/manussdk/build/python" --parallel 2
ctest --test-dir "$ROOT/vendor/manussdk/build/python" --output-on-failure
"$MANUS_PYTHON" - "$ROOT/vendor/manussdk/build/python" <<'PY'
import sys
sys.path.insert(0, sys.argv[1])
import manus_glove
assert hasattr(manus_glove.ManusGlove, 'get_raw_skeleton_snapshot')
glove = manus_glove.ManusGlove()
snapshot = glove.get_raw_skeleton_snapshot('left')
assert snapshot['skeleton'].shape == (0, 10)
assert snapshot['node_ids'] == [] and not snapshot['valid']
glove.request_disconnect()
# Cancellation before initialization must stay offline and be safe to repeat.
assert not glove.connect(mode='integrated', world_coordinates=True)
glove.disconnect()
glove.disconnect()
print(f'Manus SDK 编译和导入通过：{manus_glove.__file__}')
PY
