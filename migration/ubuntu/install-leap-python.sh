#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/common.sh"
PROJECT="$HAND_PROJECT_ROOT"
PYTHON="$HAND_VENV_ROOT/track/bin/python"
[[ -x "$PYTHON" ]] || { echo "先运行 bash migration/ubuntu/install.sh track"; exit 1; }
"$PYTHON" -c 'import platform,sys; assert sys.version_info[:2] == (3,12) and platform.machine() == "x86_64", "提供的 wheel 仅适用 CPython 3.12 / x86_64"'
[[ -f /usr/lib/ultraleap-hand-tracking-service/libLeapC.so ]] || {
  echo "请先按 README-UBUNTU.txt 安装 Ultraleap Linux 服务并执行 ldconfig。"; exit 1;
}
"$PYTHON" -m pip install "$HERE"/ultraleap/wheels/*.whl
export LEAPSDK_INSTALL_LOCATION="$("$PYTHON" -c 'import sysconfig; print(sysconfig.get_path("platlib"))')"
"$PYTHON" "$PROJECT/tools/patch_leapmotion_sdk.py"
"$PYTHON" -m pip check
"$PYTHON" -c 'import leap; print("Leap Python binding:", leap.__file__)'
echo "Leap Python binding 已安装；启动工作台后选择 Leap Motion 2。"
