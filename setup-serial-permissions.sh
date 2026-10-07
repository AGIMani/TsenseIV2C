#!/usr/bin/env bash
# Configure native Linux serial access for the user running all project scripts.
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd -P)"
target_user="${SUDO_USER:-$(id -un)}"
check_only=false

usage() {
  cat <<'HELP'
用法：
  bash setup-serial-permissions.sh                 配置当前用户（自动请求 sudo）
  bash setup-serial-permissions.sh --check         只检查，不修改、不请求 sudo
  sudo bash setup-serial-permissions.sh --user 用户名

给运行 track/play 等脚本的用户配置权限，无需分别修改每个启动脚本。
永久设置：加入 dialout 组，并安装项目自带的 H7 udev 规则。
立即生效：给当前 /dev/ttyACM*、/dev/ttyUSB* 添加该用户的读写 ACL。
组权限需要注销并重新登录一次；随后重新插拔串口也可正常访问。
本脚本不会打开串口、发送指令或启动机械手。
HELP
}

while (($#)); do
  case "$1" in
    --check) check_only=true; shift ;;
    --user)
      if (($# < 2)) || [[ -z "$2" || "$2" == -* ]]; then
        echo '--user 后面需要用户名。' >&2; exit 2
      fi
      target_user="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf '未知参数：%s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if ! id -- "$target_user" >/dev/null 2>&1; then
  printf '用户不存在：%s\n' "$target_user" >&2; exit 1
fi
if [[ "$(id -u -- "$target_user")" == 0 ]]; then
  echo '请使用普通用户运行，或通过 --user 指定实际运行网页的普通用户。' >&2
  exit 1
fi

shopt -s nullglob
ports=(/dev/ttyACM* /dev/ttyUSB*)
rule_source="$PROJECT_DIR/migration/ubuntu/70-hand-h7.rules"
rule_target=/etc/udev/rules.d/70-hand-h7.rules

if "$check_only"; then
  printf '目标用户：%s\n已登记的用户组：%s\n' "$target_user" "$(id -nG -- "$target_user")"
  if [[ "$target_user" == "$(id -un)" ]]; then
    printf '当前进程用户组：%s\n' "$(id -nG)"
  fi
  if [[ -f "$rule_target" ]] && cmp -s -- "$rule_source" "$rule_target"; then
    echo 'H7 udev 规则：已安装，与项目一致'
  else
    echo 'H7 udev 规则：未安装或与项目不同'
  fi
  if ((${#ports[@]} == 0)); then
    echo '未发现 ttyACM/ttyUSB 串口，请连接设备后再检查。'
  fi
  for port in "${ports[@]}"; do
    [[ -c "$port" ]] || continue
    ls -l -- "$port"
    if command -v getfacl >/dev/null; then getfacl -cp -- "$port"; fi
    if [[ "$target_user" == "$(id -un)" ]]; then
      if [[ -r "$port" && -w "$port" ]]; then
        echo '当前进程：有读写权限'
      else
        echo '当前进程：缺少读写权限'
      fi
    fi
  done
  exit 0
fi

if ((EUID != 0)); then
  exec sudo -- bash "$PROJECT_DIR/setup-serial-permissions.sh" --user "$target_user"
fi

for dependency in usermod install udevadm setfacl; do
  if ! command -v "$dependency" >/dev/null; then
    printf '缺少命令：%s\n' "$dependency" >&2
    if [[ "$dependency" == setfacl ]]; then echo '请先执行：sudo apt install acl' >&2; fi
    exit 1
  fi
done
[[ -f "$rule_source" ]] || { echo "缺少项目规则：$rule_source" >&2; exit 1; }
getent group dialout >/dev/null || { echo '系统不存在 dialout 组。' >&2; exit 1; }

usermod -aG dialout -- "$target_user"
install -m 0644 -- "$rule_source" "$rule_target"
udevadm control --reload-rules

for port in "${ports[@]}"; do
  [[ -c "$port" && ! -L "$port" ]] || continue
  setfacl -m "u:${target_user}:rw" -- "$port"
  printf '已授予 %s 读写权限：%s\n' "$target_user" "$port"
done

printf '\n已配置用户 %s 的串口权限，适用于该用户启动的所有项目脚本。\n' "$target_user"
if ((${#ports[@]})); then
  echo '当前串口已立即授权，可直接在网页重试连接。'
else
  echo '尚未发现串口；连接设备后可重新运行本脚本获得即时权限。'
fi
echo '请注销并重新登录一次，使 dialout 组权限对后续插拔和所有新进程生效。'
echo '日常启动 track/play 请使用普通用户，不要加 sudo。'
