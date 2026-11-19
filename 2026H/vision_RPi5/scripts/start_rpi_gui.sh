#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${repo_dir}/.venv/bin/python"
service_name="vision-scale-uart.service"

exec 9>"/tmp/nuedc-vision-debug.lock"
if ! flock -n 9; then
  command -v notify-send >/dev/null 2>&1 \
    && notify-send "H题视觉调试台" "调试台已经在运行"
  exit 1
fi

if ! systemctl is-active --quiet "${service_name}"; then
  if ! sudo -n systemctl start "${service_name}"; then
    command -v notify-send >/dev/null 2>&1 \
      && notify-send -u critical "H题视觉调试台" "正式识别服务未运行，监看台未启动"
    exit 1
  fi
fi

if [[ ! -x "${python_bin}" ]]; then
  python_bin="$(command -v python3)"
fi

cd "${repo_dir}"
export NUEDC_DEBUG_MONITOR_DIR=/dev/shm/nuedc-vision-debug
export NUEDC_DEBUG_MONITOR_ONLY=1
"${python_bin}" -m vision_function.debug_ui.app "$@"
