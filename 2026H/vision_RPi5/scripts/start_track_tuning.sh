#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${repo_dir}/.venv/bin/python"
service_name="vision-scale-uart.service"
service_was_active=0

# The visual debugger uses the same lock.  Although it does not open UART,
# allowing it to restore the production service while tuning is unsafe.
exec 9>"/tmp/nuedc-vision-debug.lock"
if ! flock -n 9; then
  command -v notify-send >/dev/null 2>&1 \
    && notify-send "H题赛道参数调试台" "视觉调试台或参数调试台已经在运行"
  exit 1
fi

restore_service() {
  if [[ "${service_was_active}" -eq 1 ]]; then
    sudo -n systemctl start "${service_name}" || true
  fi
}

trap restore_service EXIT
trap 'exit 130' INT TERM HUP

if systemctl is-active --quiet "${service_name}"; then
  if ! sudo -n systemctl stop "${service_name}"; then
    command -v notify-send >/dev/null 2>&1 \
      && notify-send -u critical "H题赛道参数调试台" "无法暂停正式视觉服务，调试台未启动"
    exit 1
  fi
  service_was_active=1
fi

if [[ ! -x "${python_bin}" ]]; then
  python_bin="$(command -v python3)"
fi

cd "${repo_dir}"
"${python_bin}" -m vision_function.track_tuning.app "$@"
