#!/bin/bash
# 在树莓派上一键部署摄像头推流服务
#
# 用法：把 deploy/ 整个目录复制到树莓派，然后
#   cd deploy && sudo bash install.sh
#
# 前提：USB UVC 摄像头已插好，且支持 MJPEG 输出
set -e

if [ "$(id -u)" -ne 0 ]; then
  echo "请用 sudo 运行" >&2
  exit 1
fi

SRC="$(cd "$(dirname "$0")" && pwd)"
RUN_USER="${SUDO_USER:-pi}"

echo "==> 检查摄像头"
if ! v4l2-ctl --list-devices 2>/dev/null | grep -q video0; then
  echo "警告: 未检测到 /dev/video0，请确认摄像头已插好"
fi
v4l2-ctl -d /dev/video0 --list-formats-ext 2>/dev/null | grep -A1 MJPG | head -20 || true

echo "==> 安装 ustreamer"
apt-get update -qq
apt-get install -y ustreamer v4l-utils

echo "==> 部署文件"
install -d /opt/camstream/www
install -m 644 "$SRC/www/index.html"     /opt/camstream/www/index.html
install -m 755 "$SRC/bench.sh"           /opt/camstream/bench.sh
install -m 755 "$SRC/camres"             /usr/local/bin/camres
install -m 644 "$SRC/ustreamer.env"      /etc/default/ustreamer
install -m 644 "$SRC/ustreamer.service"  /etc/systemd/system/ustreamer.service

# 服务以哪个用户运行取决于实际环境，需在 video 组内
sed -i "s/^User=.*/User=$RUN_USER/" /etc/systemd/system/ustreamer.service
echo "==> 服务运行用户: $RUN_USER"

echo "==> 启动服务"
systemctl daemon-reload
systemctl enable --now ustreamer
sleep 3

systemctl is-active ustreamer
IP=$(ip -4 addr show scope global | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -1)
echo
echo "部署完成，浏览器打开：http://$IP:8080/"
echo "切换分辨率：camres 1920x1080"
