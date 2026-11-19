#!/usr/bin/env bash
# Deploy shallow CNN ball detector to Raspberry Pi and install as systemd service.
#
# Usage: bash deploy_shallow_cnn.sh
#
# Prerequisites on the Windows/Mac client:
#   - sshpass installed (choco install sshpass / brew install sshpass / apt install sshpass)
#
set -euo pipefail

RPI_HOST="10.190.177.67"
RPI_USER="nailong"
RPI_PASS="nailong"
RPI_PROJECT="/home/nailong/NUEDC_2026_H_VISION"
LOCAL_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"

SSH_CMD="sshpass -p '${RPI_PASS}' ssh -o StrictHostKeyChecking=no ${RPI_USER}@${RPI_HOST}"
SCP_CMD="sshpass -p '${RPI_PASS}' scp -o StrictHostKeyChecking=no"

echo "=== 1. Pull latest code on RPi ==="
${SSH_CMD} "cd ${RPI_PROJECT} && git stash && git pull --rebase origin agent/shallow-cnn-local"

echo ""
echo "=== 2. Copy modified source files ==="
${SCP_CMD} "${LOCAL_ROOT}/vision_function/ball_detection/shallow_cnn/ncnn_runtime.py" \
    "${RPI_USER}@${RPI_HOST}:${RPI_PROJECT}/vision_function/ball_detection/shallow_cnn/ncnn_runtime.py"
${SCP_CMD} "${LOCAL_ROOT}/vision_function/scale_location/runtime.py" \
    "${RPI_USER}@${RPI_HOST}:${RPI_PROJECT}/vision_function/scale_location/runtime.py"

echo ""
echo "=== 3. Install new systemd service ==="
${SCP_CMD} "${LOCAL_ROOT}/vision_function/scale_location/deploy/vision-scale-shallow-cnn.service" \
    "${RPI_USER}@${RPI_HOST}:/tmp/vision-scale-shallow-cnn.service"
${SSH_CMD} "sudo cp /tmp/vision-scale-shallow-cnn.service /etc/systemd/system/ && sudo systemctl daemon-reload"

echo ""
echo "=== 4. Stop old YOLO service, enable shallow CNN service ==="
${SSH_CMD} "sudo systemctl stop vision-scale-uart.service 2>/dev/null || true; \
    sudo systemctl disable vision-scale-uart.service 2>/dev/null || true; \
    sudo systemctl enable vision-scale-shallow-cnn.service"

echo ""
echo "=== 5. Test run (dry-run first, 5 iterations) ==="
${SSH_CMD} "cd ${RPI_PROJECT} && .venv/bin/python -m vision_function.scale_location.runtime \
    --detector shallow_cnn \
    --shallow-cnn-model vision_function/ball_detection/model/shallow_cnn_v3_ncnn_model \
    --shallow-cnn-presence 0.5 \
    --shallow-cnn-heatmap 0.25 \
    --dry-run \
    --iterations 5 \
    --json-lines"

echo ""
echo "=== 6. Start shallow CNN service ==="
${SSH_CMD} "sudo systemctl start vision-scale-shallow-cnn.service && sleep 3 && sudo systemctl status vision-scale-shallow-cnn.service --no-pager -l"

echo ""
echo "=== Deployment complete! ==="
echo "Check logs:  ssh ${RPI_USER}@${RPI_HOST} journalctl -u vision-scale-shallow-cnn.service -f"
echo "Stop service: ssh ${RPI_USER}@${RPI_HOST} sudo systemctl stop vision-scale-shallow-cnn.service"
