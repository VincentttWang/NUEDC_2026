#!/bin/bash
# 基准测试：不同分辨率下 MJPEG 的实际帧大小与码率
PORT=8098

test_cfg() {
  desc="$1"; shift
  ustreamer "$@" --host=127.0.0.1 --port=$PORT --log-level=0 >/dev/null 2>&1 &
  pid=$!
  sleep 5
  tot=0
  for i in 1 2 3 4 5 6; do
    s=$(curl -s -o /dev/null -w "%{size_download}" http://127.0.0.1:$PORT/snapshot 2>/dev/null)
    tot=$((tot + s))
    sleep 0.25
  done
  avg=$((tot / 6))
  fps=$(curl -s http://127.0.0.1:$PORT/state 2>/dev/null | grep -o 'captured_fps": [0-9]*' | grep -o '[0-9]*')
  [ -z "$fps" ] && fps=0
  mbps=$(awk "BEGIN{printf \"%.0f\", $avg*8*$fps/1000000}")
  echo "$desc | 帧=$((avg / 1024))KB | 采集=${fps}fps | 满速码率约 ${mbps}Mbps"
  kill $pid 2>/dev/null
  wait $pid 2>/dev/null
  sleep 1
}

test_cfg "1920x1080" --device=/dev/video0 -m MJPEG -c HW -r 1920x1080 -f 60 -b 3
test_cfg "1280x720 " --device=/dev/video0 -m MJPEG -c HW -r 1280x720  -f 60 -b 3
test_cfg "800x600  " --device=/dev/video0 -m MJPEG -c HW -r 800x600   -f 60 -b 3
