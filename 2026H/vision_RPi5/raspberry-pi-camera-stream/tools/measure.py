"""测量推流的实际吞吐与到达帧率，用于判断链路是否吃紧。

    python tools/measure.py [测量秒数]

到达帧率明显低于服务端采集帧率，说明帧在网络上排队，延迟会持续累积。
连接信息见 rpi_config.py（从环境变量读取）。
"""
import json
import sys
import time
import urllib.request

from rpi_config import BASE_URL

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0

# 绕过系统代理，直连局域网
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

r = opener.open(f"{BASE_URL}/stream", timeout=10)
boundary = b"--boundarydonotcross"

t0 = time.time()
total = 0
frames = 0
buf = b""

while time.time() - t0 < DUR:
    chunk = r.read(65536)
    if not chunk:
        break
    total += len(chunk)
    buf += chunk
    n = buf.count(boundary)
    if n:
        frames += n
        # 关键：连同 boundary 本身一起丢弃，否则下一轮会重复计数
        buf = buf[buf.rfind(boundary) + len(boundary):]
r.close()

el = time.time() - t0
mbps = total * 8 / el / 1e6
fps = frames / el
avg = total / max(frames, 1)

print(f"下行吞吐    : {mbps:.1f} Mbps  ({total/1024/1024:.1f} MB / {el:.1f}s)")
print(f"到达帧率    : {fps:.1f} fps")
print(f"平均帧大小  : {avg/1024:.0f} KB")
print(f"单帧传输耗时: {avg*8/(mbps*1e6)*1000:.0f} ms  (≈ 网络造成的延迟下限)")

s = json.load(opener.open(f"{BASE_URL}/state", timeout=5))["result"]
res = s["source"]["resolution"]
print(f"服务端采集  : {s['source']['captured_fps']} fps @ {res['width']}x{res['height']}")
