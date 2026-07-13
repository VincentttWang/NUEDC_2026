# tools/probes — 板上调试探测脚本

诊断/探测用途的脚本。多数经 `../k230_tool.py run <脚本>` 在 K230 上执行，
`read_console.py` 为 PC 端直接运行。

| 脚本 | 用途 |
|------|------|
| `read_console.py <COM> [秒]` | PC 端只读监听 K230 串口输出（不打断板上程序） |
| `cvlite_rect_bench.py` | cv_lite 矩形检测有界 bench（帧率/命中统计） |
| `cvlite_live.py` | cv_lite 实时检测遥测 |
| `k230_status_probe.py` | K230 运行时与文件系统状态 |
| `k230_ai_inventory.py` | 列出板上 AI 模型/示例清单 |
| `k230_yolo_probe.py` | YOLOv8n 有界推理探测 |
| `k230_rect_probe.py` | 旧 `find_rects` 候选探测（已被 cv_lite 取代，保留备查） |
