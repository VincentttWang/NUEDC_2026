# tools — 开发与调试工具

`tools` 只保存电脑端开发工具、J-Link 命令文件，以及通过 raw REPL 临时送入 K230 的有界辅助脚本。
正式板端程序放在 `k230/`，MSPM0 固件源码放在 `app/`、`middleware/`、`core/` 和 `bsp/`。

## 目录

| 目录 | 内容 |
|---|---|
| `checks/` | Keil MDK 5/6 工程输入一致性检查 |
| `jlink/` | MSPM0 烧录、探测、寄存器读取和恢复命令文件 |
| `k230/` | K230 USB/Wi-Fi 部署、CanMV MCP、RTSP 查看和数据集工具 |
| `tools/k230/runners/` | 经 raw REPL 执行已上传板端脚本的薄包装 |
| `tools/k230/probes/` | 短时、可终止的 K230 能力与链路诊断脚本 |
| `visualizers/` | MSPM0 调试串口遥测可视化 |

## 常用入口

```powershell
# 检查两套 Keil 工程输入
python tools/checks/check_keil_project_sync.py

# 检查 Markdown 链接和已迁移的旧工具路径
python tools/checks/check_docs.py

# K230 USB raw REPL
python tools/k230/k230_tool.py list

# K230 Wi-Fi 远程开发代理
python tools/k230/k230_remote.py --host <K230-IP> status

# RTSP 查看和统计
python tools/k230/k230_video_viewer.py rtsp://<K230-IP>:8554/test --stats-frames 100

# 底盘遥测
python tools/visualizers/straight_test_viz.py --list
python tools/visualizers/speed_pid_viz.py --list
python tools/visualizers/track_follow_viz.py --list
python tools/visualizers/ball_balance_viz.py --list

# H 题循迹实时遥测（含 JY61P 航向角，默认加速度告警线 ±0.12 m/s²）
python tools/visualizers/track_follow_viz.py --port COM7
python tools/visualizers/track_follow_viz.py --port COM7 --csv track.csv --log track_raw.txt

# H3 静止守球实时遥测
python tools/visualizers/ball_balance_viz.py --port COM7 --csv ball.csv --log ball_raw.txt

# H3 S 曲线主机仿真（编译、运行全部场景并绘图）
python tools/visualizers/scurve_sim_plot.py --build --run
```

H3 图中同时显示 `xr/x` 原始/控制位置、`xref/vref/aref` 在线五次预瞄参考、实际水管角
得到的 `a0`，以及 `aff/fbp/fbv` 前馈、位置反馈和速度反馈。约束面板显示终端残差
`rx/rv/ra`；CSV 中 `ia/iv` 应分别等于 `-v` 和目标位移 `et`，可直接检查加速度/速度积分
约束。`motor/beam` 同时记录电机相对水平轴角和查表水管角，供连杆与加速度关系辨识；
`lim/brk/stuck` 用于判断继承超限状态、扰动制动与工具截停。`mspd/mkp/sf` 与
`arr/cc/ctr/sdt/smax` 用于核对步进位置环、TIMG6 PWM 装填和调度及时性。

航向角使用固件 `[TRK]` 遥测中的 `yaw` 字段，在独立曲线和窗口顶部显示，并随其他字段写入 CSV。

K230 的完整连接和恢复流程见 [`../docs/k230-development.md`](../docs/k230-development.md) 与
[`../docs/k230-remote-development.md`](../docs/k230-remote-development.md)。探针的用途见
[`tools/k230/probes/README.md`](k230/probes/README.md)。

## 维护约定

- 能长期运行或上电自启的 K230 程序放在仓库根目录 `k230/`，不要放入 `tools/k230/probes/`。
- 探针必须有界或可由 `Ctrl-C` 退出，不应承担正式功能。
- 新增探针时同步更新 `tools/k230/probes/README.md`。
- 已删除子系统专用工具应随子系统一起移除；历史背景保留在 `docs/changelog.md`，不把失效工具留在根目录。
