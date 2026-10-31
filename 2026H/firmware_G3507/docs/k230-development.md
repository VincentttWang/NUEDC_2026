# K230 固件与开发连接

> **⚠ 时效说明**：K230 是 2025E 赛题时期的视觉方案。2026H 改用树莓派承担视觉与图传
> （复用原先接 K230 的那路串口），固件侧的视觉循迹中间件已随外设精简移除。本文与
> `k230/`、`tools/k230/` 下的脚本作为历史资料保留，**不描述当前在用的系统**。

本文记录本项目当前使用的 K230 固件、升级注意事项和首选开发连接流程。

## 当前基线

| 项目 | 当前值 |
|---|---|
| 开发板 | CanMV K230 01Studio - 1G |
| 固件 | CanMV v1.8 01Studio release |
| 完整版本 | `K230-v1.8-0-gc2d1f5cc994c206d0032aaf6aaf09332f3dc3c4c` |
| 板端名称 | `k230_canmv_01studio` |
| VS Code 扩展 | `kendryte747.canmv-vscode` 0.9.6 |
| USB 调试端口 | 当前机器为 `COM15`，实际使用时应重新检测 |
| 板端主程序 | `/sdcard/main.py` |

固件从 [kendryte/canmv_k230 v1.8 release](https://github.com/kendryte/canmv_k230/releases/tag/v1.8)
获取，必须选择与 **01Studio** 板型完全对应的镜像，不可使用其他 K230/K230D 板型镜像。v1.8 更新了 IDE
调试协议、UART/USB、VICAP 流管理、Sensor/CSC 像素格式和显示绑定，因此旧固件上的媒体初始化写法不能
直接视为兼容。

## 更新固件

1. 记录当前板型和固件完整版本，备份 `/sdcard/main.py`、标定参数和其他板端文件。
2. 从官方 v1.8 release 下载 01Studio 对应镜像，并按同页提供的校验文件核对镜像。
3. 使用 01Studio 板卡支持的烧录方式写入镜像。不要保留或混用其他板型的启动文件。
4. 固件更新后把 `/sdcard` 视为可能已被清空；本次升级后原 `main.py` 确实需要重新部署。
5. 用下文的 VS Code CanMV 扩展连接，读取板卡和固件信息，确认板型、版本与上述基线一致。
6. 重新写入 `k230/vision_red_line_follow.py` 为 `/sdcard/main.py`，先由扩展显式运行并检查 Preview，
   再用真正的系统复位验证脱离 IDE 后的开机自启。

固件升级会改变底层媒体行为。升级后至少复测相机模式、双通道视场、LCD 视频层、OSD、Preview、处理帧率
和相机方向，不能只验证脚本能够导入。

## 首选连接方式：VS Code CanMV 扩展

以后日常开发、运行、文件管理、终端输出和相机取帧主要使用 VS Code 的 CanMV 扩展。扩展提供板卡检测、
连接、脚本运行、远程文件、Preview，以及同等能力的 `canmv.mcp` 服务。

### VS Code 界面流程

1. 安装并启用 `kendryte747.canmv-vscode`，打开侧边栏的 CanMV 面板。
2. 执行 `CanMV: 连接开发板`，选择自动检测到的 `CanMV K230`。当前机器通常是 `COM15`，不要写死到
   其他电脑。
3. **连接会使 K230 执行 MicroPython 软复位。** 看到 `MPY: soft reboot` 是正常行为；连接前正在运行的
   `/sdcard/main.py` 会停止，并且本固件不会因此自动恢复项目主循环。
4. 连接完成后，在远程文件树中显式运行 `/sdcard/main.py`，或打开本地脚本后执行
   `CanMV: 在 K230 上运行当前文件`。
5. 先确认终端出现 `vision_v18 sensor=...` 且脚本状态为 running，再执行 `CanMV: 启用预览`。
6. Preview 用于检查完整 800×480 图像、LCD/OSD 合成、相机方向、红线采样点和板端处理帧率。
7. 写入开机程序时使用扩展的“另存为 main.py”或远程文件上传功能；完成后用系统级复位单独验证开机路径。

扩展设置中的 `canmv.baudRate` 默认值为 `12000000`，它是 USB IDE 调试协议参数，不是 K230 与 MSPM0
之间的 TTL UART 波特率。视觉控制串口仍计划使用 `115200 8N1`，当前 `UART_ENABLED = False`。

### MCP/自动化流程

CanMV 扩展 0.9.6 注册了 `canmv.mcp`。自动化连接和抓帧应按以下顺序调用：

```text
canmv_detect_boards
-> canmv_connect_board
-> canmv_execute_file(path="/sdcard/main.py")
-> canmv_script_running
-> canmv_start_preview
-> canmv_get_latest_frame / canmv_save_latest_frame_to_host
-> canmv_stop_preview
-> canmv_disconnect_board
```

关键约束：

- `connect_board` 后必须重新执行 `main.py`，不能假定开机自启程序仍在运行。
- `start_preview` 返回 started 只表示请求已接受；仍须等待新帧并检查 `available=true`。
- 预览帧来自 IDE 帧缓冲。项目的 `Display.init(..., to_ide=True)` 必须保留。
- 连接、Preview 或 MCP 会独占 USB 调试端口。同一时刻不要再用 raw REPL 工具打开相同 COM 端口。
- 当前仓库的 `tools/k230/canmv_mcp_capture.mjs` 可通过扩展后端执行上述流程并保存
  `canmv_preview_latest.jpg`，主要用于本机自动化验证。

## 备用连接方式：raw REPL

`tools/k230/k230_tool.py` 保留为扩展不可用时的备用手段，支持 `list`、`run`、`put`、`cat`、`copy`、
`reset` 和 `hard-reset`。它通过 USB 串口的 MicroPython raw REPL 工作，默认串口速率为 115200，并使用
DTR。对 RTSP 等长期运行的服务须使用 `run --stream`，让 raw REPL 会话在服务期间保持连接。

```powershell
python tools/k230/k230_tool.py list
python tools/k230/k230_tool.py --port COM15 put k230/vision_red_line_follow.py /sdcard/main.py
python tools/k230/k230_tool.py --port COM15 cat /sdcard/main.py
```

### WAGA 手机热点 RTSP 验证

当前验证用热点为开放的 `WAGA`（2.4 GHz、无密码）。电脑和 K230 必须连接同一个热点；以实际 DHCP
地址为准，不要把示例 IP 固化进脚本。部署并保持板端服务运行：

```powershell
python tools/k230/k230_tool.py --port COM15 put k230/test.py /sdcard/rtsp_test.py
python tools/k230/k230_tool.py --port COM15 run tools/k230/runners/k230_run_rtsp_test.py --stream
```

终端打印 `virtual wbc rtsp stream on rtsp://<K230-IP>:8554/test` 后，另开终端抓取一帧或统计帧率：

```powershell
python tools/k230/k230_video_viewer.py rtsp://<K230-IP>:8554/test --snapshot k230_rtsp.jpg
python tools/k230/k230_video_viewer.py rtsp://<K230-IP>:8554/test --stats-frames 100
```

真实摄像头画面使用独立的硬件编码脚本；它选择 GC2093 的原生 1920×1080@60 模式，由 VICAP 硬件
缩放为 640×360 YUV420SP，再以 60 FPS、1000 kbit/s 编码为 H.264，不经过虚拟画布：

```powershell
python tools/k230/k230_tool.py --port COM15 put k230/rtsp_camera_stream.py /sdcard/rtsp_camera_stream.py
python tools/k230/k230_tool.py --port COM15 run tools/k230/runners/k230_run_camera_rtsp.py --stream
```

`k230_video_viewer.py` 在导入 OpenCV 前设置 FFmpeg 低缓冲选项，默认使用可靠性更高的 RTSP/TCP，
并自动把 WBC 的竖屏画面逆时针旋转为横屏。局域网稳定时可用 RTP/UDP 做低时延对比：

```powershell
python tools/k230/k230_video_viewer.py rtsp://<K230-IP>:8554/test --transport udp --buffer-size 1
```

UDP 可避免 TCP 丢包后的队头阻塞，但在手机热点丢包或客户端隔离环境中可能更不稳定；因此不要把它当作
无条件优于 TCP。`CAP_PROP_BUFFERSIZE` 只是向 OpenCV 后端请求队列长度，部分 FFmpeg 构建可能忽略该值。
若媒体服务异常退出或第二次初始化失败，先执行
`python tools/k230/k230_tool.py --port COM15 hard-reset`，再重新上传和启动。测试期间不要让 CanMV 扩展、
Serial Monitor 或其他程序占用同一串口。

当前真实摄像头 RTSP 脚本采用短 GOP（10 帧），并通过
`rtspserver_sendvideodata_byphyaddr()` 直接把编码器物理缓冲交给 RTSP 扩展，避免逐包复制为 Python
`bytes` 所带来的 CPU 与垃圾回收抖动。GOP 缩短主要改善首次出画和丢包后的恢复时间；播放器显示缓存仍
可能是端到端延迟的主要来源。

#### VLC 3.0.23 低时延实测

2026-07-29 在 WAGA 手机热点上使用 VLC 3.0.23、RTSP/TCP 和硬件 H.264 解码，各运行 12 秒进行对比。
测试使用 dummy 视频输出以排除桌面合成差异，但保留 RTSP 接收、live555 缓冲和完整解码流程。VLC 日志中
的 `Stream buffering done` 给出播放器实际建立的时间缓冲：

| VLC 参数 | 实际时间缓冲 | `picture is too late` | 解码丢帧 |
|----------|-------------:|----------------------:|---------:|
| 默认 `network-caching=1000` | 1008 ms | 41 | 6 |
| `network-caching=200` | 203--204 ms | 10--16 | 0 |
| `network-caching=100` | 204 ms | 10 | 0 |

因此观察到的 1--2 秒延迟主要来自 VLC 默认 1000 ms 网络缓存。把请求值从 200 ms 继续降到 100 ms，
实际缓冲没有再下降，说明当前 VLC/live555 路径存在约 200 ms 的有效下限。关闭 `clock-jitter` 和
`clock-synchro` 也未降低该下限，反而会增加启动欠载风险，日常使用应保留自动时钟同步。推荐关闭原有
VLC 窗口后启动独立低缓存实例，避免 URL 被转交给仍采用默认缓存的旧进程：

```powershell
D:\VLC\vlc.exe `
  --no-one-instance `
  --rtsp-tcp `
  --network-caching=200 `
  --live-caching=200 `
  --no-audio `
  rtsp://<K230-IP>:8554/test
```

此测试验证的是播放器内部缓冲和解码稳定性，不是摄像头到屏幕的绝对延迟。精确端到端测量仍需让摄像头
拍摄毫秒时钟或可同步的视觉标记。板端曾在约 31,800 帧后丢失电脑端 COM15 日志会话，原因是主机串口
`WriteFile` 失败；随后 VLC 和 OpenCV 仍能从网络拉流，证明退出的是 raw REPL 日志连接而不是 RTSP 服务。

#### 其他低时延传输路线

- **Python HTTP MJPEG 不作为低时延主方案**：历史实测中，板端硬件 JPEG 编码约 12.5--13.1 FPS，
  但电脑仅收到约 2--3 FPS；客户端连接后，MicroPython `sendall()` 还会显著拖慢视觉循环。
- **不再采用 Python 自定义 UDP-JPEG**：1400 字节分片在 15 秒内只得到 18 个完整帧并出现 20 个不完整帧，
  多次 `sendto()` 使板端编码速率降至约 2 FPS；8000 字节分片依赖 IP 分片，在手机热点上没有收到完整帧。
- **HDMI** 已验证为板内硬件直连显示，适合本地近实时观察，但不能传到电脑。
- **USB UVC gadget** 是连接电脑时最值得继续开发的非 RTSP 方案。官方原生 K230 SDK 有把开发板作为
  USB 摄像头的 UVC demo；当前 CanMV v1.8 板载 `/sdcard/examples/02-Media/uvc.py` 则是让 K230 接收
  外部 USB 摄像头，方向相反，不能直接输出 GC2093。实现板载相机到电脑需构建/定制原生 SDK 固件。
- 若必须保留无线且追求比 RTSP 更可控的延迟，应把 RTP/UDP 分包和发送下沉到原生 C/MPP 服务，避免让
  MicroPython socket 承担视频数据面。

若电脑能加入热点、K230 也已取得同网段地址，但 TCP 仍不能连接，先关闭 Clash/Mihomo 等代理软件的
TUN 模式，并确认系统中不再存在 TUN 虚拟网卡。仅设置 RTSP 不走系统代理不一定有效，因为 TUN/WFP
可能在套接字进入普通代理规则前就拦截局域网连接。可先运行
`python tools/k230/k230_tool.py --port COM15 run tools/k230/probes/k230_wifi_waga_probe.py` 独立确认板端热点关联和
DHCP，再区分板端联网故障与电脑端拦截。

若 TUN 已关闭、电脑能通过 ARP 得到 K230 的 MAC 地址、板端也打印了 `RTSP server started`，但 TCP
连接仍超时，应检查手机热点的“客户端隔离/AP isolation/禁止设备互联”设置。可先在提供热点的手机本机
用 VLC 打开 RTSP 地址：手机可播放而电脑不可播放，基本可确认是热点不允许两个接入设备互访。此时需
启用热点设备互联，或改用允许局域网客户端互访的 2.4 GHz 路由器。

### HDMI 实时摄像头显示

CanMV v1.8 固件包含 `Display.LT9611`，板端官方例程位于
`/sdcard/examples/17-Sensor/camera_single_bind_hdmi.py`。仓库中的
`k230/hdmi_camera_display.py` 使用同一条硬件链路：GC2093 输出 FHD YUV420SP，VICAP 通道直接绑定
到 `Display.LAYER_VIDEO1`，由 LT9611 输出 1920×1080 HDMI，不经过 Python 逐帧拷贝。

通过 USB 前台运行：

```powershell
python tools/k230/k230_tool.py --port COM15 run k230/hdmi_camera_display.py --stream
```

成功日志依次包含 `SENSOR_RESET_OK`、`VIDEO_LAYER_BOUND`、`HDMI_INIT_OK 1920x1080` 和
`CAMERA_RUNNING`。该命令会持续占用 USB 串口；需要运行其他 raw REPL 工具时先按 `Ctrl-C`。

当前板上部署 HDMI 前已将原 `/sdcard/app.py` 备份为 `/sdcard/app_before_hdmi.py`。恢复原 RTSP
应用并复位：

```powershell
python tools/k230/k230_tool.py --port COM15 copy /sdcard/app_before_hdmi.py /sdcard/app.py
python tools/k230/k230_tool.py --port COM15 hard-reset
```

raw REPL 不能与 VS Code CanMV 扩展同时占用 COM15。若出现“拒绝访问”，先在扩展中停止 Preview 并断开
板卡，不要反复强行打开端口。v1.8 偶尔会在脚本结束时丢失 raw REPL 的结束标记；此时应重新同步并核对
远程文件大小，不能只凭一次超时判断脚本或媒体链路失败。

## VS Code Serial Monitor

本机安装的 Microsoft `ms-vscode.vscode-serial-monitor` 0.13.1 可以直接打开 K230 的 USB CDC 串口并
收发文本。它适合持续观察 `print()` 日志，但不是 CanMV 扩展的替代品。

推荐参数：

| 设置 | 值 |
|---|---|
| Port | 自动检测 K230；当前机器为 `COM15` |
| Baud rate | `115200` |
| Data bits | `8` |
| Parity | `None` |
| Stop bits | `1` |
| Flow control | `None` |
| DTR / RTS | 当前版本默认勾选；本板实测可正常读取 |

实测用上述等价参数打开 COM15 后，Serial Monitor 能持续收到 `vision_v18` 日志，K230 主循环仍保持约
90 FPS，未因打开监视器而软复位。该结论只适用于普通串口监视：

- Serial Monitor 只能显示和发送串口字节，不提供板卡识别协议、远程文件管理、脚本状态或 Preview。
- 当前 `main.py` 不读取 USB 控制台输入，所以在 Monitor 中发送普通文本不会控制视觉程序。
- CanMV 扩展、Serial Monitor、`k230_tool.py` 和其他串口程序不能同时打开同一个 COM 端口。
- 若需要相机画面、运行远程文件或写入 `main.py`，仍应关闭 Serial Monitor 后使用 CanMV 扩展。
- 这里的 115200 是 K230 USB CDC/REPL 监视参数；K230 到 MSPM0 的 TTL UART 也计划使用 115200，
  但二者是不同的物理连接和用途。

## Wi-Fi 无线开发代理

完整设计、线协议、安全边界和恢复流程见
[`k230-remote-development.md`](k230-remote-development.md)。

板端 `/sdcard/main.py` 当前安装为 `k230/remote_dev_agent.py`，上电后持续扫描并连接开放的 2.4 GHz
热点 `WAGA`。它使用两个 TCP 端口：

| 端口 | 用途 |
|---:|---|
| 8266 | 上传文件、请求执行、查询状态和远程重启 |
| 2323 | 回放并持续输出上传程序的 `print()` 和异常信息 |

代理不含密码或加密，只能在可信、隔离的本地网络中使用。任何能访问 8266 端口的设备都能上传并执行
Python 代码。若改接公共或不可信网络，必须先增加认证，不能直接暴露该端口。

电脑端使用 `tools/k230/k230_remote.py`：

```powershell
# 查询代理和当前程序状态
python tools/k230/k230_remote.py --host 10.190.177.220 status

# 持续查看 print()/异常输出；网络中断后自动重连
python tools/k230/k230_remote.py --host 10.190.177.220 console

# 只上传文件，不重启
python tools/k230/k230_remote.py --host 10.190.177.220 put local.py /sdcard/app.py

# 上传为 app.py 并远程重启；重启后自动执行
python tools/k230/k230_remote.py --host 10.190.177.220 deploy local.py

# 当前 app.py 已结束时请求再次执行
python tools/k230/k230_remote.py --host 10.190.177.220 run /sdcard/app.py

# 远程系统复位
python tools/k230/k230_remote.py --host 10.190.177.220 restart
```

默认自启动文件为 `/sdcard/app.py`。代理自身在主线程外只使用一个网络管理线程，以便 RTSP 应用还能创建
一个编码取流线程。对于 RTSP 等不会自行结束的程序，更新时使用 `deploy`：它先完整写入临时文件并替换 app.py，再通过
系统复位停止旧程序并启动新版本。控制协议拒绝覆盖 `/sdcard/main.py`，避免一次错误的网络上传破坏恢复
入口。

安装无线代理前的原开机程序已备份为 `/sdcard/main_before_remote_dev.py`。若无线代理无法启动，可用 USB
raw REPL 恢复：

```powershell
python tools/k230/k230_tool.py --port COM15 copy /sdcard/main_before_remote_dev.py /sdcard/main.py
python tools/k230/k230_tool.py --port COM15 hard-reset
```

## 自启与复位语义

- 上电或 `machine.reset()`：走完整系统启动路径，用于验证 `/sdcard/main.py` 开机自启。
- 扩展连接：会执行 MicroPython 软复位，随后应由扩展显式执行 `/sdcard/main.py`。
- Ctrl-D/raw REPL `reset`：属于软重启调试动作，不能替代最终的开机自启验证。
- Ctrl-C/停止脚本：只中断当前循环；本项目异常退出时不会主动关闭 LCD，以保留最后一帧供排查。

## 已验证结果

在上述固件和扩展组合上已验证：

- 扩展能够识别 `CanMV K230 01Studio - 1G`，协议版本为 2。
- 扩展软复位后可重新执行 `/sdcard/main.py` 并读取终端输出。
- Preview 能获取完整的 800×480 JPEG 帧，视频层无竖纹、无右侧黑屏，方向正确。
- LCD 保持开启时视觉循环约 90 FPS；取帧约 5–8 ms，HSV 掩膜约 4–7 ms，轨迹计算约 0–1 ms。
