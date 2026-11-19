# 开发记录

记录 2026-07-29 搭建树莓派摄像头实时推流的完整过程，重点是**为什么这么选**和**踩过的坑**。

---

## 1. 环境探测

从 Windows 通过 SSH 连接树莓派。

Windows 自带 OpenSSH 客户端，但没有 `sshpass`/`plink`，无法非交互传密码。改用 Python **paramiko**，封装成 `tools/rpi.py`（命令通过 stdin 传入，避免命令行转义地狱）。

探测结果：

| 项 | 值 |
|---|---|
| 型号 | Raspberry Pi 5（`pisp_be` 设备节点特征） |
| 系统 | Debian GNU/Linux 13 (trixie), aarch64 |
| 内核 | 6.18.34 |
| 网络 | wlan0 无线，无有线（eth0 down） |

### 坑 1：`rpicam-hello --list-cameras` 报 "No cameras available"

一开始以为摄像头没接好。实际上：

```
icspring camera: icspring camer (usb-xhci-hcd.1-1):
	/dev/video0
	/dev/video1
```

这是一个 **USB UVC 摄像头**（`32e6:9251`，驱动 `uvcvideo`），不是 CSI 排线摄像头。`rpicam`/`libcamera` 系列工具**只管 CSI**，对 UVC 设备一律报「无摄像头」。用 `v4l2-ctl --list-devices` 才能看到。

`/dev/video19~35` 是 Pi 5 自带的 ISP 和硬件解码器节点，与摄像头无关。

### 摄像头能力

```
[0]: 'MJPG' (Motion-JPEG, compressed)
     1920x1080 @60fps / 1280x720 @60fps / 800x600 @60fps / 640x480 @60fps
[1]: 'YUYV' (YUYV 4:2:2)
     1920x1080 @5fps / 1280x720 @10fps / 800x600 @20fps / 640x480 @30fps
```

**关键**：摄像头自带 MJPEG 硬件编码，且只有 MJPEG 才能跑高帧率。YUYV 是未压缩的，1080p 只有 5fps。

---

## 2. 方案选择

### 为什么不用 H.264

Pi 5 **移除了硬件 H.264 编码器**（Pi 4 及之前有）。走 H.264 只能软件编码（x264），占用约一个 CPU 核心，还会增加编码延迟。

### 为什么选 MJPEG 直通

摄像头本身输出的就是 JPEG 帧。ustreamer 的 `--encoder=HW` 模式**直接转发摄像头产出的 JPEG，不做任何解码/重编码**：

- CPU 占用实测 **0.2%**
- 无编码延迟
- 浏览器原生支持 `multipart/x-mixed-replace`，`<img src>` 一行搞定，无需播放器
- TCP 传输，无抖动缓冲，延迟基本等于「传输时间」

代价是码率高（JPEG 帧间无压缩）。这是后面所有优化的核心矛盾。

### 工具选择

Debian trixie 官方源里有 **`ustreamer` 5.4**，`apt install ustreamer` 即可，无需编译。

---

## 3. 首次部署

配置为 1280x720，做成 systemd 服务开机自启。跑通了，但**用户反馈延迟明显**。

---

## 4. 延迟根因定位

### 现象

```json
"source": {"captured_fps": 22},
"stream": {"queued_fps": 22, "clients_stat": {"...": {"fps": 18}}}
```

服务端往外发 22fps，**客户端只收到 18fps**。差值不是丢帧——是帧在网络队列里堆积。这种情况下延迟**不会稳定在某个值，而是持续累积**，这正是「延迟很大」的主观感受来源。

### 量化

- 720p 单帧 JPEG ≈ **117 KB**
- 57fps 满速 → 需要约 **28 Mbps**

用 Python HTTP server 传一个大文件，实测 Pi → Windows 的**裸 TCP 吞吐只有 14 Mbps**。

**需求是链路能力的 2 倍**，必然排队。根因确认：**不是软件配置问题，是带宽不够。**

### 为什么 WiFi 只有 14 Mbps

```
*:WAGA:11:2462 MHz:100:130 Mbit/s
wlan0: 0000  70.  -23.  -256   0  0  0  20  0  0
```

信号 -23 dBm（极好），协商速率 130 Mbps，但实际只有 14 Mbps → **2.4GHz 信道 11 拥塞**（现场是商场环境，周边 WiFi 密集，共享空口时间）。当时扫描不到 5GHz。

---

## 5. 优化尝试

### 坑 2：`--quality` 对 MJPEG 源无效

想通过降低 JPEG 质量来压码率。写了 `bench.sh` 逐个配置实测：

| 配置 | 单帧 | 采集帧率 | CPU |
|---|---|---|---|
| 1280x720 HW 直通 | 117 KB | 57fps | 0.2% |
| 1280x720 CPU q=40 | **133 KB** | 58fps | 0.4% |
| 1280x720 CPU q=25 | **135 KB** | 21fps | 0.1% |
| 800x600 HW 直通 | 59 KB | 57fps | 0.2% |
| 640x480 HW 直通 | 42 KB | 57fps | 0.2% |

CPU 编码器配 `-q` **完全没有降低码率**。查 `ustreamer --help` 确认：

> It does not re-encode MJPEG to MJPEG to change the quality level

源格式是 MJPEG 时，ustreamer 不会解码再重编码，`--quality` 静默失效。

**结论：唯一有效的码率杠杆是分辨率。**

### 实施的优化

降到 640x480，并补上几个延迟相关参数：

| 参数 | 作用 |
|---|---|
| `--resolution=640x480` | 单帧 117KB → 42KB，主要收益 |
| `--tcp-nodelay` | 关闭 Nagle 算法，省掉最多 40ms 合包等待 |
| `--buffers=3`（默认 5） | 缩短 V4L2 采集队列深度 |
| `--persistent` | 设备超时不重新初始化，避免偶发卡顿 |
| `Nice=-10` + `IOSchedulingClass=realtime` | 提高调度优先级 |

效果：占用 6.3 Mbps（链路的 45%），到达 30fps 不再堆积。

---

## 6. 切换 5GHz 后重新调优

用户把树莓派接到了 5GHz。

```
*:WAGA:36:5180 MHz:90:270 Mbit/s     信号 -32 dBm
```

裸 TCP 吞吐：**168 Mbps**（12 倍提升）。

### 重新基准测试

| 分辨率 | 单帧 | 满速码率 |
|---|---|---|
| 1920x1080 | 244 KB | ~114 Mbps |
| 1280x720 | 131 KB | ~61 Mbps |
| 800x600 | 77 KB | ~36 Mbps |

### 端到端实测

| 分辨率 | 实际吞吐 | 到达帧率 | 带宽占用 | 单帧传输 |
|---|---|---|---|---|
| 1280x720 | 56.7 Mbps | **57.2 / 57 fps** | 34% | 17 ms |
| 1920x1080 | 121~127 Mbps | 54~57 / 57 fps | 72% | 19 ms |

### 最终选择 720p

1080p 也能跑，但：

- 占用 72% 带宽，**余量不足**。WiFi 吞吐会随干扰和其他设备波动，一旦掉到 110 Mbps 以下就开始排队
- 单帧传输延迟只比 720p 多 2ms，**画质的收益不体现在延迟上**
- 720p 占用 34%，有 3 倍余量，到达帧率与采集帧率完全一致（零积压）

用户想要画质时一条命令 `camres 1920x1080` 即可切换，因此把稳定的 720p 设为默认是更好的默认值。

---

## 7. 配置结构重构

为了让分辨率可以随时调整，把参数从 service 文件里抽出来：

- `/etc/default/ustreamer` — `RESOLUTION` / `FPS` / `PORT`，通过 systemd `EnvironmentFile` 注入
- `/usr/local/bin/camres` — 改配置 + 重启 + 显示实际状态，一条命令完成

---

## 8. 网页播放器

ustreamer 自带的页面比较简陋，用 `--static=/opt/camstream/www` 换成自定义页面。

设计上有两个决定：

**用 `<img src="/stream">` 而不是 fetch + canvas 手动解析**

一开始考虑用 `fetch` + `ReadableStream` 解析 multipart，只渲染最新帧、丢弃积压帧，以降低延迟。**但这个思路是错的**：TCP 会保证所有字节都送达，客户端丢帧不会让网络队列排空得更快。真正的丢帧必须发生在**服务端**——而 ustreamer 已经在做了（socket 可写时才发最新帧，这正是 `client fps < queued fps` 的由来）。

所以客户端侧无优化空间，`<img>` 反而是最优解：浏览器原生解码、硬件加速、代码最少。

**内置延迟测量时钟**

端到端延迟没法从软件内部测出来（服务端不知道浏览器何时把像素画到屏幕上）。页面上放一个 `requestAnimationFrame` 驱动的毫秒时钟，**把摄像头对准屏幕**，画面里拍到的时间与旁边实时时间的差值，就是真实的「玻璃到玻璃」延迟。这是唯一可靠的测量方法。

状态栏对比 `captured_fps` 和客户端 `fps`，直接把「是否在排队」这个关键指标暴露出来。

---

## 9. 广角畸变校正

这个摄像头是广角镜头，桶形畸变明显：桌沿、天花板结构在画面里向外鼓成弧线。

### 先确认硬件有没有现成能力

在动手写代码前，把树莓派上可能做几何校正的地方都查了一遍，结论是**都不可用**：

| 检查项 | 结果 |
|---|---|
| 摄像头 UVC 控制项 | 只有亮度/对比度/饱和度/白平衡/伽马/增益/锐度/曝光，无畸变相关项 |
| PiSP（Pi 5 的 ISP）功能块 | `input` / `tdn`(时域降噪) / `stitch`(HDR合成) / `output` / `config`，**没有 dewarp/LDC 节点** |
| `pisp-be.ko` 模块符号 | 搜 `dewarp\|ldc\|distort\|fisheye\|warp` 零命中 |
| libcamera | 无 `LensDistortion` 类控制 |

即使有，还有个更根本的障碍：**PiSP 输入只接受 YUV/RGB/Bayer，不接受 JPEG**。摄像头直接吐 MJPEG，要喂进 ISP 就得先解码，等于放弃硬件直通。

Pi 5 的 ISP 是为 **CSI 排线摄像头**的 Bayer 数据设计的，功能集中在去马赛克、降噪、HDR、色彩处理。几何校正通常由 CSI 模组自己的 libcamera tuning 文件处理，对 UVC 设备不适用。

### 为什么放在浏览器端而不是树莓派上

| | 树莓派端 | 浏览器端（选用） |
|---|---|---|
| 处理链 | 解码 JPEG → remap → 重编码 JPEG | GPU 片元着色器直接采样 |
| 树莓派 CPU | 0.2% → 150%+ | **保持 0.2%** |
| 延迟 | +20~30ms | **0** |
| 硬件直通 | 被打断 | **保持** |
| 生效范围 | 所有客户端 | 仅网页播放器 |

前面花了很大力气才把延迟压下来（MJPEG 硬件直通、`--tcp-nodelay`、缩短缓冲队列），在树莓派上做校正会把这些收益全部吐回去。浏览器端用 GPU 做，每帧的 remap 对任何现代显卡都是零成本。

代价是 `/stream` 和 `/snapshot` 输出的仍是原始畸变画面——这是有意识接受的权衡。

### 数学模型

用 Brown-Conrady 径向畸变模型。着色器里对**输出（已校正）图**上的每个点，反算它该从**输入（畸变）图**的哪个位置采样：

```glsl
vec2 c = (vUv - uCenter) * vec2(uAspect, 1.0) / uZoom;
float r2 = dot(c, c);
float f  = 1.0 + uK1 * r2 + uK2 * r2 * r2;
uv = (c * f) / vec2(uAspect, 1.0) + uCenter;
```

两个要点：

- **以画面高度为长度单位**，x 分量乘宽高比，保证径向距离各向同性（直接用 uv 会因为 16:9 的非等比而算错）
- **k1 为负**时 `f < 1`，大半径处采样点被拉向中心，输出图的周边被向外拉伸——这正是桶形校正。因为 `f < 1` 采样点始终落在源图内部，所以不会出现黑边，代价是损失一圈视野

`zoom` 用来在「视野范围」和「四角黑边」之间取舍。

### 参数怎么定的

没有做棋盘格标定。写了个脚本用**完全相同的数学模型**在本地对同一帧跑不同 k1，叠加参考网格拼成对比图，肉眼选出桌沿最直的一组：`k1=-0.28, k2=0.08`。

这是目测值不是标定值，所以页面上做成了带滑块的实时调参，并把参数存进 localStorage。要更精确的话需要棋盘格 + `cv2.calibrateCamera`，把 OpenCV 的 `K/D` 换算到本着色器的归一化约定即可（`k1_shader = k1_opencv / s²`，`s = f / H`）。

---

## 踩坑速查

### `pkill -f "xxx"` / `pgrep -f "xxx"` 把 SSH 会话自己杀了

通过 `ssh exec_command` 执行的整串命令会出现在 `bash -c` 的命令行里。如果这串命令**本身包含**要匹配的模式，`pkill -f` 就会匹配到自己的父 shell 并杀掉它，表现为 SSH 通道异常关闭（exit -1）。

规避：把脚本写成文件上传后执行，不要在一次性命令串里用 `pkill -f`。

### 后台进程挂住 SSH 通道

`nohup cmd &` 后台启动的进程会继承 stdout/stderr，paramiko 的 `exec_command` 会一直等到所有句柄关闭，导致命令「不返回」。

规避：`setsid cmd </dev/null >/dev/null 2>&1 &`，彻底脱离会话。

### 帧计数少写了 `+ len(boundary)`

`measure.py` 里统计 multipart 边界时：

```python
buf = buf[buf.rfind(boundary):]          # 错：保留的 boundary 下一轮被重复计数
buf = buf[buf.rfind(boundary) + len(boundary):]   # 对
```

第一版测出「到达 161 fps」，而源只有 57fps——虚高约 2.8 倍。**交叉验证救了这个 bug**：`吞吐 / 帧率` 算出的单帧大小是 41KB，而 `/snapshot` 实测是 121KB，两者对不上，说明帧数被高估了。

### Clash 代理

Windows 侧系统代理为 `127.0.0.1:7897`，但 `ProxyOverride` 已包含 `10.*`，访问树莓派本来就直连。实测走系统代理和强制绕过都正常，**未做任何 Clash 配置改动**。

Python 脚本里仍显式加了 `ProxyHandler({})` 绕过代理，避免环境差异导致测量失真。

---

## 最终配置

```
Raspberry Pi 5 / Debian 13 trixie
USB UVC 摄像头 (icSpring 32e6:9251) @ /dev/video0
ustreamer 5.4, MJPEG 硬件直通, 1280x720 @ 57fps
WiFi 5GHz (信道 36), 链路 168 Mbps, 占用 34%
CPU 占用 0.2%, 端到端延迟约 60~100ms
```

延迟构成估算：采集与排队 ~20ms + 网络传输 17ms + 浏览器解码显示 ~30ms。

---

## 10. 复用同摄像头的精确鱼眼标定

确认 2025 C 题项目使用的是同一只 USB 摄像头，因此复用了其 1280×720 OpenCV fisheye 棋盘格标定：17 张有效照片、重投影 RMS 0.344 px。网页片元着色器改为 Kannala-Brandt 四阶鱼眼模型，默认展示去畸变画面，同时保留原始画面切换、参考网格、参数微调和浏览器本地 PNG 截图。

树莓派仍只负责原始 MJPEG 硬件直通。这样视觉预览精度得到提升，但树莓派 CPU、网络码率和端到端延迟不变。`/stream` 与 `/snapshot` 仍是原始帧，后续钢球识别模块应使用相同的 OpenCV 标定参数自行去畸变。
