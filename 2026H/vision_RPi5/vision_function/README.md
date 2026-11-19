# 钢球识别模块

该目录实现 2026 H 题从摄像头取帧、鱼眼校正、钢球识别、厘米位置与速度估计，
到 Raspberry Pi GPIO UART 输出的完整实时视觉链路。

## 图像链路

```text
USB 摄像头硬件 MJPEG
├── uStreamer 原始直通 → 浏览器 WebGL 去畸变 → 预览/电脑端截图
└── 树莓派本机解码 → FisheyeUndistorter → 钢球检测与位置控制
```

主图传继续传输原始 MJPEG，避免树莓派重新编码 JPEG。网页和树莓派分别使用同一只摄像头的同一组 OpenCV fisheye 标定参数，因此两端的去畸变几何模型一致。

## 生产局部去畸变

```python
from pathlib import Path
from vision_function.ball_detection.preprocess import (
    TubePreprocessor,
    load_tube_rectification,
)

config = load_tube_rectification(
    Path("ball_detection/config/tube_local_rectification.json")
)
tube_image = TubePreprocessor(config)(raw_bgr_frame)
```

生产输出是 640×100 的独立水管局部校正图。它不是“先生成 640×360 全图校正
画面再裁剪”，而是把全图校正平面向画布外延伸后，直接从原始鱼眼图 remap：

```text
1280×720 原始硬件 JPEG
→ JPEG 1/2 IDCT 解码为 640×360
→ balance=0.2 校正平面中的区域 (40,126)，输出 640×100
→ YOLO 等比例 letterbox 到 320×320
```

参考尺寸下该区域为左上角 `(80,252)`、大小 1280×200，右边界达到
`x=1360`，允许保留原本落在名义 1280×720 校正画布之外的右侧管端。没有必要
生成完整去畸变图，映射不到原图的像素自动填黑。

底层也可以直接调用：

```python
from vision_function import FisheyeUndistorter

undistorter = FisheyeUndistorter(balance=0.2)
tube_image = undistorter.undistort_rectified_region(
    half_frame,
    origin_xy=(40, 126),
    output_size=(640, 100),
)
```

该方案仍使用原始 1280×720 硬件 JPEG，只是在树莓派解码阶段直接输出
640×360；网页图传分辨率不受影响。最终 v3 NCNN 在 Raspberry Pi 5、2 线程、
130 帧实测中，请求取帧到识别完成的 P95 为 26.80 ms，真实帧时间戳到识别结果
的 P95 为 26.81 ms，约 37.3 fps；当前右侧钢球 130/130 帧检出且每帧只有一个
候选。
4 线程存在明显长尾，不用于当前控制链路。
持续读取 `/stream` 会在串行推理期间积压旧帧，因此控制进程必须使用
`/snapshot`，且不使用后台 latest-frame 双线程。

钢球程序输出带帧序号和拍摄时间戳的 `BallObservation`，其中球心使用
640×100 局部坐标：

```bash
python ball_detection/detect_ball.py ball_detection/data/raw/photos
```

程序只选择最高置信度候选。控制和刻度换算使用 `center_local_xy`，即模型
letterbox 之前的 640×100 局部坐标。实时采集、位置速度估计和固定 11 字节
UART 输出已由 `scale_location/runtime.py` 串联，使用方法及协议见
[`scale_location/README.md`](scale_location/README.md)。

映射表按照输入分辨率缓存，识别循环不会反复计算相机矩阵。当前标定支持 1280×720，也可等比例缩放到 1920×1080；未经重新标定，不接受 4:3 分辨率。

网页保留原始全图和全图去畸变模式，并增加“识别ROI”模式展示同一局部坐标。
生产链路使用 `undistort_rectified_region()`；`undistort_roi()` 仅用于需要严格
等价于全图校正后裁剪的旧流程。

## 实时控制通信链路

生产控制程序使用严格串行路径，关注同一张图像从拍摄到 UART 写出的时延：

```text
USB 摄像头硬件 MJPEG（1280×720）
→ uStreamer /snapshot + X-Timestamp
→ JPEG 1/2 IDCT 解码（640×360）
→ 水管区域局部鱼眼 remap（640×100）
→ YOLO11n v3 NCNN，imgsz=320，CPU 2 线程
→ BallObservation（局部球心、置信度、拍摄时间戳）
→ 正式多刻度分段线性映射
→ 常速度 Kalman 位置/速度估计
→ 连续观测、边缘、质量和时延检查
→ 11 字节二进制协议帧
→ 单次 write(/dev/ttyAMA0)
→ GPIO14 / Pin 8
→ MCU UART RX
```

YOLO 默认置信度阈值为 `0.10`。2026-07-30 在 Raspberry Pi 5 上进行的 1800 帧
慢速往返复测检出 1791 帧（99.50%），最长连续漏检 3 帧，平均处理耗时
26.41 ms；该轮使用 `--dry-run`，未打开 UART。

每个不同的摄像头帧最多产生一个 UART 包。控制程序不使用后台采集线程、不缓存
待处理帧，也不从持续 `/stream` 队列读取旧帧。`X-Timestamp` 重复时不推理、
不发送且不增加 `SEQ`。

### 外部摄像头接口

当前摄像头端接口如下：

| 项目 | 约定 |
|---|---|
| 物理接口 | USB UVC 摄像头，树莓派设备 `/dev/video0` |
| 摄像头输出 | 硬件 MJPEG，1280×720，目标 60 fps |
| 图传服务 | uStreamer，HTTP 端口 8080 |
| 控制取帧 | `http://127.0.0.1:8080/snapshot` |
| 网页预览 | `/stream`；浏览器端完成全图或水管 ROI 校正 |
| 时间戳 | uStreamer 响应头 `X-Timestamp`，Unix 实时时基，单位秒 |
| Pi 解码 | `cv2.IMREAD_REDUCED_COLOR_2`，直接得到 640×360 BGR |

摄像头接口的失效约定：

- 缺少、重复或倒退的 `X-Timestamp` 不能产生看似正常的新测量；
- JPEG 解码失败的帧按视觉无效处理，不能沿用旧图伪装成新帧；
- 识别丢失仍对当前新帧发送一包，但清除 `VALID`；
- 连续 2 个有效观测后才置 `VALID=1`，连续丢失超过 100 ms 重置估计器；
- 网页预览与控制识别相互独立，网页截图或录像不会进入控制路径。

### 外部 UART 从设备接口

物理接线：

| Raspberry Pi 5 | UART 从设备/MCU | 说明 |
|---|---|---|
| GPIO14 / TXD0 / 物理 Pin 8 | UART RX | Pi 单向发送 |
| GND / 物理 Pin 6 | GND | 必须共地 |
| GPIO15 / RXD0 / 物理 Pin 10 | 不连接 | 当前协议不接收 MCU 数据 |

两端必须使用 3.3 V TTL。不得向树莓派 GPIO 输入 5 V。串口参数固定为
`115200 8N1`，无奇偶校验、1 个停止位、关闭软硬件流控。

目标 Pi 5 已验证 `dtoverlay=uart0-pi5` 创建 `/dev/ttyAMA0`，并将 GPIO14/15
复用为 `TXD0/RXD0`。`/dev/serial0 -> /dev/ttyAMA10` 是独立 Debug Header，
不能接 MCU。持久化配置位于 `/boot/firmware/config.txt`：

```ini
[pi5]
dtoverlay=uart0-pi5
dtoverlay=nospi10
```

#### 固定 11 字节协议

多字节整数均为小端：

| 偏移 | 字段 | 类型 | 含义 |
|---:|---|---|---|
| 0 | `SYNC0` | `uint8` | 固定 `0xA5` |
| 1 | `SYNC1` | `uint8` | 固定 `0x5A` |
| 2 | `VER` | `uint8` | 当前固定 `0x01` |
| 3 | `SEQ` | `uint8` | 每个新摄像头帧加一，255 后回绕 |
| 4 | `AGE` | `uint8` | 拍摄到即将写 UART 的测量龄，单位 ms |
| 5 | `X` | `int16 LE` | 拍摄时刻球位置，0.1 mm/LSB，中心为零 |
| 7 | `V` | `int16 LE` | 拍摄时刻球速度，1 mm/s/LSB，向右为正 |
| 9 | `FLAGS` | `uint8` | 有效性、运动、速度和质量位域 |
| 10 | `CRC8` | `uint8` | 覆盖偏移 2～9 |

`FLAGS`：

| 位 | 名称 | 从设备解释 |
|---:|---|---|
| 0 | `VALID` | 本帧位置有效；为 0 时不得使用 `X` 控制 |
| 1 | `MOVING` | 钢球处于运动状态，带启停滞回 |
| 2 | `V_VALID` | 速度估计已稳定；为 0 时不得使用 `V` 反馈 |
| 3 | `EDGE` | 钢球处于标定范围端部或范围之外 |
| 4～5 | `QUALITY` | `0..3`，3 最好 |
| 6～7 | 保留 | 固定为 0 |

定点换算：

```text
X = round(position_at_capture_cm × 100)
V = round(velocity_at_capture_cm_s × 10)
```

CRC-8 使用多项式 `0x07`、初值 `0xFF`、输入输出不反转、无最终异或。
11 字节在 115200 baud 下的线上发送时间约为 0.955 ms；MCU 应把该时间加到
`AGE`，再使用自身控制周期和摆杆倾角把拍摄时刻的 `X/V` 前推到当前时刻。
Pi 不发送已经前推的 `position_now`，避免 MCU 重复补偿。

`VALID=0` 不等于球在零点，从设备必须忽略此时的 `X`。MCU 还应检查帧头、版本、
CRC 和 `SEQ` 连续性；连续 200 ms 没有有效帧时应关闭球位置环并把摆杆恢复到
安全水平角。目标刻度不包含在该 11 字节帧内，由 MCU 自己管理；Pi 的
`--target-cm` 只用于本地日志和调试。

## 树莓派使用方法

### 1. 启动和检查摄像头

```bash
sudo systemctl start ustreamer
sudo systemctl status ustreamer --no-pager
curl --max-time 3 -o /tmp/camera-check.jpg \
  http://127.0.0.1:8080/snapshot
```

### 2. 检查 UART

```bash
ls -l /dev/ttyAMA0
pinctrl get 14
pinctrl get 15
```

期望看到 GPIO14=`TXD0`、GPIO15=`RXD0`。如果 `/dev/ttyAMA0` 不存在，检查
`/boot/firmware/config.txt` 后重启。不要用 `/dev/serial0`。

### 3. 不连接 UART 的完整 dry-run

```bash
cd /home/nailong/NUEDC_2026_H_VISION

.venv/bin/python -m vision_function.scale_location.runtime \
  --dry-run \
  --iterations 130 \
  --status-every 10 \
  --report vision_function/scale_location/output/camera_to_uart_dry_run.json
```

该模式会执行摄像头、去畸变、识别、位置速度估计和协议编码，但不会打开
GPIO UART。

### 4. 启动真实 UART 连续输出

先完成 3.3 V TTL 接线和共地，然后运行：

```bash
cd /home/nailong/NUEDC_2026_H_VISION

.venv/bin/python -m vision_function.scale_location.runtime \
  --device /dev/ttyAMA0 \
  --status-every 30
```

按 `Ctrl+C` 停止。`scale_location/config/uart.json` 中的 `position_sign=-1` 会在
状态估计入口同时反转位置和速度，使日志、协议包和 MCU 坐标保持一致。
每 30 个包打印一次 `SEQ/AGE/VALID/X/V/FLAGS` 和完整十六进制包；这些日志在
UART `write()` 之后打印，不计入发送时延。需要逐帧 JSON 诊断时添加
`--json-lines`。

当前默认使用 `scale_calibration.json`。五点静态验收的最大绝对偏差为
`0.5211 cm`，满足题目 `1 cm` 限值；正半轴约 `-0.5 cm` 偏差和 `-10 cm`
处的检出率仍列为后续优化项。

实机测试结果：130 个新帧对应 130 次硬件 UART 写入，`write_fail=0`；平均
图像采集到写入完成为 26.10 ms，最大 `AGE=28 ms`。空管中的 2 次孤立候选均
被连续观测门限阻断。有球样本从第 2 帧起 `VALID=1`，从第 4 帧起
`V_VALID=1`。原始报告见
[`scale_location/output/camera_to_uart_hardware_confirmed.json`](scale_location/output/camera_to_uart_hardware_confirmed.json)。

仓库提供
[`vision-scale-uart.service`](scale_location/deploy/vision-scale-uart.service)。目标树莓派
已将它安装到 `/etc/systemd/system/`，并验证 `enable --now`、依赖 uStreamer、
`dialout` 权限和 UART 独占均正常。当前服务已经启用并持续运行；测试期间 MCU 控制
程序没有开启，因此位置变化只用于检查检测与通信链路，不代表闭环控制结果。

```bash
sudo systemctl enable --now vision-scale-uart.service
sudo systemctl disable --now vision-scale-uart.service
systemctl status vision-scale-uart.service
journalctl -u vision-scale-uart.service -f
```

## 外部资产与仓库边界

GitHub 仓库只保存代码、配置、人工标注、小型 JSON/CSV 报告和复现说明。以下两类
二进制资产由项目成员使用仓库外方式传输，不进入普通 Git：

1. 原始图片和视频；
2. PyTorch、NCNN 等模型及训练权重。

克隆仓库后，应把外部原始数据恢复到：

```text
vision_function/ball_detection/data/raw/
vision_function/ball_detection/data/raw_similar/
```

当前生产模型应恢复到：

```text
vision_function/ball_detection/model/
├── actual_yolo11n_local_v3.pt
└── actual_yolo11n_local_v3_ncnn_model/
    ├── metadata.yaml
    ├── model.ncnn.bin
    ├── model.ncnn.param
    └── model_ncnn.py
```

树莓派实时运行至少需要完整的
`actual_yolo11n_local_v3_ncnn_model/`。训练、重新导出或继续微调时才需要 `.pt`
权重和原始数据。外部传输后应使用双方约定的 SHA-256 清单校验文件完整性。

以下目录能由原始数据和脚本重新生成，也不进入 Git：

```text
vision_function/ball_detection/data/actual_yolo_*/
vision_function/ball_detection/data/actual_video_frames_*/
vision_function/ball_detection/output/train_actual/
```

小型评估报告和 Raspberry Pi 基准 JSON 仍保留在仓库中，用于追踪准确率、时延和
模型版本。若外部资产未恢复，文档和协议单元测试仍可运行，但实时 NCNN 推理、
数据集重建和训练命令会因缺少相应文件而失败。

## 测试

```bash
python -m unittest discover -s vision_function/tests
```

钢球识别的性能基线、已完成优化和后续计划见
[`ball_detection/OPTIMIZATION.md`](ball_detection/OPTIMIZATION.md)。
实车 ROI、两轮数据集、fine-tune 和准确率记录见
[`ball_detection/ACTUAL_DATA.md`](ball_detection/ACTUAL_DATA.md)。
