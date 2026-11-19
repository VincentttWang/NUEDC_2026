# 2026 电赛 H 题：Raspberry Pi 5 视觉与图传子系统（`vision_RPi5`）

本目录为 [2026 年全国大学生电子设计竞赛 H 题「车载平衡滚球运动控制系统」](../README.md) 的 **树莓派 5 视觉、图传与现场调试子系统**，与同级的 [`firmware_G3507/`](../firmware_G3507/README.md)（TI MSPM0G3507 逻辑主控固件）通过 3.3 V TTL UART 协同工作。题目原文见 [`H题_车载平衡滚球运动控制系统.pdf`](../H题_车载平衡滚球运动控制系统.pdf)。

当前系统基于 **Raspberry Pi 5 + USB 广角摄像头（硬件 MJPEG 1280×720@60fps）** 完成以下能力：

- **局域网低延迟图传**：uStreamer 硬件 MJPEG 直通 + 浏览器 WebGL 去畸变预览、识别 ROI 预览、固定参考线、电脑端 PNG 截图与 WebM 录像；
- **水管局部鱼眼校正**：从最新 JPEG 帧 1/2 IDCT 缩放解码（640×360），直接 remap 为 640×100 水管局部去畸变图；
- **单球高速定位**：默认采用轻量 **浅层 CNN v3 NCNN**（20,411 参数，树莓派 5 纯推理 P95 **2.39 ms**、全链路 P95 **9.41 ms**），并保留 **YOLO11n NCNN** 回退路径；
- **刻度标定与状态估计**：默认采用 **彩色刻度动态标定**（支持静态多刻度分段线性标定回退），结合常速度 Kalman 滤波输出拍摄时刻的厘米位置与速度；
- **双机 UART 通信与现场调试**：通过 GPIO UART（`/dev/ttyAMA0`，115200 8N1）发送固定 11 字节视觉状态帧（`0x01`），并提供树莓派桌面 **视觉监看调试台**（共享内存零侵入监看）与 **赛道参数调试台**（双向 `0x81`/`0x82` 协议）。

---

## 系统链路

```text
USB 广角摄像头（硬件 MJPEG 1280×720，目标 60 fps）
├── uStreamer /stream
│   └── 浏览器 WebGL：原图 / 全图去畸变 / 识别 ROI + 参考线
│       └── 电脑端 PNG 截图与 WebM 录像（不占用树莓派编码算力）
└── uStreamer /snapshot + X-Timestamp
    └── Raspberry Pi 5：JPEG 1/2 IDCT 缩放解码（640×360）
        └── 水管局部鱼眼校正（640×100）
            ├── 浅层 CNN v3 NCNN（默认，640×100 原生输入）/ YOLO11n NCNN（回退）
            ├── 彩色刻度动态标定（默认）/ 静态分段线性标定（回退）
            └── 常速度 Kalman 位置与速度估计
                ├── 115200 8N1 UART（11 字节视觉状态帧 0x01）──► MSPM0G3507
                └── /dev/shm/nuedc-vision-debug ──► 树莓派桌面视觉监看台
```

控制链路严格按帧串行执行，不使用后台“最新帧”流水线，以精确度量同一帧图像从拍摄到 UART 写出的测量龄（`AGE`），并避免持续 MJPEG 流中的旧帧积压。

---

## 目录结构

| 路径 | 说明 |
|---|---|
| [`raspberry-pi-camera-stream/`](raspberry-pi-camera-stream/) | uStreamer 部署脚本、systemd 服务、WebGL 预览网页及延迟测量工具 |
| [`vision_function/ball_detection/`](vision_function/ball_detection/) | 水管局部鱼眼去畸变、浅层 CNN / YOLO11n NCNN 钢球检测、数据集构建、训练与测速基准 |
| [`vision_function/scale_location/`](vision_function/scale_location/) | 彩色刻度动态标定、静态分段线性标定、Kalman 位置速度估计、11 字节 UART 协议与实时服务 |
| [`vision_function/debug_ui/`](vision_function/debug_ui/) | 树莓派桌面视觉调试与共享内存监看台（`nuedc-vision-debug.desktop`） |
| [`vision_function/track_tuning/`](vision_function/track_tuning/) | 树莓派赛道参数桌面调试台（H2/H5/H6 13 项参数读写、校验、Profile 管理与内置帮助） |
| [`vision_function/calibration/`](vision_function/calibration/) | 当前 USB 广角摄像头的 OpenCV fisheye 四阶标定参数 |
| [`vision_function/tests/`](vision_function/tests/) | 视觉预处理、浅层 CNN、定位、协议、调试台、调参台与 UART 单元测试 |
| [`scripts/`](scripts/) | 树莓派桌面应用启动脚本（`start_rpi_gui.sh`、`start_track_tuning.sh`） |

### 详细设计文档索引

- [图传模块说明](raspberry-pi-camera-stream/README.md)（含 [开发日志](raspberry-pi-camera-stream/DEVLOG.md)）
- [视觉链路与外部接口规范](vision_function/README.md)
- [钢球检测模型准确度与速度汇总报告（浅层 CNN v3 vs YOLO11n v4）](vision_function/ball_detection/output/model_accuracy_speed_summary_20260801.md)
- [实车数据与模型演进记录](vision_function/ball_detection/ACTUAL_DATA.md)
- [视觉推理性能优化记录](vision_function/ball_detection/OPTIMIZATION.md)
- [刻度位置、速度估计与 UART 协议规范](vision_function/scale_location/README.md)
- [树莓派桌面视觉调试台说明](vision_function/debug_ui/README.md)
- [树莓派赛道参数调试台说明](vision_function/track_tuning/README.md)（含 [赛道参数作用与约束](vision_function/track_tuning/PARAMETER_HELP.md)）

---

## 克隆后的准备工作

### 1. 恢复外部数据与模型资产

原始图片、视频、PyTorch 权重（`.pt`）与 NCNN 模型二进制文件已由 `.gitignore` 排除，不进入 Git，需由团队通过外部方式传输并恢复到 `vision_RPi5/` 下对应路径：

```text
vision_function/ball_detection/data/
├── raw/
├── raw_similar/
└── raw_new/

vision_function/ball_detection/model/
├── shallow_cnn_v3_ncnn_model/       # 默认生产模型（实时运行必需）
│   ├── model.ncnn.bin
│   └── model.ncnn.param
├── actual_yolo11n_local_v3.pt       # 可选（YOLO 训练/评估）
└── actual_yolo11n_local_v3_ncnn_model/  # 可选（YOLO 回退路径）
    ├── metadata.yaml
    ├── model.ncnn.bin
    ├── model.ncnn.param
    └── model_ncnn.py
```

- 树莓派实时运行默认至少需要完整的 `shallow_cnn_v3_ncnn_model/`（若切回 `--detector yolo`，则需 `actual_yolo11n_local_v3_ncnn_model/`）。
- 仅在重新准备数据集、训练、导出或评估模型时需要原始数据与 `.pt` 权重。
- 外部传输后建议按 `model_accuracy_speed_summary_20260801.md` 中记录的 SHA-256 校验模型完整性。

### 2. 创建 Python 虚拟环境

在 `vision_RPi5/` 目录（或树莓派部署根目录）下创建虚拟环境：

```bash
cd 2026H/vision_RPi5
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install \
  -r vision_function/requirements.txt \
  -r vision_function/ball_detection/requirements.txt
```

树莓派无外网环境时，应在相同架构与 Python 版本的设备上预打包 wheel 或直接迁移已验证的 `.venv`。

---

## 树莓派部署与使用

> 下述树莓派部署命令中的 `/home/nailong/NUEDC_2026_H_VISION` 为实车树莓派上的默认部署路径（可直接将本 `vision_RPi5/` 目录内容同步至该路径）；如实际路径不同，请同步调整 `cd` 路径以及 `deploy/` 下的 `.service`、`.desktop` 与脚本配置。

### 1. 部署并启动图传服务

首次部署到树莓派：

```bash
cd raspberry-pi-camera-stream/deploy
sudo bash install.sh
```

已部署设备启动与状态检查：

```bash
sudo systemctl start ustreamer
sudo systemctl status ustreamer --no-pager
```

在同一局域网电脑浏览器打开 `http://<树莓派IP>:8080/`，支持原图、全图去畸变与“识别 ROI”三种视图，截图（PNG）和录像（WebM）均在电脑端浏览器完成，不写入树莓派。

| HTTP 接口 | 用途 |
|---|---|
| `/` | WebGL 网页预览、参考线、截图与录像 |
| `/stream` | 原始 MJPEG 连续流 |
| `/snapshot` | 原始最新 JPEG 单帧（实时识别使用） |
| `/state` | uStreamer 运行状态 JSON |

### 2. 检查摄像头分辨率与标定约束

```bash
camres 1280x720 60
curl --max-time 3 -o /tmp/camera-check.jpg http://127.0.0.1:8080/snapshot
```

当前鱼眼标定支持 `1280×720`（并可等比例用于 `1920×1080`）；未经重新标定切勿使用 4:3 分辨率进行去畸变与定位。

### 3. 不连接 MCU 的完整 Dry-Run

确认生产 NCNN 模型恢复完成后，在视觉工程根目录执行：

```bash
.venv/bin/python -m vision_function.scale_location.runtime \
  --dry-run \
  --iterations 130 \
  --status-every 10 \
  --report vision_function/scale_location/output/camera_to_uart_dry_run.json
```

默认使用 `--detector shallow_cnn` 与 `--calibration-source color-markers`；如需切换至 YOLO 回退或静态分段线性标定，可显式附加 `--detector yolo` 或 `--calibration-source fixed`。

### 4. 连接 MSPM0G3507 并启动实时 UART 输出

Raspberry Pi 5 与 MSPM0G3507 使用 **3.3 V TTL** 共地连接（`115200 8N1`）：

| Raspberry Pi 5 | MSPM0G3507（`Rpi_UART` / UART2） | 说明 |
|---|---|---|
| GPIO14 / TXD0 / 物理 Pin 8 | PA24 / UART2 RX | 视觉状态帧（`0x01`）与调参请求（`0x81`） |
| GPIO15 / RXD0 / 物理 Pin 10 | PB15 / UART2 TX | 赛道调参响应（`0x82`，仅视觉单向时可悬空） |
| GND / 物理 Pin 6 | GND | 必须共地 |

检查串口复用（确认 GPIO14 为 `TXD0`、GPIO15 为 `RXD0`，切勿使用 Debug Header 对应的 `/dev/serial0`）：

```bash
ls -l /dev/ttyAMA0
pinctrl get 14
pinctrl get 15
```

手动运行实时链路（`Ctrl+C` 停止）：

```bash
.venv/bin/python -m vision_function.scale_location.runtime \
  --device /dev/ttyAMA0 \
  --status-every 30
```

或通过 systemd 服务后台常驻运行：

```bash
sudo systemctl enable --now vision-scale-uart.service
systemctl status vision-scale-uart.service --no-pager
journalctl -u vision-scale-uart.service -f
```

### 5. 桌面调试与现场调参工具

- **视觉监看调试台**（读取 `/dev/shm/nuedc-vision-debug` 共享内存，不抢占 UART、不重复推理）：
  ```bash
  /bin/bash scripts/start_rpi_gui.sh
  ```
- **赛道参数调试台**（启动时自动暂停 `vision-scale-uart.service` 并在退出时恢复，通过双向 UART 读写 MSPM0 的 H2/H5/H6 13 项参数）：
  ```bash
  /bin/bash scripts/start_track_tuning.sh
  ```

---

## 离线检测、训练与单元测试

离线运行钢球检测：

```bash
.venv/bin/python vision_function/ball_detection/detect_ball.py \
  vision_function/ball_detection/data/raw/photos
```

数据集重建与模型训练（执行前请先阅读 [`ACTUAL_DATA.md`](vision_function/ball_detection/ACTUAL_DATA.md)）：

```bash
.venv/bin/python vision_function/ball_detection/prepare_actual_dataset.py
.venv/bin/python vision_function/ball_detection/train_actual.py
```

运行单元测试（在 `vision_RPi5/` 根目录执行）：

```bash
.venv/bin/python -m unittest discover -s vision_function/tests -q
```

> 未恢复外部模型或原始数据时，预处理、标定换算、协议、调试台与调参台单元测试仍可正常运行；涉及真实 NCNN 推理、数据集重建和训练的脚本需先恢复外部资产。

---

## 关键实测指标摘要

| 项目 | 浅层 CNN v3（默认） | YOLO11n v4（回退） | 备注 |
|---|---:|---:|---|
| 模型参数量 / NCNN bin | 20,411 / ~78 KB | ~259 万 / ~9.9 MB | 2 NCNN 线程，Raspberry Pi 5 |
| 纯推理 P95（静态 ROI） | **2.39 ms** | 18.06 ms | CNN 加速约 7.55× |
| 真实摄像头全链路 P95 | **9.41 ms** | 26.53 ms | CNN 加速约 2.82× |
| 独立验证集检出 / 误报 | 383/383（100%），0/4 | 383/383（100%），0/4 | 横向误差 P95：3.49 px vs 3.53 px |
| 最大内存 RSS | **71.7 MiB** | 412.8 MiB | CNN 约为 1/5.8 |

更多实测与验收记录见：

- [`model_accuracy_speed_summary_20260801.md`](vision_function/ball_detection/output/model_accuracy_speed_summary_20260801.md)
- [`validation_summary_20260730.json`](vision_function/scale_location/output/validation_summary_20260730.json)
- [`systemd_deployment_20260730.json`](vision_function/scale_location/output/systemd_deployment_20260730.json)
