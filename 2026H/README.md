# 2026H — 车载平衡滚球运动控制系统

2026 年全国大学生电子设计竞赛（TI 杯）H 题「车载平衡滚球运动控制系统」完整工程，包含 **TI MSPM0G3507 逻辑主控固件**、**Raspberry Pi 5 视觉与图传子系统**、**硬件电路与 3D 结构件** 以及 **设计报告**。

- 官方赛题：[`H题_车载平衡滚球运动控制系统.pdf`](H题_车载平衡滚球运动控制系统.pdf)
- 设计报告：[`report/设计报告H2026241098_王佳宁_彭嘉亮_陈煦栋.pdf`](report/设计报告H2026241098_王佳宁_彭嘉亮_陈煦栋.pdf)
- 团队成员：王佳宁、彭嘉亮、陈煦栋

---

## 目录结构

| 目录 / 文件 | 说明 | 文档入口 |
|---|---|---|
| [`firmware_G3507/`](firmware_G3507/) | **TI MSPM0G3507 逻辑主控固件**：分层源码（`app / middleware / core / bsp`）、CCS/Keil 工程、调试工具与接口文档 | [`firmware_G3507/README.md`](firmware_G3507/README.md) |
| [`vision_RPi5/`](vision_RPi5/) | **Raspberry Pi 5 视觉与图传子系统**：uStreamer+WebGL 图传、局部鱼眼校正、浅层 CNN / YOLO11n NCNN 钢球识别、彩色刻度标定、UART 通信及桌面调试/调参台 | [`vision_RPi5/README.md`](vision_RPi5/README.md) |
| [`hardware/`](hardware/) | **硬件与机械结构**：天猛星拓展板 v5 原理图/PCB/3D 模型、7 个 3D 打印连接件、25 cm 摆杆彩色刻度、摄像头图样与模块资料 | [`hardware/参考资料.md`](hardware/参考资料.md) |
| [`report/`](report/) | **设计报告**：正式提交版 PDF | [`设计报告 PDF`](report/设计报告H2026241098_王佳宁_彭嘉亮_陈煦栋.pdf) |
| [`H题_车载平衡滚球运动控制系统.pdf`](H题_车载平衡滚球运动控制系统.pdf) | **赛题原文**：2026 年电赛 H 题题目与指标说明 | — |

---

## 系统总体架构

系统采用 **「逻辑主控（MSPM0G3507）+ 视觉主控（Raspberry Pi 5）」双机协同架构**：

```text
                       5 GHz Wi-Fi（HTTP /stream + WebGL 去畸变）
USB 广角摄像头 ──► Raspberry Pi 5 ────────────────────────────────► 笔记本浏览器（实时监看 / 截图 / 录像）
   (1280×720          │  1/2 IDCT 解码 → 640×100 水管局部鱼眼校正
    MJPEG 60fps)      │  浅层 CNN v3 NCNN 钢球定位 + 彩色刻度动态标定 + Kalman 滤波
                      │
                      │  UART（115200 8N1，3.3 V TTL，固定 11 字节帧）
                      │  · 0x01：视觉位置/速度/状态帧（Pi → MCU）
                      │  · 0x81 / 0x82：H2/H5/H6 赛道参数双向调参帧（Pi ↔ MCU）
                      ▼
               TI MSPM0G3507（firmware_G3507）
                 ├── 循迹闭环：Yahboom 8 路红外 + JY61P 陀螺仪 → TB6612 直流电机 + 霍尔编码器（S 曲线加减速）
                 ├── 摆杆平衡：视觉位置外环 + 步进电机/GMR 编码器角度内环 + 底盘切向加速度前馈补偿
                 └── 人机交互：OLED 菜单 + 独立按键 + Debug_Ex 遥测串口
```

### 双机物理接线与通信约定

| Raspberry Pi 5（`/dev/ttyAMA0`） | TI MSPM0G3507（`Rpi_UART` / UART2） | 说明 |
|---|---|---|
| GPIO14 / TXD0 / Pin 8 | PA24 / UART2 RX | 视觉状态帧（`0x01`）与调参请求（`0x81`） |
| GPIO15 / RXD0 / Pin 10 | PB15 / UART2 TX | 赛道调参响应（`0x82`） |
| GND / Pin 6 | GND | 3.3 V TTL 必须共地 |

- 串口参数固定为 `115200 8N1`，多字节整数小端序，固定 11 字节帧长，帧头 `0xA5 0x5A`，末尾覆盖字节 2～9 的 CRC-8（多项式 `0x07`，初值 `0xFF`）。
- 视觉状态帧协议详见 [`vision_RPi5/vision_function/scale_location/README.md`](vision_RPi5/vision_function/scale_location/README.md)；赛道调参双端协议详见 [`firmware_G3507/docs/interfaces/rpi_track_tuning.md`](firmware_G3507/docs/interfaces/rpi_track_tuning.md) 与 [`vision_RPi5/vision_function/track_tuning/README.md`](vision_RPi5/vision_function/track_tuning/README.md)。

---

## 赛题要求与实测指标

| 任务 | 赛题要求 | 实测结果 | 结论 |
|---|---|---|---|
| 1. 图传监测 | 稳定实时显示、可录制回放 | 58 fps，时延 <30 ms，覆盖 25 cm 摆杆 | 满足 |
| 2. 循线一圈 + 停车 A | ≤20 s，停车偏差 ≤2 cm | 10.63 s，停车偏差 0.9 cm | 满足 |
| 3. 静止往返 ±5 cm | O → +5 cm → -5 cm，≤5 s，误差 ≤1 cm | 3.92 s，误差 0.6 cm | 满足 |
| 4. AB 段中心保持 | ≤8 s，钢球稳定在 O，误差 ≤1 cm | 7.02 s，误差 0.7 cm | 满足 |
| 5. 整圈中心保持 | ≤30 s，钢球稳定在 O，误差 ≤1 cm | 24.31 s，误差 0.8 cm | 满足 |
| 6. 任意位置整圈 | ≤30 s，稳定在指定位置，误差 ≤1 cm | 约 24 s，误差 0.9 cm | 满足 |

---

## 快速开始

### 1. MSPM0G3507 固件侧（`firmware_G3507/`）

- 基于 `mspm0_sdk_2_10_00_04`，支持 Keil MDK 5/6（Arm Compiler 6.22）与 CCS。
- 源码采用单向分层架构：`app → middleware → bsp / core`。
- 构建、烧录、引脚与标定步骤见：
  - [`firmware_G3507/README.md`](firmware_G3507/README.md)
  - [`firmware_G3507/docs/build-guide.md`](firmware_G3507/docs/build-guide.md)
  - [`firmware_G3507/docs/architecture.md`](firmware_G3507/docs/architecture.md)
  - [`firmware_G3507/docs/step-motor-calibration.md`](firmware_G3507/docs/step-motor-calibration.md)

### 2. Raspberry Pi 5 视觉与图传侧（`vision_RPi5/`）

- 恢复团队外部传输的 NCNN 生产模型（`vision_function/ball_detection/model/shallow_cnn_v3_ncnn_model/`）并安装 Python 虚拟环境。
- 部署图传服务 `ustreamer`（HTTP 端口 `8080`）及视觉实时服务 `vision-scale-uart.service`。
- 详细部署、Dry-Run、桌面视觉监看台与赛道调参台说明见：
  - [`vision_RPi5/README.md`](vision_RPi5/README.md)
  - [`vision_RPi5/raspberry-pi-camera-stream/README.md`](vision_RPi5/raspberry-pi-camera-stream/README.md)
  - [`vision_RPi5/vision_function/README.md`](vision_RPi5/vision_function/README.md)
  - [`vision_RPi5/vision_function/ball_detection/output/model_accuracy_speed_summary_20260801.md`](vision_RPi5/vision_function/ball_detection/output/model_accuracy_speed_summary_20260801.md)

### 3. 硬件与结构资料（`hardware/`）

- [`hardware/主控板/SCH_天猛星小车拓展板v5.pdf`](hardware/主控板/SCH_天猛星小车拓展板v5.pdf) — 主控拓展板 v5 原理图
- [`hardware/3D打印模型/`](hardware/3D打印模型/) — 巡线转接件、摄像头底座、水管合页与步进电机连接件（`.SLDPRT`）
- [`hardware/水管彩色刻度/`](hardware/水管彩色刻度/) — 25 cm 摆杆彩色刻度（PDF / PSD）
- [`hardware/参考资料.md`](hardware/参考资料.md) — 电机、传感器与调试器资料索引
