# 钢球刻度位置与速度估计方案

本模块位于钢球检测和运动控制之间。输入是 `ball_detection` 输出的去畸变球心、
摄像头帧时间戳和目标刻度，输出是钢球当前位置、相对目标位移、控制误差和速度。

当前已完成正式多刻度分段线性标定和 Raspberry Pi 5 五点静态验收，最大绝对偏差
为 `0.5211 cm`，暂按题目 `1 cm` 限值接受；后续仍需优化正半轴偏差、左端检出率
并使用动态数据重新调整速度估计参数。

## 1. 赛题约束

题目中摆杆由长 25 cm 的 PPR 水管制成，刻度间隔为 0.1 cm，中心点定义为
`0 cm`，图示有效刻度约为 `-12 cm` 至 `+12 cm`。主要控制要求包括：

- 静止车辆上，小球在 5 s 内从 `+5 cm` 运动至 `-5 cm` 并稳定；
- 指定位置和行驶过程中的最大位置误差绝对值不超过 1 cm；
- 摄像头必须覆盖整个摆杆并判断钢球位置。

当前 Raspberry Pi 5 实测帧时间戳到识别结果的 P95 延迟为 26.81 ms，控制链路
约为 37.3 fps。因此位置和速度必须携带拍摄时间戳，并补偿视觉处理延迟。

## 2. 坐标定义

统一使用厘米和秒：

- 摆杆中心刻度为 `0 cm`；
- 图像中沿摆杆向右为正，向左为负；
- 目标刻度 `target_cm` 的默认有效范围为 `[-12.0, 12.0]`；
- 速度 `velocity_cm_s` 向右为正、向左为负。

同时输出两种误差，避免控制方向歧义：

```text
displacement_cm  = position_now_cm - target_cm
control_error_cm = target_cm - position_now_cm
```

下游闭环控制器使用 `control_error_cm` 和 `velocity_now_cm_s`。例如目标为
`+5.0 cm`、当前位置为 `+3.2 cm` 时：

```text
displacement_cm  = -1.8 cm
control_error_cm = +1.8 cm  # 仍需向右运动
```

## 3. 输入坐标

位置换算使用 `BallDetector` 输出的 `center_local_xy`，即鱼眼校正后
640x100 水管局部图中的球心坐标，不使用原始鱼眼坐标或模型 letterbox 坐标。

局部坐标的约束是：

- 标定点、水管轴线和球心可以使用同一个坐标系；
- 无需先恢复至 1280x720，避免混用缩放倍率；
- 局部画布参数改变时必须拒绝加载旧标定。

标定配置必须绑定以下参数，任一参数改变后应拒绝加载旧标定：

- 摄像头标定文件及其校验值；
- 局部输出分辨率 640x100；
- `fisheye_balance=0.2`；
- 校正平面参考区域 `(80,252,1280,200)`；
- 图像左右方向和摄像头安装方向。

## 4. 二维球心到摆杆一维位置

### 4.1 轴向投影

设校正后的球心为 `p=(x,y)`，摆杆轴线原点为 `p0`，向右的单位方向向量为
`d`，则球心在摆杆轴线上的像素坐标为：

```text
u = dot(p - p0, d)
```

这样即使摆杆在图像中有少量倾斜，也不会直接把图像横坐标误认为轴向坐标。

### 4.2 为什么不只解三角形

若只使用水管左右端点，按照球心在两端之间的比例计算 25 cm 物理位置，本质上
就是两个端点之间的线性插值，也可视为相似三角形法在一维轴线上的形式。

严格的三维几何求解需要额外知道：

- 摄像头相对水管的准确高度、位置和姿态；
- 水管每一帧所在的三维平面和摆动角度；
- 图像射线与水管平面的交点。

单摄像头只提供射线，水管摆动时若没有准确角度或平面信息，无法仅凭一个球心
像素确定其三维深度。安装高度和角度的测量误差也会直接变成系统性位置误差。

空间直线上的物理坐标 `s` 与透视图像坐标 `u` 一般满足分式关系：

```text
u(s) = (a*s + b) / (c*s + d)
```

只有水管近似平行于成像平面时才近似线性。当前水管相对摄像头基本固定、画面中
倾斜很小且已完成鱼眼校正，因此优先用真实刻度直接标定综合成像关系，比依赖不易
精确测量的摄像头高度和安装角度更稳健。

### 4.3 多刻度分段线性标定

拍摄一张刻度清晰的去畸变图，无需把钢球精确放置在刻度上。人工点击多个已知
刻度位置，例如：

```text
-12, -9, -6, -3, 0, +3, +6, +9, +12 cm
```

用这些点拟合摆杆轴线，再将每个标定点投影成 `u`，保存单调递增的
`(u_px, scale_cm)` 锚点。运行时在相邻锚点间做分段线性插值。

分段线性插值的优点：

- 能吸收轻微透视和鱼眼校正残差；
- 不要求水管实际长度恰好为 25 cm；
- 不要求准确测量摄像头外参；
- 不会像高阶多项式一样在水管两端振荡；
- 标定结果可直接叠加到图像上人工检查。

如果只标定左右端点，该方法自然退化为普通的 25 cm 比例换算。超出标定锚点
范围时返回无效状态，不做无提示的外推或截断。

### 4.4 后续动态几何修正

第一版使用固定轴线。取得车辆振动数据后，可在每帧检测摆杆两端或刻度带，将
球心归一化到当前摆杆局部坐标：

```text
lambda = ball_axis_distance_px / current_rod_length_px
position_cm = calibration(lambda)
```

这可以补偿摆杆在画面中的平移、旋转和少量投影长度变化。如果以后能从编码器或
倾角传感器得到准确摆角，再增加相机射线与水管平面求交的完整几何模型；刻度标定
仍可用于修正几何模型的安装偏差。

## 5. 标定配置格式

建议保存为 `config/scale_calibration.json`：

```json
{
  "schema_version": 1,
  "coordinate_space": "tube_local_rectified_640x100",
  "reference_image_size": [640, 100],
  "fisheye_balance": 0.2,
  "positive_direction": "image_right",
  "axis_origin_xy": [60.0, 50.0],
  "axis_direction_xy": [1.0, 0.0],
  "anchors": [
    {"axis_px": 10.0, "scale_cm": -12.0},
    {"axis_px": 300.0, "scale_cm": 0.0},
    {"axis_px": 570.0, "scale_cm": 12.0}
  ],
  "valid_range_cm": [-12.0, 12.0]
}
```

示例数值仅表示格式，不能直接用于实车。

## 6. 位置和速度状态估计

### 6.1 不使用相邻帧直接差分

当前 v2 模型的动态球心 P95 误差约为 5.87 ROI 像素。若直接计算：

```text
velocity = (position[n] - position[n-1]) / dt
```

约 28 ms 的采样间隔会明显放大位置噪声。长窗口滑动平均虽然能降低噪声，但会
引入明显相位延迟。例如 5 帧居中窗口自身约增加 56 ms 延迟。

### 6.2 带时间戳的 Kalman 状态估计

使用摄像头 `/snapshot` 响应中的 `X-Timestamp`，建立状态：

```text
state = [position_cm, velocity_cm_s, acceleration_cm_s2]
```

测量量只有刻度位置。状态预测使用相邻摄像头帧时间戳之差，而不是处理循环耗时。
测量置信度较低时增大测量噪声，异常跳变通过创新门限拒绝。

需要处理：

- 初始化后至少积累 3 至 5 个有效观测，才将速度标记为有效；
- 缺失 1 至 3 帧时短时间只做预测；
- 连续缺失超过约 100 ms 后输出 `valid=false`；
- 时间戳倒退、重复或间隔异常时重置估计器；
- 目标刻度改变不重置位置和速度状态。

如果没有足够数据稳定估计加速度，第一版可使用 `[position, velocity]` 常速度
Kalman 模型，但接口仍保留加速度字段。

## 7. 视觉时延补偿

检测结果对应的是图像拍摄时刻，不是程序输出时刻。设：

```text
delay = output_timestamp - capture_timestamp
```

将滤波状态预测到当前控制时刻：

```text
position_now = position_at_capture
             + velocity_at_capture * delay
             + 0.5 * acceleration * delay^2

velocity_now = velocity_at_capture + acceleration * delay
```

当前 P95 延迟约为 26.81 ms。如果小球速度为 20 cm/s，不补偿会产生约
0.55 cm 的陈旧位置；速度为 50 cm/s 时约为 1.38 cm，已经超过赛题 1 cm
误差要求。

预测时长应限制在约 50 ms 内。超过限制说明链路异常，此时输出无效或降级状态，
不能继续向远期外推。

若能够获得摆杆角度或舵机命令，可将其作为加速度预测输入。实际滚动加速度与
`g*sin(angle)` 相关，系数通过实车实验辨识，可以降低舵机刚动作时纯视觉速度
估计的响应延迟。

## 8. 对外接口

建议让刻度模块与 YOLO 解耦：

```python
locator = ScaleLocator("config/scale_calibration.json")
locator.set_target(5.0)

state = locator.update(
    center_xy=detection["center_local_xy"],
    capture_timestamp=frame_timestamp,
    confidence=detection["confidence"],
    output_timestamp=current_time,
)
```

输出数据结构：

```python
ScaleState(
    valid=True,
    target_cm=5.0,
    position_at_capture_cm=3.15,
    velocity_at_capture_cm_s=18.2,
    acceleration_cm_s2=30.0,
    position_now_cm=3.66,
    velocity_now_cm_s=19.0,
    displacement_cm=-1.34,
    control_error_cm=1.34,
    capture_timestamp=0.0,
    measurement_age_ms=27.5,
    confidence=0.73,
)
```

`position_now_cm` 和 `velocity_now_cm_s` 只用于 Pi 本地日志、预览或以后由 Pi
直接闭环控制。按照第 11 节 UART 协议与当前 MCU 分工，Pi 实际发送拍摄时刻的
`position_at_capture_cm` 和 `velocity_at_capture_cm_s`，并发送测量龄 `AGE`，由
MCU 使用自身掌握的摆杆倾角把状态前推到控制时刻。不能发送已经前推过的
`position_now_cm` 再让 MCU 前推，否则会重复补偿时延。

## 9. 计划目录结构

```text
scale_location/
├── README.md
├── __init__.py
├── types.py
├── calibration.py
├── mapper.py
├── estimator.py
├── pipeline.py
├── protocol.py
├── uart_transport.py
├── runtime.py
├── calibrate_scale.py
├── tests/
│   ├── test_mapper.py
│   ├── test_estimator.py
│   ├── test_protocol.py
│   └── test_pipeline.py
└── config/
    ├── scale_calibration.json
    ├── motion_filter.json
    └── uart.json
```

- `calibration.py`：配置读取、坐标系和兼容性校验；
- `mapper.py`：轴向投影与分段线性刻度插值；
- `estimator.py`：位置、速度、加速度状态估计；
- `pipeline.py`：目标管理、时延补偿和最终输出；
- `protocol.py`：定点转换、固定 11 字节打包和 CRC-8；
- `uart_transport.py`：串口配置、整包写入和发送统计；
- `runtime.py`：串联 `/snapshot`、检测、定位、状态估计和 UART；
- `calibrate_scale.py`：人工点击刻度、拟合并生成标定预览；
- `types.py`：输入观测和输出状态数据结构。

## 10. 实施和验证顺序

1. 拍摄一张实际刻度清晰的去畸变图；
2. 实现人工刻度点击、轴线拟合和标定文件生成；
3. 实现固定轴线的分段线性位置换算；
4. 使用现有视频回放实现并调整 Kalman 位置和速度估计；
5. 加入从拍摄时刻预测至控制时刻的延迟补偿；
6. 在树莓派实时输出位置、速度和状态有效性；
7. 使用已知位置和受控运动数据验证绝对位置与速度误差；
8. 收集车辆振动数据后评估是否需要每帧摆杆轴线修正。

建议验收指标：

- 静止小球的位置 P95 抖动不超过 0.3 cm；
- 静止速度估计应稳定接近 0 cm/s；
- 标定范围内绝对位置误差目标不超过 0.5 cm，为赛题 1 cm 指标留出余量；
- 丢失检测和异常时间戳时不得继续输出看似有效的陈旧控制量；
- 输出中的符号方向、单位和时间戳必须通过单元测试固定。

## 11. `ball_detection` 输入契约

检测模块和刻度模块之间使用明确的数据结构，不让刻度模块依赖 YOLO 的原始结果
对象或模型 letterbox 坐标：

```python
@dataclass(frozen=True)
class BallObservation:
    frame_sequence: int
    capture_timestamp_s: float
    found: bool
    center_local_xy: tuple[float, float] | None
    confidence: float | None
    box_local_xyxy: tuple[float, float, float, float] | None = None
    candidate_count: int = 0
```

推荐调用接口：

```python
observation, crop = detector.detect(
    frame,
    capture_timestamp_s=frame_timestamp,
    frame_sequence=frame_sequence,
)
```

字段约束：

- `capture_timestamp_s` 必须是当前 JPEG 的 uStreamer `X-Timestamp` 原值，不能在
  检测完成后重新打时间戳；
- `frame_sequence` 对每个不同的摄像头帧递增，而不是只对成功检出的帧递增；
- `found=false` 时，球心、检测框和置信度均为 `None`；
- `center_local_xy` 是 `640x100` 水管局部校正平面内、进入模型 letterbox 前的
  浮点坐标；
- `confidence` 保留最高候选的原始置信度，不在检测模块内提前分档；
- `box_local_xyxy` 和 `candidate_count` 只用于质量分析、日志和故障排查。

`MOVING`、`EDGE`、`V_VALID` 和 `QUALITY` 不由 `ball_detection` 输出：

- `MOVING` 由刻度模块根据物理位置变化、速度和滞回阈值生成；
- `EDGE` 由标定有效范围和端部安全区生成；
- `V_VALID` 由状态估计器是否已积累足够观测生成；
- `QUALITY` 综合检测置信度、Kalman 创新残差、连续性和边缘状态生成。

速度也不能由检测模块做相邻帧直接差分，必须由刻度模块使用拍摄时间戳统一估计。

控制运行入口使用 `X-Timestamp` 判断 `/snapshot` 是否返回了重复帧。重复帧不再次
推理、不增加协议 `SEQ`、也不发送重复测量。每个不同的新帧都发送一包，包括
没有检出球的帧。

## 12. Pi 到 MCU 的 UART 协议

### 12.1 物理层和发送时机

当前链路为单向 Pi 到 MCU：

```text
Pi GPIO14 / TX / 物理 Pin 8  -> MSPM0 PA9 / UART1 RX
Pi GND / 物理 Pin 6          -> MSPM0 GND
Pi GPIO15 / RX / 物理 Pin 10 -> 单向协议下不连接
```

两端使用 3.3 V TTL 并共地，不得向 Raspberry Pi GPIO 输入 5 V。串口参数固定为
`115200 8N1`，关闭软、硬件流控。发送与摄像头新帧锁定，每取得一个不同的新帧并
完成处理后发送一包，不额外使用定时发送线程。

### 12.2 固定 11 字节帧

| 偏移 | 字节 | 字段 | 类型 | 含义 |
|---:|---:|---|---|---|
| 0 | 1 | `SYNC0` | `0xA5` | 帧头 |
| 1 | 1 | `SYNC1` | `0x5A` | 帧头 |
| 2 | 1 | `VER` | `uint8` | 当前为 `0x01` |
| 3 | 1 | `SEQ` | `uint8` | 每个新摄像头帧加一，溢出回绕 |
| 4 | 1 | `AGE` | `uint8` | 测量龄，单位 ms |
| 5 | 2 | `X` | `int16 LE` | 球位置，0.1 mm/LSB，摆杆中心为零 |
| 7 | 2 | `V` | `int16 LE` | 球速度，1 mm/s/LSB，向右为正 |
| 9 | 1 | `FLAGS` | `uint8` | 状态位域 |
| 10 | 1 | `CRC8` | `uint8` | 覆盖偏移 2 至 9 |

`FLAGS` 定义：

| 位 | 名称 | 含义 |
|---:|---|---|
| 0 | `VALID` | 本帧检出球且位置测量被估计器接受 |
| 1 | `MOVING` | 位置变化超过配置的运动阈值 |
| 2 | `V_VALID` | 速度估计已稳定可用 |
| 3 | `EDGE` | 球位于可测范围端部或范围之外 |
| 4-5 | `QUALITY` | `0..3`，3 最好 |
| 6-7 | 保留 | 固定置零 |

定点转换为：

```python
x_01mm = round(position_at_capture_cm * 100)
velocity_mm_s = round(velocity_at_capture_cm_s * 10)
```

编码包体可使用：

```python
body = struct.pack(
    "<BBBhhB",
    0x01,
    sequence,
    age_ms,
    x_01mm,
    velocity_mm_s,
    flags,
)
packet = b"\xA5\x5A" + body + bytes([crc8(body)])
```

定点值超过 `int16` 量程时不能静默截断，必须清除 `VALID` 和 `V_VALID` 并记录
故障。未检出时仍发送一包并令 `VALID=0`；`X/V` 可保留最近估计状态，但 MCU
必须按标志忽略无效值。

### 12.3 测量龄

`AGE` 必须在紧邻整包 `write()` 前计算：

```text
AGE = 即将写出的时刻 - 当前帧 capture_timestamp
```

计算 `AGE` 使用与 `X-Timestamp` 匹配的系统实时时基。11 字节必须通过一次
`write(packet)` 写入，串口发送线程不得积压多帧。`AGE > 255 ms` 时发送
`AGE=255` 并清除 `VALID/V_VALID`，不允许 uint8 回绕。

MCU 在收到末字节时记录本地时刻，并计算：

```text
measurement_age = AGE + 0.955 ms + 收到末字节后的本地耗时
```

其中 `0.955 ms = 11 * 10 / 115200`。MCU 使用该测量龄、收到的拍摄时刻 `X/V`
以及自身倾角信息完成前推。

### 12.4 CRC-8

CRC 参数固定为：

```text
多项式       0x07 (x^8 + x^2 + x + 1)
初值         0xFF
输入/输出    均不反转
最终异或     无
覆盖范围     VER 至 FLAGS，偏移 2..9，共 8 字节
```

实现通信前必须生成至少三个固定协议测试向量，同时用于 Python 单元测试和 MCU C
单元测试，先验证字节序、单位和 CRC，再进行硬件接线。

### 12.5 Raspberry Pi 5 配置约束

Pi 5 默认主串口位于独立 Debug Header，不能假定 `/dev/serial0` 自动对应
GPIO14/15。部署时应：

1. 使用 `dtoverlay -h uart0-pi5` 核对系统提供的 Pi 5 UART overlay 和引脚参数；
2. 在 `/boot/firmware/config.txt` 启用 GPIO14/15 对应的 UART，并关闭对应的
   Linux serial console；
3. 区分 Pi 5 独立 Debug Header 与 GPIO14/15 UART。当前实机
   `/dev/serial0 -> ttyAMA10` 和 `BOOT_UART=1` 只用于独立 Debug Header，MCU
   使用 `uart0-pi5` 创建的 `/dev/ttyAMA0`，两者不冲突；不得把 MCU 接到
   `/dev/serial0`；
4. 重启后用 `readlink -f /dev/serial*` 和 `pinctrl get 14 15` 验证实际设备节点及
   引脚复用；
5. `uart.json` 保存验证后的设备节点，例如 `/dev/ttyAMA0`，而不是未经确认地
   硬编码 `/dev/serial0`。

## 13. 通信相关失效策略

Pi 侧必须保证：

- 时间戳缺失、倒退、重复或间隔异常时不产生看似正常的新测量；
- 检测丢失时继续按新帧发送 `VALID=0`，使 MCU 能区分视觉失效与 UART 断链；
- 速度估计未初始化、重置或观测间隔异常时清除 `V_VALID`；
- UART 写入失败、短写或超时时计数并输出明确日志，不阻塞视觉循环无限重试；
- 记录 `tx_frames`、`write_fail`、`invalid`、`max_age` 和实测发送帧率。

MCU 侧按协议文档执行 CRC/版本/序号检查；`VALID=0` 不得解释成 `X=0`，
`V_VALID=0` 时不得使用速度反馈项，连续 200 ms 没有有效帧时关闭球位置环并将
摆杆恢复到安全水平角。

## 14. 更新后的实施顺序

1. 将 `BallDetector` 输出改为第 11 节的 `BallObservation`，并保持旧字典接口的
   短期兼容测试；
2. 使用当前安装状态下的清晰刻度图完成 `640x100` 局部坐标标定；
3. 实现位置映射、端部判断和带拍摄时间戳的 Kalman 状态估计；
4. 使用现有动态视频调整过程噪声、运动滞回阈值和质量分档；
5. 实现协议编码、CRC 和 Python/MCU 共用固定测试向量；
6. 使用逻辑分析仪验证 GPIO14 上每个新帧对应一个完整 11 字节包；
7. 接入 MCU，验证 `SEQ`、CRC、`AGE`、无球帧和 200 ms 失效保护；
8. 使用已知位置和受控运动数据验收绝对位置、速度、时延补偿和端部行为。

## 15. 当前临时标定实现（2026-07-30）

本轮只实现测试性质的 25 cm 假设标定，不修改 `ball_detection` 输出接口。数据来自：

```text
ball_detection/data/raw/boundary/empty/
├── snapshot-2026-07-30T04-08-48-126Z.png
├── snapshot-2026-07-30T04-08-49-855Z.png
└── snapshot-2026-07-30T04-08-52-113Z.png
```

三张原图均使用当前 `tube_local_rectification.json` 转换到 `640x100` 水管局部
校正平面。取三帧中值图后量取：

| 临时锚点 | 局部坐标 `(x,y)` | 假设物理位置 |
|---|---:|---:|
| 水管左端 | `(59.0, 50.0)` | `-12.5 cm` |
| 黑色胶带中心 | `(286.0, 50.0)` | `0.0 cm` |
| 水管右端 | `(582.0, 50.0)` | `+12.5 cm` |

三帧分别估计出的胶带横坐标约为 `285.73`、`286.27`、`286.03 px`，因此临时
配置取 `286.0 px`。左右两段分别独立做线性插值，以吸收当前透视比例不对称：

```text
左半段：227 px 对应 12.5 cm
右半段：296 px 对应 12.5 cm
```

配置保存在
`config/scale_calibration_temporary_25cm.json`，并显式标记为
`temporary_test_only`。配置记录了三张源图、去畸变配置及其 SHA-256，禁止将它
重命名成正式标定文件。正式多刻度标定完成后替换配置并重置运动估计器即可，位置
换算代码和 UART 协议不变。

临时标定的快速检查：

```python
from vision_function.scale_location import load_scale_calibration

calibration = load_scale_calibration(
    "vision_function/scale_location/config/scale_calibration_temporary_25cm.json"
)
measurement = calibration.measure((286.0, 50.0))
assert measurement.valid
assert measurement.position_cm == 0.0
```

现有边界数据中的球心代入该临时标定后，左侧极限附近约为 `-11.5 cm`，右侧
极限附近约为 `+11.7 cm`，与钢球半径导致球心不能到达水管几何端点的现象一致。
这些数值只能用于当前测试，不作为最终精度结论。

### 15.1 正式多刻度标定工具

`calibrate_scale.py` 使用与生产检测相同的 `tube_local_rectification.json`，把
1280x720 原图或 640x360 缩小图转换成 640x100 水管局部校正图。传入多张图片时
先取中值，再依次点击已知刻度，最后拟合轴线并生成分段线性标定配置和预览图。

保持摄像头、水管和照明为正式安装状态，移走钢球并拍摄数张刻度清晰的图片。例如
直接使用一个只包含本次标定照片的目录：

```bash
python -m vision_function.scale_location.calibrate_scale \
  path/to/scale-images \
  --marks=-12,-9,-6,-3,0,3,6,9,12
```

点击必须按照窗口底部提示从左到右进行。按 `U` 或退格撤销，按 `R` 清空，按
`1`、`2`、`3` 即时切换显示倍率，全部点击完成后按回车确认，按 `Esc` 取消。
提示栏单独显示在图像下方，不遮挡 ROI。默认输出：

```text
vision_function/scale_location/config/scale_calibration.json
vision_function/scale_location/output/scale_calibration_preview.png
```

输出已存在时工具会拒绝覆盖；人工检查预览无误后，使用 `--force` 明确替换。若要
复现已有点击，可传入 `--points points.json`，其格式为：

```json
[
  {"scale_cm": -12.0, "x": 61.2, "y": 47.8},
  {"scale_cm": 0.0, "x": 286.4, "y": 50.1},
  {"scale_cm": 12.0, "x": 579.7, "y": 57.2}
]
```

工具会检查物理刻度和拟合后的轴向像素是否严格递增、点位是否位于 640x100 图内，
并在配置中记录源图和局部校正配置的 SHA-256。正式配置现已成为运行时默认值，也可
使用以下参数显式选择：

```bash
.venv/bin/python -m vision_function.scale_location.runtime \
  --calibration vision_function/scale_location/config/scale_calibration.json \
  --dry-run \
  --status-every 10
```

先在 `-10/-5/0/+5/+10 cm` 等已知位置完成静态复核，再接入 UART。修改摄像头
位置、安装方向、鱼眼参数或局部校正区域后，旧标定立即失效，必须重新生成。

## 16. 当前通信实现（2026-07-30）

已实现：

- `protocol.py`：物理量定点化、FLAGS、CRC-8、11 字节编解码和严格范围检查；
- `uart_transport.py`：Linux/POSIX `115200 8N1` 配置、单次 `os.write()`、发送前
  `AGE` 计算、`SEQ` 回绕、超龄失效和短写报错；
- `config/uart.json`：待在 Pi 5 上核实设备节点的部署配置；
- `test_scale_protocol.py` 和 `test_uart_transport.py`：固定向量、字节序、CRC、
  测量龄、超龄与短写测试。

三组供 MCU C 端互验的固定协议向量如下：

| 场景 | 完整 11 字节十六进制包 |
|---|---|
| 零位置、`AGE=27`、有效且速度有效、质量 3 | `A5 5A 01 00 1B 00 00 00 00 35 E9` |
| `X=-12.5 cm`、`V=-30 cm/s`、SEQ/AGE=255 | `A5 5A 01 FF FF 1E FB D4 FE 2F 41` |
| `X=+12.5 cm`、`V=+30 cm/s`、无效、运动、边缘、质量 1 | `A5 5A 01 01 00 E2 04 2C 01 1A 5A` |

发送层可单独使用：

```python
from vision_function.scale_location.uart_transport import (
    FrameLockedSender,
    PosixUart,
    UartSettings,
)

settings = UartSettings.load("vision_function/scale_location/config/uart.json")
with PosixUart(settings) as uart:
    sender = FrameLockedSender(uart)
    sender.send(
        capture_timestamp_s=frame_timestamp,
        position_cm=position_at_capture_cm,
        velocity_cm_s=velocity_at_capture_cm_s,
        valid=True,
        moving=True,
        velocity_valid=True,
        edge=False,
        quality=3,
    )
```

目标 Raspberry Pi 5 已于 2026-07-30 实测确认：加载 `uart0-pi5` 后
`/dev/ttyAMA0` 对应 GPIO14=`TXD0`、GPIO15=`RXD0`，用户 `nailong` 属于
`dialout` 组。`/dev/serial0 -> ttyAMA10` 是独立调试串口，不能用于 MCU。
`/boot/firmware/config.txt` 已持久化 `dtoverlay=uart0-pi5`，原文件备份为
`config.txt.before-scale-uart-20260730`。

## 17. 图像采集到 UART 的完整运行链路（2026-07-30）

现已按第 11、12 节接口打通严格串行的数据流：

```text
uStreamer /snapshot + X-Timestamp
→ JPEG 1/2 IDCT 解码为 640x360
→ 640x100 水管局部鱼眼 remap
→ v3 NCNN 单球识别（imgsz=320，2 线程）
→ BallObservation
→ 正式多刻度分段线性位置映射
→ 带拍摄时间戳的常速度 Kalman 状态估计
→ ScaleState
→ AGE/FLAGS/定点化/CRC-8
→ 单次 write() 发出 11 字节 UART 包
```

`BallDetector.detect()` 现在直接返回第 11 节定义的 `BallObservation`：

```python
observation, tube_crop = detector.detect(
    reduced_frame,
    capture_timestamp_s=frame_timestamp,
    frame_sequence=frame_sequence,
)
```

迁移期间如有旧离线程序需要原字典，可调用 `detector.to_legacy_dict(observation)`
或 `detector.detect_legacy(frame)`。实时链路不再使用旧字典。

`runtime.py` 保证：

- 只有 `X-Timestamp` 不同的新帧才推理并发送，重复 `/snapshot` 不增加 `SEQ`；
- 每个不同的新帧发送一包，无球、越界或估计拒绝时发送 `VALID=0`；
- 连续 2 个有效观测后才置 `VALID=1`，连续丢失超过 100 ms 后重置估计器，避免
  空管中的孤立单帧误检进入控制环；
- `X` 和 `V` 是拍摄时刻的滤波位置、速度，不是前推后的 `position_now`；
- `AGE` 在 UART `write()` 前计算，11 字节只调用一次 `write()`；
- 时间戳缺失、倒退、超过 255 ms、短写和 UART 异常都有明确失效处理或计数；
- `--target-cm` 只影响 Pi 日志中的位移和控制误差。固定 11 字节协议不含目标值，
  MCU 的目标仍由 MCU 自己管理。

先用真实摄像头走完整协议但不打开串口：

```bash
cd /home/nailong/NUEDC_2026_H_VISION
.venv/bin/python -m vision_function.scale_location.runtime \
  --dry-run \
  --iterations 130 \
  --status-every 10 \
  --report vision_function/scale_location/output/camera_to_uart_dry_run.json
```

确认 GPIO14 对应设备节点且 Pi 与 MCU 使用 3.3 V TTL 共地后，去掉 `--dry-run`
并明确指定核实过的节点：

```bash
.venv/bin/python -m vision_function.scale_location.runtime \
  --device /dev/ttyAMA0 \
  --status-every 30
```

当前默认使用 `scale_calibration.json`。2026-07-30 五点静态 dry-run 共采集
500 帧，最大绝对偏差为 `0.5211 cm`；该结果暂时接受，但尚未替代动态运动、车体
振动和闭环控制验收。

### 17.1 运动漏检诊断抓图

运行时可选择保存未检出或低置信度的 640x100 水管局部图。该功能默认关闭，仅用于
离线诊断；写图会扰动实时延迟，因此应配合 `--dry-run` 使用：

```bash
.venv/bin/python -m vision_function.scale_location.runtime \
  --dry-run \
  --iterations 1800 \
  --diagnostic-dir vision_function/scale_location/output/dynamic_diagnostics \
  --diagnostic-confidence-below 0.5 \
  --diagnostic-max-images 500 \
  --report vision_function/scale_location/output/dynamic_diagnostics_report.json
```

目录内的 `manifest.json` 记录每张图片的帧序号、时间戳、检出状态、置信度和球心。
达到图片上限后程序继续统计符合条件的帧，但不再写图。诊断目录属于运行产物，不应
强制提交 Git；筛选出的代表性样本应按数据集流程单独归档。

2026-07-30 的慢速动态测试在默认 `0.25` 阈值下检出 1703/1800 帧。对 97 张
漏检 ROI 离线重放后，`0.10` 阈值找回 82 张（84.5%）；完整 v3 数据集中的
31 张无球样本在 `0.10` 阈值下均无误报，最高伪候选置信度为 `0.0642`，而
`0.05` 已产生 10/31 误报。因此生产默认值调整为 `0.10`，不建议继续下调；仍可
通过 `--conf` 显式覆盖以进行对照测试。

默认值部署后又完成一轮不写诊断图片的 1800 帧实机动态复测：检出 1791 帧
（99.50%），1788 帧进入有效状态，最长连续漏检 3 帧，位置覆盖
`-9.63～+12.13 cm`，共 30 次过零。平均处理耗时为 26.41 ms；相较 `0.25`
阈值测试的 94.61% 检出率和 9 帧最长漏检，动态连续性明显改善。

### 17.2 Raspberry Pi 5 实机结果

2026-07-30 在 `/dev/ttyAMA0` 上使用真实摄像头完成 130 帧硬件发送测试：

- 130 个新帧对应 130 次完整 11 字节写入，`write_fail=0`；
- 无重复帧、时间戳缺失、倒退或 JPEG 解码失败；
- 平均采集到 UART 写入完成 26.10 ms，最大协议 `AGE=28 ms`；
- 空管场景产生 2 次非连续候选，连续 2 帧确认门限将其全部保持为
  `VALID=0`，没有孤立误报进入控制环；
- 保存的实车有球样本连续发送时，第 1 帧确认、从第 2 帧起 `VALID=1`、从第 4
  帧起 `V_VALID=1`；静止位置约 `+10.09 cm`，AGE 为 22～23 ms。

空管硬件原始报告见
[`camera_to_uart_hardware_confirmed.json`](output/camera_to_uart_hardware_confirmed.json)。

正式标定和 `0.10` 检测阈值部署后，又执行了 300 帧真实 MCU UART 复验：

- 300 个新帧全部写入 `/dev/ttyAMA0`，`write_fail=0`；
- 300 帧全部检出，除启动确认帧外均为有效状态；
- 平均拍摄到写入完成 26.51 ms，最大协议 `AGE=28 ms`；
- 静态起始位置约 `-1.40 cm`，发出的包通过本地 CRC 和字段编码检查。

随后持续发送测试覆盖了约 `-11～+8 cm` 的快速往返，速度达到约 `±23 cm/s`。
该测试期间 MCU 控制程序没有开启，运动属于测试输入，只能评价视觉检测、状态估计和
UART 发送，不能用于判断闭环方向、控制器增益或稳定性。

本轮可追溯结果文件：

- [`validation_summary_20260730.json`](output/validation_summary_20260730.json)：
  静态标定、阈值扫描、动态复测和 UART 安全结论汇总；
- [`dynamic_slow_20260730_report.json`](output/dynamic_slow_20260730_report.json)：
  `0.25` 阈值动态基线；
- [`dynamic_conf010_valid_20260730_report.json`](output/dynamic_conf010_valid_20260730_report.json)：
  `0.10` 阈值动态复测原始报告；
- [`camera_to_mcu_uart_conf010_20260730.json`](output/camera_to_mcu_uart_conf010_20260730.json)：
  300 帧真实 MCU UART 复验原始报告。

## 18. 坐标反号与 systemd 部署（2026-07-30）

MCU 的物理正方向与最初视觉标定方向相反。现在由 `config/uart.json` 显式配置：

```json
"position_sign": -1
```

反号发生在标定测量进入状态估计的位置，因此位置、速度、运行日志和 UART 协议字段
会一起反号；原始标定锚点和证据图片保持不变。缺少该字段的旧配置默认使用 `+1`，
只有 `-1` 和 `+1` 是合法值。

仓库中的 [`vision-scale-uart.service`](deploy/vision-scale-uart.service) 是目标
Raspberry Pi 5 的 systemd 单元，依赖 `ustreamer.service`，以用户 `nailong` 和
补充组 `dialout` 运行。标准管理命令为：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now vision-scale-uart.service
sudo systemctl disable --now vision-scale-uart.service
systemctl status vision-scale-uart.service
journalctl -u vision-scale-uart.service -f
```

部署时先完成了 60 帧 dry-run，确认原 `-11.4 cm` 方向变为 `+11.4 cm`；随后
180 帧真实 UART 测试全部检出并发送，`write_fail=0`，平均处理耗时 26.90 ms。
systemd 单元安装后曾达到 `enabled/active`、重启次数 0，并独占 `/dev/ttyAMA0`。

延时监控观察到位置从 `+10.22 cm` 跨到 `-9.41 cm`、速度最低约
`-34.97 cm/s`，并出现连续 `VALID=0` 外推帧。由于当时尚未明确 MCU 控制程序没有
开启，这段测试运动一度被误判为闭环异常并临时执行了 `disable --now`。澄清后服务已
重新 `enable --now`；最终状态为 `enabled/active`、重启次数 0，并由服务主进程独占
`/dev/ttyAMA0`。这些快速运动数据仍提示 MCU 将来必须严格遵守 `VALID` 标志，但不构成
当前控制稳定性结论。

原始和部署报告：

- [`position_sign_inverted_uart_20260730.json`](output/position_sign_inverted_uart_20260730.json)；
- [`systemd_deployment_20260730.json`](output/systemd_deployment_20260730.json)。

### Color-marker runtime calibration

The production calibration source is now the physical yellow/green/magenta ruler.
config/color_markers.json was fitted from the five 1280x200 rectified samples in
calibration_assets/color_ruler and is applied to the 640x100 BallDetector crop.
The runtime color search ROI is [20, 62, 600, 30]. Across the 949 extracted tilt
samples, classified marker pixels occupied x=67..597 and extended no lower than
y=86. The wider horizontal search therefore increases endpoint margin, while the
slightly tighter vertical search removes empty rows. Its 0.30 column fraction
preserves the previous absolute requirement of nine matching pixels per column.
Marker centers found inside that region fit the ruler axis. Up to one narrow internal marker may be reconstructed
from two reliable neighbours. A single wide internal marker is accepted only when
the marker-center spacing check still passes, while an anomalous outermost marker
is trimmed to produce a smaller valid calibration range. Zero, multiple-damage,
unreliable-neighbour, and bad-spacing cases remain invalid. Width is therefore a
secondary quality signal instead of a whole-frame veto. Detection runs every two processed frames,
retries on the next frame after rejection, and expires geometry after 120 ms.

The softer width policy was replayed over all 949 extracted 640x100 tilt samples:
949/949 produced valid geometry, 930 recovered the full -11..+11 cm range, and
19 safely returned a partial range. The maximum accepted spacing CV was 0.095,
below the unchanged 0.12 geometry limit. This replay is a same-environment
regression check and does not replace independent lighting and occlusion tests.

An optional oriented-strip pass is enabled by `oriented_roi_enabled`. The fixed
ROI remains the initializer and unconditional fallback. When the robust marker
axis reaches `oriented_angle_threshold_degrees` (currently 1 degree), all accepted
marker centres fit a rotated `640x30` strip. Nearest-neighbour affine sampling
straightens that strip before the normal colour sequence detector runs, then the
resulting origin, direction, and marker centres are mapped back into the original
640x100 coordinate space. A recent retained geometry may seed the next oriented
pass; stale geometry is never used. Failure to extract or validate the strip
immediately retries the fixed ROI on the same frame.

The enabled path preserved the fixed-ROI result on all 949 real samples (949
valid, 930 full range, 19 partial). Their fitted angles stayed below 0.56 degree,
so none paid the affine-remap cost. In a separate synthetic stress replay made by
rotating 100 real samples, the adaptive path retained 97 full-range detections at
both +3 and -3 degrees; the fixed ROI retained 9 and 0 respectively. These
synthetic rotations validate the fallback boundary but do not replace captured
large-angle footage.

The default runtime path is:

    camera -> BallDetector -> color-marker ScaleCalibration -> ScalePipeline -> UART

No UART packet field, serial setting, BallObservation contract, or detector output
coordinate has changed. If no current valid marker geometry exists, the ball
measurement is withheld and UART VALID is cleared; the fixed calibration is never
used silently. Use --calibration-source fixed to restore the previous fixed
calibration explicitly. Use --marker-shadow to validate marker geometry while the
fixed calibration remains active.

