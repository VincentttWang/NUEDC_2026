# 钢球识别性能优化记录

本文记录树莓派 5 钢球识别链路的性能基线、已完成优化和后续优化顺序。
所有稳态测试均使用 `batch=1`，统计 100 帧或以上，并以 P95 作为主要指标。

## 水管局部校正迁移（2026-07-30）

边界样本确认旧的 570×80 ROI 在物理极限位置余量不足。生产预处理已改为：

```text
1280×720 原始硬件 JPEG
→ 1/2 IDCT 解码为 640×360
→ balance=0.2 无限延伸校正平面的 origin=(40,126)
→ 直接 remap 为 640×100 水管局部图
→ YOLO letterbox 到 320×320
```

参考坐标下区域为 `(left=80, top=252, width=1280, height=200)`，右边界
`x=1360` 可以超过名义 1280×720 校正画布。该方式保持 v2 的焦距和球像素尺寸，
不生成完整去畸变图。

旧 v2 NCNN 在新边界样本上的初测结果为：左极限 2/5、中心 3/3、右极限
2/5、空管 0/3 误检。加入极限样本及 50 张当前无手部背景的实时 JPEG 后，最终
v3 NCNN 在默认 `conf=0.25` 下达到左极限 5/5、中心 3/3、右极限 5/5、当前
实时 JPEG 50/50，全部 13 张原始空管图无误报。

## v3 最终实机流程（2026-07-30）

最终模型和局部 remap 已部署到 Raspberry Pi 5。使用当前右侧钢球进行 10 帧
预热和 130 帧稳态测试，达到 130/130 帧检出、总候选 130，未出现跨帧漏检或
多候选。生产阈值为 `conf=0.25`，NCNN 固定 2 线程。

| 环节 | 平均用时 | P95 |
|---|---:|---:|
| `/snapshot` 获取最新 JPEG | 0.624 ms | 0.704 ms |
| JPEG 1/2 IDCT 解码 | 4.376 ms | 4.447 ms |
| 640×100 水管局部 remap | 1.234 ms | 1.294 ms |
| YOLO 整体调用 | 20.250 ms | 20.568 ms |
| 其中 NCNN 推理 | 18.208 ms | 18.488 ms |
| **请求取帧到识别完成** | **26.499 ms** | **26.804 ms** |
| **帧时间戳到识别完成** | **26.307 ms** | **26.814 ms** |

控制链路等效约 37.3 fps。测试温度从 57.6°C 上升至 60.9°C，
`throttled=0x0`。诊断 JPEG 编码不属于控制流程；若启用，其 P95 为 0.595 ms。
完整实机原始报告见
[`camera_preprocess_20260730T025504+0800.json`](output/rpi_benchmark_local_v3_final/camera_preprocess_20260730T025504+0800.json)。

## v2 实车流程（2026-07-30 迁移前树莓派基线）

2026-07-30 最终机械结构数据已经完成第二轮动态视频 fine-tune，准确率、数据限制和
复现步骤见 [`ACTUAL_DATA.md`](ACTUAL_DATA.md)。新流程使用：

```text
1280×720 硬件 MJPEG
→ /snapshot 获取服务器最新完成帧
→ JPEG 1/2 IDCT 缩放解码为 640×360 BGR
→ balance=0.2 鱼眼校正
→ 固定 ROI (60, 140, 630, 220)，大小 570×80
→ actual_yolo11n_v2 NCNN FP32，imgsz=320
→ 只选择最高置信度候选
→ 球心恢复到校正后 1280×720 坐标
```

本地 NCNN 已通过 21/21 有球图、10/10 空管图和 407/407 动态验证图检查。
模型与链路已同步到 Raspberry Pi 5，使用正在运行的 1280×720 硬件 MJPEG
摄像头、`/snapshot`、JPEG 1/2 IDCT 解码和 570×80 ROI 完成 130 帧稳态测试。

原始报告：

- [`NCNN 2 线程首轮`](output/rpi_benchmark_v2/camera_preprocess_20260730T000635+0800.json)；
- [`NCNN 4 线程`](output/rpi_benchmark_v2/camera_preprocess_20260730T000652+0800.json)；
- [`NCNN 2 线程热机复测`](output/rpi_benchmark_v2/camera_preprocess_20260730T000713+0800.json)。

热机 2 线程分项结果：

| 环节 | 平均用时 | P95 |
|---|---:|---:|
| `/snapshot` 获取最新 JPEG | 0.64 ms | 0.75 ms |
| JPEG 1/2 IDCT 解码 | 4.37 ms | 4.47 ms |
| balance=0.2、570×80 ROI remap | 0.87 ms | 0.92 ms |
| YOLO 整体调用 | 20.83 ms | 21.72 ms |
| 其中 NCNN 推理 | 18.94 ms | 19.67 ms |
| **请求取帧到识别完成** | **26.72 ms** | **27.78 ms** |
| **帧时间戳到识别完成** | **26.50 ms** | **27.52 ms** |

两轮 2 线程控制 P95 分别为 27.24 ms 和 27.78 ms，热机复测等效约 36.0 fps。
测试末温度 70.3°C，`throttled=0x0`。4 线程虽然最低单帧更快，但核竞争造成
控制 P95 增至 37.23 ms、最大值 120.23 ms，因此当前固定使用 NCNN 2 线程。

下文的 25.08 ms 属于 2026-07-29 旧 ROI/旧模型基线。v2 P95 增加约 2.7 ms，
主要来自 ROI 从 425×75 增至 570×80、JPEG 解码和模型调用开销增加。

## 已部署性能基线（2026-07-29）

```text
1280×720 硬件 MJPEG
→ /snapshot 获取服务器最新完成帧
→ JPEG 1/2 IDCT 缩放解码为 640×360 BGR
→ 半尺寸固定 ROI 鱼眼 remap
→ YOLO11n NCNN FP32，imgsz=320，NCNN 2 线程
→ 提取钢球中心并乘 2，恢复到校正后 1280×720 坐标
```

固定 ROI 使用校正后 1280×720 坐标：

```text
xyxy = (300, 270, 1150, 420)
size = 850×150
```

实际解码和 remap 使用半尺寸坐标：

```text
xyxy = (150, 135, 575, 210)
size = 425×75
```

## 已部署基线性能

来源：[`半尺寸 snapshot`](output/rpi_benchmark/camera_preprocess_20260729T194911+0800.json)。
同一轮完整分辨率基线见
[`完整尺寸 snapshot`](output/rpi_benchmark/camera_preprocess_20260729T194855+0800.json)。

| 环节 | 平均用时 | P95 | 平均占比 |
|---|---:|---:|---:|
| `/snapshot` 获取最新硬件 MJPEG 帧 | 0.59 ms | 0.66 ms | 2.4% |
| JPEG 1/2 IDCT 解码 | 3.57 ms | 3.63 ms | 14.5% |
| 半尺寸固定 ROI 鱼眼 remap | 0.63 ms | 0.70 ms | 2.6% |
| ROI 复制 | 0 ms | 0 ms | 0% |
| YOLO 整体调用 | 19.89 ms | 20.17 ms | 80.6% |
| 其中：神经网络推理 | 18.13 ms | 18.40 ms | 73.4% |
| **请求取帧到识别完成** | **24.69 ms** | **25.08 ms** | **100%** |
| **帧时间戳到识别完成** | **24.41 ms** | **25.08 ms** | — |

当前等效控制帧率约为 **39.88 fps**。与同一轮完整分辨率 snapshot 相比，
真实帧时间戳到识别完成的 P95 从 29.24 ms 降至 25.08 ms，降低 14.2%；
请求取帧到识别完成的等效帧率从 34.22 fps 提高到 39.88 fps。

一次性开销：

- NCNN 包装器初始化约 1.38 ms，权重在首次预测时延迟加载并由预热阶段吸收；
- 半尺寸 ROI 映射首次建立并完成首帧 remap 约 1.91 ms；
- 映射表后续直接从缓存复用。

生产控制链路不应包含 JPEG 重编码。若额外编码半尺寸校正 ROI，P95 约增加
0.38 ms。

## 已完成优化

### 浅层 CNN 一维定位与树莓派实测（2026-07-31）

状态：**已完成本机训练、NCNN 导出和 Raspberry Pi 5 独立测速，尚未替换生产
YOLO**。

实现位于 [`shallow_cnn/`](shallow_cnn/)，直接使用局部校正得到的原生
`640×100` 水管 ROI，不做外部 resize 或 square letterbox。模型共有 20,411
个参数，输入为 RGB float32 `[1,3,100,640]`，输出：

```text
presence_logits [1]
heatmap_logits [1,160]
offset_logits [1,160]
```

每个热图格对应原 ROI 的 4 px，再通过格内偏移恢复连续横坐标。导出的
`model.ncnn.bin` 为 80,044 bytes；当前 YOLO 的 NCNN bin 为 10,354,940
bytes，浅层模型权重约缩小 129 倍。

#### NCNN 数值与准确率检查

NCNN 在现有 387 张验证图上的结果为：

| 指标 | 浅层 CNN NCNN | 当前 YOLO NCNN |
|---|---:|---:|
| 正样本检出 | 383/383 | 383/383 |
| 负样本误报 | 0/4 | 0/4 |
| 定位误差平均值 | 1.58 px（横向） | 1.00 px（二维中心） |
| 定位误差 P95 | 4.29 px（横向） | 3.86 px（二维中心） |
| 定位误差最大值 | 7.49 px（横向） | 7.77 px（二维中心） |

两个模型的误差口径并不完全相同，但现有结果表明浅层 CNN 的定位精度略低于
YOLO，检出结论暂时一致。NCNN 与 PyTorch 在全部验证图上没有检出分歧，横坐标
最大转换差仅 `0.000029 px`。原始报告见
[`ncnn_validation.json`](output/shallow_cnn_v1/ncnn_validation.json)。

当前仅有 4 张验证负样本，不能据此证明模型对实车反光、遮挡、空管和复杂背景
具有足够鲁棒性。

#### Raspberry Pi 5 纯推理

环境为 aarch64 Pi 5、Python 3.13.5、OpenCV 5.0.0、NCNN 1.0.20260526。
模型文件保存 FP32 权重，测速使用 NCNN 默认开启的 FP16 packed/storage/
arithmetic。运行时已支持通过 `--precision fp32` 显式关闭这些选项，但本轮
树莓派结果均为默认高性能模式。

测试时 `ustreamer.service` 和现有 `vision-scale-uart.service` 均保持运行，
因此结果包含正在运行的生产 YOLO 负载。每种线程配置预热 30 次并测量 500 次：

| NCNN 线程 | 纯推理 P95 | BGR 转换、推理和坐标解码 P95 | 等效帧率 |
|---:|---:|---:|---:|
| 1 | 6.38 ms | 7.38 ms | 135.5 fps |
| **2** | **5.35 ms** | **6.65 ms** | **150.3 fps** |
| 4 | 50.83 ms | 52.47 ms | 19.1 fps |

4 线程会与现有 2 线程 YOLO 进程争抢 Pi 5 的四个核心，调度抖动远大于计算收益，
因此浅层 CNN 推荐固定使用 2 线程。原始报告：

- [`1 线程`](output/shallow_cnn_v1/rpi_benchmark/static_t1_20260731T162204+0800.json)；
- [`2 线程`](output/shallow_cnn_v1/rpi_benchmark/static_t2_20260731T162209+0800.json)；
- [`4 线程`](output/shallow_cnn_v1/rpi_benchmark/static_t4_20260731T162252+0800.json)。

#### Raspberry Pi 5 完整实时链路

使用 2 线程连续运行三轮，每轮预热 10 帧并测量 130 个不同的 `/snapshot` 帧。
流程保持为：

```text
/snapshot
→ JPEG 1/2 IDCT 解码为 640×360
→ 直接 remap 为 640×100 水管局部图
→ BGR 转 RGB/CHW
→ 浅层 CNN NCNN
→ 横坐标解码
```

| 指标 | 三轮 P95 范围 | 中位轮 P95 | 当前 YOLO P95 |
|---|---:|---:|---:|
| 神经网络推理 | 4.71～7.61 ms | 5.60 ms | 18.49 ms |
| 请求取帧到坐标输出 | 12.75～18.03 ms | 13.68 ms | 26.80 ms |
| 帧时间戳到坐标输出 | 12.66～17.53 ms | 13.35 ms | 26.81 ms |
| 等效控制帧率 | 55.5～78.4 fps | 73.1 fps | 37.3 fps |

三轮共 390/390 帧检出钢球。结束温度为 58.2～59.8°C，全部
`throttled=0x0`。中位轮相对当前 YOLO 将完整链路 P95 降低约 49%；即使使用
三轮最差 P95，仍降低约 33%。原始报告：

- [`首轮`](output/shallow_cnn_v1/rpi_benchmark/camera_t2_20260731T162316+0800.json)；
- [`第二轮`](output/shallow_cnn_v1/rpi_benchmark/camera_t2_20260731T162413+0800.json)；
- [`第三轮`](output/shallow_cnn_v1/rpi_benchmark/camera_t2_20260731T162415+0800.json)。

实时测试画面只有一个位置较稳定的钢球，390/390 检出只能验证运行链路，不能
代替带标注的泛化准确率测试。当前结论是：**浅层 CNN 的速度和资源占用足以进入
生产影子运行，但必须扩充独立负样本、边界、反光和高速运动数据后，才能决定是否
替换 YOLO。**

### NCNN FP32 推理后端

状态：**已完成并部署到树莓派**。

模型目录：[`model/best_ncnn_model`](model/best_ncnn_model)。运行依赖固定为
Ultralytics 8.4.110 和 NCNN 1.0.20260526。树莓派无网络时，将匹配
Python 3.13/aarch64 的 NCNN wheel 复制到设备后使用 `--no-index --no-deps`
安装。

准确性结果见
[`output/backend_comparison/report.json`](output/backend_comparison/report.json)：

- 13 张 ROI 图片中 PyTorch 与 NCNN 均为 13/13 检出；
- 球心平均偏差 0.38 px，最大偏差 1.59 px；
- 最大置信度绝对差 0.0824；
- 当前空摄像头场景 100/100 帧均无检出。

树莓派静态 ROI 基准：

原始报告：
[`PyTorch 2 线程`](output/rpi_benchmark/static_roi_320_20260729T185020+0800.json)、
[`NCNN 2 线程`](output/rpi_benchmark/static_roi_320_20260729T185248+0800.json)、
[`NCNN 4 线程`](output/rpi_benchmark/static_roi_320_20260729T185302+0800.json)、
[`FP16 2 线程`](output/rpi_benchmark/static_roi_320_20260729T191815+0800.json)、
[`FP16 4 线程`](output/rpi_benchmark/static_roi_320_20260729T191822+0800.json)。

| 后端 | 线程 | 推理 P95 | `predict()` P95 | P99 | 等效帧率 |
|---|---:|---:|---:|---:|---:|
| PyTorch | 2 | 26.06 ms | 27.49 ms | 28.19 ms | 36.38 fps |
| NCNN FP32 | 2 | 18.01 ms | 20.11 ms | 20.35 ms | 49.73 fps |
| NCNN FP32 | 4 | 15.60 ms | 17.78 ms | 18.90 ms | 56.23 fps |
| NCNN FP16 | 2 | 18.21 ms | 20.18 ms | 20.48 ms | 49.56 fps |
| NCNN FP16 | 4 | 17.02 ms | 19.09 ms | 30.39 ms | 52.39 fps |

实时摄像头完整链路：

原始报告：
[`PyTorch 2 线程`](output/rpi_benchmark/camera_preprocess_20260729T181949+0800.json)、
[`NCNN 2 线程`](output/rpi_benchmark/camera_preprocess_20260729T185328+0800.json)、
[`NCNN 4 线程`](output/rpi_benchmark/camera_preprocess_20260729T185321+0800.json)、
[`FP16 2 线程`](output/rpi_benchmark/camera_preprocess_20260729T191845+0800.json)、
[`FP16 4 线程`](output/rpi_benchmark/camera_preprocess_20260729T191852+0800.json)。

| 后端 | 线程 | YOLO P95 | 控制总计 P95 | 控制总计 P99 | 等效帧率 |
|---|---:|---:|---:|---:|---:|
| PyTorch | 2 | 27.41 ms | 36.21 ms | 37.11 ms | 27.62 fps |
| NCNN FP32 | 2 | 19.97 ms | 28.50 ms | 28.79 ms | 35.09 fps |
| NCNN FP32 | 4 | 20.43 ms | 28.82 ms | 35.73 ms | 34.70 fps |
| NCNN FP16 | 2 | 19.92 ms | 28.41 ms | 28.56 ms | 35.20 fps |
| NCNN FP16 | 4 | 19.18 ms | 27.99 ms | 34.31 ms | 35.73 fps |

顺序控制链路推荐 2 线程：虽然 4 线程的静态推理更快，但实时测试尾延迟更大；
2 线程 P99 更稳定，并为采集、控制和后续流水线保留两个 CPU 核心。

FP16 模型已导出到
[`model/best_fp16_ncnn_model`](model/best_fp16_ncnn_model)，大小约 5.1 MB。
本地准确性为 13/13，球心最大偏差 1.58 px，并已部署到树莓派完成测速。
FP16 在 2 线程下与 FP32 基本持平，4 线程下静态 P95 和 P99 反而更差；实时
结果的微小差异处于运行波动范围。因此 FP16 只保留为节省约 5 MB 存储的候选，
不替换数值精度更高、性能更稳定的 FP32 推荐模型。

复现导出时应在 x86_64 电脑上安装
[`requirements-export.txt`](requirements-export.txt)，然后执行：

```bash
yolo export model=model/best.pt format=ncnn imgsz=320
yolo export model=model/best.pt format=ncnn imgsz=320 quantize=16
```

树莓派基准命令示例：

```bash
python benchmark_rpi.py static --input roi \
  --model model/best_ncnn_model --imgsz 320 --ncnn-threads 2

python benchmark_rpi.py camera --remap-mode roi --with-yolo \
  --model model/best_ncnn_model --imgsz 320 --ncnn-threads 2
```

### 固定 ROI 直接 remap

状态：**已完成**，提交 `30e63d9`。

直接从原始帧生成校正后坐标系中的 850×150 ROI，结果与整帧校正后裁剪逐像素一致。

| 指标 | 整帧 remap | ROI remap | 改善 |
|---|---:|---:|---:|
| 首次建图并校正 | 53.70 ms | 7.33 ms | 降低 86.4% |
| 稳态 remap P95 | 16.59 ms | 2.28 ms | 7.27 倍 |
| 摄像头前处理 P95 | 22.96 ms | 8.97 ms | 降低 60.9% |
| 含 YOLO 完整链路 P95 | 51.20 ms | 36.21 ms | 降低 29.3% |
| 等效帧率 | 19.53 fps | 27.62 fps | 提高 41.4% |

### JPEG 1/2 缩放解码与采集方式

状态：**已完成实测，采用缩放解码并保留 `/snapshot`**。

OpenCV 5.0.0 已内置 libjpeg-turbo 3.1.2，不需要额外安装 JPEG 解码库。
`cv2.IMREAD_REDUCED_COLOR_2` 利用 JPEG IDCT 缩放直接输出 640×360，而不是先
完整解码 1280×720 再执行一次 resize。鱼眼参数和 ROI 坐标按 1/2 等比例缩放，
YOLO 输出坐标最后乘 2 恢复到 1280×720 校正坐标系。

13 张固定图片验证见
[`ncnn_decode_scale2_report.json`](output/backend_comparison/ncnn_decode_scale2_report.json)：

- 完整尺寸和半尺寸均为 13/13 检出；
- 半尺寸相对完整尺寸的球心平均偏差 0.48 px，最大偏差 1.29 px；
- 平均置信度从 0.6866 变为 0.6937。

树莓派 130 帧实时结果：

| 方案 | 解码 P95 | remap P95 | 请求到结果 P95 | 帧时间戳到结果 P95 |
|---|---:|---:|---:|---:|
| `/snapshot`，1280×720 解码 | 6.33 ms | 2.40 ms | 29.22 ms | 29.24 ms |
| `/snapshot`，640×360 缩放解码 | 3.63 ms | 0.70 ms | 25.08 ms | 25.08 ms |
| 持续 `/stream`，640×360 缩放解码 | 3.52 ms | 0.65 ms | 24.24 ms | **1350.30 ms** |

持续流的函数执行时间看似更低，是因为它立即从 TCP 缓冲区读出旧帧；在约
24 ms 的串行处理速度低于相机 60 fps 时，帧会不断积压。该测试中帧龄 P95
已经达到 1.33 s，最终端到端 P95 为 1.35 s。因此控制链路必须继续使用每次
返回服务器最新完成帧的 `/snapshot`。原始持续流报告见
[`camera_preprocess_20260729T194926+0800.json`](output/rpi_benchmark/camera_preprocess_20260729T194926+0800.json)。

基准程序同时记录 uStreamer 的 `X-Timestamp`。以后评价采集优化时，应以
`frame_timestamp_to_control_result` 为主，不能只看函数调用耗时。

## 不采用的方案

### 最新帧双线程流水线

状态：**不采用**。

双线程可以让前处理与 YOLO 重叠，提高单位时间输出数量，但不能缩短同一张图
从采集到处理完成的路径；线程调度和“最新帧”槽位还可能增加帧龄抖动。本任务
关注单帧端到端延迟，因此生产控制链路保持严格串行：

```text
最新 snapshot → 缩放解码 → ROI remap → YOLO → 输出位置
```

若将来需要同时提供网页预览，预览仍由独立的 uStreamer/浏览器链路完成，不能
把控制识别改成以吞吐率为目标的帧队列。

## 后续优化清单

### P1：测试更小输入尺寸

依次测试：

```text
imgsz = 288
imgsz = 256
```

每种尺寸必须检查：

- 13 张图片检出率；
- 最低置信度；
- 与 `imgsz=320` 的球心偏差；
- 暗光、运动模糊和管槽两端位置；
- 树莓派 P95 和温度。

未通过准确率要求时保持 `imgsz=320`。

### P2：YOLO 低频检测与轻量跟踪

方案：YOLO 每 2～3 帧重新检测一次，中间帧使用局部模板匹配、光流或一维搜索更新位置。置信度下降、运动突变或目标丢失时立即重新运行 YOLO。

目标：YOLO 维持 10～15 Hz，钢球位置更新达到 30～50 Hz。

### P2：浅层 CNN 影子运行与数据扩充

一维位置模型的实现、NCNN 转换和 Raspberry Pi 5 性能测试已经完成，结果见
“浅层 CNN 一维定位与树莓派实测”。下一阶段不是继续压缩网络，而是：

1. 采集更多实车空管、人物或机械结构进入 ROI、强反光和曝光变化负样本；
2. 增加左右物理极限、高速运动和运动模糊的独立测试集；
3. 在生产进程中让 CNN 与 YOLO 对同一 ROI 并行输出，但暂时只采用 YOLO 结果；
4. 记录两者的检出分歧、横坐标偏差、连续性和实际控制效果；
5. 通过独立数据门槛后，再评估将 CNN 切换为主检测器、YOLO 作为降级或复核路径。

### P3：生产链路清理

控制进程中禁止每帧执行：

- `cv2.imencode` / `cv2.imwrite`；
- 检测框和文字绘制；
- 终端日志打印；
- 完整视频保存。

调试数据应异步采样保存，网页预览继续使用 uStreamer 原始 MJPEG 和浏览器端鱼眼校正。

## 准确率与鲁棒性待办

当前空场景没有安装管槽，固定 ROI 会覆盖人物、纸箱和机械结构，模型可能产生误检。性能优化不能替代数据完善，后续必须：

1. 采集实车空管槽和深色底板负样本；
2. 增加不同曝光、阴影、反光和运动模糊数据；
3. 使用管槽轴线和钢球尺寸范围过滤 YOLO 输出；
4. 对连续帧使用速度、加速度和位置边界约束；
5. 最终安装后重新标定固定 ROI。

## 每次优化的统一测试要求

每项优化必须保留以下结果：

- 模型和后端版本；
- 输入分辨率、ROI、`imgsz` 和线程数；
- 预热次数不少于 10；
- 测试帧数不少于 100；
- 平均值、P50、P95、P99和最大值；
- 13 张固定图片的检出率和球心坐标；
- 实时空场景误检情况；
- 测试前后温度及 `vcgencmd get_throttled`；
- 与当前推荐流程的差异结论。
