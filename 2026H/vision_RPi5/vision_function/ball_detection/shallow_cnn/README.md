# 浅层 CNN 钢球定位实验

本目录实现固定 `640x100` 水管 ROI 内的单钢球存在性判断和一维横坐标定位。
它与生产 YOLO 并行开发，当前不会改变默认检测器。

## 模型输入输出

输入保持原生 ROI 分辨率，不做 resize 或 square letterbox：

```text
RGB float32 [N, 3, 100, 640]
```

模型输出：

- `presence_logits [N]`：是否存在钢球；
- `heatmap_logits [N, 160]`：横向位置热图，每格对应原 ROI 4 px；
- `offset_logits [N, 160]`：格点内连续偏移。

## 训练

默认直接读取 `data/actual_yolo_local_v3` 的 ROI 图片和 YOLO 标签，不依赖
`data.yaml`：

```powershell
python -m vision_function.ball_detection.shallow_cnn.train
```

快速检查数据、前向、反向和保存流程：

```powershell
python -m vision_function.ball_detection.shallow_cnn.train --epochs 1 --limit 128
```

使用新增倾斜录像样本池训练 v2（需要 CUDA 时显式指定设备）：

```powershell
conda run -n steelball python -m `
  vision_function.ball_detection.shallow_cnn.train `
  --config vision_function/ball_detection/config/shallow_cnn_v2.yaml `
  --device cuda
```

v2 将 `actual_yolo_local_v3/train` 与尚未切分的
`actual_video_frames_raw_new/all` 合并训练，验证仍只使用独立采集的
`actual_yolo_local_v3/val`。不要把同一段新录像的相邻帧随机拆入验证集。

从 v2 权重继续训练去重后的 v3：

```powershell
conda run -n steelball python -m `
  vision_function.ball_detection.shallow_cnn.train `
  --config vision_function/ball_detection/config/shallow_cnn_v3.yaml `
  --device cuda
```

v3 开启按图像内容 SHA-256 去重。`actual_video_frames_raw_new_v2` 已包含
`actual_video_frames_raw_new` 的全部 375 张样本，因此配置只引用 v2 目录；加载器
还会去除旧训练集中的历史重复样本，并在同图标签冲突时直接终止训练。

训练产物默认写入：

```text
vision_function/ball_detection/model/shallow_cnn_v1.pt
vision_function/ball_detection/output/shallow_cnn_v1/training/
```

## 离线评估

```powershell
python -m vision_function.ball_detection.shallow_cnn.evaluate `
  --weights vision_function/ball_detection/model/shallow_cnn_v1.pt `
  --data vision_function/ball_detection/data/actual_yolo_local_v3 `
  --split val `
  --report vision_function/ball_detection/output/shallow_cnn_v1/validation.json
```

核心指标是正样本检出率、空管误报率和横坐标误差，而不是二维检测 mAP。

## NCNN 导出与一致性检查

导出保持原生 `640x100` 输入，不做 resize；`fp16=0` 保持 FP32 权重：

```powershell
conda run -n steelball python -m `
  vision_function.ball_detection.shallow_cnn.export_ncnn `
  --pnnx C:\Users\Alan\.conda\envs\steelball\Scripts\pnnx.exe
```

生成的 `model.ncnn.param` 和 `model.ncnn.bin` 位于
`model/shallow_cnn_v1_ncnn_model/`。转换后应运行逐图一致性检查：

```powershell
conda run -n steelball python -m `
  vision_function.ball_detection.shallow_cnn.evaluate_ncnn `
  --report vision_function/ball_detection/output/shallow_cnn_v1/ncnn_validation.json
```

运行时默认显式启用 NCNN 的 FP16 packed/storage/arithmetic 以适配 ARM CPU，但
模型权重文件仍为 FP32。可传入 `--precision fp32` 关闭所有 FP16 运行选项。

## 本机速度基准

推荐使用现有 `steelball` Conda 环境，它已验证能识别 RTX 4060：

```powershell
conda run -n steelball python -m `
  vision_function.ball_detection.shallow_cnn.benchmark_local `
  --device cuda

conda run -n steelball python -m `
  vision_function.ball_detection.shallow_cnn.benchmark_local `
  --device cpu --threads 2
```

该基准分别记录纯模型推理，以及 ROI 转张量、推理和坐标解码的完整调用时间。
本机 GPU 结果不能代替 Raspberry Pi 5 NCNN 基准。

## 当前边界

- 已完成 PyTorch 训练、评估、NCNN 导出和 Raspberry Pi 5 独立测速；
- NCNN 运行时尚未替换生产 YOLO 检测器；
- 当前验证负样本很少，不能仅凭现有验证集决定替换生产 YOLO。

## v3 去重微调结果（2026-08-01）

训练前逐图检查结果：两批新数据之间有 375 张内容完全相同的图像，旧训练集内部
另有 190 张历史重复图；所有重复图的标签一致。去重后使用 1593 张唯一训练图
（1345 张有球、248 张空管），与 387 张独立验证图的哈希交集为 0。

从 `shallow_cnn_v2.pt` 初始化，使用 RTX 4060 Laptop GPU、`lr=3e-4`、
batch 32 微调 20 epoch。按误报、漏检、验证 P95 选择的最佳权重来自第 4 epoch：

| 数据 | 模型 | 正样本检出 | 负样本误报 | 横坐标平均误差 | 横坐标 P95 |
|---|---|---:|---:|---:|---:|
| 独立旧验证集 | v3 | 383/383 | 0/4 | 0.99 px | 3.49 px |
| 完整新样本并集 | v2 | 710/710 | 0/239 | 2.34 px | 10.67 px |
| 完整新样本并集 | v3 | 710/710 | 0/239 | 1.73 px | 5.60 px |

新增的 `有球的视频1.webm` 上，P95 从 11.02 px 降至 5.71 px；旧的
`带球的视频.webm` 上则从 4.22 px 变为 5.33 px。完整新样本属于训练数据，相关
数字用于检查拟合和回归，不代表独立泛化准确率。权重为
`model/shallow_cnn_v3.pt`，报告位于 `output/shallow_cnn_v3/`。

v3 导出 FP32 NCNN 权重后在 Raspberry Pi 5 上使用 2 个 NCNN 线程测试。生产
`ustreamer` 和 `vision-scale-uart` 服务均保持运行：静态 500 次完整调用 P95
为 6.18 ms；130 个不同摄像头帧的取帧、半尺寸 JPEG 解码、局部校正、推理和
坐标解码总 P95 为 14.62 ms，相当于 68.4 fps。两项测试均无降频，摄像头测试
130/130 帧检出。原始报告见 `output/shallow_cnn_v3/rpi_benchmark/`。

随后经授权临时停止 `vision-scale-uart`、保留摄像头 `ustreamer`，完成独占测速并
自动恢复服务。静态纯推理/完整调用 P95 分别为 2.39/3.08 ms；真实摄像头链路
纯推理/控制总 P95 分别为 2.63/9.41 ms，相当于 106.2 fps。报告见
`output/shallow_cnn_v3/rpi_benchmark_standalone/`。

## v2 GPU 训练结果（2026-08-01）

使用 RTX 4060 Laptop GPU 从头训练 40 epoch，训练集 1209 张（其中 189 张
负样本），独立验证集 387 张（其中 4 张负样本）。按误报、漏检、横坐标 P95
依次选出的最佳权重来自第 27 epoch：

| 数据 | 正样本检出 | 负样本误报 | 横坐标平均误差 | 横坐标 P95 |
|---|---:|---:|---:|---:|
| 独立旧验证集 | 383/383 | 0/4 | 0.99 px | 3.48 px |
| 新训练样本池 | 213/213 | 0/162 | 1.48 px | 4.22 px |

新样本池是训练数据，后一行只能确认拟合与回归修复，不能作为独立泛化指标。
权重为 `model/shallow_cnn_v2.pt`，评估报告位于
`output/shallow_cnn_v2/`。

## v1 本机基线（2026-07-30）

模型为 20,411 个参数，最佳权重来自第 13 epoch。现有 387 张验证图结果：

| 指标 | 结果 |
|---|---:|
| 正样本检出 | 383/383 |
| 负样本误报 | 0/4 |
| 横坐标平均误差 | 1.58 px |
| 横坐标 P95 | 4.34 px |
| 横坐标 P99 | 6.03 px |
| 横坐标最大误差 | 7.49 px |

本机 300 次、batch=1 基准：

| 环境 | 纯推理 P95 | ROI 转张量、推理和解码 P95 |
|---|---:|---:|
| RTX 4060 CUDA | 3.00 ms | 3.03 ms |
| 本机 CPU 2 线程 | 4.00 ms | 4.37 ms |

原始报告位于 `output/shallow_cnn_v1/`。这些结果证明实现闭环成立，但验证集只有
4 张负样本，因此当前默认检测器仍保持 YOLO。

## NCNN 与 Raspberry Pi 5 结果（2026-07-31）

NCNN 在全部 387 张验证图上与 PyTorch 的检出结论完全一致，二者横坐标
最大差值为 `0.000029 px`。NCNN 验证结果仍为 383/383 个正样本检出、0/4 个
负样本误报、平均误差 1.58 px、P95 误差 4.29 px。报告见
[`ncnn_validation.json`](../output/shallow_cnn_v1/ncnn_validation.json)。

同一验证集上的当前 YOLO 也是 383/383 检出、0/4 误报，其二维中心误差平均
1.00 px、P95 3.86 px。浅层 CNN 的指标为横向误差，口径并不完全相同，但现有
结果表明它的定位精度略低于 YOLO，而检出结论暂时一致。

树莓派为 Pi 5、Python 3.13.5、OpenCV 5.0.0、NCNN 1.0.20260526，使用 FP32
权重和 NCNN 默认启用的 FP16 中间存储与算术。测试时
`ustreamer` 和当前 `vision-scale-uart` 生产服务均保持运行，结果包含现有生产
负载；输入直接使用校正后的原生 `640x100` ROI，不缩放。

500 次静态 ROI 结果：

| NCNN 线程 | 纯推理 P95 | ROI 转换、推理和解码 P95 |
|---:|---:|---:|
| 1 | 6.38 ms | 7.38 ms |
| **2** | **5.35 ms** | **6.65 ms** |
| 4 | 50.83 ms | 52.47 ms |

4 线程会与现有 2 线程 YOLO 生产进程争抢 Pi 5 的 4 个核心，因此选择 2 线程。
原始报告见 [`rpi_benchmark/`](../output/shallow_cnn_v1/rpi_benchmark/)。

2 线程完整实时链路连续运行三轮，每轮测量 130 个不同的 `/snapshot` 帧：

| 指标 | 三轮 P95 | 中位轮 P95 | 当前 YOLO P95 |
|---|---:|---:|---:|
| 神经网络推理 | 4.71～7.61 ms | 5.60 ms | 18.49 ms |
| 获取帧至坐标输出 | 12.75～18.03 ms | 13.68 ms | 26.80 ms |
| 帧时间戳至坐标输出 | 12.66～17.53 ms | 13.35 ms | 26.81 ms |
| 等效控制帧率 | 55.5～78.4 fps | 73.1 fps | 37.3 fps |

三轮共 390/390 帧检出钢球，结束温度 58.2～59.8°C，均为
`throttled=0x0`。中位轮相对当前 YOLO 将完整链路 P95 降低约 49%，但实时画面
只有一个稳定钢球，不能代替带标注的泛化准确率测试。

## 已排除的首轮结构

- 将横向特征从 160 格提高到 320 格后，验证 P95 和边缘最大误差均变差；
- 增加 soft-argmax 连续坐标辅助损失后，验证 P95 也没有改善；
- v1 因此固定使用 160 格热图和最高格点内的连续偏移。
