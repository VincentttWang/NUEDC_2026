# K230 Target YOLO Prototype

This directory contains the prototype path for combining a lightweight YOLO
detector with the geometric target refinement in `k230/rect_target.py`.

## Current state

- `target_dataset/`: 80 automatically labelled 320x240 seed images.
- `runs/target_yolov8n_224_overfit/weights/best.pt`: proof-of-concept model.
- `k230/rect_target.py`: enables AI automatically when
  `/sdcard/target_yolov8n_224.kmodel` exists and otherwise uses CV only.

The seed images are near-duplicates from one scene. The resulting validation
metrics only prove that the training and runtime interfaces are wired
correctly. Do not treat the model as competition-ready until the dataset has
multiple distances, view angles, backgrounds and illumination conditions.

## Collect

Place a complete target in view and vary its distance and angle while running:

```powershell
python tools/k230_collect_yolo_dataset.py COM15
```

The collector only accepts complete, high-magnitude CV rectangles and writes
YOLO labels beside each RGB image.

## Train

```powershell
$env:YOLO_CONFIG_DIR="$PWD/ai/.ultralytics"
yolo detect train model=yolov8n.pt data="$PWD/ai/target_dataset/dataset.yaml" `
  epochs=100 imgsz=224 batch=16 workers=0 device=cpu `
  project="$PWD/ai/runs" name=target_yolov8n_224
```

Use pretrained weights for the real dataset. The checked prototype was trained
from `yolov8n.yaml` only because downloading weights was unavailable.

## Export and convert

Install the versions required by the active CanMV firmware, then export:

```powershell
yolo export model="ai/runs/target_yolov8n_224/weights/best.pt" `
  format=onnx imgsz=224 opset=11 simplify=False
```

Convert the ONNX model with the matching Canaan nncase and `nncase-kpu`
packages, using representative images from `target_dataset/images/train` for
PTQ calibration. Copy the result to:

The connected firmware reports nncase runtime 2.9.0. With the matching Python
3.10 toolchain installed under `ai/toolchain/python310`, run:

```powershell
$env:PATH="$PWD/ai/toolchain/python310/Lib/site-packages;$env:PATH"
ai/toolchain/python310/python.exe ai/convert_yolov8_k230.py `
  --model ai/runs/target_yolov8n_224_overfit/weights/best.onnx `
  --dataset ai/target_dataset/images/train `
  --output ai/target_yolov8n_224.kmodel
```

Deploy the generated model as:

```text
/sdcard/target_yolov8n_224.kmodel
```

On the next boot, `rect_target.py` prints `AI=1`, runs YOLO every four frames,
expands its box into a CV ROI, and periodically falls back to full-frame CV.
