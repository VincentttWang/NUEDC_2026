"""Fine-tune the existing YOLO11n detector on the final vehicle dataset."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL = ROOT / "model" / "actual_yolo11n_v2.pt"
DEFAULT_DATA = ROOT / "data" / "actual_yolo_local_v3" / "data.yaml"
DEFAULT_PROJECT = ROOT / "output" / "train_actual"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--project", type=Path, default=DEFAULT_PROJECT)
    parser.add_argument("--name", default="yolo11n_finetune_local_v3")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--freeze", type=int, default=10)
    parser.add_argument("--lr0", type=float, default=0.0001)
    args = parser.parse_args()

    model = YOLO(str(args.model))
    model.train(
        data=str(args.data),
        epochs=args.epochs,
        patience=15,
        batch=args.batch,
        imgsz=args.imgsz,
        device=args.device,
        workers=args.workers,
        project=str(args.project.resolve()),
        name=args.name,
        exist_ok=True,
        pretrained=True,
        freeze=args.freeze,
        optimizer="AdamW",
        lr0=args.lr0,
        lrf=0.01,
        cos_lr=True,
        weight_decay=0.0005,
        warmup_epochs=3.0,
        rect=True,
        cache="ram",
        seed=2026,
        deterministic=True,
        amp=False,
        degrees=2.0,
        translate=0.02,
        scale=0.05,
        shear=0.0,
        perspective=0.0,
        flipud=0.0,
        fliplr=0.0,
        mosaic=0.0,
        mixup=0.0,
        copy_paste=0.0,
        hsv_h=0.01,
        hsv_s=0.2,
        hsv_v=0.15,
        plots=True,
        verbose=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
