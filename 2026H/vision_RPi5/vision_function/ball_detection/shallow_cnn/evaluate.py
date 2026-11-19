"""Evaluate shallow-CNN presence and horizontal position accuracy."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch.utils.data import DataLoader

from .dataset import YoloRoiDataset
from .decode import decode_batch
from .model import ShallowBallNet


ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = ROOT.parents[2]
DEFAULT_DATA = ROOT.parent / "data" / "actual_yolo_local_v3"
DEFAULT_WEIGHTS = ROOT.parent / "model" / "shallow_cnn_v1.pt"


def percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    return round(float(np.percentile(np.asarray(values), quantile)), 4)


@torch.inference_mode()
def evaluate_model(
    model: ShallowBallNet,
    loader: DataLoader[dict[str, Any]],
    device: torch.device,
    *,
    presence_threshold: float,
    heatmap_threshold: float,
) -> dict[str, Any]:
    model.eval()
    positive_total = positive_detected = 0
    negative_total = false_positives = 0
    errors: list[float] = []
    rows: list[dict[str, Any]] = []
    for batch in loader:
        images = batch["image"].to(device)
        presence_logits, heatmap_logits, offset_logits = model(images)
        presence, peaks, indices, positions = decode_batch(
            presence_logits, heatmap_logits, offset_logits
        )
        targets_present = batch["presence"].bool()
        targets_x = batch["x_local"]
        found = (presence >= presence_threshold) & (peaks >= heatmap_threshold)
        for path, target_present, target_x, detected, probability, peak, index, x in zip(
            batch["path"],
            targets_present,
            targets_x,
            found.cpu(),
            presence.cpu(),
            peaks.cpu(),
            indices.cpu(),
            positions.cpu(),
            strict=True,
        ):
            has_ball = bool(target_present.item())
            is_found = bool(detected.item())
            error = None
            if has_ball:
                positive_total += 1
                if is_found:
                    positive_detected += 1
                    error = abs(float(x.item()) - float(target_x.item()))
                    errors.append(error)
            else:
                negative_total += 1
                false_positives += int(is_found)
            rows.append(
                {
                    "image": Path(path).name,
                    "has_ground_truth": has_ball,
                    "found": is_found,
                    "presence_probability": round(float(probability.item()), 6),
                    "heatmap_peak": round(float(peak.item()), 6),
                    "bin_index": int(index.item()),
                    "x_local": round(float(x.item()), 4) if is_found else None,
                    "x_error_px": None if error is None else round(error, 4),
                }
            )
    summary = {
        "positive_detected": positive_detected,
        "positive_total": positive_total,
        "recall": (
            None if positive_total == 0 else round(positive_detected / positive_total, 6)
        ),
        "negative_false_positive": false_positives,
        "negative_total": negative_total,
        "false_positive_rate": (
            None if negative_total == 0 else round(false_positives / negative_total, 6)
        ),
        "x_error_px_mean": None if not errors else round(float(np.mean(errors)), 4),
        "x_error_px_p50": percentile(errors, 50),
        "x_error_px_p95": percentile(errors, 95),
        "x_error_px_p99": percentile(errors, 99),
        "x_error_px_max": None if not errors else round(max(errors), 4),
    }
    return {"summary": summary, "images": rows}


def load_checkpoint(path: Path, device: torch.device) -> tuple[ShallowBallNet, dict[str, Any]]:
    checkpoint = torch.load(path, map_location=device, weights_only=False)
    model = ShallowBallNet(
        tuple(checkpoint.get("model_channels", (16, 24, 32, 48))),
        output_bins=int(checkpoint.get("model_output_bins", 160)),
    )
    model.load_state_dict(checkpoint["model"])
    model.to(device)
    return model, checkpoint


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--split", default="val")
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--presence-threshold", type=float, default=0.5)
    parser.add_argument("--heatmap-threshold", type=float, default=0.25)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    device = torch.device(args.device)
    model, checkpoint = load_checkpoint(args.weights, device)
    dataset = YoloRoiDataset(args.data, args.split)
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=False, num_workers=0)
    report = evaluate_model(
        model,
        loader,
        device,
        presence_threshold=args.presence_threshold,
        heatmap_threshold=args.heatmap_threshold,
    )
    report.update(
        {
            "weights": str(args.weights),
            "data": str(args.data),
            "split": args.split,
            "epoch": checkpoint.get("epoch"),
            "presence_threshold": args.presence_threshold,
            "heatmap_threshold": args.heatmap_threshold,
        }
    )
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text, encoding="utf-8")
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
