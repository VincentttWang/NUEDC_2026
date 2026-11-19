"""Compare the exported NCNN model with PyTorch and ground truth."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import torch

from .dataset import YoloRoiDataset
from .decode import decode_batch
from .evaluate import DEFAULT_DATA, DEFAULT_WEIGHTS, load_checkpoint, percentile
from .ncnn_runtime import DEFAULT_MODEL, ShallowCNNNCNN


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--split", default="val")
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--precision", choices=("fp16", "fp32"), default="fp16")
    parser.add_argument("--presence-threshold", type=float, default=0.5)
    parser.add_argument("--heatmap-threshold", type=float, default=0.25)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    torch_model, checkpoint = load_checkpoint(args.weights, torch.device("cpu"))
    torch_model.eval()
    ncnn_model = ShallowCNNNCNN(
        args.model,
        threads=args.threads,
        use_fp16=args.precision == "fp16",
    )
    dataset = YoloRoiDataset(args.data, args.split)

    positive_total = positive_detected = 0
    negative_total = false_positives = 0
    errors: list[float] = []
    torch_ncnn_x_deltas: list[float] = []
    presence_deltas: list[float] = []
    peak_deltas: list[float] = []
    found_disagreements = 0

    with torch.inference_mode():
        for sample in dataset:
            path = Path(sample["path"])
            bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if bgr is None:
                raise RuntimeError(f"Cannot decode {path}")
            ncnn_prediction = ncnn_model.predict(
                bgr,
                presence_threshold=args.presence_threshold,
                heatmap_threshold=args.heatmap_threshold,
            )

            outputs = torch_model(sample["image"].unsqueeze(0))
            presence, peaks, _indices, positions = decode_batch(*outputs)
            torch_found = bool(
                presence[0] >= args.presence_threshold
                and peaks[0] >= args.heatmap_threshold
            )
            found_disagreements += int(torch_found != ncnn_prediction.found)
            presence_deltas.append(
                abs(float(presence[0]) - ncnn_prediction.presence_probability)
            )
            peak_deltas.append(abs(float(peaks[0]) - ncnn_prediction.heatmap_peak))
            if torch_found and ncnn_prediction.found:
                assert ncnn_prediction.x_local is not None
                torch_ncnn_x_deltas.append(
                    abs(float(positions[0]) - ncnn_prediction.x_local)
                )

            has_ball = bool(sample["presence"])
            if has_ball:
                positive_total += 1
                if ncnn_prediction.found:
                    positive_detected += 1
                    assert ncnn_prediction.x_local is not None
                    error = abs(ncnn_prediction.x_local - float(sample["x_local"]))
                    errors.append(error)
            else:
                negative_total += 1
                false_positives += int(ncnn_prediction.found)

    summary = {
        "samples": len(dataset),
        "checkpoint_epoch": checkpoint.get("epoch"),
        "positive_detected": positive_detected,
        "positive_total": positive_total,
        "negative_false_positive": false_positives,
        "negative_total": negative_total,
        "x_error_px_mean": round(float(np.mean(errors)), 4),
        "x_error_px_p95": percentile(errors, 95),
        "x_error_px_max": round(max(errors), 4),
        "torch_ncnn_found_disagreements": found_disagreements,
        "torch_ncnn_x_delta_px_max": round(max(torch_ncnn_x_deltas), 6),
        "torch_ncnn_presence_probability_delta_max": round(max(presence_deltas), 8),
        "torch_ncnn_heatmap_peak_delta_max": round(max(peak_deltas), 8),
        "ncnn_threads": args.threads,
        "ncnn_precision": args.precision,
    }
    text = json.dumps(summary, ensure_ascii=False, indent=2) + "\n"
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
