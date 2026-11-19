"""Benchmark shallow-CNN batch-1 inference on the local CPU or GPU."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import perf_counter
from typing import Callable

import cv2
import numpy as np
import torch

from .decode import decode_predictions
from .detector import ShallowCNNBallDetector
from .evaluate import DEFAULT_DATA, DEFAULT_WEIGHTS, load_checkpoint
from .model import parameter_count


def summarize(values_ms: list[float]) -> dict[str, float | int]:
    values = np.asarray(values_ms, dtype=np.float64)
    return {
        "count": len(values_ms),
        "mean": round(float(values.mean()), 4),
        "p50": round(float(np.percentile(values, 50)), 4),
        "p95": round(float(np.percentile(values, 95)), 4),
        "p99": round(float(np.percentile(values, 99)), 4),
        "maximum": round(float(values.max()), 4),
    }


def synchronize(device: torch.device) -> None:
    if device.type == "cuda":
        torch.cuda.synchronize(device)


def measure(
    operation: Callable[[], object],
    *,
    device: torch.device,
    warmup: int,
    iterations: int,
) -> list[float]:
    for _ in range(warmup):
        operation()
    synchronize(device)
    timings: list[float] = []
    for _ in range(iterations):
        started = perf_counter()
        operation()
        synchronize(device)
        timings.append((perf_counter() - started) * 1000.0)
    return timings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--image", type=Path)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    if args.iterations < 1 or args.warmup < 0:
        raise SystemExit("iterations must be positive and warmup non-negative")
    device = torch.device(args.device)
    if device.type == "cpu":
        torch.set_num_threads(args.threads)
    model, checkpoint = load_checkpoint(args.weights, device)
    model.eval()

    image_path = args.image
    if image_path is None:
        candidates = sorted((DEFAULT_DATA / "images" / "val").glob("*.png"))
        if not candidates:
            raise SystemExit(f"No validation PNG images below {DEFAULT_DATA}")
        image_path = candidates[0]
    bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise SystemExit(f"Cannot read image: {image_path}")
    tensor = ShallowCNNBallDetector.roi_to_tensor(bgr).to(device)

    @torch.inference_mode()
    def inference_only() -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        return model(tensor)

    @torch.inference_mode()
    def complete_call() -> object:
        input_tensor = ShallowCNNBallDetector.roi_to_tensor(bgr).to(device)
        return decode_predictions(*model(input_tensor))

    inference_ms = measure(
        inference_only,
        device=device,
        warmup=args.warmup,
        iterations=args.iterations,
    )
    complete_ms = measure(
        complete_call,
        device=device,
        warmup=args.warmup,
        iterations=args.iterations,
    )
    report = {
        "weights": str(args.weights),
        "checkpoint_epoch": checkpoint.get("epoch"),
        "image": str(image_path),
        "device": str(device),
        "device_name": (
            torch.cuda.get_device_name(device) if device.type == "cuda" else "CPU"
        ),
        "torch": torch.__version__,
        "threads": torch.get_num_threads() if device.type == "cpu" else None,
        "parameters": parameter_count(model),
        "warmup": args.warmup,
        "timing_ms": {
            "inference_only": summarize(inference_ms),
            "roi_tensor_model_decode": summarize(complete_ms),
        },
    }
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
