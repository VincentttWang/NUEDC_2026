"""Train the shallow single-ball position network."""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path
from typing import Any

import numpy as np
import torch
import yaml
from torch.optim import AdamW
from torch.optim.lr_scheduler import CosineAnnealingLR
from torch.utils.data import DataLoader, WeightedRandomSampler

from .dataset import CombinedYoloRoiDataset, YoloRoiDataset
from .evaluate import evaluate_model
from .losses import ShallowBallLoss
from .model import ShallowBallNet, parameter_count


ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = ROOT.parents[2]
DEFAULT_CONFIG = ROOT.parent / "config" / "shallow_cnn_v1.yaml"


def resolve_repository_path(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def make_train_loader(
    dataset: CombinedYoloRoiDataset,
    *,
    batch_size: int,
    negative_fraction: float,
) -> DataLoader[dict[str, Any]]:
    positives = sum(dataset.presences)
    negatives = len(dataset) - positives
    if positives and negatives and 0.0 < negative_fraction < 1.0:
        positive_weight = (1.0 - negative_fraction) / positives
        negative_weight = negative_fraction / negatives
        weights = [positive_weight if present else negative_weight for present in dataset.presences]
        sampler = WeightedRandomSampler(weights, len(dataset), replacement=True)
        return DataLoader(dataset, batch_size=batch_size, sampler=sampler, num_workers=0)
    return DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=0)


def make_dataset(
    config: dict[str, Any],
    group: str,
    *,
    augment: bool,
    seed: int,
    limit: int | None,
) -> CombinedYoloRoiDataset:
    configured_sources = config.get("datasets", {}).get(group)
    if configured_sources is None:
        configured_sources = [
            {"root": config["dataset_root"], "split": group}
        ]
    datasets = []
    for source_index, source in enumerate(configured_sources):
        datasets.append(
            YoloRoiDataset(
                resolve_repository_path(source["root"]),
                str(source["split"]),
                augment=augment,
                seed=seed + source_index,
                limit=limit,
                output_bins=int(config["model"]["output_bins"]),
                sigma_bins=float(config["targets"]["sigma_bins"]),
            )
        )
    return CombinedYoloRoiDataset(
        datasets,
        deduplicate_by_content=bool(config.get("deduplicate_by_content", False)),
    )


def move_targets(batch: dict[str, Any], device: torch.device) -> dict[str, torch.Tensor]:
    return {
        key: batch[key].to(device)
        for key in ("presence", "heatmap", "offset_index", "offset")
    }


def validation_score(summary: dict[str, Any]) -> tuple[float, float, float]:
    false_positives = float(summary["negative_false_positive"])
    misses = float(summary["positive_total"] - summary["positive_detected"])
    p95 = float(summary["x_error_px_p95"] or 1e9)
    return false_positives, misses, p95


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--epochs", type=int)
    parser.add_argument("--batch-size", type=int)
    parser.add_argument("--device")
    parser.add_argument(
        "--initial-weights",
        type=Path,
        help="Initialize model parameters from an existing shallow-CNN checkpoint.",
    )
    parser.add_argument("--limit", type=int, help="Limit each split for a smoke test")
    args = parser.parse_args()

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    seed = int(config["seed"])
    seed_everything(seed)
    device_name = args.device or config["device"]
    if device_name == "auto":
        device_name = "cuda" if torch.cuda.is_available() else "cpu"
    device = torch.device(device_name)
    epochs = args.epochs or int(config["training"]["epochs"])
    batch_size = args.batch_size or int(config["training"]["batch_size"])
    weights_path = resolve_repository_path(config["weights_path"])
    output_dir = resolve_repository_path(config["output_dir"])
    output_dir.mkdir(parents=True, exist_ok=True)
    weights_path.parent.mkdir(parents=True, exist_ok=True)

    train_dataset = make_dataset(
        config,
        "train",
        augment=True,
        seed=seed,
        limit=args.limit,
    )
    val_dataset = make_dataset(
        config,
        "val",
        augment=False,
        seed=seed,
        limit=args.limit,
    )
    train_loader = make_train_loader(
        train_dataset,
        batch_size=batch_size,
        negative_fraction=float(config["training"]["negative_sample_fraction"]),
    )
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=0)

    channels = tuple(int(value) for value in config["model"]["channels"])
    output_bins = int(config["model"]["output_bins"])
    model = ShallowBallNet(channels, output_bins=output_bins).to(device)
    initial_weights_value = args.initial_weights or config.get("initial_weights_path")
    initial_weights_path = (
        None
        if initial_weights_value is None
        else resolve_repository_path(initial_weights_value)
    )
    if initial_weights_path is not None:
        initial_checkpoint = torch.load(
            initial_weights_path, map_location=device, weights_only=False
        )
        initial_channels = tuple(initial_checkpoint.get("model_channels", channels))
        initial_output_bins = int(
            initial_checkpoint.get("model_output_bins", output_bins)
        )
        if initial_channels != channels or initial_output_bins != output_bins:
            raise ValueError(
                "Initial checkpoint architecture does not match the configured model"
            )
        model.load_state_dict(initial_checkpoint["model"])
    criterion = ShallowBallLoss(**config["loss"])
    optimizer = AdamW(
        model.parameters(),
        lr=float(config["training"]["learning_rate"]),
        weight_decay=float(config["training"]["weight_decay"]),
    )
    scheduler = CosineAnnealingLR(optimizer, T_max=max(1, epochs))
    presence_threshold = float(config["thresholds"]["presence"])
    heatmap_threshold = float(config["thresholds"]["heatmap"])
    print(
        json.dumps(
            {
                "device": str(device),
                "parameters": parameter_count(model),
                "train_images": len(train_dataset),
                "train_negatives": len(train_dataset) - sum(train_dataset.presences),
                "val_images": len(val_dataset),
                "val_negatives": len(val_dataset) - sum(val_dataset.presences),
                "initial_weights": (
                    None if initial_weights_path is None else str(initial_weights_path)
                ),
            },
            ensure_ascii=False,
        )
    )

    history: list[dict[str, Any]] = []
    best_score: tuple[float, float, float] | None = None
    for epoch in range(1, epochs + 1):
        model.train()
        accumulated = {"total": 0.0, "presence": 0.0, "heatmap": 0.0, "offset": 0.0}
        batches = 0
        for batch in train_loader:
            images = batch["image"].to(device)
            targets = move_targets(batch, device)
            optimizer.zero_grad(set_to_none=True)
            losses = criterion(model(images), targets)
            losses.total.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
            optimizer.step()
            for name in accumulated:
                accumulated[name] += float(getattr(losses, name).detach().cpu())
            batches += 1
        scheduler.step()
        train_losses = {name: value / max(1, batches) for name, value in accumulated.items()}
        validation = evaluate_model(
            model,
            val_loader,
            device,
            presence_threshold=presence_threshold,
            heatmap_threshold=heatmap_threshold,
        )
        row = {
            "epoch": epoch,
            "learning_rate": optimizer.param_groups[0]["lr"],
            "train_loss": train_losses,
            "validation": validation["summary"],
        }
        history.append(row)
        print(json.dumps(row, ensure_ascii=False))

        checkpoint = {
            "schema_version": 1,
            "epoch": epoch,
            "model_channels": channels,
            "model_output_bins": output_bins,
            "model": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "config": config,
            "validation": validation["summary"],
        }
        torch.save(checkpoint, output_dir / "last.pt")
        score = validation_score(validation["summary"])
        if best_score is None or score < best_score:
            best_score = score
            torch.save(checkpoint, weights_path)
            torch.save(checkpoint, output_dir / "best.pt")

    report = {
        "config": str(args.config),
        "datasets": config.get(
            "datasets",
            {
                "train": config.get("dataset_root"),
                "val": config.get("dataset_root"),
            },
        ),
        "weights": str(weights_path),
        "initial_weights": (
            None if initial_weights_path is None else str(initial_weights_path)
        ),
        "device": str(device),
        "parameters": parameter_count(model),
        "history": history,
    }
    (output_dir / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
