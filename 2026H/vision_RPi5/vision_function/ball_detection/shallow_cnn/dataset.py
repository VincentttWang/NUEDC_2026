"""Dataset loader for 640x100 ROI images with YOLO center labels."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

from .decode import encode_position_target


IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg"}


@dataclass(frozen=True)
class RoiSample:
    image_path: Path
    label_path: Path
    x_local: float | None

    @property
    def present(self) -> bool:
        return self.x_local is not None


def read_yolo_center(label_path: Path, image_width: int) -> float | None:
    text = label_path.read_text(encoding="utf-8").strip()
    if not text:
        return None
    lines = text.splitlines()
    if len(lines) != 1:
        raise ValueError(f"Expected at most one ball in {label_path}, got {len(lines)}")
    values = lines[0].split()
    if len(values) != 5:
        raise ValueError(f"Invalid YOLO label in {label_path}: {lines[0]}")
    class_id, center_x, _center_y, _width, _height = map(float, values)
    if int(class_id) != 0:
        raise ValueError(f"Unexpected class {class_id} in {label_path}")
    if not 0.0 <= center_x <= 1.0:
        raise ValueError(f"Normalized x is outside [0, 1] in {label_path}")
    return min(float(image_width) - 1e-4, center_x * image_width)


def discover_samples(
    dataset_root: Path,
    split: str,
    *,
    image_width: int = 640,
) -> list[RoiSample]:
    image_dir = dataset_root / "images" / split
    label_dir = dataset_root / "labels" / split
    if not image_dir.is_dir() or not label_dir.is_dir():
        raise FileNotFoundError(
            f"Expected images/{split} and labels/{split} below {dataset_root}"
        )
    image_paths = sorted(
        path for path in image_dir.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES
    )
    if not image_paths:
        raise RuntimeError(f"No images found in {image_dir}")

    samples: list[RoiSample] = []
    for image_path in image_paths:
        label_path = label_dir / f"{image_path.stem}.txt"
        if not label_path.is_file():
            raise FileNotFoundError(f"Missing label for {image_path}: {label_path}")
        samples.append(
            RoiSample(
                image_path=image_path,
                label_path=label_path,
                x_local=read_yolo_center(label_path, image_width),
            )
        )
    return samples


class YoloRoiDataset(Dataset[dict[str, object]]):
    def __init__(
        self,
        dataset_root: Path,
        split: str,
        *,
        augment: bool = False,
        seed: int = 2026,
        limit: int | None = None,
        input_size: tuple[int, int] = (640, 100),
        output_bins: int = 160,
        sigma_bins: float = 1.5,
    ) -> None:
        self.input_width, self.input_height = input_size
        self.output_bins = output_bins
        self.sigma_bins = sigma_bins
        self.augment = augment
        self.rng = np.random.default_rng(seed)
        samples = discover_samples(
            Path(dataset_root), split, image_width=self.input_width
        )
        if limit is not None and limit < len(samples):
            indices = self.rng.permutation(len(samples))[:limit]
            samples = [samples[int(index)] for index in sorted(indices)]
        self.samples = samples
        self.presences = [sample.present for sample in samples]

    def __len__(self) -> int:
        return len(self.samples)

    def _augment_image(self, rgb: np.ndarray) -> np.ndarray:
        image = rgb.astype(np.float32)
        if self.rng.random() < 0.8:
            contrast = self.rng.uniform(0.85, 1.15)
            brightness = self.rng.uniform(-18.0, 18.0)
            image = image * contrast + brightness
        if self.rng.random() < 0.25:
            gamma = self.rng.uniform(0.8, 1.2)
            image = 255.0 * np.power(np.clip(image, 0, 255) / 255.0, gamma)
        if self.rng.random() < 0.2:
            kernel_size = int(self.rng.choice((3, 5)))
            kernel = np.zeros((kernel_size, kernel_size), dtype=np.float32)
            kernel[kernel_size // 2, :] = 1.0 / kernel_size
            image = cv2.filter2D(image, -1, kernel)
        if self.rng.random() < 0.25:
            image += self.rng.normal(0.0, self.rng.uniform(1.0, 5.0), image.shape)
        if self.rng.random() < 0.25:
            shift_y = int(self.rng.integers(-3, 4))
            transform = np.float32([[1, 0, 0], [0, 1, shift_y]])
            image = cv2.warpAffine(
                image,
                transform,
                (self.input_width, self.input_height),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )
        return np.clip(image, 0, 255).astype(np.uint8)

    def __getitem__(self, index: int) -> dict[str, object]:
        sample = self.samples[index]
        bgr = cv2.imread(str(sample.image_path), cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError(f"Cannot read image: {sample.image_path}")
        if bgr.shape != (self.input_height, self.input_width, 3):
            raise ValueError(
                f"Expected {(self.input_height, self.input_width, 3)}, "
                f"got {bgr.shape} for {sample.image_path}"
            )
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if self.augment:
            rgb = self._augment_image(rgb)
        image = torch.from_numpy(np.ascontiguousarray(rgb.transpose(2, 0, 1)))
        image = image.to(torch.float32).div_(255.0)

        presence = float(sample.present)
        if sample.x_local is None:
            heatmap = torch.zeros(self.output_bins, dtype=torch.float32)
            offset_index = 0
            offset = 0.0
            x_local = -1.0
        else:
            heatmap, offset_index, offset = encode_position_target(
                sample.x_local,
                input_width=self.input_width,
                output_bins=self.output_bins,
                sigma_bins=self.sigma_bins,
            )
            x_local = sample.x_local
        return {
            "image": image,
            "presence": torch.tensor(presence, dtype=torch.float32),
            "heatmap": heatmap,
            "offset_index": torch.tensor(offset_index, dtype=torch.long),
            "offset": torch.tensor(offset, dtype=torch.float32),
            "x_local": torch.tensor(x_local, dtype=torch.float32),
            "path": str(sample.image_path),
        }


class CombinedYoloRoiDataset(Dataset[dict[str, object]]):
    """Combine independently split ROI datasets without copying their files."""

    def __init__(
        self,
        datasets: Sequence[YoloRoiDataset],
        *,
        deduplicate_by_content: bool = False,
    ) -> None:
        if not datasets:
            raise ValueError("At least one dataset is required")
        self.datasets = list(datasets)
        self.index_map: list[tuple[int, int]] = []
        seen: dict[str, RoiSample] = {}
        for dataset_index, dataset in enumerate(self.datasets):
            for sample_index, sample in enumerate(dataset.samples):
                if deduplicate_by_content:
                    digest = hashlib.sha256(sample.image_path.read_bytes()).hexdigest()
                    previous = seen.get(digest)
                    if previous is not None:
                        if previous.x_local != sample.x_local:
                            raise ValueError(
                                "Duplicate image has conflicting labels: "
                                f"{previous.label_path} and {sample.label_path}"
                            )
                        continue
                    seen[digest] = sample
                self.index_map.append((dataset_index, sample_index))
        self.presences = [
            self.datasets[dataset_index].presences[sample_index]
            for dataset_index, sample_index in self.index_map
        ]

    def __len__(self) -> int:
        return len(self.index_map)

    def __getitem__(self, index: int) -> dict[str, object]:
        if index < 0:
            index += len(self)
        if not 0 <= index < len(self):
            raise IndexError(index)
        dataset_index, sample_index = self.index_map[index]
        return self.datasets[dataset_index][sample_index]
