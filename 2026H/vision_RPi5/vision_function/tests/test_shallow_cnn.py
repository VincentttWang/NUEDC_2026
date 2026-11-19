from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
import torch

from vision_function.ball_detection.shallow_cnn.dataset import (
    CombinedYoloRoiDataset,
    YoloRoiDataset,
)
from vision_function.ball_detection.shallow_cnn.decode import (
    decode_batch,
    encode_position_target,
)
from vision_function.ball_detection.shallow_cnn.losses import ShallowBallLoss
from vision_function.ball_detection.shallow_cnn.model import (
    ShallowBallNet,
    parameter_count,
)


class ShallowCNNTest(unittest.TestCase):
    def test_position_target_round_trip(self) -> None:
        for expected_x in (0.1, 127.25, 305.5, 639.8):
            _heatmap, index, offset = encode_position_target(expected_x)
            presence_logits = torch.tensor([10.0])
            heatmap_logits = torch.full((1, 160), -10.0)
            heatmap_logits[0, index] = 10.0
            probability = min(1.0 - 1e-6, max(1e-6, offset + 0.5))
            raw_offset = math.log(probability / (1.0 - probability))
            offset_logits = torch.zeros((1, 160))
            offset_logits[0, index] = raw_offset
            _presence, _peak, decoded_index, x_local = decode_batch(
                presence_logits, heatmap_logits, offset_logits
            )
            self.assertEqual(int(decoded_index.item()), index)
            self.assertAlmostEqual(float(x_local.item()), expected_x, places=3)

    def test_model_shapes_and_size(self) -> None:
        model = ShallowBallNet()
        model.eval()
        with torch.inference_mode():
            presence, heatmap, offset = model(torch.zeros(2, 3, 100, 640))
        self.assertEqual(presence.shape, (2,))
        self.assertEqual(heatmap.shape, (2, 160))
        self.assertEqual(offset.shape, (2, 160))
        self.assertLess(parameter_count(model), 100_000)

    def test_loss_is_finite_and_backpropagates(self) -> None:
        model = ShallowBallNet()
        images = torch.rand(2, 3, 100, 640)
        heatmap, index, offset = encode_position_target(305.5)
        targets = {
            "presence": torch.tensor([1.0, 0.0]),
            "heatmap": torch.stack((heatmap, torch.zeros_like(heatmap))),
            "offset_index": torch.tensor([index, 0]),
            "offset": torch.tensor([offset, 0.0]),
        }
        loss = ShallowBallLoss()(model(images), targets).total
        self.assertTrue(torch.isfinite(loss))
        loss.backward()
        self.assertTrue(any(parameter.grad is not None for parameter in model.parameters()))

    def test_dataset_reads_positive_and_negative_labels(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            image_dir = root / "images" / "train"
            label_dir = root / "labels" / "train"
            image_dir.mkdir(parents=True)
            label_dir.mkdir(parents=True)
            image = np.zeros((100, 640, 3), dtype=np.uint8)
            cv2.imwrite(str(image_dir / "positive.png"), image)
            cv2.imwrite(str(image_dir / "negative.png"), image)
            (label_dir / "positive.txt").write_text(
                "0 0.25 0.5 0.05 0.2\n", encoding="utf-8"
            )
            (label_dir / "negative.txt").write_text("", encoding="utf-8")
            dataset = YoloRoiDataset(root, "train")
            samples = {Path(item["path"]).stem: item for item in dataset}
            self.assertEqual(float(samples["positive"]["presence"]), 1.0)
            self.assertAlmostEqual(float(samples["positive"]["x_local"]), 160.0)
            self.assertEqual(float(samples["negative"]["presence"]), 0.0)
            self.assertEqual(float(samples["negative"]["x_local"]), -1.0)

    def test_combined_dataset_preserves_all_sources(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            datasets = []
            for source_index, present in enumerate((True, False)):
                source = root / f"source_{source_index}"
                image_dir = source / "images" / "all"
                label_dir = source / "labels" / "all"
                image_dir.mkdir(parents=True)
                label_dir.mkdir(parents=True)
                cv2.imwrite(
                    str(image_dir / "sample.png"),
                    np.zeros((100, 640, 3), dtype=np.uint8),
                )
                (label_dir / "sample.txt").write_text(
                    "0 0.25 0.5 0.05 0.2\n" if present else "",
                    encoding="utf-8",
                )
                datasets.append(YoloRoiDataset(source, "all"))
            combined = CombinedYoloRoiDataset(datasets)
            self.assertEqual(len(combined), 2)
            self.assertEqual(combined.presences, [True, False])
            self.assertEqual(float(combined[0]["presence"]), 1.0)
            self.assertEqual(float(combined[-1]["presence"]), 0.0)

    def test_combined_dataset_deduplicates_identical_images(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            datasets = []
            for source_index in range(2):
                source = root / f"source_{source_index}"
                image_dir = source / "images" / "all"
                label_dir = source / "labels" / "all"
                image_dir.mkdir(parents=True)
                label_dir.mkdir(parents=True)
                cv2.imwrite(
                    str(image_dir / f"sample_{source_index}.png"),
                    np.zeros((100, 640, 3), dtype=np.uint8),
                )
                (label_dir / f"sample_{source_index}.txt").write_text(
                    "0 0.25 0.5 0.05 0.2\n", encoding="utf-8"
                )
                datasets.append(YoloRoiDataset(source, "all"))
            combined = CombinedYoloRoiDataset(
                datasets, deduplicate_by_content=True
            )
            self.assertEqual(len(combined), 1)
            self.assertEqual(combined.presences, [True])


if __name__ == "__main__":
    unittest.main()
