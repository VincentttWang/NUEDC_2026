"""Low-overhead shared-memory preview bridge for the production runtime.

The UART process is the only process that performs inference.  It publishes the
already processed frame and typed result to ``/dev/shm`` for the desktop UI.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from time import time_ns
from typing import Any

import cv2
import numpy as np


class DebugFramePublisher:
    """Atomically publish a two-slot latest-frame snapshot in a tmpfs directory."""

    def __init__(self, directory: Path | str, *, roi_jpeg_quality: int = 90) -> None:
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.roi_jpeg_quality = int(roi_jpeg_quality)
        self._generation = 0

    def publish(
        self,
        source_jpeg: bytes,
        tube_frame: np.ndarray | None,
        result: dict[str, Any],
    ) -> None:
        if tube_frame is None or tube_frame.size == 0:
            return
        ok, encoded_roi = cv2.imencode(
            ".jpg",
            tube_frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.roi_jpeg_quality],
        )
        if not ok:
            return

        self._generation += 1
        slot = self._generation % 2
        source_name = f"source-{slot}.jpg"
        roi_name = f"roi-{slot}.jpg"
        self._replace_bytes(source_name, source_jpeg)
        self._replace_bytes(roi_name, encoded_roi.tobytes())

        payload = {
            "schema_version": 1,
            "generation": self._generation,
            "published_ns": time_ns(),
            "source_image": source_name,
            "roi_image": roi_name,
            "result": result,
        }
        metadata_tmp = self.directory / ".latest.json.tmp"
        metadata_tmp.write_text(
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
        os.replace(metadata_tmp, self.directory / "latest.json")

    def _replace_bytes(self, name: str, payload: bytes) -> None:
        target = self.directory / name
        temporary = self.directory / f".{name}.tmp"
        temporary.write_bytes(payload)
        os.replace(temporary, target)


def decode_published_image(
    path: Path, flags: int = cv2.IMREAD_COLOR
) -> np.ndarray:
    encoded = np.frombuffer(path.read_bytes(), dtype=np.uint8)
    frame = cv2.imdecode(encoded, flags)
    if frame is None:
        raise RuntimeError(f"无法解码调试画面：{path}")
    return frame
