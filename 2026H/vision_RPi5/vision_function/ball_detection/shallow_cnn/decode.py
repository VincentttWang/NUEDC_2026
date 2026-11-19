"""Encode continuous ball coordinates and decode shallow-CNN predictions."""

from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass(frozen=True)
class PositionPrediction:
    found: bool
    x_local: float | None
    presence_probability: float
    heatmap_peak: float
    bin_index: int

    @property
    def confidence(self) -> float:
        return (self.presence_probability * self.heatmap_peak) ** 0.5


def encode_position_target(
    x_local: float,
    *,
    input_width: int = 640,
    output_bins: int = 160,
    sigma_bins: float = 1.5,
) -> tuple[torch.Tensor, int, float]:
    """Create a Gaussian heatmap and an offset within the containing bin."""
    if not 0.0 <= x_local < float(input_width):
        raise ValueError(f"x_local must be in [0, {input_width}), got {x_local}")
    if output_bins < 2 or input_width % output_bins:
        raise ValueError("input_width must be exactly divisible by output_bins")
    if sigma_bins <= 0:
        raise ValueError("sigma_bins must be positive")

    bin_size = input_width / output_bins
    continuous_index = x_local / bin_size - 0.5
    bin_index = min(output_bins - 1, max(0, int(x_local // bin_size)))
    offset = x_local / bin_size - (bin_index + 0.5)
    coordinates = torch.arange(output_bins, dtype=torch.float32)
    heatmap = torch.exp(-0.5 * ((coordinates - continuous_index) / sigma_bins) ** 2)
    heatmap /= heatmap.max().clamp_min(1e-6)
    return heatmap, bin_index, float(offset)


def decode_batch(
    presence_logits: torch.Tensor,
    heatmap_logits: torch.Tensor,
    offset_logits: torch.Tensor,
    *,
    input_width: int = 640,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Vectorized differentiable decode used by evaluation and export checks."""
    if heatmap_logits.ndim != 2 or offset_logits.shape != heatmap_logits.shape:
        raise ValueError("heatmap and offset outputs must have shape [batch, bins]")
    if presence_logits.shape != (heatmap_logits.shape[0],):
        raise ValueError("presence output must have shape [batch]")
    bins = heatmap_logits.shape[1]
    if input_width % bins:
        raise ValueError("input_width must be exactly divisible by output bins")

    heatmap_probabilities = heatmap_logits.sigmoid()
    heatmap_peak, indices = heatmap_probabilities.max(dim=1)
    selected_offsets = torch.gather(offset_logits, 1, indices[:, None]).squeeze(1)
    selected_offsets = selected_offsets.sigmoid() - 0.5
    bin_size = float(input_width) / bins
    x_local = (indices.to(offset_logits.dtype) + 0.5 + selected_offsets) * bin_size
    return presence_logits.sigmoid(), heatmap_peak, indices, x_local


def decode_predictions(
    presence_logits: torch.Tensor,
    heatmap_logits: torch.Tensor,
    offset_logits: torch.Tensor,
    *,
    presence_threshold: float = 0.5,
    heatmap_threshold: float = 0.25,
    input_width: int = 640,
) -> list[PositionPrediction]:
    presence, peaks, indices, positions = decode_batch(
        presence_logits, heatmap_logits, offset_logits, input_width=input_width
    )
    predictions: list[PositionPrediction] = []
    for probability, peak, index, position in zip(
        presence.detach().cpu(),
        peaks.detach().cpu(),
        indices.detach().cpu(),
        positions.detach().cpu(),
        strict=True,
    ):
        is_found = bool(
            probability.item() >= presence_threshold
            and peak.item() >= heatmap_threshold
        )
        predictions.append(
            PositionPrediction(
                found=is_found,
                x_local=float(position.item()) if is_found else None,
                presence_probability=float(probability.item()),
                heatmap_peak=float(peak.item()),
                bin_index=int(index.item()),
            )
        )
    return predictions
