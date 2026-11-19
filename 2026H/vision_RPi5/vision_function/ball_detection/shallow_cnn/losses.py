"""Losses for presence, heatmap and within-bin position offset."""

from __future__ import annotations

from dataclasses import dataclass

import torch
import torch.nn.functional as functional
from torch import nn


@dataclass(frozen=True)
class LossOutput:
    total: torch.Tensor
    presence: torch.Tensor
    heatmap: torch.Tensor
    offset: torch.Tensor


def quality_focal_loss(
    logits: torch.Tensor,
    targets: torch.Tensor,
    *,
    gamma: float = 2.0,
    positive_weight: float = 8.0,
) -> torch.Tensor:
    probabilities = logits.sigmoid()
    modulation = (targets - probabilities).abs().pow(gamma)
    weights = 1.0 + positive_weight * targets
    loss = functional.binary_cross_entropy_with_logits(
        logits, targets, reduction="none"
    )
    return (loss * modulation * weights).mean()


class ShallowBallLoss(nn.Module):
    def __init__(
        self,
        *,
        presence_weight: float = 1.0,
        heatmap_weight: float = 2.0,
        offset_weight: float = 1.0,
    ) -> None:
        super().__init__()
        self.presence_weight = presence_weight
        self.heatmap_weight = heatmap_weight
        self.offset_weight = offset_weight

    def forward(
        self,
        outputs: tuple[torch.Tensor, torch.Tensor, torch.Tensor],
        targets: dict[str, torch.Tensor],
    ) -> LossOutput:
        presence_logits, heatmap_logits, offset_logits = outputs
        presence_targets = targets["presence"].to(presence_logits.dtype)
        presence_loss = functional.binary_cross_entropy_with_logits(
            presence_logits, presence_targets
        )

        positive_mask = presence_targets > 0.5
        if positive_mask.any():
            heatmap_loss = quality_focal_loss(
                heatmap_logits[positive_mask], targets["heatmap"][positive_mask]
            )
            indices = targets["offset_index"][positive_mask].long()
            selected_offset_logits = torch.gather(
                offset_logits[positive_mask], 1, indices[:, None]
            ).squeeze(1)
            selected_offsets = selected_offset_logits.sigmoid() - 0.5
            offset_loss = functional.smooth_l1_loss(
                selected_offsets, targets["offset"][positive_mask]
            )
        else:
            heatmap_loss = heatmap_logits.sum() * 0.0
            offset_loss = offset_logits.sum() * 0.0

        total = (
            self.presence_weight * presence_loss
            + self.heatmap_weight * heatmap_loss
            + self.offset_weight * offset_loss
        )
        return LossOutput(total, presence_loss, heatmap_loss, offset_loss)
