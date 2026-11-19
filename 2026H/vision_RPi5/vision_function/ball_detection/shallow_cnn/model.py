"""Compact CNN specialized for the fixed 640x100 tube ROI."""

from __future__ import annotations

import torch
from torch import nn


class ConvNormAct(nn.Sequential):
    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        *,
        kernel_size: int = 3,
        stride: tuple[int, int] = (1, 1),
        groups: int = 1,
    ) -> None:
        padding = kernel_size // 2
        super().__init__(
            nn.Conv2d(
                in_channels,
                out_channels,
                kernel_size,
                stride=stride,
                padding=padding,
                groups=groups,
                bias=False,
            ),
            nn.BatchNorm2d(out_channels),
            nn.SiLU(inplace=True),
        )


class DepthwiseSeparableBlock(nn.Module):
    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        *,
        stride: tuple[int, int] = (1, 1),
        residual: bool = False,
    ) -> None:
        super().__init__()
        self.depthwise = ConvNormAct(
            in_channels,
            in_channels,
            stride=stride,
            groups=in_channels,
        )
        self.pointwise = ConvNormAct(
            in_channels,
            out_channels,
            kernel_size=1,
        )
        self.use_residual = residual and stride == (1, 1) and in_channels == out_channels

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        outputs = self.pointwise(self.depthwise(inputs))
        return outputs + inputs if self.use_residual else outputs


class ShallowBallNet(nn.Module):
    """Predict ball presence plus a high-resolution horizontal heatmap and offset."""

    input_height = 100
    input_width = 640
    output_bins = 160

    def __init__(
        self,
        channels: tuple[int, int, int, int] = (16, 24, 32, 48),
        output_bins: int = 160,
    ) -> None:
        super().__init__()
        if output_bins not in (160, 320):
            raise ValueError("output_bins must be 160 or 320")
        self.output_bins = output_bins
        c1, c2, c3, c4 = channels
        self.features = nn.Sequential(
            ConvNormAct(3, c1, stride=(2, 2)),       # 100x640 -> 50x320
            DepthwiseSeparableBlock(c1, c2, stride=(2, 1)),  # -> 25x320
            DepthwiseSeparableBlock(c2, c3, stride=(2, 2)),  # -> 13x160
            DepthwiseSeparableBlock(c3, c4),
            DepthwiseSeparableBlock(c4, c4, residual=True),
            DepthwiseSeparableBlock(c4, c4, residual=True),
        )
        self.axis_neck = nn.Sequential(
            nn.Conv1d(c4 * 2, 32, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm1d(32),
            nn.SiLU(inplace=True),
        )
        self.heatmap_head = nn.Conv1d(32, 1, kernel_size=1)
        self.offset_head = nn.Conv1d(32, 1, kernel_size=1)
        self.presence_head = nn.Sequential(
            nn.Linear(64, 16),
            nn.SiLU(inplace=True),
            nn.Linear(16, 1),
        )
        self._initialize_heads()

    def _initialize_heads(self) -> None:
        nn.init.constant_(self.heatmap_head.bias, -2.2)
        nn.init.zeros_(self.offset_head.bias)
        nn.init.zeros_(self.presence_head[-1].bias)

    def forward(
        self, inputs: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        features = self.features(inputs)
        mean_features = features.mean(dim=2)
        max_features = features.amax(dim=2)
        axis_features = self.axis_neck(torch.cat((mean_features, max_features), dim=1))

        heatmap_logits = self.heatmap_head(axis_features).squeeze(1)
        offset_logits = self.offset_head(axis_features).squeeze(1)
        presence_features = torch.cat(
            (axis_features.mean(dim=2), axis_features.amax(dim=2)), dim=1
        )
        presence_logits = self.presence_head(presence_features).squeeze(1)
        return presence_logits, heatmap_logits, offset_logits


def parameter_count(model: nn.Module) -> int:
    return sum(parameter.numel() for parameter in model.parameters())
