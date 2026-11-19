"""Shallow CNN for single-ball presence and one-dimensional localization."""

from .decode import PositionPrediction, decode_predictions, encode_position_target
from .model import ShallowBallNet

__all__ = [
    "PositionPrediction",
    "ShallowBallNet",
    "decode_predictions",
    "encode_position_target",
]
