"""Map detector observations to physical state and UART status flags."""

from __future__ import annotations

import math
from time import time
from typing import Callable

from .calibration import ScaleCalibration
from .estimator import MotionEstimator
from .types import BallObservation, ScaleState


class ScalePipeline:
    def __init__(
        self,
        calibration: ScaleCalibration,
        estimator: MotionEstimator,
        *,
        target_cm: float = 0.0,
        position_sign: int = 1,
        clock: Callable[[], float] = time,
    ) -> None:
        if calibration.image_size != (640, 100):
            raise ValueError("The live detector requires a 640x100 scale calibration")
        if position_sign not in (-1, 1):
            raise ValueError("position_sign must be -1 or 1")
        self.calibration = calibration
        self.estimator = estimator
        self.position_sign = position_sign
        self.clock = clock
        self._moving = False
        self.set_target(target_cm)

    def set_target(self, target_cm: float) -> None:
        target_cm = float(target_cm)
        calibration_low, calibration_high = self.calibration.valid_range_cm
        low = min(
            self.position_sign * calibration_low,
            self.position_sign * calibration_high,
        )
        high = max(
            self.position_sign * calibration_low,
            self.position_sign * calibration_high,
        )
        if not math.isfinite(target_cm) or not low <= target_cm <= high:
            raise ValueError(f"target_cm must be in [{low}, {high}]")
        self.target_cm = target_cm

    def set_calibration(self, calibration: ScaleCalibration) -> None:
        """Replace only the pixel-to-centimetre mapping at runtime."""
        if calibration.image_size != (640, 100):
            raise ValueError("The live detector requires a 640x100 scale calibration")
        calibration_low, calibration_high = calibration.valid_range_cm
        low = min(
            self.position_sign * calibration_low,
            self.position_sign * calibration_high,
        )
        high = max(
            self.position_sign * calibration_low,
            self.position_sign * calibration_high,
        )
        if not low <= self.target_cm <= high:
            raise ValueError(
                f"target_cm {self.target_cm} is outside new calibration range "
                f"[{low}, {high}]"
            )
        self.calibration = calibration

    def reset(self) -> None:
        self.estimator.reset()
        self._moving = False

    @staticmethod
    def _quality(confidence: float | None, edge: bool, accepted: bool) -> int:
        if not accepted or confidence is None:
            return 0
        if confidence >= 0.70:
            quality = 3
        elif confidence >= 0.45:
            quality = 2
        else:
            quality = 1
        return min(quality, 2) if edge else quality

    def update(
        self,
        observation: BallObservation,
        *,
        output_timestamp_s: float | None = None,
    ) -> ScaleState:
        output_timestamp_s = (
            float(self.clock()) if output_timestamp_s is None else float(output_timestamp_s)
        )
        age_s = output_timestamp_s - observation.capture_timestamp_s
        if not math.isfinite(age_s) or age_s < -0.005:
            raise ValueError("Output timestamp precedes the camera timestamp")
        age_s = max(0.0, age_s)

        measurement = None
        if observation.found and observation.center_local_xy is not None:
            measurement = self.calibration.measure(observation.center_local_xy)
        measured_position = (
            self.position_sign * measurement.position_cm
            if measurement is not None and measurement.valid
            else None
        )
        estimate = self.estimator.update(
            measured_position,
            observation.capture_timestamp_s,
            observation.confidence,
        )
        valid = bool(
            observation.found
            and measurement is not None
            and measurement.valid
            and estimate.accepted
            and estimate.accepted_observations
            >= self.estimator.config.min_valid_observations
        )
        velocity_valid = valid and estimate.velocity_valid
        edge = bool(measurement.edge) if measurement is not None else False

        speed = abs(estimate.velocity_cm_s)
        config = self.estimator.config
        if velocity_valid:
            if self._moving:
                self._moving = speed >= config.moving_stop_cm_s
            else:
                self._moving = speed >= config.moving_start_cm_s
        elif not estimate.initialized:
            self._moving = False

        prediction_s = min(age_s, config.max_prediction_s)
        prediction_valid = age_s <= config.max_prediction_s
        position_now = estimate.position_cm + estimate.velocity_cm_s * prediction_s
        velocity_now = estimate.velocity_cm_s
        state_valid = valid and prediction_valid
        state_velocity_valid = velocity_valid and prediction_valid
        quality = self._quality(observation.confidence, edge, state_valid)
        if not prediction_valid:
            quality = 0

        displacement = position_now - self.target_cm
        return ScaleState(
            frame_sequence=observation.frame_sequence,
            valid=state_valid,
            velocity_valid=state_velocity_valid,
            moving=self._moving,
            edge=edge,
            quality=quality,
            target_cm=self.target_cm,
            position_at_capture_cm=estimate.position_cm,
            velocity_at_capture_cm_s=estimate.velocity_cm_s,
            acceleration_cm_s2=0.0,
            position_now_cm=position_now,
            velocity_now_cm_s=velocity_now,
            displacement_cm=displacement,
            control_error_cm=-displacement,
            capture_timestamp_s=observation.capture_timestamp_s,
            output_timestamp_s=output_timestamp_s,
            measurement_age_ms=age_s * 1000.0,
            confidence=observation.confidence,
            innovation_cm=estimate.innovation_cm,
        )
