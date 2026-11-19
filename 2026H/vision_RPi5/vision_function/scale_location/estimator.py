"""Timestamped constant-velocity Kalman estimator for the steel ball."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class MotionFilterConfig:
    measurement_sigma_cm: float = 0.18
    acceleration_sigma_cm_s2: float = 80.0
    initial_velocity_sigma_cm_s: float = 20.0
    innovation_gate_sigma: float = 4.0
    innovation_floor_cm: float = 1.5
    max_frame_interval_s: float = 0.2
    loss_timeout_s: float = 0.1
    min_valid_observations: int = 2
    min_velocity_observations: int = 4
    min_velocity_span_s: float = 0.05
    moving_start_cm_s: float = 2.0
    moving_stop_cm_s: float = 1.0
    max_prediction_s: float = 0.05

    @classmethod
    def load(cls, path: Path | str) -> "MotionFilterConfig":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        allowed = set(cls.__dataclass_fields__)
        unknown = set(payload) - allowed - {"schema_version", "note"}
        if unknown:
            raise ValueError(f"Unknown motion filter fields: {sorted(unknown)}")
        values = {key: payload[key] for key in allowed if key in payload}
        config = cls(**values)
        config.validate()
        return config

    def validate(self) -> None:
        positive = (
            "measurement_sigma_cm",
            "acceleration_sigma_cm_s2",
            "initial_velocity_sigma_cm_s",
            "innovation_gate_sigma",
            "innovation_floor_cm",
            "max_frame_interval_s",
            "loss_timeout_s",
            "min_velocity_span_s",
            "moving_start_cm_s",
            "moving_stop_cm_s",
            "max_prediction_s",
        )
        if any(not math.isfinite(getattr(self, key)) or getattr(self, key) <= 0 for key in positive):
            raise ValueError("Motion filter numeric settings must be finite and positive")
        if self.min_valid_observations < 1:
            raise ValueError("min_valid_observations must be at least 1")
        if self.min_velocity_observations < self.min_valid_observations:
            raise ValueError("min_velocity_observations must be at least 2")
        if self.moving_stop_cm_s > self.moving_start_cm_s:
            raise ValueError("moving_stop_cm_s cannot exceed moving_start_cm_s")


@dataclass(frozen=True)
class EstimateResult:
    initialized: bool
    accepted: bool
    position_cm: float
    velocity_cm_s: float
    velocity_valid: bool
    innovation_cm: float | None
    accepted_observations: int


class MotionEstimator:
    """A small 2-state Kalman filter driven by camera capture timestamps."""

    def __init__(self, config: MotionFilterConfig | None = None) -> None:
        self.config = config or MotionFilterConfig()
        self.config.validate()
        self.reset()

    def reset(self) -> None:
        self._initialized = False
        self._timestamp_s: float | None = None
        self._first_accepted_timestamp_s: float | None = None
        self._last_accepted_timestamp_s: float | None = None
        self._accepted_count = 0
        self._position = 0.0
        self._velocity = 0.0
        self._p00 = 0.0
        self._p01 = 0.0
        self._p10 = 0.0
        self._p11 = 0.0

    @property
    def initialized(self) -> bool:
        return self._initialized

    def _initialize(self, position_cm: float, timestamp_s: float, confidence: float) -> None:
        sigma = self._measurement_sigma(confidence)
        self._initialized = True
        self._timestamp_s = timestamp_s
        self._first_accepted_timestamp_s = timestamp_s
        self._last_accepted_timestamp_s = timestamp_s
        self._accepted_count = 1
        self._position = position_cm
        self._velocity = 0.0
        self._p00 = sigma * sigma
        self._p01 = 0.0
        self._p10 = 0.0
        self._p11 = self.config.initial_velocity_sigma_cm_s**2

    def _measurement_sigma(self, confidence: float) -> float:
        bounded = min(1.0, max(0.25, float(confidence)))
        return self.config.measurement_sigma_cm / bounded

    def _predict(self, dt: float) -> None:
        self._position += dt * self._velocity
        q = self.config.acceleration_sigma_cm_s2**2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        p00 = self._p00 + dt * (self._p01 + self._p10) + dt2 * self._p11
        p01 = self._p01 + dt * self._p11
        p10 = self._p10 + dt * self._p11
        p11 = self._p11
        self._p00 = p00 + q * dt4 / 4.0
        self._p01 = p01 + q * dt3 / 2.0
        self._p10 = p10 + q * dt3 / 2.0
        self._p11 = p11 + q * dt2

    def update(
        self,
        position_cm: float | None,
        timestamp_s: float,
        confidence: float | None,
    ) -> EstimateResult:
        timestamp_s = float(timestamp_s)
        if not math.isfinite(timestamp_s):
            raise ValueError("timestamp_s must be finite")
        if position_cm is not None and not math.isfinite(position_cm):
            raise ValueError("position_cm must be finite")

        if not self._initialized:
            if position_cm is None:
                return EstimateResult(False, False, 0.0, 0.0, False, None, 0)
            self._initialize(position_cm, timestamp_s, confidence or 0.25)
            return EstimateResult(True, True, self._position, 0.0, False, 0.0, 1)

        assert self._timestamp_s is not None
        dt = timestamp_s - self._timestamp_s
        if dt <= 0.0 or dt > self.config.max_frame_interval_s:
            self.reset()
            if position_cm is None:
                return EstimateResult(False, False, 0.0, 0.0, False, None, 0)
            self._initialize(position_cm, timestamp_s, confidence or 0.25)
            return EstimateResult(True, True, self._position, 0.0, False, 0.0, 1)

        self._predict(dt)
        self._timestamp_s = timestamp_s
        accepted = False
        innovation: float | None = None
        if position_cm is not None:
            sigma = self._measurement_sigma(confidence or 0.25)
            measurement_variance = sigma * sigma
            innovation = position_cm - self._position
            innovation_variance = self._p00 + measurement_variance
            gate = max(
                self.config.innovation_floor_cm,
                self.config.innovation_gate_sigma * math.sqrt(innovation_variance),
            )
            if abs(innovation) <= gate:
                k0 = self._p00 / innovation_variance
                k1 = self._p10 / innovation_variance
                p00, p01 = self._p00, self._p01
                self._position += k0 * innovation
                self._velocity += k1 * innovation
                self._p00 = (1.0 - k0) * p00
                self._p01 = (1.0 - k0) * p01
                self._p10 -= k1 * p00
                self._p11 -= k1 * p01
                self._accepted_count += 1
                self._last_accepted_timestamp_s = timestamp_s
                accepted = True

        if position_cm is None:
            assert self._last_accepted_timestamp_s is not None
            if timestamp_s - self._last_accepted_timestamp_s > self.config.loss_timeout_s:
                self.reset()
                return EstimateResult(False, False, 0.0, 0.0, False, None, 0)

        assert self._first_accepted_timestamp_s is not None
        velocity_valid = (
            self._accepted_count >= self.config.min_velocity_observations
            and timestamp_s - self._first_accepted_timestamp_s
            >= self.config.min_velocity_span_s
        )
        return EstimateResult(
            True,
            accepted,
            self._position,
            self._velocity,
            velocity_valid,
            innovation,
            self._accepted_count,
        )
