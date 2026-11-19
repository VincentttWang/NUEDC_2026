from __future__ import annotations

import unittest

from vision_function.scale_location.calibration import load_scale_calibration
from vision_function.scale_location.estimator import MotionEstimator, MotionFilterConfig
from vision_function.scale_location.pipeline import ScalePipeline
from vision_function.scale_location.types import BallObservation


CALIBRATION = (
    "vision_function/scale_location/config/scale_calibration_temporary_25cm.json"
)


def observation(sequence: int, timestamp: float, position_cm: float) -> BallObservation:
    x = 286.0 + position_cm / 12.5 * 296.0
    return BallObservation(
        frame_sequence=sequence,
        capture_timestamp_s=timestamp,
        found=True,
        center_local_xy=(x, 50.0),
        confidence=0.8,
        box_local_xyxy=(x - 10.0, 40.0, x + 10.0, 60.0),
        candidate_count=1,
    )


class ScalePipelineTests(unittest.TestCase):
    def make_pipeline(self) -> ScalePipeline:
        config = MotionFilterConfig(
            measurement_sigma_cm=0.1,
            acceleration_sigma_cm_s2=80.0,
            min_velocity_observations=4,
            min_velocity_span_s=0.05,
        )
        return ScalePipeline(
            load_scale_calibration(CALIBRATION), MotionEstimator(config), target_cm=5.0
        )

    def test_position_velocity_target_and_flags_are_produced(self) -> None:
        pipeline = self.make_pipeline()
        states = []
        for index in range(6):
            timestamp = 100.0 + index * 0.03
            states.append(
                pipeline.update(
                    observation(index, timestamp, index * 0.3),
                    output_timestamp_s=timestamp + 0.025,
                )
            )
        state = states[-1]
        self.assertTrue(state.valid)
        self.assertTrue(state.velocity_valid)
        self.assertTrue(state.moving)
        self.assertEqual(state.quality, 3)
        self.assertAlmostEqual(state.position_at_capture_cm, 1.5, delta=0.2)
        self.assertGreater(state.velocity_at_capture_cm_s, 5.0)
        self.assertAlmostEqual(
            state.control_error_cm, state.target_cm - state.position_now_cm
        )
        self.assertAlmostEqual(state.measurement_age_ms, 25.0, places=6)

    def test_position_sign_inverts_position_and_velocity_together(self) -> None:
        config = MotionFilterConfig(
            measurement_sigma_cm=0.1,
            acceleration_sigma_cm_s2=80.0,
            min_velocity_observations=4,
            min_velocity_span_s=0.05,
        )
        pipeline = ScalePipeline(
            load_scale_calibration(CALIBRATION),
            MotionEstimator(config),
            position_sign=-1,
        )
        states = []
        for index in range(6):
            timestamp = 200.0 + index * 0.03
            states.append(
                pipeline.update(
                    observation(index, timestamp, index * 0.3),
                    output_timestamp_s=timestamp + 0.025,
                )
            )
        state = states[-1]
        self.assertAlmostEqual(state.position_at_capture_cm, -1.5, delta=0.2)
        self.assertLess(state.velocity_at_capture_cm_s, -5.0)

    def test_rejects_invalid_position_sign(self) -> None:
        with self.assertRaisesRegex(ValueError, "position_sign"):
            ScalePipeline(
                load_scale_calibration(CALIBRATION),
                MotionEstimator(MotionFilterConfig()),
                position_sign=0,
            )

    def test_missing_detection_retains_state_but_clears_validity(self) -> None:
        pipeline = self.make_pipeline()
        first = pipeline.update(
            observation(0, 10.0, 1.0), output_timestamp_s=10.02
        )
        missing = BallObservation(1, 10.03, False, None, None, None, 0)
        second = pipeline.update(missing, output_timestamp_s=10.05)
        self.assertFalse(first.valid)
        self.assertFalse(second.valid)
        self.assertFalse(second.velocity_valid)
        self.assertAlmostEqual(second.position_at_capture_cm, first.position_at_capture_cm)

    def test_out_of_range_measurement_is_clamped_and_marks_edge(self) -> None:
        pipeline = self.make_pipeline()
        outside = BallObservation(
            0, 1.0, True, (630.0, 50.0), 0.9, (620.0, 40.0, 639.0, 60.0), 1
        )
        state = pipeline.update(outside, output_timestamp_s=1.02)
        # The measurement is clamped (not discarded), but a single edge observation
        # is not enough to satisfy the estimator's min_valid_observations requirement.
        self.assertFalse(state.valid)
        self.assertTrue(state.edge)
        self.assertEqual(state.quality, 0)

    def test_single_detection_is_not_valid_and_long_loss_resets_state(self) -> None:
        pipeline = self.make_pipeline()
        first = pipeline.update(
            observation(0, 20.0, 3.0), output_timestamp_s=20.02
        )
        self.assertFalse(first.valid)
        missing = BallObservation(1, 20.11, False, None, None, None, 0)
        lost = pipeline.update(missing, output_timestamp_s=20.13)
        self.assertFalse(lost.valid)
        self.assertEqual(lost.position_at_capture_cm, 0.0)
        reacquired = pipeline.update(
            observation(2, 20.14, 3.0), output_timestamp_s=20.16
        )
        self.assertFalse(reacquired.valid)


if __name__ == "__main__":
    unittest.main()
