import unittest

from vision_function.ball_detection.extract_actual_video_frames import (
    choose_annotation,
    sampling_interval,
)


class VideoSamplingTest(unittest.TestCase):
    def test_sampling_interval_tracks_speed(self) -> None:
        self.assertEqual(sampling_interval(0), 1.0)
        self.assertEqual(sampling_interval(20), 0.5)
        self.assertEqual(sampling_interval(80), 0.25)
        self.assertEqual(sampling_interval(160), 0.1)

    def test_hough_is_used_when_detectors_agree(self) -> None:
        annotation, source, distance = choose_annotation(
            (100.0, 40.0, 12.0),
            (88.0, 28.0, 112.0, 52.0),
            0.8,
        )
        self.assertEqual(annotation, (100.0, 40.0, 12.0))
        self.assertEqual(source, "agreement")
        self.assertAlmostEqual(distance or 0.0, 0.0)

    def test_yolo_fills_hough_miss(self) -> None:
        annotation, source, distance = choose_annotation(
            None,
            (88.0, 28.0, 112.0, 52.0),
            0.8,
        )
        self.assertEqual(annotation, (100.0, 40.0, 12.0))
        self.assertEqual(source, "yolo_only")
        self.assertIsNone(distance)

    def test_yolo_wins_large_disagreement(self) -> None:
        annotation, source, distance = choose_annotation(
            (500.0, 40.0, 12.0),
            (88.0, 28.0, 112.0, 52.0),
            0.8,
        )
        self.assertEqual(annotation, (100.0, 40.0, 12.0))
        self.assertEqual(source, "disagreement_yolo")
        self.assertGreater(distance or 0.0, 300.0)


if __name__ == "__main__":
    unittest.main()
