"""track_follow_viz 的纯解析/计算单元测试。"""
import unittest

from track_follow_viz import TrackTelemetry, filtered_measured_accel, parse_track_line


SAMPLE = (
    "[TRK] t=1250 run=7 req=5 seg=S2 st=FOLLOW fs=LINE gm=1 sen=1 mask=18 n=2 err=0.0 cor=-1.25 "
    "vc=0.123 ac=0.120 vs=0.020 wref=30.0 wz=28.5 yaw=87.3 "
    "href=85.0 herr=-2.3 hs=1 "
    "vl=0.110 vr=0.120 dl=31.0 dr=32.0 "
    "sl=0.150 sr=0.151 s=0.151 rem=0.000 drop=0"
)


class TrackFollowVizTest(unittest.TestCase):
    def test_parse_data_line(self):
        parsed = parse_track_line(SAMPLE)
        self.assertEqual(parsed["t"], 1250)
        self.assertEqual(parsed["run"], 7)
        self.assertEqual(parsed["req"], 5)
        self.assertEqual(parsed["seg"], "S2")
        self.assertEqual(parsed["st"], "FOLLOW")
        self.assertEqual(parsed["fs"], "LINE")
        self.assertEqual(parsed["gm"], 1)
        self.assertEqual(parsed["mask"], 0x18)
        self.assertAlmostEqual(parsed["ac"], 0.12)
        self.assertAlmostEqual(parsed["vs"], 0.02)
        self.assertAlmostEqual(parsed["wref"], 30.0)
        self.assertAlmostEqual(parsed["wz"], 28.5)
        self.assertAlmostEqual(parsed["yaw"], 87.3)
        self.assertAlmostEqual(parsed["href"], 85.0)
        self.assertAlmostEqual(parsed["herr"], -2.3)
        self.assertEqual(parsed["hs"], 1)
        self.assertAlmostEqual(parsed["turn"], 0.0)

    def test_parse_legacy_line_without_steer_command(self):
        parsed = parse_track_line(
            SAMPLE.replace("run=7 ", "")
                  .replace("fs=LINE ", "")
                  .replace("gm=1 ", "")
                  .replace("vs=0.020 ", "")
                  .replace("wref=30.0 ", "")
                  .replace("wz=28.5 ", "")
                  .replace("yaw=87.3 ", ""))
        self.assertAlmostEqual(parsed["vs"], 0.0)
        self.assertAlmostEqual(parsed["wref"], 0.0)
        self.assertAlmostEqual(parsed["wz"], 0.0)
        self.assertAlmostEqual(parsed["yaw"], 0.0)
        self.assertAlmostEqual(parsed["turn"], 0.0)

    def test_parse_turn_angle(self):
        parsed = parse_track_line(SAMPLE.replace(
            "yaw=87.3 ", "yaw=87.3 turn=176.4 "))
        self.assertAlmostEqual(parsed["turn"], 176.4)

    def test_ignore_event_line(self):
        self.assertIsNone(parse_track_line("[TRK] finish detected t=23000 s=5.95"))

    def test_reject_missing_field(self):
        with self.assertRaises(ValueError):
            parse_track_line("[TRK] t=1 st=FOLLOW")

    def test_measured_acceleration(self):
        accel = filtered_measured_accel(
            [0, 100, 200], [0.0, 0.1, 0.2], [0.0, 0.1, 0.2], alpha=1.0)
        self.assertEqual(len(accel), 3)
        self.assertAlmostEqual(accel[1], 1.0)
        self.assertAlmostEqual(accel[2], 1.0)

    def test_enter_event_clears_previous_run(self):
        telemetry = TrackTelemetry()
        telemetry.feed_line(SAMPLE)
        self.assertTrue(telemetry.snapshot()["t"])
        telemetry.feed_line("[TRK] --- enter rear->axle=0.070m ---")
        self.assertFalse(telemetry.snapshot()["t"])


if __name__ == "__main__":
    unittest.main()
