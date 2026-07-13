# 持续运行(30s) + LCD 实时显示 + 每秒回传遥测。用改写后的 rect_target 真实检测链。
import sys, time, gc
sys.path.append("/sdcard")
import rect_target as rt

rt.DISPLAY_MODE = "none"    # headless: LCD 路径(to_ide=False)会卡, 与 cv_lite 无关
DURATION_MS = 25000
FULL_ROI = [0, 0, rt.DETECT_WIDTH, rt.DETECT_HEIGHT]

def main():
    rt.camera_init()
    try:
        t0 = time.ticks_ms()
        sec_t = t0
        frames = sec_frames = sec_valid = 0
        last_center = None
        while time.ticks_diff(time.ticks_ms(), t0) < DURATION_MS:
            rt.os.exitpoint()
            img = rt.sensor.snapshot()
            rects = rt.detect_rects(img)
            cand = rt.select_best_rect(img, rects, True) if rects else None
            if cand is not None:
                best, corners, center = cand
                last_center = center
                sec_valid += 1
                txc = rt.mirror_point(center)
                txcorners = [rt.mirror_point(p) for p in corners]
                try:
                    rt.send_vision_frame(txc[0], txc[1],
                                         rt.IMG_CENTER_X, rt.IMG_CENTER_Y, txcorners)
                except Exception:
                    pass
            else:
                try:
                    rt.send_vision_frame()
                except Exception:
                    pass
            frames += 1
            sec_frames += 1
            img = None
            now = time.ticks_ms()
            if time.ticks_diff(now, sec_t) >= 1000:
                fps = sec_frames * 1000.0 / time.ticks_diff(now, sec_t)
                print("t=%02ds fps=%.1f lock=%d/%d center=%s" % (
                    time.ticks_diff(now, t0) // 1000, fps,
                    sec_valid, sec_frames, last_center))
                sec_t = now
                sec_frames = sec_valid = 0
            if frames % 30 == 0:
                gc.collect()
        print("=== LIVE DONE total_frames", frames, "===")
    finally:
        rt.camera_deinit()

main()
