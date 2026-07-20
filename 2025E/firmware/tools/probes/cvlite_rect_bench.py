# 上板 bench: 导入改写后的 rect_target, 用真实 detect_rects + select_best_rect
# 跑有限帧, 测 cv_lite 路径的帧率与检测情况, 结束干净释放相机。无 LCD/AI, 纯检测。
import sys, time, gc
sys.path.append("/sdcard")
import rect_target as rt

rt.DISPLAY_MODE = "none"   # 纯检测测帧率, 不走 LCD

N = 120

def main():
    print("cv_lite fns present:",
          hasattr(rt.cv_lite, "grayscale_find_rectangles_with_corners"))
    rt.camera_init()
    try:
        for _ in range(5):
            rt.sensor.snapshot()   # 预热
        t0 = time.ticks_ms()
        raw_total = 0
        valid_frames = 0
        first_hit = None
        for i in range(N):
            img = rt.sensor.snapshot()
            rects = rt.detect_rects(img)
            raw_total += len(rects)
            cand = rt.select_best_rect(img, rects, True) if rects else None
            if cand is not None:
                valid_frames += 1
                if first_hit is None:
                    first_hit = (i, cand[2], cand[1])
            img = None
            if i % 30 == 0:
                gc.collect()
        dt = time.ticks_diff(time.ticks_ms(), t0)
        print("=== RESULT ===")
        print("frames", N, "ms", dt, "fps", round(N * 1000.0 / dt, 2))
        print("raw_rects_total", raw_total,
              "avg_per_frame", round(raw_total / float(N), 2))
        print("valid_frames", valid_frames)
        if first_hit:
            print("first_hit frame", first_hit[0],
                  "center", first_hit[1], "corners", first_hit[2])
        else:
            print("no valid target this run (无靶在视野内?)")
    finally:
        rt.camera_deinit()

main()
print("=== BENCH DONE ===")
