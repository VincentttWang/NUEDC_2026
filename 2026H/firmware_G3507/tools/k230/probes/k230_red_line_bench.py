"""Finite K230 v1.8 benchmark for the red-line candidate."""
import sys

sys.path.append("/sdcard")
import vision_red_line_follow_candidate as detector

detector.RUN_DURATION_MS = 10000
detector.DISPLAY_LCD = True
detector.UART_ENABLED = False
detector.LOG_INTERVAL = 30
detector.main()
print("RED_LINE_BENCH_DONE")
