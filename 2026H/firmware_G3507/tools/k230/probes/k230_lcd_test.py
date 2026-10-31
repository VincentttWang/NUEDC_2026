"""Short, non-persistent ST7701 LCD self-test for the connected K230."""
import os
import time

import image
from media.display import Display
from media.media import MediaManager


LCD_WIDTH = 800
LCD_HEIGHT = 480


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    display_ready = False
    media_ready = False
    try:
        # ARGB8888 is the framebuffer format used by Canaan's ST7701 example.
        canvas = image.Image(LCD_WIDTH, LCD_HEIGHT, image.ARGB8888)
        Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT,
                     to_ide=True)
        display_ready = True
        MediaManager.init()
        media_ready = True

        canvas.clear()
        canvas.draw_string_advanced(70, 55, 52, "K230 LCD SELF TEST",
                                    color=(255, 255, 255))
        canvas.draw_string_advanced(70, 145, 64, "RED", color=(255, 0, 0))
        canvas.draw_string_advanced(300, 145, 64, "GREEN", color=(0, 255, 0))
        canvas.draw_string_advanced(600, 145, 64, "BLUE", color=(0, 0, 255))
        canvas.draw_string_advanced(70, 270, 42, "ST7701 800x480 ARGB8888",
                                    color=(255, 255, 0))
        canvas.draw_string_advanced(70, 355, 34, "display path is alive",
                                    color=(0, 255, 255))
        Display.show_image(canvas)
        print("LCD_TEST_SHOWN format=ARGB8888 size=800x480")
        for remaining in range(8, 0, -1):
            print("LCD_TEST_REMAINING %d" % remaining)
            time.sleep(1)
            os.exitpoint()
    finally:
        # Release media pools cleanly before main.py is restarted. The physical
        # panel is reinitialized immediately by the following soft reset.
        if display_ready:
            try:
                Display.deinit()
            except Exception:
                pass
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except Exception:
            pass
        time.sleep_ms(100)
        if media_ready:
            try:
                MediaManager.deinit()
            except Exception:
                pass


main()
