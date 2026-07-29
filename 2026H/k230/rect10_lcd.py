# ------------------------------------------------------------------------
# rect10_lcd.py
# 基于 rect10.py 的 LCD 显示版本：
# 1. 识别流程、16:9 比例 (320x180)、LAB 阈值、动态 ROI 追踪与形态学参数与 rect10.py 一致。
# 2. 主要差异：显示后端从 IDE 虚拟显示改为 LCD 屏幕 (ST7701)，关闭 to_ide 镜像输出。
# 3. 显示时复用一张 800x480 RGB565 LCD 帧，通过 draw_image 将 320x180 画面拉伸到 LCD 全屏。
# ------------------------------------------------------------------------
import time, os, gc, sys, math
import image

from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin
from machine import FPIOA

# --- 摄像头视场角 (FOV) 预估值 ---
H_FOV_DEG = 60.0              # 水平视场角 (角度)
V_FOV_DEG = 35.0              # 垂直视场角 (角度)

# 预计算 FOV 相关的常数
H_TAN_HALF_FOV = math.tan(math.radians(H_FOV_DEG / 2.0))  # 水平视场角一半的正切值，用于偏转角计算
V_TAN_HALF_FOV = math.tan(math.radians(V_FOV_DEG / 2.0))  # 垂直视场角一半的正切值，用于偏转角计算

# 1. 严格同步跑通的分辨率设置
SENSOR_ID = 2                 # 摄像头传感器 ID (ID=2 对应 1920x1080 基础采集模式)
SENSOR_BASE_WIDTH = 1920      # 摄像头原始采集宽度 (16:9 宽画幅)
SENSOR_BASE_HEIGHT = 1080     # 摄像头原始采集高度 (16:9 宽画幅)
SENSOR_FPS = 60               # 摄像头采集帧率 (FPS)
DISPLAY_FPS = 15              # 显示刷新率 (FPS)
DETECT_WIDTH = ALIGN_UP(320, 16)  # 算法图像处理宽度 (16字节对齐，设为 320)
DETECT_HEIGHT = 180           # 算法图像处理高度 (16:9 比例，设为 180)
LCD_WIDTH = 800               # LCD 物理屏幕宽度 (像素)
LCD_HEIGHT = 480              # LCD 物理屏幕高度 (像素)
LCD_X_SCALE = LCD_WIDTH / DETECT_WIDTH  # 图像在 LCD 水平方向拉伸倍数
LCD_Y_SCALE = LCD_HEIGHT / DETECT_HEIGHT # 图像在 LCD 垂直方向拉伸倍数

IMG_CENTER_X = DETECT_WIDTH // 2   # 图像几何中心 X 坐标
IMG_CENTER_Y = DETECT_HEIGHT // 2  # 图像几何中心 Y 坐标

# LAB 阈值：格式为 (L_min, L_max, A_min, A_max, B_min, B_max)
# 这组值需要按现场光照在 IDE 阈值工具里微调。
LAB_TARGET_THRESHOLD = (0, 24, -18, 15, -17, 22)  # 识别黑色边框的 LAB 色彩空间阈值
LAB_BINARY_INVERT = True      # 二值化是否反相 (True 表示将阈值外的亮色区域设为白色，阈值内的黑色边框设为白色)
DEBUG_CANDIDATES = True       # 是否开启候选矩形调试模式 (若开启会绘制并打印过滤原因)

# --- 状态机定义 ---
STATE_SEARCHING = 0           # 状态机状态：搜寻状态 (全屏搜索目标)
STATE_LOCKED = 1              # 状态机状态：锁定状态 (在局部 ROI 框内精准跟踪)
STATE_COASTING = 2            # 状态机状态：滑行状态 (目标短暂丢失时的滑行重获)

# --- 靶标验证与局部 ROI 追踪配置 (与 rect10.py 保持一致) ---
MIN_ASPECT_RATIO = 0.85       # 候选靶标的最小长宽比
MAX_ASPECT_RATIO = 1.65       # 候选靶标的最大长宽比
MIN_AREA = 2250               # 候选靶标在 320x180 画面下的最小面积 (像素点数)
MAX_AREA = 26250              # 候选靶标在 320x180 画面下的最大面积 (像素点数)
MIN_DENSITY_MEAN = 70         # 靶标内部二值化后白色像素平均亮度阈值 (用于判断空心度和过滤杂噪)

# ROI 局部追踪参数
ROI_MARGIN = 25               # 跟踪成功后，局部搜索框向四周外扩的像素余量 (防止运动出框)
MAX_COASTING_FRAMES = 3       # 目标短暂丢失时，允许保持滑行状态的最大帧数
ROI_EXPAND_MARGIN = 80        # 局部追踪丢失后，每次向外扩展搜索区域的像素量
MAX_ROI_EXPAND_STEPS = 2      # 局部追踪丢失后，最大允许扩张 ROI 的次数，超限则退回全屏搜索
ROI_FIND_RECTS_THRESHOLD = 8000  # 局部 ROI 追踪时的矩形边缘梯度阈值
FULLSCREEN_FIND_RECTS_THRESHOLD = 8000  # 全屏搜索时的矩形边缘梯度阈值
FULLSCREEN_SEARCH_INTERVAL = 1  # 全屏搜索帧间隔 (每帧都执行)

# 形态学滤波闭运算迭代次数 (1:1 保持边框尺寸)
MORPH_ITERATIONS = 1          # 膨胀与腐蚀的迭代次数，用于边缘缝合与降噪

sensor = None
gpio2_pin = None

def gpio_init():
    global gpio2_pin
    fpioa = FPIOA()
    fpioa.set_function(2, FPIOA.GPIO2)
    gpio2_pin = Pin(2, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
    gpio2_pin.value(0)

def set_gpio2_high(is_high):
    if gpio2_pin:
        gpio2_pin.value(1 if is_high else 0)

def camera_init():
    global sensor
    # 构造 Sensor 对象
    sensor = Sensor(id=SENSOR_ID, width=SENSOR_BASE_WIDTH, height=SENSOR_BASE_HEIGHT, fps=SENSOR_FPS)
    sensor.reset()

    # set chn0 output size
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    # LAB 阈值需要彩色输入
    sensor.set_pixformat(Sensor.RGB565)

    # use LCD as display output
    Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT, fps=DISPLAY_FPS, to_ide=False)
    # init media manager
    MediaManager.init()
    # sensor start run
    sensor.run()
    gpio_init()

def camera_deinit():
    global sensor
    set_gpio2_high(False)
    if sensor:
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()

# ------------------------------------------------------------------------
# 坐标解算辅助函数 (采用投影交点法)
# ------------------------------------------------------------------------
def get_target_center(corners):
    x1, y1 = corners[0]; x2, y2 = corners[2]
    x3, y3 = corners[1]; x4, y4 = corners[3]
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 0.0001:
        return (x1 + x2 + x3 + x4) // 4, (y1 + y2 + y3 + y4) // 4
    ix = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) // denom
    iy = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) // denom
    return int(ix), int(iy)

# ------------------------------------------------------------------------
# 校验单个矩形是否符合靶标特征 (几何 + 占比过滤)
# ------------------------------------------------------------------------
def validate_target_candidate(img, r):
    w, h = r.w(), r.h()
    area = w * h
    aspect_ratio = float(w) / h if h != 0 else 0

    if not (MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO):
        return False
    if not (MIN_AREA <= area <= MAX_AREA):
        return False

    stat = img.statistics(roi=r.rect())
    if stat.mean() < MIN_DENSITY_MEAN:
        return False

    return True

def get_candidate_reject_reason(img, r):
    w, h = r.w(), r.h()
    area = w * h
    aspect_ratio = float(w) / h if h != 0 else 0
    stat = img.statistics(roi=r.rect())
    mean = stat.mean()

    if not (MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO):
        return "aspect", area, aspect_ratio, mean
    if not (MIN_AREA <= area <= MAX_AREA):
        return "area", area, aspect_ratio, mean
    if mean < MIN_DENSITY_MEAN:
        return "mean", area, aspect_ratio, mean
    return "ok", area, aspect_ratio, mean

# ------------------------------------------------------------------------
# 根据检测到的矩形，计算并裁剪下一帧的局部搜索 ROI
# ------------------------------------------------------------------------
def calculate_search_roi(r):
    rx, ry, rw, rh = r.rect()

    # 向四周延伸外扩 margin 像素
    x = rx - ROI_MARGIN
    y = ry - ROI_MARGIN
    w = rw + 2 * ROI_MARGIN
    h = rh + 2 * ROI_MARGIN

    # 裁剪到图像物理边界，防止报错
    x1 = max(0, x)
    y1 = max(0, y)
    x2 = min(DETECT_WIDTH, x + w)
    y2 = min(DETECT_HEIGHT, y + h)

    return [x1, y1, x2 - x1, y2 - y1]

def expand_search_roi(roi, margin):
    rx, ry, rw, rh = roi
    x1 = max(0, rx - margin)
    y1 = max(0, ry - margin)
    x2 = min(DETECT_WIDTH, rx + rw + margin)
    y2 = min(DETECT_HEIGHT, ry + rh + margin)
    return [x1, y1, x2 - x1, y2 - y1]

def is_fullscreen_roi(roi):
    return roi[0] == 0 and roi[1] == 0 and roi[2] >= DETECT_WIDTH and roi[3] >= DETECT_HEIGHT

def capture_picture():
    # --- 瞬时帧率计算变量 (滑动窗口，以1秒为区间统计真实帧率) ---
    frame_times = []
    fps_val = 0.0

    # --- 追踪状态与 ROI 缓存变量 ---
    tracking_state = STATE_SEARCHING
    coast_counter = 0

    # 初始状态下的追踪区域为全图
    search_roi = [0, 0, DETECT_WIDTH, DETECT_HEIGHT]

    # 缓存的历史解算数据 (用于丢失滑行输出)
    last_rect = None          # 矩形外框 [x, y, w, h]
    last_cx = 0               # 靶心 x
    last_cy = 0               # 靶心 y
    last_dx = 0
    last_dy = 0
    last_yaw = 0.0
    last_pitch = 0.0
    roi_expand_steps = 0
    fullscreen_search_counter = 0
    display_img = image.Image(LCD_WIDTH, LCD_HEIGHT, image.RGB565)

    while True:
        # 计算滑动窗口瞬时帧率
        now_time = time.ticks_ms()
        frame_times.append(now_time)
        # 移出 1 秒 (1000 毫秒) 之前的帧时间戳
        while frame_times and time.ticks_diff(now_time, frame_times[0]) > 1000:
            frame_times.pop(0)

        # 计算最近 1 秒内的 FPS
        if len(frame_times) > 1:
            fps_val = (len(frame_times) - 1) * 1000.0 / time.ticks_diff(now_time, frame_times[0])
        else:
            fps_val = 0.0

        try:
            os.exitpoint()
            global sensor
            # 1. 抓取原始彩色图像
            img = sensor.snapshot()

            active_fullscreen = is_fullscreen_roi(search_roi)
            run_rect_search = True
            fullscreen_search_counter = 0

            # 2. 图像处理与算法运行 (全部局限在 active_roi 内，大幅降低 CPU 开销)
            # 与 rect_07.py 唯一算法差异：这里使用 LAB 阈值，而不是灰度阈值。
            img.binary([LAB_TARGET_THRESHOLD], invert=LAB_BINARY_INVERT, roi=search_roi)

            if run_rect_search:
                # 执行形态学闭运算以提高边缘稳定性（缝合边缘并去除毛刺）
                img.dilate(MORPH_ITERATIONS, roi=search_roi)
                img.erode(MORPH_ITERATIONS, roi=search_roi)

            rects = None
            if run_rect_search:
                rect_threshold = FULLSCREEN_FIND_RECTS_THRESHOLD if active_fullscreen else ROI_FIND_RECTS_THRESHOLD
                # 运行矩形识别，同样只限制在搜索区域内
                rects = img.find_rects(threshold=rect_threshold, roi=search_roi)

            best_rect = None

            if rects:
                # 单次遍历选出通过特征校验且面积最大的矩形，避免额外候选列表占用内存
                for r in rects:
                    if DEBUG_CANDIDATES:
                        reason, area, aspect_ratio, mean = get_candidate_reject_reason(img, r)
                        img.draw_rectangle([v for v in r.rect()], color=255, thickness=1)
                        print(f"Rect cand -> reason:{reason} rect:{r.rect()} area:{area} aspect:{aspect_ratio:.2f} mean:{mean:.1f}")
                        if reason == "ok":
                            if best_rect is None or area > (best_rect.w() * best_rect.h()):
                                best_rect = r
                    elif validate_target_candidate(img, r):
                        if best_rect is None or (r.w() * r.h()) > (best_rect.w() * best_rect.h()):
                            best_rect = r

            # 3. 状态机逻辑处理与 ROI 动态更新
            if best_rect is not None:
                set_gpio2_high(True)
                # --- 成功锁定了有效靶标 ---
                tracking_state = STATE_LOCKED
                coast_counter = 0
                roi_expand_steps = 0

                # 动态计算并更新下一帧的局部搜索区域 ROI
                search_roi = calculate_search_roi(best_rect)

                corners = best_rect.corners()
                t_cx, t_cy = get_target_center(corners)

                dx = t_cx - IMG_CENTER_X
                dy = t_cy - IMG_CENTER_Y

                angle_x_rad = math.atan((dx / (DETECT_WIDTH / 2.0)) * H_TAN_HALF_FOV)
                angle_y_rad = math.atan((dy / (DETECT_HEIGHT / 2.0)) * V_TAN_HALF_FOV)

                yaw_angle = math.degrees(angle_x_rad)
                pitch_angle = math.degrees(angle_y_rad)

                # 绘制靶标框和十字中心
                img.draw_rectangle([v for v in best_rect.rect()], color=255, thickness=2)
                img.draw_cross(t_cx, t_cy, color=255, size=15)

                # 绘制当前正在生效的局部 ROI 边框 (用细线框标记，非常直观)
                img.draw_rectangle(search_roi, color=255, thickness=1)

                # 辅助参考线和文本
                img.draw_cross(IMG_CENTER_X, IMG_CENTER_Y, color=255, size=10)
                img.draw_line(IMG_CENTER_X, IMG_CENTER_Y, t_cx, t_cy, color=255)
                img.draw_string(t_cx + 10, t_cy + 10, "dx:%d dy:%d" % (dx, dy), color=255, scale=2)
                img.draw_string(t_cx + 10, t_cy + 30, "Yaw:%.1f Pitch:%.1f" % (yaw_angle, pitch_angle), color=255, scale=2)

                # 缓存当前的锁死坐标，留作下一次丢失时滑行使用
                last_rect = [v for v in best_rect.rect()]
                last_cx, last_cy = t_cx, t_cy
                last_dx, last_dy = dx, dy
                last_yaw, last_pitch = yaw_angle, pitch_angle

                print(f"Target LOCKED (LAB ROI) -> dx: {dx:3d}, dy: {dy:3d} | Yaw: {yaw_angle:5.1f}°, Pitch: {pitch_angle:5.1f}° | ROI: {search_roi} | FPS: {fps_val:.1f}")

            else:
                set_gpio2_high(False)
                # --- 当前搜索范围内未能识别到有效靶标 (遮挡或运动出框) ---
                if tracking_state == STATE_LOCKED:
                    tracking_state = STATE_COASTING
                    coast_counter = MAX_COASTING_FRAMES
                elif tracking_state == STATE_COASTING:
                    coast_counter -= 1
                    if coast_counter <= 0:
                        if (not active_fullscreen) and roi_expand_steps < MAX_ROI_EXPAND_STEPS:
                            roi_expand_steps += 1
                            search_roi = expand_search_roi(search_roi, ROI_EXPAND_MARGIN)
                            coast_counter = MAX_COASTING_FRAMES
                            print(f"Target Reacquire Expand ROI [{roi_expand_steps}] -> ROI: {search_roi} | FPS: {fps_val:.1f}")
                        else:
                            # 扩展 ROI 后依然找不到，最后才退回全屏低频搜索
                            tracking_state = STATE_SEARCHING
                            roi_expand_steps = 0
                            search_roi = [0, 0, DETECT_WIDTH, DETECT_HEIGHT]

                if tracking_state == STATE_COASTING:
                    # Coast 状态下继续使用上一帧数据，并且绘制当前的局部搜索框
                    img.draw_rectangle(last_rect, color=255, thickness=2)
                    img.draw_cross(last_cx, last_cy, color=255, size=15)
                    img.draw_rectangle(search_roi, color=255, thickness=1) # 依然把绿框留在屏幕上

                    img.draw_cross(IMG_CENTER_X, IMG_CENTER_Y, color=255, size=10)
                    img.draw_line(IMG_CENTER_X, IMG_CENTER_Y, last_cx, last_cy, color=255)
                    img.draw_string(last_cx + 10, last_cy + 10, "dx:%d dy:%d (Coast)" % (last_dx, last_dy), color=255, scale=2)
                    img.draw_string(last_cx + 10, last_cy + 30, "Yaw:%.1f Pitch:%.1f" % (last_yaw, last_pitch), color=255, scale=2)

                    print(f"Target Coasting [{coast_counter}] -> Keep ROI: {search_roi} | FPS: {fps_val:.1f}")
                else:
                    # 搜寻状态下，把搜索框设回全屏，并提示
                    if run_rect_search:
                        img.draw_string(10, 40, "Searching Fullscreen...", color=255, scale=2)
                        print(f"Target Searching Fullscreen... | FPS: {fps_val:.1f}")
                    else:
                        img.draw_string(10, 40, "Searching Skip...", color=255, scale=2)
                        print(f"Target Searching Skip... | FPS: {fps_val:.1f}")

            # 4. 绘制 FPS
            img.draw_string(10, 10, "FPS: %.2f" % fps_val, color=255, scale=2)

            # 5. 显示到 LCD 全屏
            display_img.clear()
            display_img.draw_image(img, 0, 0, x_scale=LCD_X_SCALE, y_scale=LCD_Y_SCALE)
            Display.show_image(display_img)

            # 6. 内存回收
            rects = None
            best_rect = None
            img = None
            gc.collect()

        except KeyboardInterrupt as e:
            print("user stop")
            break
        except MemoryError as e:
            set_gpio2_high(False)
            tracking_state = STATE_SEARCHING
            coast_counter = 0
            roi_expand_steps = 0
            fullscreen_search_counter = 0
            search_roi = [0, 0, DETECT_WIDTH, DETECT_HEIGHT]
            last_rect = None
            last_cx = 0
            last_cy = 0
            last_dx = 0
            last_dy = 0
            last_yaw = 0.0
            last_pitch = 0.0
            rects = None
            best_rect = None
            img = None
            gc.collect()
            print("MemoryError: reset to throttled fullscreen LAB search and continue")
            continue
        except BaseException as e:
            import sys
            sys.print_exception(e)
            break

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_is_init = False
    try:
        print("--- rect10_lcd 启动 (Sensor id=2 + 16:9 LAB 动态 ROI LCD 版) ---")
        camera_init()
        camera_is_init = True
        print("camera capture start with LAB recognition")
        capture_picture()
    except Exception as e:
        import sys
        sys.print_exception(e)
    finally:
        if camera_is_init:
            print("camera deinit")
            camera_deinit()

if __name__ == "__main__":
    main()
