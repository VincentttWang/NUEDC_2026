# ------------------------------------------------------------------------
# rect_07.py
# 基于 rect_04.py (形态学滤波版) 的动态局部 ROI 追踪版 (方案七)：
# 1. 放弃 rect_05 的全图寻找+移动距离限制方案，改用动态感兴趣区域 (ROI) 局部搜索。
# 2. 初始状态 (SEARCHING) 全图寻找靶标。一旦通过初筛（几何过滤与内部像素均值校验），
#    则在下一帧将识别区域限制在略大于靶标矩形尺寸 of 局部 ROI 内。
# 3. 图像二值化、形态学膨胀/腐蚀、以及矩形搜寻全部局限在局部 ROI 内进行。
#    * 优势：像素处理量从 76800 暴降至 15000 左右，大幅提升处理速度(FPS)；
#    * 优势：外部背景噪声（如其他远处的矩形、光斑）被彻底物理隔离，不会产生任何干扰。
# 4. 支持丢失滑行机制 (COASTING)：若在局部 ROI 发生短暂丢失，维持该 ROI 滑行 3 帧；
#    若持续丢失，则将 ROI 重置为全图，重回 SEARCHING 状态重新捕捉。
# ------------------------------------------------------------------------
import time, os, gc, sys, math

from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin
from machine import FPIOA
from machine import UART

# --- 摄像头视场角 (FOV) 预估值 ---
H_FOV_DEG = 60.0
V_FOV_DEG = 35.0

# 预计算 FOV 相关的常数
H_TAN_HALF_FOV = math.tan(math.radians(H_FOV_DEG / 2.0))
V_TAN_HALF_FOV = math.tan(math.radians(V_FOV_DEG / 2.0))

# 1. 严格同步跑通的分辨率设置
DETECT_WIDTH = ALIGN_UP(320, 16)
DETECT_HEIGHT = 240

IMG_CENTER_X = DETECT_WIDTH // 2
IMG_CENTER_Y = DETECT_HEIGHT // 2

# --- MSPM0 UART 通信配置 ---
# 默认使用 K230 UART2，115200 8N1。若实际接线使用其他 FPIOA 引脚，只需修改
# K230_UART_TX_PIN / K230_UART_RX_PIN，不需要改动帧协议。
K230_UART_TX_PIN = 11
K230_UART_RX_PIN = 12
K230_UART_BAUDRATE = 115200

FRAME_START = 0x12
FRAME_END = 0x5B

uart2 = None

# 用户要求的二值化阈值
target_threshold = (66, 163)

# --- 状态机定义 ---
STATE_SEARCHING = 0
STATE_LOCKED = 1
STATE_COASTING = 2

# --- 靶标验证与局部 ROI 追踪配置 (方案七核心) ---
MIN_ASPECT_RATIO = 1.1      # A4靶标最小长宽比
MAX_ASPECT_RATIO = 1.8      # A4靶标最大长宽比
MIN_AREA = 3000             # A4靶标在 320x240 分辨率下的最小面积（像素）
MAX_AREA = 35000            # A4靶标在 320x240 分辨率下的最大面积（像素）
MIN_DENSITY_MEAN = 170      # 靶标内部二值化后白色像素平均亮度阈值

# ROI 局部追踪参数
ROI_MARGIN = 25             # 局部搜索框在靶标矩形四周外扩的像素余量 (防止目标移动出框)
MAX_COASTING_FRAMES = 3    # 目标短暂丢失时的最大维持帧数

sensor = None
gpio2_pin = None

def uart_init():
    global uart2
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(K230_UART_RX_PIN, FPIOA.UART2_RXD)
    uart2 = UART(UART.UART2,
                 baudrate=K230_UART_BAUDRATE,
                 bits=UART.EIGHTBITS,
                 parity=UART.PARITY_NONE,
                 stop=UART.STOPBITS_ONE)

def append_u16_be(frame, value):
    value = int(value)
    if value < 0:
        value = 0
    if value > 65535:
        value = 65535
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)

def normalize_rect_corners(corners):
    if not corners or len(corners) < 4:
        return [(0, 0), (0, 0), (0, 0), (0, 0)]

    sorted_by_y = sorted(corners, key=lambda p: p[1])
    top = sorted(sorted_by_y[:2], key=lambda p: p[0])
    bottom = sorted(sorted_by_y[2:], key=lambda p: p[0])

    bottom_left = bottom[0]
    bottom_right = bottom[1]
    top_right = top[1]
    top_left = top[0]
    return [bottom_left, bottom_right, top_right, top_left]

def send_vision_frame(target_x=0, target_y=0, laser_x=0, laser_y=0, corners=None):
    if uart2 is None:
        return

    frame = bytearray()
    frame.append(FRAME_START)
    frame.append(0x00)

    # Laser 段沿用旧协议含义：目标中心 x/y + 激光点 x/y。
    # 当前脚本尚未识别激光点，暂用图像中心作为激光点，使 MSPM0 可执行“相机中心对准靶心”的闭环。
    append_u16_be(frame, target_x)
    append_u16_be(frame, target_y)
    append_u16_be(frame, laser_x)
    append_u16_be(frame, laser_y)

    for x, y in normalize_rect_corners(corners):
        append_u16_be(frame, x)
        append_u16_be(frame, y)

    frame.append(FRAME_END)
    uart2.write(frame)

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
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()

    # set chn0 output size
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    # set chn0 output format (纯灰度极速)
    sensor.set_pixformat(Sensor.GRAYSCALE)

    # use IDE as display output
    Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=100, to_ide=True)
    # init media manager
    MediaManager.init()
    # sensor start run
    sensor.run()
    gpio_init()
    uart_init()

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
            # 1. 抓取原始灰度图像
            img = sensor.snapshot()

            # 2. 图像处理与算法运行 (全部局限在 active_roi 内，大幅降低 CPU 开销)
            # 对局部/全局搜索区域进行二值化
            img.binary([target_threshold], roi=search_roi)

            # 在同一搜索区域执行形态学滤波
            img.dilate(1, roi=search_roi)
            img.erode(1, roi=search_roi)

            # 运行矩形识别，同样只限制在搜索区域内
            rects = img.find_rects(threshold=8000, roi=search_roi)

            best_rect = None

            if rects:
                # 过滤出符合靶标特征 of 矩形
                valid_candidates = []
                for r in rects:
                    if validate_target_candidate(img, r):
                        valid_candidates.append(r)

                if valid_candidates:
                    # 信任通过过滤且面积最大的矩形
                    best_rect = max(valid_candidates, key=lambda r: r.w() * r.h())

            # 3. 状态机逻辑处理与 ROI 动态更新
            if best_rect is not None:
                set_gpio2_high(True)
                # --- 成功锁定了有效靶标 ---
                tracking_state = STATE_LOCKED
                coast_counter = 0

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

                send_vision_frame(t_cx, t_cy, IMG_CENTER_X, IMG_CENTER_Y, corners)

                print(f"Target LOCKED (ROI) -> dx: {dx:3d}, dy: {dy:3d} | Yaw: {yaw_angle:5.1f}°, Pitch: {pitch_angle:5.1f}° | ROI: {search_roi} | FPS: {fps_val:.1f}")

            else:
                set_gpio2_high(False)
                # --- 当前搜索范围内未能识别到有效靶标 (遮挡或运动出框) ---
                if tracking_state == STATE_LOCKED:
                    tracking_state = STATE_COASTING
                    coast_counter = MAX_COASTING_FRAMES
                elif tracking_state == STATE_COASTING:
                    coast_counter -= 1
                    if coast_counter <= 0:
                        # 维持局部搜索依然找不到，说明目标彻底丢失，重置为全图搜索
                        tracking_state = STATE_SEARCHING
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
                    send_vision_frame(last_cx, last_cy, IMG_CENTER_X, IMG_CENTER_Y, None)

                    print(f"Target Coasting [{coast_counter}] -> Keep ROI: {search_roi} | FPS: {fps_val:.1f}")
                else:
                    # 搜寻状态下，把搜索框设回全屏，并提示
                    img.draw_string(10, 40, "Searching Fullscreen...", color=255, scale=2)
                    send_vision_frame()
                    print(f"Target Searching Fullscreen... | FPS: {fps_val:.1f}")

            # 4. 绘制 FPS
            img.draw_string(10, 10, "FPS: %.2f" % fps_val, color=255, scale=2)

            # 5. 显示到屏幕
            Display.show_image(img)

            # 6. 内存回收
            img = None
            gc.collect()

        except KeyboardInterrupt as e:
            print("user stop")
            break
        except BaseException as e:
            import sys
            sys.print_exception(e)
            break

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_is_init = False
    try:
        print("--- rect_07 启动 (动态局部 ROI 追踪版) ---")
        camera_init()
        camera_is_init = True
        print("camera capture start with recognition")
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
