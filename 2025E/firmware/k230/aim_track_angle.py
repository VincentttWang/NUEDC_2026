# ------------------------------------------------------------------------
# aim_track_angle.py -- K230 角度误差追踪 (S7 视觉管线)
#
# cv_lite 灰度矩形检测 + SEARCH/LOCK/COAST 三态状态机 + ROI 跟踪门 +
# 几何/尺寸/时间三层筛选; 输出"靶心相对图像中心"的视场角误差。
# 输出层为角度域恒速 Kalman: 平滑检测噪声 + 3σ 新息门去野值 + COAST 短暂
# 丢帧速度外推桥接(仍 VALID) + 发送前按实测处理延迟前推(源头补链路延迟)。
# 激光偏移【同轴先置零】(IDEAL_*=0, CALIB_OFFSET=0), 真实偏移由 S10 光点标定填入。
#
# 协议: A5 5A status seq yaw_i16_be pitch_i16_be checksum, 单位 0.01 deg。
#   status: 0=VALID(锁定) 1=NOT_FOUND(从未锁定) 2=LOST(曾锁定,当前丢失/COAST)。
#   MCU 端 auto_aim 仅在新帧+VALID 时取误差喂视觉慢校正, 非 VALID 自动冻结 bias。
# 另配 27 字节 rect 帧 (0x12..0x5B) 供搜索/画圆读角点。
# ------------------------------------------------------------------------
import time, os, gc, math
import image
import cv_lite
from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin, FPIOA, UART

# ===== 图像与矩形检测 =====
# 传感器工作在原生 1920x1080(16:9) 全阵列: 避免 1280x960(4:3) 裁剪丢失约 1/3 水平 FOV
# (Sensor(320,240) 会落到 1280x960 裁剪模式)。处理分辨率取 16:9 的 480x270:
#   - 与 H/V_FOV(60x35, 本就是 16:9 全 FOV) 自洽, FOV/角度常量无需改;
#   - 缩放比 1920/480 = 1080/270 = 4, 与旧 1280x960->320x240 的 ÷4 相同, 靶像素
#     面积/ROI/跳变等检测阈值(1280x960 为裁剪窗、像素节距不变)维持原标定不必重调。
SENSOR_WIDTH = 1920
SENSOR_HEIGHT = 1080
DETECT_WIDTH = 480
DETECT_HEIGHT = 270
IMG_CX = DETECT_WIDTH // 2
IMG_CY = DETECT_HEIGHT // 2
CVLITE_SHAPE = [DETECT_HEIGHT, DETECT_WIDTH]

# cv_lite 矩形原语参数 (几何层的一部分: max_angle_cos 约束矩形度)。
CANNY_LO, CANNY_HI = 50, 150
APPROX_EPS = 0.04
AREA_MIN_RATIO = 0.002
MAX_ANGLE_COS = 0.3
GAUSS = 5

# ===== 三层筛选阈值 =====
# 几何层: 长宽比区间 (cv_lite 已保证矩形度)。
MIN_ASPECT = 0.4
MAX_ASPECT = 2.5
# 尺寸层: bbox 面积区间 (最强判据; 靶 area~16500 vs 背景噪声 <700, 现场标定)。
MIN_AREA = 1000
MAX_AREA = 60000
# 时间层: 锁定确认 / 跳变拒斥 / COAST 容忍。
LOCK_CONFIRM_FRAMES = 3     # SEARCH 中同一目标连续一致多少帧才 LOCK
COAST_MAX_FRAMES = 8        # LOCK 后连续丢失多少帧才回 SEARCH
SEARCH_JUMP_PX = 60         # SEARCH 确认时判"同一目标"的中心跳变上限
ROI_EXTRA_PX = 40           # ROI 门半径 = 0.5*max(bbox) + 该值
ROI_COAST_SCALE = 1.6       # COAST 时 ROI 半径放大 (目标可能已偏出)

# ===== 角度域恒速 Kalman (LOCK/COAST 输出层, 替代 EMA) =====
# 每轴独立 2 态 [角度, 角速度]: 平滑检测噪声 + 3σ 新息门去野值 +
# COAST 短暂丢帧用速度外推继续输出 + 发送前按实测处理延迟前推(补一半以上链路延迟)。
KF_MEAS_SIGMA_DEG = 0.15    # 测量噪声 σ (deg); 静态摆车录 10s 按散布重标
KF_ACCEL_SIGMA = 300.0      # 过程噪声: 视线角加速度 σ (deg/s²), 覆盖转弯机动
KF_GATE_SIGMA2 = 9.0        # 新息门 (3σ)²: 超出视为野值拒收
KF_GATE_REJECT_MAX = 3      # 连续拒收帧数上限: 达到则硬复位到测量 (世界真变了)
KF_COAST_VALID_FRAMES = 3   # COAST 前 N 帧仍以 VALID 输出预测值 (糊帧/短遮挡桥接)
KF_LEAD_BASE_MS = 25.0      # 延迟前推基础量 (曝光+读出+MCU 消费半周期的估计)
KF_R = KF_MEAS_SIGMA_DEG * KF_MEAS_SIGMA_DEG


class AxisKF:
    """单轴 2 态恒速 Kalman: a(deg), r(deg/s)。标量化实现, 30fps 开销可忽略。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.inited = False
        self.a = 0.0
        self.r = 0.0
        self.p00 = 0.0
        self.p01 = 0.0
        self.p11 = 0.0
        self.rejects = 0

    def start(self, z):
        self.inited = True
        self.a = z
        self.r = 0.0
        self.p00 = KF_R * 4.0
        self.p01 = 0.0
        self.p11 = 400.0        # 初始速度不确定 σ~20deg/s
        self.rejects = 0

    def predict(self, dt):
        if not self.inited or dt <= 0.0:
            return
        self.a += self.r * dt
        p00 = self.p00 + dt * (2.0 * self.p01 + dt * self.p11)
        p01 = self.p01 + dt * self.p11
        q = KF_ACCEL_SIGMA * KF_ACCEL_SIGMA
        dt2 = dt * dt
        self.p00 = p00 + q * dt2 * dt2 * 0.25
        self.p01 = p01 + q * dt2 * dt * 0.5
        self.p11 = self.p11 + q * dt2

    def update(self, z):
        if not self.inited:
            self.start(z)
            return True
        nu = z - self.a
        s = self.p00 + KF_R
        if nu * nu > KF_GATE_SIGMA2 * s:
            # 野值拒收; 连续多帧超门 -> 目标真移动了, 硬复位跟上
            self.rejects += 1
            if self.rejects >= KF_GATE_REJECT_MAX:
                self.start(z)
            return False
        self.rejects = 0
        k0 = self.p00 / s
        k1 = self.p01 / s
        self.a += k0 * nu
        self.r += k1 * nu
        p00 = (1.0 - k0) * self.p00
        p01 = (1.0 - k0) * self.p01
        p11 = self.p11 - k1 * self.p01
        self.p00, self.p01, self.p11 = p00, p01, p11
        return True

    def output(self, lead_s):
        return self.a + self.r * lead_s

# 摄像头倒装补偿: 只影响发给 MSPM0 的控制坐标 (角度误差)。
MIRROR_180 = True

# ===== 角度模型 (视场角) =====
H_FOV_DEG = 60.0
V_FOV_DEG = 35.0
H_TAN_HALF_FOV = math.tan(math.radians(H_FOV_DEG / 2.0))
V_TAN_HALF_FOV = math.tan(math.radians(V_FOV_DEG / 2.0))

# ===== 激光-相机偏移: S10 光点标定实测值 (laser_offset_calib.py, 2026-07-15) =====
# 实测: yaw 光斑角恒定 -1.86°(无视差); pitch 随距离 1.66(远)~3.16(近) -> 竖直视差,
# 尺量镜头中心-出光口竖直间距 ~1.5cm(与反解量级吻合, 采信物理量测)。
TARGET_HEIGHT_CM = 17.3     # 靶实际高度 (cm), 供 CALIB_OFFSET 的像素换算
CALIB_OFFSET_X_CM = 0.0     # 横向: yaw 无视差, 保持 0
CALIB_OFFSET_Y_CM = -1.5    # 竖直视差 1.5cm; 符号经镜像链推导(补偿加在镜像前的原始
                            # 坐标, MIRROR_180 翻转其作用方向)。上板验证: 近距(0.5m)
                            # 收敛后光斑若竖直偏离靶心且越近越偏 -> 翻此符号。
IDEAL_YAW_DEG = -0.32       # 光斑 yaw 纯角度偏置 (无距离依赖)
IDEAL_PITCH_DEG = 0.73      # 光斑 pitch 纯角度偏置: 由 d_y=1.5cm 从近端(1.44)/远端
                            # (1.09)双向锚定取中; 残差 <0.4° 由视觉 bias 吸收

# ===== UART1 (GPIO3/4, 对接 MSPM0 UART2) =====
UART_TX_PIN, UART_RX_PIN = 3, 4
UART_BAUD = 115200
FRAME_SOF0, FRAME_SOF1 = 0xA5, 0x5A
ANGLE_SCALE = 100.0
ANGLE_STATUS_VALID = 0
ANGLE_STATUS_NOT_FOUND = 1
ANGLE_STATUS_LOST = 2
VISION_FRAME_START, VISION_FRAME_END = 0x12, 0x5B

# ===== 激光与 LCD =====
LASER_PIN = 2
DISPLAY_LCD = True          # 现场肉眼确认锁定; headless 可提速到 77-87fps
LCD_WIDTH, LCD_HEIGHT = 800, 480
DISPLAY_INTERVAL = 2

DEBUG = True
LOG_INTERVAL = 30

# ===== 状态机 =====
STATE_SEARCH, STATE_LOCK, STATE_COAST = 0, 1, 2
STATE_NAME = ("SEARCH", "LOCK", "COAST")

uart = None
laser = None
sensor = None
display_img = None
laser_requested = False
laser_command_buffer = bytearray()
angle_tx_seq = 0

# 追踪状态
tk_state = STATE_SEARCH
tk_last_center = None        # 上帧原始图像坐标下的靶心 (ROI/跳变判据基准)
tk_last_bbox = (0, 0)
tk_out_center = None         # 最近测得的靶心 (原始坐标, 供显示/ROI 可视化)
tk_confirm_center = None
tk_confirm_count = 0
tk_coast_count = 0
tk_target_seen = False       # 是否曾经锁定 (区分 NOT_FOUND / LOST)

# 角度域 KF (输出层): 平滑/野值门/COAST 预测/延迟前推
kf_yaw = AxisKF()
kf_pitch = AxisKF()


def dist2(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def mirror_pt(x, y):
    if not MIRROR_180:
        return int(x), int(y)
    mx = max(0, min(DETECT_WIDTH - 1, 2 * IMG_CX - int(x)))
    my = max(0, min(DETECT_HEIGHT - 1, 2 * IMG_CY - int(y)))
    return mx, my


def diag_center(corners):
    # 两条对角线交点; 退化时取四点平均。
    x1, y1 = corners[0]; x2, y2 = corners[2]
    x3, y3 = corners[1]; x4, y4 = corners[3]
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if -1e-3 < den < 1e-3:
        return ((x1 + x2 + x3 + x4) // 4, (y1 + y2 + y3 + y4) // 4)
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
    return int(px), int(py)


def filter_candidates(rects):
    # 几何 + 尺寸两层筛选 -> [(rect, corners, (cx,cy), area), ...]。
    out = []
    if not rects:
        return out
    for r in rects:
        w, h = r[2], r[3]
        if w <= 0 or h <= 0:
            continue
        area = w * h
        if area < MIN_AREA or area > MAX_AREA:
            continue
        aspect = float(w) / float(h)
        if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
            continue
        corners = [(r[4], r[5]), (r[6], r[7]), (r[8], r[9]), (r[10], r[11])]
        out.append((r, corners, diag_center(corners), area))
    return out


def pick_largest(cands):
    best = None
    for c in cands:
        if best is None or c[3] > best[3]:
            best = c
    return best


def pick_largest_in_roi(cands, center, radius):
    r2 = radius * radius
    best = None
    for c in cands:
        if dist2(c[2], center) > r2:
            continue
        if best is None or c[3] > best[3]:
            best = c
    return best


def roi_radius(bbox):
    return 0.5 * float(max(bbox[0], bbox[1])) + ROI_EXTRA_PX


def track_update(cands):
    # 三态状态机: 返回 (status, best_candidate_or_None)。
    # SEARCH: 全帧选最大, 同一目标连续 N 帧一致 -> LOCK。
    # LOCK  : 只收 ROI 门内候选, 输出 EMA 平滑靶心; 丢失 -> COAST。
    # COAST : 短暂丢失保持并放大 ROI 找回; 超时 -> SEARCH。
    global tk_state, tk_last_center, tk_last_bbox, tk_out_center
    global tk_confirm_center, tk_confirm_count, tk_coast_count, tk_target_seen

    if tk_state == STATE_SEARCH:
        best = pick_largest(cands)
        if best is not None:
            c = best[2]
            if tk_confirm_center is not None and dist2(c, tk_confirm_center) <= SEARCH_JUMP_PX * SEARCH_JUMP_PX:
                tk_confirm_count += 1
            else:
                tk_confirm_count = 1
            tk_confirm_center = c
            if tk_confirm_count >= LOCK_CONFIRM_FRAMES:
                tk_state = STATE_LOCK
                tk_last_center = c
                tk_last_bbox = (best[0][2], best[0][3])
                tk_out_center = c
                tk_coast_count = 0
                tk_target_seen = True
                return ANGLE_STATUS_VALID, best
        else:
            tk_confirm_count = 0
            tk_confirm_center = None
        return (ANGLE_STATUS_LOST if tk_target_seen else ANGLE_STATUS_NOT_FOUND), None

    # LOCK / COAST: ROI 门跟踪
    radius = roi_radius(tk_last_bbox)
    if tk_state == STATE_COAST:
        radius *= ROI_COAST_SCALE
    best = pick_largest_in_roi(cands, tk_last_center, radius)
    if best is not None:
        c = best[2]
        tk_out_center = c            # 原始测量; 平滑/野值交给角度域 KF
        tk_last_center = c
        tk_last_bbox = (best[0][2], best[0][3])
        tk_coast_count = 0
        tk_state = STATE_LOCK
        return ANGLE_STATUS_VALID, best

    tk_coast_count += 1
    if tk_coast_count > COAST_MAX_FRAMES:
        tk_state = STATE_SEARCH
        tk_confirm_count = 0
        tk_confirm_center = None
        return (ANGLE_STATUS_LOST if tk_target_seen else ANGLE_STATUS_NOT_FOUND), None
    tk_state = STATE_COAST
    return ANGLE_STATUS_LOST, None


def append_i16_be(frame, value):
    value = max(-32768, min(32767, int(round(value)))) & 0xFFFF
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)


def append_u16_be(frame, value):
    value = max(0, min(65535, int(value)))
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)


def normalize_rect_corners(corners):
    # MCU 几何层约定: [左下、右下、右上、左上]。
    ordered_y = sorted(corners, key=lambda point: point[1])
    top = sorted(ordered_y[:2], key=lambda point: point[0])
    bottom = sorted(ordered_y[2:], key=lambda point: point[0])
    return [bottom[0], bottom[1], top[1], top[0]]


def send_angle_frame(yaw_err_deg, pitch_err_deg, status):
    global angle_tx_seq
    if uart is None:
        return
    frame = bytearray()
    frame.append(FRAME_SOF0)
    frame.append(FRAME_SOF1)
    frame.append(status & 0xFF)
    frame.append(angle_tx_seq & 0xFF)
    angle_tx_seq = (angle_tx_seq + 1) & 0xFF
    append_i16_be(frame, yaw_err_deg * ANGLE_SCALE)
    append_i16_be(frame, pitch_err_deg * ANGLE_SCALE)
    frame.append(sum(frame) & 0xFF)
    uart.write(frame)


def send_vision_frame(target, corners):
    if uart is None:
        return
    frame = bytearray()
    frame.append(VISION_FRAME_START)
    frame.append(0)
    append_u16_be(frame, target[0])
    append_u16_be(frame, target[1])
    append_u16_be(frame, IMG_CX)
    append_u16_be(frame, IMG_CY)
    for x, y in normalize_rect_corners(corners):
        append_u16_be(frame, x)
        append_u16_be(frame, y)
    frame.append(VISION_FRAME_END)
    uart.write(frame)


def angle_error(target_x, target_y):
    # 靶心相对图像中心 -> 视场角 -> 相对理想(激光)指向的误差。同轴时 IDEAL_*=0。
    dx = target_x - IMG_CX
    dy = target_y - IMG_CY
    yaw_deg = math.degrees(math.atan((dx / (DETECT_WIDTH / 2.0)) * H_TAN_HALF_FOV))
    pitch_deg = math.degrees(math.atan((dy / (DETECT_HEIGHT / 2.0)) * V_TAN_HALF_FOV))
    return yaw_deg, pitch_deg, yaw_deg - IDEAL_YAW_DEG, pitch_deg - IDEAL_PITCH_DEG


def calib_target(center, bbox_h):
    # 激光-相机不同心时按物理偏移做像素补偿; 同轴(偏移0)时即靶心。
    if TARGET_HEIGHT_CM > 0 and (CALIB_OFFSET_X_CM != 0.0 or CALIB_OFFSET_Y_CM != 0.0):
        ppc = float(bbox_h) / TARGET_HEIGHT_CM
        return (int(center[0] - CALIB_OFFSET_X_CM * ppc),
                int(center[1] - CALIB_OFFSET_Y_CM * ppc))
    return (int(center[0]), int(center[1]))


def set_laser(on):
    if laser is not None:
        laser.value(1 if on else 0)


def poll_laser_command():
    # MSPM0 仅发 "LASER=0\n" 或 "LASER=1\n"; 逐字节累积兼容分段接收。
    global laser_requested, laser_command_buffer
    if uart is None or uart.any() <= 0:
        return
    data = uart.read()
    if not data:
        return
    for byte in data:
        if byte == 10 or byte == 13:
            if laser_command_buffer == b"LASER=1":
                laser_requested = True
            elif laser_command_buffer == b"LASER=0":
                laser_requested = False
            laser_command_buffer = bytearray()
        elif len(laser_command_buffer) < 16:
            laser_command_buffer.append(byte)
        else:
            laser_command_buffer = bytearray()


def camera_init():
    global sensor, uart, laser, display_img
    # 用原生全阵列构造 (1920x1080 全 FOV), 再由 ISP 缩放到处理分辨率; 若直接用
    # DETECT_WIDTH/HEIGHT 构造会落到 1280x960 裁剪模式而丢失水平 FOV。
    sensor = Sensor(width=SENSOR_WIDTH, height=SENSOR_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    if DISPLAY_LCD:
        Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
        display_img = image.Image(LCD_WIDTH, LCD_HEIGHT, image.RGB565)
    MediaManager.init()
    sensor.run()
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, baudrate=UART_BAUD,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    fpioa.set_function(LASER_PIN, FPIOA.GPIO2)
    laser = Pin(LASER_PIN, Pin.OUT)
    set_laser(False)


def camera_deinit():
    set_laser(False)
    try:
        if sensor:
            sensor.stop()
    except Exception:
        pass
    try:
        if DISPLAY_LCD:
            Display.deinit()
    except Exception:
        pass
    try:
        MediaManager.deinit()
    except Exception:
        pass


def show(img):
    if not DISPLAY_LCD or display_img is None:
        return
    display_img.clear()
    display_img.draw_image(img, 0, 0,
                           x_scale=LCD_WIDTH / DETECT_WIDTH,
                           y_scale=LCD_HEIGHT / DETECT_HEIGHT)
    Display.show_image(display_img)


def draw_overlay(img, state, best, out_center, target, yaw_err, pitch_err, fps):
    img.draw_cross(IMG_CX, IMG_CY, color=255, size=10, thickness=1)
    if state != STATE_SEARCH and out_center is not None:
        # ROI 门可视化
        radius = int(roi_radius(tk_last_bbox) * (ROI_COAST_SCALE if state == STATE_COAST else 1.0))
        img.draw_circle(int(tk_last_center[0]), int(tk_last_center[1]), radius, color=255, thickness=1)
    if best is not None:
        corners = best[1]
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) & 3]
            img.draw_line(int(x0), int(y0), int(x1), int(y1), color=255, thickness=2)
        img.draw_cross(int(out_center[0]), int(out_center[1]), color=255, size=12, thickness=2)
        img.draw_circle(int(target[0]), int(target[1]), 4, color=255, fill=True)
        img.draw_line(IMG_CX, IMG_CY, int(target[0]), int(target[1]), color=255, thickness=1)
        img.draw_string_advanced(4, 26, 18, "Y %.2f P %.2f" % (yaw_err, pitch_err), color=255)
    img.draw_string_advanced(4, 4, 18, STATE_NAME[state], color=255)
    img.draw_string_advanced(4, 48, 18, "FPS:%.1f" % fps, color=255)


def main():
    global sensor
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_init()
    frame_count = 0
    fps = 0.0
    fps_window_start_ms = time.ticks_ms()
    fps_window_count = 0
    prev_frame_ms = time.ticks_ms()   # KF dt 基准
    try:
        while True:
            os.exitpoint()
            poll_laser_command()
            img = sensor.snapshot()
            fps_window_count += 1
            now_ms = time.ticks_ms()
            fps_elapsed_ms = time.ticks_diff(now_ms, fps_window_start_ms)
            if fps_elapsed_ms >= 1000:
                fps = fps_window_count * 1000.0 / fps_elapsed_ms
                fps_window_start_ms = now_ms
                fps_window_count = 0

            t_cap_ms = time.ticks_ms()   # 本帧捕获基准 (延迟前推用)
            kf_dt = time.ticks_diff(t_cap_ms, prev_frame_ms) / 1000.0
            prev_frame_ms = t_cap_ms
            if kf_dt <= 0.0 or kf_dt > 0.2:
                kf_dt = 0.033

            rects = cv_lite.grayscale_find_rectangles_with_corners(
                CVLITE_SHAPE, img.to_numpy_ref(), CANNY_LO, CANNY_HI,
                APPROX_EPS, AREA_MIN_RATIO, MAX_ANGLE_COS, GAUSS)
            cands = filter_candidates(rects)
            status, best = track_update(cands)

            raw_target = None
            yaw_err = 0.0
            pitch_err = 0.0
            if status == ANGLE_STATUS_VALID and best is not None:
                # 测量: 本帧原始靶心 -> 角度; KF 预测+新息门更新 (拒收野值时输出预测)
                raw_target = calib_target(best[2], best[0][3])
                target = mirror_pt(raw_target[0], raw_target[1])
                _, _, z_yaw, z_pitch = angle_error(target[0], target[1])
                kf_yaw.predict(kf_dt)
                kf_pitch.predict(kf_dt)
                kf_yaw.update(z_yaw)
                kf_pitch.update(z_pitch)
                lead_s = (time.ticks_diff(time.ticks_ms(), t_cap_ms) + KF_LEAD_BASE_MS) / 1000.0
                yaw_err = kf_yaw.output(lead_s)
                pitch_err = kf_pitch.output(lead_s)
                send_angle_frame(yaw_err, pitch_err, ANGLE_STATUS_VALID)
                send_vision_frame(target, [mirror_pt(x, y) for x, y in best[1]])
            elif (tk_state == STATE_COAST and kf_yaw.inited
                  and tk_coast_count <= KF_COAST_VALID_FRAMES):
                # 短暂丢帧: 速度外推桥接, 仍以 VALID 输出 (超过 N 帧转 LOST, MCU 冻结)
                kf_yaw.predict(kf_dt)
                kf_pitch.predict(kf_dt)
                lead_s = (time.ticks_diff(time.ticks_ms(), t_cap_ms) + KF_LEAD_BASE_MS) / 1000.0
                yaw_err = kf_yaw.output(lead_s)
                pitch_err = kf_pitch.output(lead_s)
                send_angle_frame(yaw_err, pitch_err, ANGLE_STATUS_VALID)
                send_vision_frame((0, 0), [(0, 0), (0, 0), (0, 0), (0, 0)])
            else:
                if tk_state == STATE_SEARCH:
                    kf_yaw.reset()
                    kf_pitch.reset()
                send_angle_frame(0.0, 0.0, status)
                send_vision_frame((0, 0), [(0, 0), (0, 0), (0, 0), (0, 0)])
            set_laser(laser_requested)

            if DISPLAY_LCD and (frame_count % DISPLAY_INTERVAL == 0):
                draw_overlay(img, tk_state, best, tk_out_center,
                             raw_target if (status == ANGLE_STATUS_VALID and best is not None) else None,
                             yaw_err, pitch_err, fps)
                show(img)

            frame_count += 1
            if DEBUG and (frame_count % LOG_INTERVAL == 0):
                if status == ANGLE_STATUS_VALID:
                    print("%s c=(%d,%d) err=(%.2f,%.2f) fps=%.1f" %
                          (STATE_NAME[tk_state], tk_out_center[0], tk_out_center[1],
                           yaw_err, pitch_err, fps))
                else:
                    print("%s (no valid) fps=%.1f" % (STATE_NAME[tk_state], fps))
            img = None
            if frame_count % 30 == 0:
                gc.collect()
    except KeyboardInterrupt:
        pass
    except BaseException as exc:
        import sys
        sys.print_exception(exc)
    finally:
        camera_deinit()


if __name__ == "__main__":
    main()
