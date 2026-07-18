#include "gimbal_tracking.h"

#include "bsp_time.h"
#include "geometry/geometry.h"
#include "gimbal.h"

#include <stddef.h>
#include <string.h>

static GIMBAL_TRACKING_CONFIG s_gimbal_tracking_config;
static GIMBAL_TRACKING_STATE s_gimbal_tracking_state;
static PID_CONTROLLER s_yaw_pid;
static PID_CONTROLLER s_pitch_pid;
static PID_CONTROLLER s_yaw_angle_pid;
static PID_CONTROLLER s_pitch_angle_pid;
static bool s_gimbal_tracking_initialized;
static uint32_t s_last_frame_ms;
static uint32_t s_last_angle_frame_count;
static bool s_angle_protocol_active;

static GIMBAL_TRACKING_CONFIG GimbalTracking_DefaultConfig(void){
    PID_CONFIG yaw_pid_config = {
        .kp = GIMBAL_TRACKING_DEFAULT_YAW_PID_KP,
        .ki = GIMBAL_TRACKING_DEFAULT_YAW_PID_KI,
        .kd = GIMBAL_TRACKING_DEFAULT_YAW_PID_KD,
        .integral_limit = GIMBAL_TRACKING_DEFAULT_YAW_PID_INTEGRAL_LIMIT,
        .output_limit = GIMBAL_TRACKING_DEFAULT_YAW_PID_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };
    PID_CONFIG pitch_pid_config = {
        .kp = GIMBAL_TRACKING_DEFAULT_PITCH_PID_KP,
        .ki = GIMBAL_TRACKING_DEFAULT_PITCH_PID_KI,
        .kd = GIMBAL_TRACKING_DEFAULT_PITCH_PID_KD,
        .integral_limit = GIMBAL_TRACKING_DEFAULT_PITCH_PID_INTEGRAL_LIMIT,
        .output_limit = GIMBAL_TRACKING_DEFAULT_PITCH_PID_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };
    PID_CONFIG yaw_angle_pid_config = yaw_pid_config;
    PID_CONFIG pitch_angle_pid_config = pitch_pid_config;

    yaw_angle_pid_config.kp = GIMBAL_TRACKING_DEFAULT_ANGLE_YAW_PID_KP;
    yaw_angle_pid_config.ki = GIMBAL_TRACKING_DEFAULT_ANGLE_YAW_PID_KI;
    yaw_angle_pid_config.kd = GIMBAL_TRACKING_DEFAULT_ANGLE_YAW_PID_KD;
    yaw_angle_pid_config.integral_limit = GIMBAL_TRACKING_DEFAULT_ANGLE_YAW_PID_INTEGRAL_LIMIT;
    pitch_angle_pid_config.kp = GIMBAL_TRACKING_DEFAULT_ANGLE_PITCH_PID_KP;
    pitch_angle_pid_config.ki = GIMBAL_TRACKING_DEFAULT_ANGLE_PITCH_PID_KI;
    pitch_angle_pid_config.kd = GIMBAL_TRACKING_DEFAULT_ANGLE_PITCH_PID_KD;
    pitch_angle_pid_config.integral_limit = GIMBAL_TRACKING_DEFAULT_ANGLE_PITCH_PID_INTEGRAL_LIMIT;
    GIMBAL_TRACKING_CONFIG config = {
        .yaw_pid = yaw_pid_config,
        .pitch_pid = pitch_pid_config,
        .yaw_angle_pid = yaw_angle_pid_config,
        .pitch_angle_pid = pitch_angle_pid_config,
        .image_height = GIMBAL_TRACKING_DEFAULT_IMAGE_HEIGHT,
        .paper_width = GIMBAL_TRACKING_DEFAULT_PAPER_WIDTH,
        .paper_height = GIMBAL_TRACKING_DEFAULT_PAPER_HEIGHT,
        .circle_radius = GIMBAL_TRACKING_DEFAULT_CIRCLE_RADIUS,
        /* 输出方向系数: 隔离电机安装方向 (实机标定)。
         * 实测: yaw 需反向 (-1); pitch 方向与 yaw 相反, 取 +1。 */
        .yaw_output_sign = -1.0f,
        .pitch_output_sign = 1.0f,
        .link_timeout_ms = GIMBAL_TRACKING_DEFAULT_LINK_TIMEOUT_MS,
    };

    return config;
}

static void GimbalTracking_EnsureInitialized(void){
    if (!s_gimbal_tracking_initialized){
        GimbalTracking_Init(NULL);
    }
}

static CORE_POINT2F GimbalTracking_ImagePoint(uint16_t x, uint16_t y){
    /*
     * 当前 K230 端已经在发送前处理摄像头倒装问题，并按 320x240 图像坐标
     * 直接发送目标点和图像中心。MCU 侧不再额外翻转 y 轴，避免二次修正。
     */
    CORE_POINT2F point = {
        .x = (float)x,
        .y = (float)y,
    };

    return point;
}

static BSP_STATUS GimbalTracking_HandleTargetNotReady(void){
    uint32_t now_ms = BSP_Time_GetMs();

    /*
     * 目标丢失(Lost): 立即把云台速度置 0 并停转, 但保持继续接收数据 —— 下次收到
     * 有效目标(Lock)会在 UpdateXxx 中直接恢复跟踪。
     * 注意: 旧实现先做 CheckLinkTimeout 早退, 一旦超时 s_last_frame_ms 不再更新,
     * 导致永久判超时、Lock 后也无法恢复(这正是"Lost 后不再追踪"的根因); 现改为
     * 每帧都先读目标, 丢失才走此处停转, 不再阻塞恢复。
     */
    s_gimbal_tracking_state.target_valid = false;
    s_gimbal_tracking_state.laser_valid = false;
    s_gimbal_tracking_state.yaw_speed = 0.0f;

    /* 超过 link_timeout_ms 未见有效目标才置超时标志(供 UI 显示/E2 判定), 不阻塞恢复。 */
    if ((s_gimbal_tracking_config.link_timeout_ms != 0U) &&
        ((now_ms - s_last_frame_ms) >= s_gimbal_tracking_config.link_timeout_ms)){
        s_gimbal_tracking_state.link_timeout = true;
    }

    /* 丢失目标: yaw 停转, pitch 停在当前位置保持。 */
    (void)Gimbal_HoldOnTargetLost();
    s_gimbal_tracking_state.pitch_speed = Gimbal_GetSpeed().pitch_deg_s;

    /* 清零角度 PID 积分: 丢失期间不再累积, 重获目标从新鲜误差起步, 避免陈旧积分甩动。
     * (像素模式不用这两个控制器, 清零无副作用。) */
    s_yaw_angle_pid.state.integral = 0.0f;
    s_pitch_angle_pid.state.integral = 0.0f;
    return BSP_STATUS_NOT_READY;
}

static void GimbalTracking_MarkTargetReady(void){
    s_last_frame_ms = BSP_Time_GetMs();
    s_gimbal_tracking_state.link_timeout = false;
}

static BSP_STATUS GimbalTracking_ReadLaser(CORE_POINT2F *target,
                                           CORE_POINT2F *laser){
    const CANMV_TARGET_DATA *data = CanMvUart_GetTargetData(CANMV_TARGET_LASER);

    s_gimbal_tracking_state.laser_status = CanMvUart_GetStatus(CANMV_TARGET_LASER);

    if ((data == NULL) || (data->status != CANMV_STATUS_OK) || (data->count < 4U)){
        s_gimbal_tracking_state.laser_valid = false;
        return BSP_STATUS_NOT_READY;
    }

    /* 前两个值为目标中心，旧协议用 0,0 表示未识别到目标。 */
    if ((data->value[0] == 0U) || (data->value[1] == 0U)){
        s_gimbal_tracking_state.target_valid = false;
        return BSP_STATUS_NOT_READY;
    }

    if (target != NULL){
        *target = GimbalTracking_ImagePoint(data->value[0], data->value[1]);
    }

    if (laser != NULL){
        *laser = GimbalTracking_ImagePoint(data->value[2], data->value[3]);
    }

    s_gimbal_tracking_state.target_valid = true;
    s_gimbal_tracking_state.laser_valid = true;

    return BSP_STATUS_OK;
}

static BSP_STATUS GimbalTracking_ReadAngleError(float *yaw_error_deg,
                                                 float *pitch_error_deg){
    const CANMV_TARGET_DATA *data = CanMvUart_GetTargetData(CANMV_TARGET_ANGLE);

    if ((data == NULL) || (data->status != CANMV_STATUS_OK) || (data->count < 2U)){
        return BSP_STATUS_NOT_READY;
    }
    if (yaw_error_deg != NULL){
        *yaw_error_deg = (float)(int16_t)data->value[0] / CANMV_ANGLE_SCALE;
    }
    if (pitch_error_deg != NULL){
        *pitch_error_deg = (float)(int16_t)data->value[1] / CANMV_ANGLE_SCALE;
    }
    return BSP_STATUS_OK;
}

static BSP_STATUS GimbalTracking_ReadRect(GEOMETRY_RECT2F *rect){
    uint16_t rect_data[8];
    uint8_t count = CanMvUart_GetData(CANMV_TARGET_RECT, rect_data, 8U);

    s_gimbal_tracking_state.rect_status = CanMvUart_GetStatus(CANMV_TARGET_RECT);

    if ((s_gimbal_tracking_state.rect_status != CANMV_STATUS_OK) || (count < 8U)){
        return BSP_STATUS_NOT_READY;
    }

    Geometry_RectFromArray(rect_data, rect);
    return BSP_STATUS_OK;
}

void GimbalTracking_Init(const GIMBAL_TRACKING_CONFIG *config){
    if (config == NULL){
        s_gimbal_tracking_config = GimbalTracking_DefaultConfig();
    } else{
        s_gimbal_tracking_config = *config;
    }

    PID_Init(&s_yaw_pid, &s_gimbal_tracking_config.yaw_pid);
    PID_Init(&s_pitch_pid, &s_gimbal_tracking_config.pitch_pid);
    PID_Init(&s_yaw_angle_pid, &s_gimbal_tracking_config.yaw_angle_pid);
    PID_Init(&s_pitch_angle_pid, &s_gimbal_tracking_config.pitch_angle_pid);
    GimbalTracking_Reset();
    s_gimbal_tracking_initialized = true;
}

void GimbalTracking_Reset(void){
    PID_Reset(&s_yaw_pid);
    PID_Reset(&s_pitch_pid);
    PID_Reset(&s_yaw_angle_pid);
    PID_Reset(&s_pitch_angle_pid);
    s_last_frame_ms = BSP_Time_GetMs();
    s_last_angle_frame_count = g_canmv_uart_angle_frame_count;
    s_angle_protocol_active = false;

    s_gimbal_tracking_state.target.x = 0.0f;
    s_gimbal_tracking_state.target.y = 0.0f;
    s_gimbal_tracking_state.laser.x = 0.0f;
    s_gimbal_tracking_state.laser.y = 0.0f;
    s_gimbal_tracking_state.error.x = 0.0f;
    s_gimbal_tracking_state.error.y = 0.0f;
    s_gimbal_tracking_state.yaw_speed = 0.0f;
    s_gimbal_tracking_state.pitch_speed = 0.0f;
    s_gimbal_tracking_state.laser_status = CANMV_STATUS_INIT;
    s_gimbal_tracking_state.rect_status = CANMV_STATUS_INIT;
    s_gimbal_tracking_state.target_valid = false;
    s_gimbal_tracking_state.laser_valid = false;
    s_gimbal_tracking_state.angle_mode = false;
    s_gimbal_tracking_state.link_timeout = false;
}

BSP_STATUS GimbalTracking_UpdateAngle(float dt_s){
    GimbalTracking_EnsureInitialized();

    /* 只在收到新角度帧时更新 PID，帧间保持上一速度指令。 */
    if (g_canmv_uart_angle_frame_count != s_last_angle_frame_count){
        float yaw_error_deg;
        float pitch_error_deg;

        s_last_angle_frame_count = g_canmv_uart_angle_frame_count;
        s_angle_protocol_active = true;
        if (GimbalTracking_ReadAngleError(&yaw_error_deg, &pitch_error_deg) != BSP_STATUS_OK){
            return GimbalTracking_HandleTargetNotReady();
        }
        GimbalTracking_MarkTargetReady();
        return GimbalTracking_TrackAngleErrors(yaw_error_deg, pitch_error_deg, dt_s);
    }

    if (s_angle_protocol_active){
        uint32_t now_ms = BSP_Time_GetMs();
        if ((s_gimbal_tracking_config.link_timeout_ms != 0U) &&
            ((now_ms - s_last_frame_ms) >= s_gimbal_tracking_config.link_timeout_ms)){
            return GimbalTracking_HandleTargetNotReady();
        }
        return BSP_STATUS_OK;
    }

    return GimbalTracking_HandleTargetNotReady();
}

BSP_STATUS GimbalTracking_UpdateLaserCenter(float dt_s){
    GimbalTracking_EnsureInitialized();

    /* 角度协议一旦出现即优先，并持续使用；否则兼容旧像素激光协议。 */
    if ((g_canmv_uart_angle_frame_count != s_last_angle_frame_count) ||
        s_angle_protocol_active){
        return GimbalTracking_UpdateAngle(dt_s);
    }

    /* 每帧先读最新目标: Lock 则跟踪, Lost 则停转但继续接收(不永久锁死)。 */
    CORE_POINT2F target;
    CORE_POINT2F laser;
    BSP_STATUS status = GimbalTracking_ReadLaser(&target, &laser);

    if (status != BSP_STATUS_OK){
        return GimbalTracking_HandleTargetNotReady();
    }

    GimbalTracking_MarkTargetReady();
    return GimbalTracking_TrackPoints(target, laser, dt_s);
}

BSP_STATUS GimbalTracking_UpdateRectCircle(int32_t edge_index,
                                           float angle_offset_deg,
                                           float dt_s){
    GimbalTracking_EnsureInitialized();

    CORE_POINT2F laser_target;
    CORE_POINT2F laser;
    BSP_STATUS status = GimbalTracking_ReadLaser(&laser_target, &laser);
    (void)laser_target;

    if (status != BSP_STATUS_OK){
        return GimbalTracking_HandleTargetNotReady();
    }

    GEOMETRY_RECT2F rect;
    status = GimbalTracking_ReadRect(&rect);

    if (status != BSP_STATUS_OK){
        s_gimbal_tracking_state.target_valid = false;
        return GimbalTracking_HandleTargetNotReady();
    }

    CORE_POINT2F paper_center = {
        .x = s_gimbal_tracking_config.paper_width / 2.0f,
        .y = s_gimbal_tracking_config.paper_height / 2.0f,
    };

    /* 先在题目纸面坐标系中生成圆轨迹目标，再映射到 CanMV 识别到的四边形。 */
    float target_angle_deg = (float)(edge_index - 1) * 90.0f + angle_offset_deg;
    CORE_POINT2F paper_target = Geometry_CirclePointDeg(paper_center,
                                                        s_gimbal_tracking_config.circle_radius,
                                                        target_angle_deg);
    CORE_POINT2F target = Geometry_PaperToRectPoint(paper_target,
                                                    s_gimbal_tracking_config.paper_width,
                                                    s_gimbal_tracking_config.paper_height,
                                                    &rect);

    s_gimbal_tracking_state.target_valid = true;
    GimbalTracking_MarkTargetReady();
    return GimbalTracking_TrackPoints(target, laser, dt_s);
}

BSP_STATUS GimbalTracking_UpdateRectCenter(float dt_s){
    GimbalTracking_EnsureInitialized();

    CORE_POINT2F laser_target;
    CORE_POINT2F laser;
    BSP_STATUS status = GimbalTracking_ReadLaser(&laser_target, &laser);
    (void)laser_target;

    if (status != BSP_STATUS_OK){
        return GimbalTracking_HandleTargetNotReady();
    }

    GEOMETRY_RECT2F rect;
    status = GimbalTracking_ReadRect(&rect);

    if (status != BSP_STATUS_OK){
        s_gimbal_tracking_state.target_valid = false;
        return GimbalTracking_HandleTargetNotReady();
    }

    CORE_POINT2F raw_center = Geometry_RectBilinearInterpolate(&rect, 0.5f, 0.5f);
    CORE_POINT2F target = GimbalTracking_ImagePoint((uint16_t)raw_center.x,
                                                    (uint16_t)raw_center.y);

    s_gimbal_tracking_state.target_valid = true;
    GimbalTracking_MarkTargetReady();
    return GimbalTracking_TrackPoints(target, laser, dt_s);
}

bool GimbalTracking_IsRectValid(void){
    GimbalTracking_EnsureInitialized();

    GEOMETRY_RECT2F rect;
    return GimbalTracking_ReadRect(&rect) == BSP_STATUS_OK;
}

/*
 * 最小速度地板 + 像素死区: 补偿 yaw 轴底层整数 RPM 台阶 (最小 6 deg/s)。
 *   - |axis_error| <= 死区: 输出 0, 中心处停住不抖;
 *   - 死区外但 |speed| < 最小驱动速度: 抬到 ±min, 保证任何真实误差都能驱动电机。
 */
static float GimbalTracking_ApplyMinSpeed(float speed, float axis_error, float min_move){
    float abs_err = (axis_error < 0.0f) ? -axis_error : axis_error;

    if (abs_err <= GIMBAL_TRACKING_DEFAULT_DEADBAND_PX){
        return 0.0f;
    }

    float abs_speed = (speed < 0.0f) ? -speed : speed;

    if ((abs_speed > 0.0f) && (abs_speed < min_move)){
        return (speed > 0.0f) ? min_move : -min_move;
    }

    return speed;
}

BSP_STATUS GimbalTracking_TrackPoints(CORE_POINT2F target,
                                      CORE_POINT2F laser,
                                      float dt_s){
    GimbalTracking_EnsureInitialized();

    s_gimbal_tracking_state.target = target;
    s_gimbal_tracking_state.laser = laser;
    s_gimbal_tracking_state.error.x = target.x - laser.x;
    s_gimbal_tracking_state.error.y = target.y - laser.y;
    s_gimbal_tracking_state.target_valid = true;
    s_gimbal_tracking_state.laser_valid = true;
    s_gimbal_tracking_state.angle_mode = false;

    float yaw_output = PID_Update(&s_yaw_pid, target.x, laser.x, dt_s);
    float pitch_output = PID_Update(&s_pitch_pid, target.y, laser.y, dt_s);

    /* 输出方向系数用于隔离电机安装方向差异，避免在 PID 误差定义中混入硬件极性。 */
    float yaw_speed = s_gimbal_tracking_config.yaw_output_sign * yaw_output;
    float pitch_speed = s_gimbal_tracking_config.pitch_output_sign * pitch_output;

    /*
     * yaw(速度模式): 死区外加最小速度地板, 越过底层整数 RPM 台阶。
     * pitch(位置模式): 只保留像素死区, 不加地板。
     */
    yaw_speed = GimbalTracking_ApplyMinSpeed(yaw_speed, s_gimbal_tracking_state.error.x,
                                             GIMBAL_TRACKING_DEFAULT_MIN_MOVE_YAW_DEG_S);
    pitch_speed = GimbalTracking_ApplyMinSpeed(pitch_speed, s_gimbal_tracking_state.error.y, 0.0f);

    s_gimbal_tracking_state.yaw_speed = yaw_speed;
    s_gimbal_tracking_state.pitch_speed = pitch_speed;

    return Gimbal_SetSpeed(yaw_speed, pitch_speed);
}

BSP_STATUS GimbalTracking_TrackAngleErrors(float yaw_error_deg,
                                           float pitch_error_deg,
                                           float dt_s){
    float yaw_output;
    float pitch_output;
    float yaw_speed;
    float pitch_speed;

    GimbalTracking_EnsureInitialized();
    s_gimbal_tracking_state.target.x = yaw_error_deg;
    s_gimbal_tracking_state.target.y = pitch_error_deg;
    s_gimbal_tracking_state.laser.x = 0.0f;
    s_gimbal_tracking_state.laser.y = 0.0f;
    s_gimbal_tracking_state.error.x = yaw_error_deg;
    s_gimbal_tracking_state.error.y = pitch_error_deg;
    s_gimbal_tracking_state.target_valid = true;
    s_gimbal_tracking_state.laser_valid = true;
    s_gimbal_tracking_state.angle_mode = true;

    yaw_output = PID_Update(&s_yaw_angle_pid, yaw_error_deg, 0.0f, dt_s);
    pitch_output = PID_Update(&s_pitch_angle_pid, pitch_error_deg, 0.0f, dt_s);
    yaw_speed = s_gimbal_tracking_config.yaw_output_sign * yaw_output;
    pitch_speed = s_gimbal_tracking_config.pitch_output_sign * pitch_output;
    /*
     * 不设角度死区: 由 I 项把稳态误差持续调零(死区会让 I 卡在边缘无法收敛)。
     * yaw 仅保留最小速度地板越过整数 RPM 台阶, 且只在输出【非零】时抬升——
     * 误差/输出恰为 0 时保持 0, 不会自造中心极限环。pitch 位置模式不加地板。
     */
    if ((yaw_speed > 0.0f) &&
        (yaw_speed < GIMBAL_TRACKING_DEFAULT_MIN_MOVE_YAW_DEG_S)){
        yaw_speed = GIMBAL_TRACKING_DEFAULT_MIN_MOVE_YAW_DEG_S;
    } else if ((yaw_speed < 0.0f) &&
               (yaw_speed > -GIMBAL_TRACKING_DEFAULT_MIN_MOVE_YAW_DEG_S)){
        yaw_speed = -GIMBAL_TRACKING_DEFAULT_MIN_MOVE_YAW_DEG_S;
    }
    s_gimbal_tracking_state.yaw_speed = yaw_speed;
    s_gimbal_tracking_state.pitch_speed = pitch_speed;
    return Gimbal_SetSpeed(yaw_speed, pitch_speed);
}

BSP_STATUS GimbalTracking_Stop(void){
    GimbalTracking_EnsureInitialized();

    s_gimbal_tracking_state.yaw_speed = 0.0f;
    s_gimbal_tracking_state.pitch_speed = 0.0f;
    return Gimbal_Stop();
}

GIMBAL_TRACKING_STATE GimbalTracking_GetState(void){
    return s_gimbal_tracking_state;
}

void GimbalTracking_SetGain(const char *key, float value){
    GimbalTracking_EnsureInitialized();
    if (key == NULL){
        return;
    }

    PID_CONFIG *y = &s_gimbal_tracking_config.yaw_pid;
    PID_CONFIG *p = &s_gimbal_tracking_config.pitch_pid;

    if (strcmp(key, "ykp") == 0){
        y->kp = value;
    } else if (strcmp(key, "yki") == 0){
        y->ki = value;
    } else if (strcmp(key, "ykd") == 0){
        y->kd = value;
    } else if (strcmp(key, "pkp") == 0){
        p->kp = value;
    } else if (strcmp(key, "pki") == 0){
        p->ki = value;
    } else if (strcmp(key, "pkd") == 0){
        p->kd = value;
    } else if (strcmp(key, "olim") == 0){
        y->output_limit = value;
        p->output_limit = value;
    } else{
        return;
    }

    /* 即时应用到在用控制器: 只替换 config, 保留积分/状态, 调参不产生跳变。 */
    s_yaw_pid.config = *y;
    s_pitch_pid.config = *p;
}

GIMBAL_TRACKING_CONFIG GimbalTracking_GetConfig(void){
    GimbalTracking_EnsureInitialized();
    return s_gimbal_tracking_config;
}
