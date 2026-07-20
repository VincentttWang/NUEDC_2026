#include "gimbal_tracking.h"

#include "bsp_time.h"
#include "geometry/geometry.h"
#include "gimbal.h"

#include <stddef.h>

static GIMBAL_TRACKING_CONFIG s_gimbal_tracking_config;
static GIMBAL_TRACKING_STATE s_gimbal_tracking_state;
static PID_CONTROLLER s_yaw_pid;
static PID_CONTROLLER s_pitch_pid;
static bool s_gimbal_tracking_initialized;
static uint32_t s_last_frame_ms;

static GIMBAL_TRACKING_CONFIG GimbalTracking_DefaultConfig(void){
    PID_CONFIG pid_config = {
        .kp = GIMBAL_TRACKING_DEFAULT_PID_KP,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 1000.0f,
        .output_limit = GIMBAL_TRACKING_DEFAULT_PID_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };
    GIMBAL_TRACKING_CONFIG config = {
        .yaw_pid = pid_config,
        .pitch_pid = pid_config,
        .image_height = GIMBAL_TRACKING_DEFAULT_IMAGE_HEIGHT,
        .paper_width = GIMBAL_TRACKING_DEFAULT_PAPER_WIDTH,
        .paper_height = GIMBAL_TRACKING_DEFAULT_PAPER_HEIGHT,
        .circle_radius = GIMBAL_TRACKING_DEFAULT_CIRCLE_RADIUS,
        .yaw_output_sign = 1.0f,
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

    if ((s_gimbal_tracking_config.link_timeout_ms == 0U) ||
        ((now_ms - s_last_frame_ms) < s_gimbal_tracking_config.link_timeout_ms)){
        s_gimbal_tracking_state.link_timeout = false;
        return BSP_STATUS_NOT_READY;
    }

    bool was_timeout = s_gimbal_tracking_state.link_timeout;
    s_gimbal_tracking_state.link_timeout = true;
    s_gimbal_tracking_state.target_valid = false;
    s_gimbal_tracking_state.laser_valid = false;
    s_gimbal_tracking_state.yaw_speed = 0.0f;
    s_gimbal_tracking_state.pitch_speed = 0.0f;
    if (!was_timeout){
        (void)Gimbal_Stop();
    }

    return BSP_STATUS_NOT_READY;
}

static void GimbalTracking_MarkTargetReady(void){
    s_last_frame_ms = BSP_Time_GetMs();
    s_gimbal_tracking_state.link_timeout = false;
}

static bool GimbalTracking_CheckLinkTimeout(void){
    uint32_t now_ms = BSP_Time_GetMs();

    if ((s_gimbal_tracking_config.link_timeout_ms == 0U) ||
        ((now_ms - s_last_frame_ms) < s_gimbal_tracking_config.link_timeout_ms)){
        s_gimbal_tracking_state.link_timeout = false;
        return false;
    }

    (void)GimbalTracking_HandleTargetNotReady();
    return s_gimbal_tracking_state.link_timeout;
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
    GimbalTracking_Reset();
    s_gimbal_tracking_initialized = true;
}

void GimbalTracking_Reset(void){
    PID_Reset(&s_yaw_pid);
    PID_Reset(&s_pitch_pid);
    s_last_frame_ms = BSP_Time_GetMs();

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
    s_gimbal_tracking_state.link_timeout = false;
}

BSP_STATUS GimbalTracking_UpdateLaserCenter(float dt_s){
    GimbalTracking_EnsureInitialized();

    if (GimbalTracking_CheckLinkTimeout()){
        return BSP_STATUS_NOT_READY;
    }

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

    if (GimbalTracking_CheckLinkTimeout()){
        return BSP_STATUS_NOT_READY;
    }

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

    if (GimbalTracking_CheckLinkTimeout()){
        return BSP_STATUS_NOT_READY;
    }

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

    float yaw_output = PID_Update(&s_yaw_pid, target.x, laser.x, dt_s);
    float pitch_output = PID_Update(&s_pitch_pid, target.y, laser.y, dt_s);

    /* 输出方向系数用于隔离电机安装方向差异，避免在 PID 误差定义中混入硬件极性。 */
    s_gimbal_tracking_state.yaw_speed = s_gimbal_tracking_config.yaw_output_sign * yaw_output;
    s_gimbal_tracking_state.pitch_speed = s_gimbal_tracking_config.pitch_output_sign * pitch_output;

    return Gimbal_SetSpeed(s_gimbal_tracking_state.yaw_speed,
                           s_gimbal_tracking_state.pitch_speed);
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
