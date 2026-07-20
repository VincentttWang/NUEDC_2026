#include "localization.h"

#include <math.h>

static LOCALIZATION_STATE s_loc;

static float Localization_HalfSide(void){
    return s_loc.track_side_m / 2.0f;
}

void Localization_Init(KINEMATICS_POSE start_pose,
                       float track_side_m,
                       float initial_distance_m){
    s_loc.pose = start_pose;
    s_loc.pose.heading_deg = Kinematics_NormalizeAngleDeg(start_pose.heading_deg);
    s_loc.track_side_m = (track_side_m > 0.0f) ? track_side_m
                                               : LOCALIZATION_DEFAULT_TRACK_SIDE_M;
    s_loc.perimeter_m = 4.0f * s_loc.track_side_m;
    s_loc.last_distance_m = initial_distance_m;
    s_loc.travelled_m = 0.0f;
    s_loc.pending_corr_x_m = 0.0f;
    s_loc.pending_corr_y_m = 0.0f;
    s_loc.initialized = true;
}

void Localization_Update(float distance_m, float heading_deg){
    if (!s_loc.initialized){
        KINEMATICS_POSE zero = {0.0f, 0.0f, 0.0f};
        Localization_Init(zero, LOCALIZATION_DEFAULT_TRACK_SIDE_M, distance_m);
    }

    /* 兼容累计里程输入；发挥任务使用 UpdateDelta 注入车心修正后的增量。 */
    float delta_m = distance_m - s_loc.last_distance_m;
    s_loc.last_distance_m = distance_m;

    Localization_UpdateDelta(delta_m, heading_deg);
}

void Localization_UpdateDelta(float center_delta_m, float heading_deg){
    if (!s_loc.initialized){
        KINEMATICS_POSE zero = {0.0f, 0.0f, 0.0f};
        Localization_Init(zero, LOCALIZATION_DEFAULT_TRACK_SIDE_M, 0.0f);
    }

    /* 航向直接采用 IMU 融合值 (低漂移); 位置沿该航向积分位移。 */
    float heading_rad = KINEMATICS_DEG_TO_RAD(heading_deg);
    s_loc.pose.x_m += center_delta_m * cosf(heading_rad);
    s_loc.pose.y_m += center_delta_m * sinf(heading_rad);
    s_loc.pose.heading_deg = Kinematics_NormalizeAngleDeg(heading_deg);
    s_loc.last_center_delta_m = center_delta_m;

    /* 圈进度按行进路程累加 (前进为主, 用绝对值累计避免抖动倒扣)。 */
    s_loc.travelled_m += (center_delta_m < 0.0f) ? -center_delta_m : center_delta_m;

    /* 渐变锚定: 逐帧摊平 anchor 存下的位置修正 (指数收敛), 避免位姿一帧突变。 */
    float apply_x = s_loc.pending_corr_x_m * LOCALIZATION_ANCHOR_BLEND_ALPHA;
    float apply_y = s_loc.pending_corr_y_m * LOCALIZATION_ANCHOR_BLEND_ALPHA;
    s_loc.pose.x_m += apply_x;
    s_loc.pose.y_m += apply_y;
    s_loc.pending_corr_x_m -= apply_x;
    s_loc.pending_corr_y_m -= apply_y;
}

float Localization_CorrectWheelDelta(float wheel_delta_m,
                                     float gyro_z_deg_s,
                                     float encoder_lateral_offset_m,
                                     float dt_s){
    if (dt_s <= 0.0f){
        return wheel_delta_m;
    }

    /* v_wheel = v_center - omega * y，因此 v_center = v_wheel + omega * y。 */
    float omega_rad_s = KINEMATICS_DEG_TO_RAD(gyro_z_deg_s);
    return wheel_delta_m + omega_rad_s * encoder_lateral_offset_m * dt_s;
}

void Localization_UpdateFromWheel(float wheel_delta_m,
                                  float gyro_z_deg_s,
                                  float encoder_lateral_offset_m,
                                  float heading_deg,
                                  float dt_s){
    float center_delta_m = Localization_CorrectWheelDelta(wheel_delta_m,
                                                          gyro_z_deg_s,
                                                          encoder_lateral_offset_m,
                                                          dt_s);
    Localization_UpdateDelta(center_delta_m, heading_deg);
}

CORE_POINT2F Localization_CornerPoint(LOCALIZATION_CORNER corner){
    float half = Localization_HalfSide();
    float side = s_loc.track_side_m;
    CORE_POINT2F point = {0.0f, 0.0f};

    switch (corner){
    case LOCALIZATION_CORNER_A:
        point.x = -half;
        point.y = 0.0f;
        break;
    case LOCALIZATION_CORNER_B:
        point.x = half;
        point.y = 0.0f;
        break;
    case LOCALIZATION_CORNER_C:
        point.x = -half;
        point.y = -side;
        break;
    case LOCALIZATION_CORNER_D:
        point.x = half;
        point.y = -side;
        break;
    default:
        break;
    }

    return point;
}

void Localization_ResetToCorner(LOCALIZATION_CORNER corner){
    if (corner >= LOCALIZATION_CORNER_COUNT){
        return;
    }
    CORE_POINT2F p = Localization_CornerPoint(corner);
    s_loc.pose.x_m = p.x;
    s_loc.pose.y_m = p.y;
    /* 立即 snap: 清掉未完成的渐变待修正量。 */
    s_loc.pending_corr_x_m = 0.0f;
    s_loc.pending_corr_y_m = 0.0f;
}

void Localization_ResetToCornerAtTravel(LOCALIZATION_CORNER corner,
                                        float travelled_m){
    if ((corner >= LOCALIZATION_CORNER_COUNT) || (travelled_m < 0.0f)){
        return;
    }

    /*
     * 渐变锚定: 弧长立即重锚 (供圈进度/画圆相位); 位置修正存为待施加量, 由 UpdateDelta
     * 逐帧摊平, 避免位姿一帧突变 → 几何 yaw 指令阶跃 → 云台被 snap 猛甩过冲。
     * (若需旧的立即 snap: 令 LOCALIZATION_ANCHOR_BLEND_ALPHA=1.0, 或改调 ResetToCorner。)
     */
    CORE_POINT2F p = Localization_CornerPoint(corner);
    s_loc.pending_corr_x_m = p.x - s_loc.pose.x_m;
    s_loc.pending_corr_y_m = p.y - s_loc.pose.y_m;
    s_loc.travelled_m = travelled_m;
}

KINEMATICS_POSE Localization_GetPose(void){
    return s_loc.pose;
}

float Localization_GetTravelled(void){
    return s_loc.travelled_m;
}

float Localization_GetLapProgress(void){
    if (s_loc.perimeter_m <= 0.0f){
        return 0.0f;
    }
    float lap = s_loc.travelled_m / s_loc.perimeter_m;
    return lap - floorf(lap);
}

uint32_t Localization_GetLapCount(void){
    if (s_loc.perimeter_m <= 0.0f){
        return 0U;
    }
    return (uint32_t)(s_loc.travelled_m / s_loc.perimeter_m);
}

LOCALIZATION_STATE Localization_GetState(void){
    return s_loc;
}
