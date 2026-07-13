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
    s_loc.initialized = true;
}

void Localization_Update(float distance_m, float heading_deg){
    if (!s_loc.initialized){
        KINEMATICS_POSE zero = {0.0f, 0.0f, 0.0f};
        Localization_Init(zero, LOCALIZATION_DEFAULT_TRACK_SIDE_M, distance_m);
    }

    /* 里程增量: 编码器累计里程为单调量, 取差分得到本步位移 (可正可负)。 */
    float delta_m = distance_m - s_loc.last_distance_m;
    s_loc.last_distance_m = distance_m;

    /* 航向直接采用 IMU 融合值 (低漂移); 位置沿该航向积分位移。 */
    float heading_rad = KINEMATICS_DEG_TO_RAD(heading_deg);
    s_loc.pose.x_m += delta_m * cosf(heading_rad);
    s_loc.pose.y_m += delta_m * sinf(heading_rad);
    s_loc.pose.heading_deg = Kinematics_NormalizeAngleDeg(heading_deg);

    /* 圈进度按行进路程累加 (前进为主, 用绝对值累计避免抖动倒扣)。 */
    s_loc.travelled_m += (delta_m < 0.0f) ? -delta_m : delta_m;
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
