#include "aim_solver.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

static AIM_SOLVER_CONFIG s_aim_config;
static bool s_aim_initialized;

AIM_SOLVER_CONFIG AimSolver_DefaultConfig(void){
    AIM_SOLVER_CONFIG config = {
        .target_center = {
            .x_m = AIM_SOLVER_DEFAULT_TARGET_X_M,
            .y_m = AIM_SOLVER_DEFAULT_TARGET_OFFSET_M,
        },
        .height_diff_m = AIM_SOLVER_DEFAULT_HEIGHT_DIFF_M,
        .mount_x_m = AIM_SOLVER_DEFAULT_MOUNT_X_M,
        .mount_y_m = AIM_SOLVER_DEFAULT_MOUNT_Y_M,
        .yaw_zero_offset_deg = AIM_SOLVER_DEFAULT_YAW_ZERO_OFFSET_DEG,
        .laser_lateral_offset_m = AIM_SOLVER_DEFAULT_LASER_LATERAL_OFFSET_M,
        .pitch_zero_offset_deg = AIM_SOLVER_DEFAULT_PITCH_ZERO_OFFSET_DEG,
        .pitch_beam_per_motor = AIM_SOLVER_DEFAULT_PITCH_BEAM_PER_MOTOR,
        .laser_vertical_offset_m = AIM_SOLVER_DEFAULT_LASER_VERTICAL_OFFSET_M,
        .circle_radius_m = AIM_SOLVER_DEFAULT_CIRCLE_RADIUS_M,
    };

    return config;
}

static void AimSolver_EnsureInitialized(void){
    if (!s_aim_initialized){
        AimSolver_Init(NULL);
    }
}

void AimSolver_Init(const AIM_SOLVER_CONFIG *config){
    if (config == NULL){
        s_aim_config = AimSolver_DefaultConfig();
    } else{
        s_aim_config = *config;
    }
    s_aim_initialized = true;
}

AIM_SOLVER_CONFIG AimSolver_GetConfig(void){
    AimSolver_EnsureInitialized();
    return s_aim_config;
}

void AimSolver_SetTargetCenter(AIM_POINT2F target_center){
    AimSolver_EnsureInitialized();
    s_aim_config.target_center = target_center;
}

AIM_SOLVER_RESULT AimSolver_Solve(KINEMATICS_POSE pose,
                                  AIM_POINT2F target_xy,
                                  float dz_m){
    AimSolver_EnsureInitialized();

    float heading_rad = KINEMATICS_DEG_TO_RAD(pose.heading_deg);
    float cos_h = cosf(heading_rad);
    float sin_h = sinf(heading_rad);

    /* 云台 yaw 轴支点的世界坐标: 车体安装偏移经航向旋转后叠加到车位置。 */
    float gx = pose.x_m + s_aim_config.mount_x_m * cos_h - s_aim_config.mount_y_m * sin_h;
    float gy = pose.y_m + s_aim_config.mount_x_m * sin_h + s_aim_config.mount_y_m * cos_h;

    /* 云台支点 → 目标点的水平向量。 */
    float dx = target_xy.x_m - gx;
    float dy = target_xy.y_m - gy;

    float horizontal = sqrtf(dx * dx + dy * dy);
    if (horizontal < AIM_SOLVER_HORIZONTAL_EPS){
        horizontal = AIM_SOLVER_HORIZONTAL_EPS;
    }

    /* 世界系绝对方位角 (支点→目标方向)。 */
    float world_bearing_deg = KINEMATICS_RAD_TO_DEG(atan2f(dy, dx));

    /* 激光束线不过 yaw 转轴的偏轴修正: 光束平行于云台指向、横向平移 ℓ (左正),
     * 命中条件 yaw = bearing − asin(ℓ/r), 修正量随距离 1/r 变化。 */
    float lateral_corr_deg = 0.0f;
    if (s_aim_config.laser_lateral_offset_m != 0.0f){
        float ratio = s_aim_config.laser_lateral_offset_m / horizontal;
        ratio = Kinematics_Clamp(ratio, -0.5f, 0.5f);   /* 病态近距保护 (≤30°) */
        lateral_corr_deg = KINEMATICS_RAD_TO_DEG(asinf(ratio));
    }

    /* 转到车体/云台系: 减去车航向、偏轴修正与机械零位补偿。 */
    float yaw_cmd_deg = Kinematics_NormalizeAngleDeg(world_bearing_deg
                                                     - pose.heading_deg
                                                     - lateral_corr_deg
                                                     - s_aim_config.yaw_zero_offset_deg);

    /* pitch (2D 简化): 几何仰角是【光束角】, 指令/零位是【电机角】, pitch 有
     * 减速时须按 1/k 折算 (k=光束角/电机角), 否则斜率错 k 倍 → 随距离漂移。
     * 竖直偏轴修正: 束线不过 pitch 转轴时, 命中仰角 = atan2 − asin(ℓp/r) (光束角域)。 */
    float beam_ratio = (s_aim_config.pitch_beam_per_motor > 0.0f)
                           ? s_aim_config.pitch_beam_per_motor
                           : 1.0f;
    float beam_pitch_deg = KINEMATICS_RAD_TO_DEG(atan2f(dz_m, horizontal));
    if (s_aim_config.laser_vertical_offset_m != 0.0f){
        float vratio = s_aim_config.laser_vertical_offset_m / horizontal;
        vratio = Kinematics_Clamp(vratio, -0.5f, 0.5f);
        beam_pitch_deg -= KINEMATICS_RAD_TO_DEG(asinf(vratio));
    }
    float pitch_cmd_deg = beam_pitch_deg / beam_ratio
                          + s_aim_config.pitch_zero_offset_deg;

    AIM_SOLVER_RESULT result = {
        .yaw_cmd_deg = yaw_cmd_deg,
        .pitch_cmd_deg = pitch_cmd_deg,
        .range_m = sqrtf(horizontal * horizontal + dz_m * dz_m),
        .horizontal_m = horizontal,
    };

    return result;
}

AIM_SOLVER_RESULT AimSolver_SolveCenter(KINEMATICS_POSE pose){
    AimSolver_EnsureInitialized();
    return AimSolver_Solve(pose, s_aim_config.target_center, s_aim_config.height_diff_m);
}

void AimSolver_CirclePoint(float phase_deg, AIM_POINT2F *out_xy, float *out_dz){
    AimSolver_EnsureInitialized();

    float phase_rad = KINEMATICS_DEG_TO_RAD(phase_deg);
    float radius = s_aim_config.circle_radius_m;

    /*
     * 靶面是 Y = 靶心 Y 的竖直平面 (平行于 AB), 由世界 X (水平) 和竖直方向张成。
     * 圆点在该平面内绕靶心画圆: 水平 X 方向为 cos, 竖直方向 (高差) 为 sin。
     */
    if (out_xy != NULL){
        out_xy->x_m = s_aim_config.target_center.x_m + radius * cosf(phase_rad);
        out_xy->y_m = s_aim_config.target_center.y_m;
    }
    if (out_dz != NULL){
        *out_dz = s_aim_config.height_diff_m + radius * sinf(phase_rad);
    }
}

AIM_SOLVER_RESULT AimSolver_SolveCircle(KINEMATICS_POSE pose, float phase_deg){
    AIM_POINT2F xy;
    float dz;
    AimSolver_CirclePoint(phase_deg, &xy, &dz);
    return AimSolver_Solve(pose, xy, dz);
}

float AimSolver_ProgressToPhaseDeg(float lap_progress, float phase0_deg){
    return Kinematics_NormalizeAngleDeg(lap_progress * 360.0f + phase0_deg);
}
