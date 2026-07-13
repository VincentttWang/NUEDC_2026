#include "aim_solver.h"

#include <math.h>
#include <stddef.h>

static AIM_SOLVER_CONFIG s_aim_config;
static bool s_aim_initialized;

AIM_SOLVER_CONFIG AimSolver_DefaultConfig(void){
    AIM_SOLVER_CONFIG config = {
        .target_center = {
            .x_m = AIM_SOLVER_DEFAULT_TARGET_X_M,
            .y_m = AIM_SOLVER_DEFAULT_TARGET_OFFSET_M,
            .z_m = AIM_SOLVER_DEFAULT_TARGET_Z_M,
        },
        .mount_x_m = AIM_SOLVER_DEFAULT_MOUNT_X_M,
        .mount_y_m = AIM_SOLVER_DEFAULT_MOUNT_Y_M,
        .mount_z_m = AIM_SOLVER_DEFAULT_MOUNT_Z_M,
        .yaw_zero_offset_deg = AIM_SOLVER_DEFAULT_YAW_ZERO_OFFSET_DEG,
        .pitch_zero_offset_deg = AIM_SOLVER_DEFAULT_PITCH_ZERO_OFFSET_DEG,
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

void AimSolver_SetTargetCenter(AIM_VEC3F target_center){
    AimSolver_EnsureInitialized();
    s_aim_config.target_center = target_center;
}

AIM_SOLVER_RESULT AimSolver_Solve(KINEMATICS_POSE pose, AIM_VEC3F target_world){
    AimSolver_EnsureInitialized();

    float heading_rad = KINEMATICS_DEG_TO_RAD(pose.heading_deg);
    float cos_h = cosf(heading_rad);
    float sin_h = sinf(heading_rad);

    /* 云台 yaw 轴支点的世界坐标: 车体安装偏移经航向旋转后叠加到车位置。 */
    float gx = pose.x_m + s_aim_config.mount_x_m * cos_h - s_aim_config.mount_y_m * sin_h;
    float gy = pose.y_m + s_aim_config.mount_x_m * sin_h + s_aim_config.mount_y_m * cos_h;
    float gz = s_aim_config.mount_z_m;

    /* 云台支点 → 目标点向量。 */
    float dx = target_world.x_m - gx;
    float dy = target_world.y_m - gy;
    float dz = target_world.z_m - gz;

    float horizontal = sqrtf(dx * dx + dy * dy);

    /* 世界系绝对方位角 (激光在水平面内应指向的方向)。 */
    float world_bearing_deg = KINEMATICS_RAD_TO_DEG(atan2f(dy, dx));

    /* 转到车体/云台系: 减去车航向与机械零位补偿。 */
    float yaw_cmd_deg = Kinematics_NormalizeAngleDeg(world_bearing_deg
                                                     - pose.heading_deg
                                                     - s_aim_config.yaw_zero_offset_deg);

    /* pitch: 仰角 = atan2(高度差, 水平距离) + 机械零位补偿。 */
    float pitch_cmd_deg = KINEMATICS_RAD_TO_DEG(atan2f(dz, horizontal))
                          + s_aim_config.pitch_zero_offset_deg;

    AIM_SOLVER_RESULT result = {
        .yaw_cmd_deg = yaw_cmd_deg,
        .pitch_cmd_deg = pitch_cmd_deg,
        .range_m = sqrtf(horizontal * horizontal + dz * dz),
        .horizontal_m = horizontal,
    };

    return result;
}

AIM_SOLVER_RESULT AimSolver_SolveCenter(KINEMATICS_POSE pose){
    AimSolver_EnsureInitialized();
    return AimSolver_Solve(pose, s_aim_config.target_center);
}

AIM_VEC3F AimSolver_CircleTarget(float phase_deg){
    AimSolver_EnsureInitialized();

    float phase_rad = KINEMATICS_DEG_TO_RAD(phase_deg);
    float radius = s_aim_config.circle_radius_m;

    /*
     * 靶面是 Y = 靶心 Y 的竖直平面 (平行于 AB), 由世界 X (水平) 和 Z (竖直) 张成。
     * 圆点在该平面内绕靶心画圆: X 方向为 cos, Z 方向为 sin。
     */
    AIM_VEC3F point = {
        .x_m = s_aim_config.target_center.x_m + radius * cosf(phase_rad),
        .y_m = s_aim_config.target_center.y_m,
        .z_m = s_aim_config.target_center.z_m + radius * sinf(phase_rad),
    };

    return point;
}

AIM_SOLVER_RESULT AimSolver_SolveCircle(KINEMATICS_POSE pose, float phase_deg){
    return AimSolver_Solve(pose, AimSolver_CircleTarget(phase_deg));
}

float AimSolver_ProgressToPhaseDeg(float lap_progress, float phase0_deg){
    return Kinematics_NormalizeAngleDeg(lap_progress * 360.0f + phase0_deg);
}

void AimVisionBias_Init(AIM_VISION_BIAS *bias, float ki, float limit){
    if (bias == NULL){
        return;
    }
    bias->yaw_bias_deg = 0.0f;
    bias->pitch_bias_deg = 0.0f;
    bias->ki = ki;
    bias->bias_limit_deg = limit;
}

void AimVisionBias_Update(AIM_VISION_BIAS *bias,
                          float yaw_err_deg,
                          float pitch_err_deg,
                          bool valid,
                          float dt_s){
    if (bias == NULL){
        return;
    }
    /* 视觉无效: bias 保持不动, 前馈照常输出, 保证激光连续。 */
    if (!valid || (dt_s <= 0.0f)){
        return;
    }

    bias->yaw_bias_deg += bias->ki * yaw_err_deg * dt_s;
    bias->pitch_bias_deg += bias->ki * pitch_err_deg * dt_s;

    bias->yaw_bias_deg = Kinematics_Clamp(bias->yaw_bias_deg,
                                          -bias->bias_limit_deg, bias->bias_limit_deg);
    bias->pitch_bias_deg = Kinematics_Clamp(bias->pitch_bias_deg,
                                            -bias->bias_limit_deg, bias->bias_limit_deg);
}
