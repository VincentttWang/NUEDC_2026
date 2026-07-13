#include "aim_fusion.h"

#include "kinematics/kinematics.h"

#include <math.h>
#include <stddef.h>

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

AIM_RATE_FF AimFusion_MotionRate(KINEMATICS_POSE pose,
                                 AIM_POINT2F target_xy,
                                 float dz_m,
                                 float v_center_mps,
                                 float omega_deg_s){
    float psi_rad = KINEMATICS_DEG_TO_RAD(pose.heading_deg);

    /* 速率项忽略 mount 偏移 (二阶小量); 支点近似取车位置。 */
    float dx = target_xy.x_m - pose.x_m;
    float dy = target_xy.y_m - pose.y_m;
    float rh = sqrtf(dx * dx + dy * dy);
    if (rh < AIM_FUSION_RH_EPS){
        rh = AIM_FUSION_RH_EPS;
    }

    float bearing_rad = atan2f(dy, dx);
    float dbeta_rad = bearing_rad - psi_rad;   /* 车体系视线角 */
    float sin_db = sinf(dbeta_rad);
    float cos_db = cosf(dbeta_rad);

    AIM_RATE_FF out;
    /* yaw: 平移掠过引起的视线转速 − 车体自转。 */
    out.yaw_rate_deg_s = KINEMATICS_RAD_TO_DEG(v_center_mps * sin_db / rh) - omega_deg_s;
    out.yaw_rate_deg_s = Kinematics_Clamp(out.yaw_rate_deg_s,
                                          -AIM_FUSION_MAX_YAW_RATE_DEG_S,
                                          AIM_FUSION_MAX_YAW_RATE_DEG_S);
    /* pitch: 距离变化引起的仰角变率 (Δz 小 → 量级小)。 */
    out.pitch_rate_deg_s = KINEMATICS_RAD_TO_DEG(
        dz_m * v_center_mps * cos_db / (rh * rh + dz_m * dz_m));

    return out;
}

AIM_FUSION_OUTPUT AimFusion_Combine(AIM_SOLVER_RESULT feedforward,
                                    const AIM_VISION_BIAS *bias,
                                    AIM_RATE_FF rate){
    float yaw_bias = (bias != NULL) ? bias->yaw_bias_deg : 0.0f;
    float pitch_bias = (bias != NULL) ? bias->pitch_bias_deg : 0.0f;

    AIM_FUSION_OUTPUT out;
    out.yaw_cmd_deg = feedforward.yaw_cmd_deg + yaw_bias;
    out.pitch_cmd_deg = feedforward.pitch_cmd_deg + pitch_bias;
    out.yaw_rate_ff_deg_s = rate.yaw_rate_deg_s;
    out.pitch_rate_ff_deg_s = rate.pitch_rate_deg_s;
    return out;
}
