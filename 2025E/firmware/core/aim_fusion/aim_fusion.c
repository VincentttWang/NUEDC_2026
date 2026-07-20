#include "aim_fusion.h"

#include "kinematics/kinematics.h"

#include <math.h>
#include <stddef.h>

void AimVisionBias_Init(AIM_VISION_BIAS *bias, AIM_VISION_GAIN yaw,
                        AIM_VISION_GAIN pitch){
    if (bias == NULL){
        return;
    }
    bias->yaw_bias_deg = 0.0f;
    bias->pitch_bias_deg = 0.0f;
    bias->yaw = yaw;
    bias->pitch = pitch;
    bias->frame_count = 0U;
}

/* 单轴衰减增益: g(n)=max(gain_min, gain0/(1+n))。 */
static float AimVisionBias_AxisGain(const AIM_VISION_GAIN *axis, uint32_t n){
    float gain = axis->gain0 / (1.0f + (float)n);
    return (gain < axis->gain_min) ? axis->gain_min : gain;
}

void AimVisionBias_Update(AIM_VISION_BIAS *bias,
                          float yaw_err_deg,
                          float pitch_err_deg,
                          bool valid){
    if (bias == NULL){
        return;
    }
    /* 视觉无效: bias 冻结, 前馈照常输出, 保证激光连续 (帧计数不清零)。 */
    if (!valid){
        return;
    }

    /* KF 式衰减增益, 逐轴独立: 首帧快收敛, 随帧数衰减到各自稳态。 */
    float yaw_gain = AimVisionBias_AxisGain(&bias->yaw, bias->frame_count);
    float pitch_gain = AimVisionBias_AxisGain(&bias->pitch, bias->frame_count);
    bias->frame_count++;

    bias->yaw_bias_deg += yaw_gain * yaw_err_deg;
    bias->pitch_bias_deg += pitch_gain * pitch_err_deg;

    bias->yaw_bias_deg = Kinematics_Clamp(bias->yaw_bias_deg,
                                          -bias->yaw.limit_deg, bias->yaw.limit_deg);
    bias->pitch_bias_deg = Kinematics_Clamp(bias->pitch_bias_deg,
                                            -bias->pitch.limit_deg,
                                            bias->pitch.limit_deg);
}

void AimVisionBias_SetSteadyGain(AIM_VISION_BIAS *bias){
    if ((bias == NULL) || (bias->yaw.gain_min <= 0.0f) ||
        (bias->pitch.gain_min <= 0.0f)){
        return;
    }
    /* g(n)=gain0/(1+n) <= gain_min => n >= gain0/gain_min - 1; 两轴都满足取较大者。 */
    uint32_t n_yaw = (uint32_t)(bias->yaw.gain0 / bias->yaw.gain_min) + 1U;
    uint32_t n_pitch = (uint32_t)(bias->pitch.gain0 / bias->pitch.gain_min) + 1U;
    bias->frame_count = (n_yaw > n_pitch) ? n_yaw : n_pitch;
}

AIM_RATE_FF AimFusion_MotionRate(KINEMATICS_POSE pose,
                                 AIM_POINT2F target_xy,
                                 float dz_m,
                                 float v_center_mps,
                                 float omega_deg_s,
                                 float mount_x_m,
                                 float mount_y_m){
    float psi_rad = KINEMATICS_DEG_TO_RAD(pose.heading_deg);
    float cos_h = cosf(psi_rad);
    float sin_h = sinf(psi_rad);
    float omega_rad_s = KINEMATICS_DEG_TO_RAD(omega_deg_s);

    /* 云台支点: mount 偏移经航向旋转到世界系 (与 aim_solver 静态解一致)。 */
    float rx = mount_x_m * cos_h - mount_y_m * sin_h;
    float ry = mount_x_m * sin_h + mount_y_m * cos_h;
    float dx = target_xy.x_m - (pose.x_m + rx);
    float dy = target_xy.y_m - (pose.y_m + ry);
    float rh_sq = dx * dx + dy * dy;
    if (rh_sq < AIM_FUSION_RH_EPS * AIM_FUSION_RH_EPS){
        rh_sq = AIM_FUSION_RH_EPS * AIM_FUSION_RH_EPS;
    }
    float rh = sqrtf(rh_sq);

    /* 支点世界速度 = 车心平移 + 自转甩臂 ω×r (ω 逆时针为正, ẑ×r=(−ry,rx))。 */
    float vx = v_center_mps * cos_h - omega_rad_s * ry;
    float vy = v_center_mps * sin_h + omega_rad_s * rx;

    AIM_RATE_FF out;
    /* yaw: 视线方位角变率 d(atan2)/dt = (dy·vx − dx·vy)/Rh², 再减车体自转。 */
    out.yaw_rate_deg_s = KINEMATICS_RAD_TO_DEG((dy * vx - dx * vy) / rh_sq)
                         - omega_deg_s;
    out.yaw_rate_deg_s = Kinematics_Clamp(out.yaw_rate_deg_s,
                                          -AIM_FUSION_MAX_YAW_RATE_DEG_S,
                                          AIM_FUSION_MAX_YAW_RATE_DEG_S);
    /* pitch: 径向接近速度 v_rad = v_g·r̂ 引起的仰角变率 (Δz 小 → 量级小)。 */
    float v_radial = (vx * dx + vy * dy) / rh;
    out.pitch_rate_deg_s = KINEMATICS_RAD_TO_DEG(
        dz_m * v_radial / (rh_sq + dz_m * dz_m));

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
