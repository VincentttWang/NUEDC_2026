/**
 * @file  ball_balance.h
 * @brief 一维滚球在线终端约束控制（纯计算，不直接访问硬件）。
 *
 * 每拍从实测 (x,v,水管角) 重新规划一条到 (target,0,0) 的五次轨迹，只执行
 * 前方一个短预瞄点后再次规划。完整规划严格满足：
 *   integral(a dt) = -v0
 *   integral(v dt) = target - x0
 * 因而把“移动到目标”和“在目标速度归零”统一成一个约束，不再另设停止距离模型。
 *
 * 正方向约定：水管正倾角使球向正方向加速。调用方必须保证视觉 x/v 同号。
 */
#ifndef BALL_BALANCE_H
#define BALL_BALANCE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 在线五次轨迹的运动约束。 */
    float rolling_acceleration_gain_mm_s2;
    float profile_max_velocity_mm_s;
    float profile_max_acceleration_mm_s2;
    float profile_min_duration_s;
    float profile_max_duration_s;
    float profile_lookahead_s;

    /* 模型前馈后的轻量轨迹误差反馈。 */
    float position_feedback_gain_deg_per_mm;
    float velocity_feedback_gain_deg_per_mm_s;
    float feedback_limit_deg;

    /* 动力学水平点慢速学习。 */
    float zero_trim_ki_deg_per_mm_s;
    float zero_trim_leak_tau_s;
    float zero_trim_limit_deg;

    /* 水管物理角和输出变化率约束。 */
    float angle_min_deg;
    float angle_max_deg;
    float control_angle_limit_deg;
    float angle_rate_near_deg_s;
    float angle_rate_far_deg_s;
    float angle_rate_brake_deg_s;
    float angle_rate_transition_mm;

    /* 截停/卡滞仅作诊断并冻结零偏学习，不擅自执行 safe return。 */
    float stuck_min_error_mm;
    float stuck_max_position_change_mm;
    float stuck_min_command_deg;
    float stuck_time_s;

    float trim_min_position_mm;
    float trim_max_position_mm;
    float trim_min_speed_mm_s;
    float trim_max_speed_mm_s;

    float settled_position_mm;
    float settled_speed_mm_s;
    float settled_time_s;
} BALL_BALANCE_CONFIG;

typedef struct {
    float target_x_mm;
    float zero_trim_deg;
    float output_deg;
    float previous_position_mm;
    float stagnant_elapsed_s;
    float settled_elapsed_s;
    bool  have_previous_position;
    bool  intercepted;
    bool  settled;
} BALL_BALANCE_CONTROLLER;

typedef struct {
    float x_mm;
    float velocity_mm_s;
    float actual_angle_deg;
    float dt_s;
    bool  moving;
    bool  allow_zero_trim;
} BALL_BALANCE_INPUT;

typedef struct {
    float angle_deg;
    float x_ref_mm;
    float velocity_ref_mm_s;
    float acceleration_ref_mm_s2;
    float feedforward_deg;
    float position_feedback_deg;
    float velocity_feedback_deg;
    float feedback_deg;
    float position_error_mm;
    float velocity_error_mm_s;
    float model_acceleration_mm_s2;
    float profile_duration_s;
    float profile_lookahead_s;
    float profile_peak_velocity_mm_s;
    float profile_peak_acceleration_mm_s2;
    float acceleration_integral_mm_s;
    float velocity_integral_mm;
    float terminal_position_residual_mm;
    float terminal_velocity_residual_mm_s;
    float terminal_acceleration_residual_mm_s2;
    float angle_rate_limit_deg_s;
    float zero_trim_deg;
    bool  profile_limit_relaxed;
    bool  saturated;
    bool  rate_limited;
    bool  braking;
    bool  intercepted;
    bool  settled;
} BALL_BALANCE_OUTPUT;

void BallBalance_Init(BALL_BALANCE_CONTROLLER *controller);

/** 清空动态状态，并令下一拍从给定实际水管角开始做斜率限制。 */
void BallBalance_Reset(BALL_BALANCE_CONTROLLER *controller,
                       float current_angle_deg);

void BallBalance_SetTarget(BALL_BALANCE_CONTROLLER *controller,
                           float target_x_mm);

/** 设置经实测得到的动力学零倾角先验，单位 deg。 */
void BallBalance_SetZeroTrim(BALL_BALANCE_CONTROLLER *controller,
                            float zero_trim_deg);

/** 执行一次在线终端约束规划 + 前馈/反馈合成。 */
bool BallBalance_Update(BALL_BALANCE_CONTROLLER *controller,
                        const BALL_BALANCE_CONFIG *config,
                        const BALL_BALANCE_INPUT *input,
                        BALL_BALANCE_OUTPUT *output);

#ifdef __cplusplus
}
#endif

#endif /* BALL_BALANCE_H */
