#include "ball_balance.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define TEST_K_G_MM_S2_PER_RAD 7004.75f

static const BALL_BALANCE_CONFIG CONFIG = {
    .rolling_acceleration_gain_mm_s2 = TEST_K_G_MM_S2_PER_RAD,
    .profile_max_velocity_mm_s = 90.0f,
    .profile_max_acceleration_mm_s2 = 280.0f,
    .profile_min_duration_s = 0.35f,
    .profile_max_duration_s = 3.00f,
    .profile_lookahead_s = 0.12f,
    .position_feedback_gain_deg_per_mm = 0.025f,
    .velocity_feedback_gain_deg_per_mm_s = 0.018f,
    .feedback_limit_deg = 1.5f,
    .zero_trim_ki_deg_per_mm_s = 0.004f,
    .zero_trim_leak_tau_s = 180.0f,
    .zero_trim_limit_deg = 0.6f,
    .angle_min_deg = -5.2f,
    .angle_max_deg = 5.1f,
    .control_angle_limit_deg = 4.0f,
    .angle_rate_near_deg_s = 70.0f,
    .angle_rate_far_deg_s = 40.0f,
    .angle_rate_brake_deg_s = 100.0f,
    .angle_rate_transition_mm = 30.0f,
    .stuck_min_error_mm = 20.0f,
    .stuck_max_position_change_mm = 0.5f,
    .stuck_min_command_deg = 1.2f,
    .stuck_time_s = 0.8f,
    .trim_min_position_mm = 2.0f,
    .trim_max_position_mm = 20.0f,
    .trim_min_speed_mm_s = 0.0f,
    .trim_max_speed_mm_s = 8.0f,
    .settled_position_mm = 2.0f,
    .settled_speed_mm_s = 5.0f,
    .settled_time_s = 1.0f,
};

static BALL_BALANCE_INPUT Input(float x_mm, float velocity_mm_s, bool moving){
    BALL_BALANCE_INPUT input = {
        .x_mm = x_mm,
        .velocity_mm_s = velocity_mm_s,
        .actual_angle_deg = 0.0f,
        .dt_s = 0.02f,
        .moving = moving,
        .allow_zero_trim = false,
    };
    return input;
}

static void AssertTerminalConstraints(const BALL_BALANCE_OUTPUT *output,
                                      float x0_mm,
                                      float v0_mm_s,
                                      float target_mm){
    /* integral(a dt)=Delta-v；integral(v dt)=Delta-x。 */
    assert(fabsf(output->acceleration_integral_mm_s + v0_mm_s) < 0.02f);
    assert(fabsf(output->velocity_integral_mm - (target_mm - x0_mm)) < 0.02f);
    assert(fabsf(output->terminal_position_residual_mm) < 0.02f);
    assert(fabsf(output->terminal_velocity_residual_mm_s) < 0.02f);
    assert(fabsf(output->terminal_acceleration_residual_mm_s2) < 0.05f);
}

int main(void){
    BALL_BALANCE_CONTROLLER controller;
    BALL_BALANCE_OUTPUT output;
    BallBalance_Init(&controller);
    BallBalance_SetTarget(&controller, 0.0f);

    /* 静止在正侧：预瞄参考、前馈和反馈都应指向中心。 */
    BALL_BALANCE_INPUT input = Input(10.0f, 0.0f, false);
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    AssertTerminalConstraints(&output, 10.0f, 0.0f, 0.0f);
    assert(output.x_ref_mm < 10.0f);
    assert(output.velocity_ref_mm_s < 0.0f);
    assert(output.acceleration_ref_mm_s2 < 0.0f);
    assert(output.feedforward_deg < 0.0f);
    assert(output.feedback_deg < 0.0f);
    assert(output.profile_peak_velocity_mm_s <=
           CONFIG.profile_max_velocity_mm_s * 1.01f);
    assert(output.profile_peak_acceleration_mm_s2 <=
           CONFIG.profile_max_acceleration_mm_s2 * 1.01f);
    assert(!output.profile_limit_relaxed);

    /* 距离更远时规划时长和参考速度增大，但仍满足同一终端约束。 */
    BALL_BALANCE_CONTROLLER far_controller;
    BALL_BALANCE_OUTPUT far_output;
    BallBalance_Init(&far_controller);
    input = Input(100.0f, 0.0f, false);
    assert(BallBalance_Update(&far_controller, &CONFIG, &input, &far_output));
    AssertTerminalConstraints(&far_output, 100.0f, 0.0f, 0.0f);
    assert(far_output.profile_duration_s > output.profile_duration_s);
    assert(far_output.angle_rate_limit_deg_s < output.angle_rate_limit_deg_s);

    /* 高速靠近目标：在线重规划必须给出反向加速度，并在终点速度归零。 */
    BallBalance_Init(&controller);
    input = Input(20.0f, -100.0f, true);
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    AssertTerminalConstraints(&output, 20.0f, -100.0f, 0.0f);
    assert(output.acceleration_ref_mm_s2 > 0.0f);
    assert(output.braking);
    assert(output.angle_rate_limit_deg_s == CONFIG.angle_rate_brake_deg_s);

    /* 当前倾角直接决定规划初始加速度，验证 a=K_G*sin(theta-zero)。 */
    BallBalance_Init(&controller);
    input = Input(0.0f, 0.0f, false);
    input.actual_angle_deg = 4.0f;
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    float expected_acceleration = TEST_K_G_MM_S2_PER_RAD *
                                  sinf(4.0f * 0.01745329251994329577f);
    assert(fabsf(output.model_acceleration_mm_s2 - expected_acceleration) < 0.1f);
    AssertTerminalConstraints(&output, 0.0f, 0.0f, 0.0f);

    /* 若初始速度已经超出规划上限，仍保证终端约束并显式标记约束放宽。 */
    BallBalance_Init(&controller);
    input = Input(0.0f, 200.0f, true);
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    AssertTerminalConstraints(&output, 0.0f, 200.0f, 0.0f);
    assert(output.profile_limit_relaxed);

    /* 零偏是可调先验；控制限角始终相对零偏而非绝对零度。 */
    BallBalance_Init(&controller);
    BallBalance_SetZeroTrim(&controller, -0.4f);
    input = Input(0.0f, 0.0f, false);
    input.actual_angle_deg = -0.4f;
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    assert(fabsf(output.model_acceleration_mm_s2) < 0.01f);
    assert(fabsf(output.zero_trim_deg + 0.4f) < 0.001f);

    /* 中心低速窗口允许消除静差；冻结后偏置缓慢泄漏。 */
    BallBalance_Init(&controller);
    input = Input(5.0f, 0.0f, false);
    input.allow_zero_trim = true;
    for (unsigned i = 0U; i < 100U; i++){
        assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    }
    assert(output.zero_trim_deg < -0.03f);
    float learned_trim = fabsf(output.zero_trim_deg);
    input.allow_zero_trim = false;
    for (unsigned i = 0U; i < 100U; i++){
        assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    }
    assert(fabsf(output.zero_trim_deg) < learned_trim);

    /* 工具截停只形成诊断，不执行依赖不可靠水平位的 safe return。 */
    BallBalance_Init(&controller);
    input = Input(25.0f, 0.0f, false);
    for (unsigned i = 0U; i < 50U; i++){
        assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    }
    assert(output.intercepted);
    assert(fabsf(output.angle_deg - output.zero_trim_deg) >
           CONFIG.stuck_min_command_deg);
    input = Input(23.0f, -10.0f, true);
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    assert(!output.intercepted);

    BallBalance_Init(&controller);
    input = Input(1.0f, 2.0f, true);
    for (unsigned i = 0U; i < 51U; i++){
        assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    }
    assert(output.settled);
    input = Input(3.0f, 2.0f, true);
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    assert(!output.settled);

    /* 理想双积分对象闭环烟测：100 mm 回中，并能吸收一次 +100 mm/s 外部拨动。 */
    BallBalance_Init(&controller);
    float x_mm = 100.0f;
    float velocity_mm_s = 0.0f;
    float beam_deg = 0.0f;
    for (unsigned i = 0U; i < 1000U; i++){
        if (i == 250U){ velocity_mm_s += 100.0f; }
        input = Input(x_mm, velocity_mm_s, fabsf(velocity_mm_s) > 0.5f);
        input.actual_angle_deg = beam_deg;
        assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
        /* 近似步进位置环：KP=6 s^-1，120°/s 电机轴经局部连杆增益约为 20°/s 水管。 */
        float beam_rate_deg_s = 6.0f * (output.angle_deg - beam_deg);
        if (beam_rate_deg_s > 20.0f){ beam_rate_deg_s = 20.0f; }
        if (beam_rate_deg_s < -20.0f){ beam_rate_deg_s = -20.0f; }
        beam_deg += beam_rate_deg_s * input.dt_s;
        float acceleration_mm_s2 = TEST_K_G_MM_S2_PER_RAD *
            sinf(beam_deg * 0.01745329251994329577f);
        velocity_mm_s += acceleration_mm_s2 * input.dt_s;
        x_mm += velocity_mm_s * input.dt_s;
        assert(fabsf(x_mm) < 500.0f);
    }
    assert(fabsf(x_mm) < 3.0f);
    assert(fabsf(velocity_mm_s) < 5.0f);
    printf("receding profile smoke final: x=%.3f mm v=%.3f mm/s\n",
           (double)x_mm, (double)velocity_mm_s);

    puts("ball_balance tests passed");
    return 0;
}
