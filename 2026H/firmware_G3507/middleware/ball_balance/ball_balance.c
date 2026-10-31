/**
 * @file  ball_balance.c
 * @brief 一维滚球在线终端约束控制实现。
 */
#include "ball_balance.h"

#include <math.h>
#include <stddef.h>

#define BALL_BALANCE_DEG_TO_RAD 0.01745329251994329577f
#define BALL_BALANCE_RAD_TO_DEG 57.295779513082320876f
#define BALL_BALANCE_PEAK_ACC_FACTOR 5.773502691896258f
#define BALL_BALANCE_PEAK_VEL_FACTOR 1.875f
#define BALL_BALANCE_PROFILE_SAMPLES 17U
#define BALL_BALANCE_DURATION_ITERATIONS 4U

typedef struct {
    float x0_mm;
    float v0_mm_s;
    float a0_mm_s2;
    float target_mm;
    float duration_s;
    float c3_mm;
    float c4_mm;
    float c5_mm;
    float peak_velocity_mm_s;
    float peak_acceleration_mm_s2;
    bool  limit_relaxed;
} BALL_BALANCE_PROFILE;

static float BallBalance_Clamp(float value, float low, float high){
    if (value < low){ return low; }
    if (value > high){ return high; }
    return value;
}

static float BallBalance_Max(float a, float b){
    return (a > b) ? a : b;
}

static bool BallBalance_ConfigValid(const BALL_BALANCE_CONFIG *config){
    return (config != NULL) &&
           (config->rolling_acceleration_gain_mm_s2 > 0.0f) &&
           (config->profile_max_velocity_mm_s > 0.0f) &&
           (config->profile_max_acceleration_mm_s2 > 0.0f) &&
           (config->profile_min_duration_s > 0.0f) &&
           (config->profile_max_duration_s >= config->profile_min_duration_s) &&
           (config->profile_lookahead_s > 0.0f) &&
           (config->position_feedback_gain_deg_per_mm >= 0.0f) &&
           (config->velocity_feedback_gain_deg_per_mm_s >= 0.0f) &&
           (config->feedback_limit_deg >= 0.0f) &&
           (config->zero_trim_ki_deg_per_mm_s >= 0.0f) &&
           (config->zero_trim_leak_tau_s > 0.0f) &&
           (config->zero_trim_limit_deg >= 0.0f) &&
           (config->angle_min_deg < config->angle_max_deg) &&
           (config->control_angle_limit_deg > 0.0f) &&
           (config->control_angle_limit_deg <= config->angle_max_deg) &&
           (config->control_angle_limit_deg <= -config->angle_min_deg) &&
           (config->angle_rate_near_deg_s > 0.0f) &&
           (config->angle_rate_far_deg_s > 0.0f) &&
           (config->angle_rate_brake_deg_s >= config->angle_rate_near_deg_s) &&
           (config->angle_rate_near_deg_s >= config->angle_rate_far_deg_s) &&
           (config->angle_rate_transition_mm > 0.0f) &&
           (config->stuck_min_error_mm >= 0.0f) &&
           (config->stuck_max_position_change_mm >= 0.0f) &&
           (config->stuck_min_command_deg >= 0.0f) &&
           (config->stuck_time_s >= 0.0f) &&
           (config->settled_position_mm >= 0.0f) &&
           (config->settled_speed_mm_s >= 0.0f) &&
           (config->settled_time_s >= 0.0f);
}

/*
 * x = x0 + v0*t + a0*t^2/2 + c3*s^3 + c4*s^4 + c5*s^5, s=t/T。
 * 系数由末端 x=target、v=0、a=0 唯一确定。因此完整轨迹天然满足：
 *   integral(a dt)=v(T)-v(0)=-v0
 *   integral(v dt)=x(T)-x(0)=target-x0
 */
static void BallBalance_SolveProfile(BALL_BALANCE_PROFILE *profile,
                                     float duration_s){
    float t2 = duration_s * duration_s;
    float dx = profile->target_mm -
        (profile->x0_mm + profile->v0_mm_s * duration_s +
         0.5f * profile->a0_mm_s2 * t2);
    float dv = -(profile->v0_mm_s + profile->a0_mm_s2 * duration_s) * duration_s;
    float da = -profile->a0_mm_s2 * t2;

    profile->duration_s = duration_s;
    profile->c3_mm = 10.0f * dx - 4.0f * dv + 0.5f * da;
    profile->c4_mm = -15.0f * dx + 7.0f * dv - da;
    profile->c5_mm = 6.0f * dx - 3.0f * dv + 0.5f * da;
}

static void BallBalance_EvaluateProfile(const BALL_BALANCE_PROFILE *profile,
                                        float time_s,
                                        float *x_mm,
                                        float *v_mm_s,
                                        float *a_mm_s2){
    float t = BallBalance_Clamp(time_s, 0.0f, profile->duration_s);
    float s = t / profile->duration_s;
    float s2 = s * s;
    float s3 = s2 * s;
    float s4 = s3 * s;
    float s5 = s4 * s;
    float t2 = profile->duration_s * profile->duration_s;

    *x_mm = profile->x0_mm + profile->v0_mm_s * t +
        0.5f * profile->a0_mm_s2 * t * t +
        profile->c3_mm * s3 + profile->c4_mm * s4 + profile->c5_mm * s5;
    *v_mm_s = profile->v0_mm_s + profile->a0_mm_s2 * t +
        (3.0f * profile->c3_mm * s2 + 4.0f * profile->c4_mm * s3 +
         5.0f * profile->c5_mm * s4) / profile->duration_s;
    *a_mm_s2 = profile->a0_mm_s2 +
        (6.0f * profile->c3_mm * s + 12.0f * profile->c4_mm * s2 +
         20.0f * profile->c5_mm * s3) / t2;
}

static void BallBalance_FindPeaks(BALL_BALANCE_PROFILE *profile){
    profile->peak_velocity_mm_s = 0.0f;
    profile->peak_acceleration_mm_s2 = 0.0f;
    for (unsigned i = 0U; i < BALL_BALANCE_PROFILE_SAMPLES; i++){
        float time_s = profile->duration_s * (float)i /
            (float)(BALL_BALANCE_PROFILE_SAMPLES - 1U);
        float x;
        float v;
        float a;
        BallBalance_EvaluateProfile(profile, time_s, &x, &v, &a);
        v = fabsf(v);
        a = fabsf(a);
        if (v > profile->peak_velocity_mm_s){ profile->peak_velocity_mm_s = v; }
        if (a > profile->peak_acceleration_mm_s2){
            profile->peak_acceleration_mm_s2 = a;
        }
    }
}

static void BallBalance_PlanProfile(const BALL_BALANCE_CONFIG *config,
                                    const BALL_BALANCE_INPUT *input,
                                    float target_mm,
                                    float zero_trim_deg,
                                    BALL_BALANCE_PROFILE *profile){
    profile->x0_mm = input->x_mm;
    profile->v0_mm_s = input->velocity_mm_s;
    profile->a0_mm_s2 = config->rolling_acceleration_gain_mm_s2 *
        sinf((input->actual_angle_deg - zero_trim_deg) * BALL_BALANCE_DEG_TO_RAD);
    profile->target_mm = target_mm;
    profile->limit_relaxed = false;

    float distance_mm = fabsf(target_mm - input->x_mm);
    float duration_s = sqrtf(BALL_BALANCE_PEAK_ACC_FACTOR * distance_mm /
                             config->profile_max_acceleration_mm_s2);
    duration_s = BallBalance_Max(duration_s,
        BALL_BALANCE_PEAK_VEL_FACTOR * distance_mm /
        config->profile_max_velocity_mm_s);
    duration_s = BallBalance_Max(duration_s,
        2.0f * fabsf(input->velocity_mm_s) /
        config->profile_max_acceleration_mm_s2);
    duration_s = BallBalance_Clamp(duration_s,
                                   config->profile_min_duration_s,
                                   config->profile_max_duration_s);

    float allowed_velocity = BallBalance_Max(config->profile_max_velocity_mm_s,
                                              fabsf(profile->v0_mm_s));
    float allowed_acceleration = BallBalance_Max(
        config->profile_max_acceleration_mm_s2, fabsf(profile->a0_mm_s2));
    for (unsigned iteration = 0U;
         iteration < BALL_BALANCE_DURATION_ITERATIONS; iteration++){
        BallBalance_SolveProfile(profile, duration_s);
        BallBalance_FindPeaks(profile);
        float scale_v = profile->peak_velocity_mm_s / allowed_velocity;
        float scale_a = sqrtf(profile->peak_acceleration_mm_s2 /
                              allowed_acceleration);
        float scale = BallBalance_Max(scale_v, scale_a);
        if (scale <= 1.001f){ break; }
        float longer_s = duration_s * scale * 1.01f;
        if (longer_s >= config->profile_max_duration_s){
            duration_s = config->profile_max_duration_s;
            break;
        }
        duration_s = longer_s;
    }
    BallBalance_SolveProfile(profile, duration_s);
    BallBalance_FindPeaks(profile);
    profile->limit_relaxed =
        (profile->peak_velocity_mm_s > config->profile_max_velocity_mm_s * 1.01f) ||
        (profile->peak_acceleration_mm_s2 >
         config->profile_max_acceleration_mm_s2 * 1.01f);
}

void BallBalance_Init(BALL_BALANCE_CONTROLLER *controller){
    if (controller == NULL){ return; }
    controller->target_x_mm = 0.0f;
    controller->zero_trim_deg = 0.0f;
    controller->output_deg = 0.0f;
    controller->previous_position_mm = 0.0f;
    controller->stagnant_elapsed_s = 0.0f;
    controller->settled_elapsed_s = 0.0f;
    controller->have_previous_position = false;
    controller->intercepted = false;
    controller->settled = false;
}

void BallBalance_Reset(BALL_BALANCE_CONTROLLER *controller,
                       float current_angle_deg){
    if (controller == NULL){ return; }
    controller->zero_trim_deg = 0.0f;
    controller->output_deg = current_angle_deg;
    controller->previous_position_mm = 0.0f;
    controller->stagnant_elapsed_s = 0.0f;
    controller->settled_elapsed_s = 0.0f;
    controller->have_previous_position = false;
    controller->intercepted = false;
    controller->settled = false;
}

void BallBalance_SetTarget(BALL_BALANCE_CONTROLLER *controller,
                           float target_x_mm){
    if (controller == NULL){ return; }
    controller->target_x_mm = target_x_mm;
    controller->settled_elapsed_s = 0.0f;
    controller->settled = false;
}

void BallBalance_SetZeroTrim(BALL_BALANCE_CONTROLLER *controller,
                            float zero_trim_deg){
    if (controller == NULL){ return; }
    controller->zero_trim_deg = zero_trim_deg;
}

bool BallBalance_Update(BALL_BALANCE_CONTROLLER *controller,
                        const BALL_BALANCE_CONFIG *config,
                        const BALL_BALANCE_INPUT *input,
                        BALL_BALANCE_OUTPUT *output){
    if ((controller == NULL) || (input == NULL) || (output == NULL) ||
        !BallBalance_ConfigValid(config) || (input->dt_s <= 0.0f)){
        return false;
    }

    float target_error_mm = controller->target_x_mm - input->x_mm;
    float abs_error = fabsf(target_error_mm);
    float abs_speed = fabsf(input->velocity_mm_s);

    bool trim_window = input->allow_zero_trim && !controller->intercepted &&
                       (abs_error >= config->trim_min_position_mm) &&
                       (abs_error <= config->trim_max_position_mm) &&
                       (abs_speed >= config->trim_min_speed_mm_s) &&
                       (abs_speed <= config->trim_max_speed_mm_s);
    if (trim_window){
        controller->zero_trim_deg += config->zero_trim_ki_deg_per_mm_s *
                                     target_error_mm * input->dt_s;
        controller->zero_trim_deg = BallBalance_Clamp(
            controller->zero_trim_deg,
            -config->zero_trim_limit_deg,
            config->zero_trim_limit_deg);
    } else{
        float leak = BallBalance_Clamp(input->dt_s /
            config->zero_trim_leak_tau_s, 0.0f, 1.0f);
        controller->zero_trim_deg *= 1.0f - leak;
    }

    BALL_BALANCE_PROFILE profile;
    BallBalance_PlanProfile(config, input, controller->target_x_mm,
                            controller->zero_trim_deg, &profile);
    float lookahead_s = BallBalance_Clamp(config->profile_lookahead_s,
                                          input->dt_s,
                                          profile.duration_s);
    float x_ref;
    float velocity_ref;
    float acceleration_ref;
    BallBalance_EvaluateProfile(&profile, lookahead_s,
                                &x_ref, &velocity_ref, &acceleration_ref);

    float acceleration_ratio = BallBalance_Clamp(
        acceleration_ref / config->rolling_acceleration_gain_mm_s2,
        -0.999f, 0.999f);
    float feedforward_deg = asinf(acceleration_ratio) * BALL_BALANCE_RAD_TO_DEG;
    float position_error_mm = x_ref - input->x_mm;
    float velocity_error_mm_s = velocity_ref - input->velocity_mm_s;
    float position_feedback_deg = config->position_feedback_gain_deg_per_mm *
                                  position_error_mm;
    float velocity_feedback_deg = config->velocity_feedback_gain_deg_per_mm_s *
                                  velocity_error_mm_s;
    float raw_feedback_deg = position_feedback_deg + velocity_feedback_deg;
    float feedback_deg = BallBalance_Clamp(raw_feedback_deg,
                                            -config->feedback_limit_deg,
                                            config->feedback_limit_deg);

    float requested_deg = controller->zero_trim_deg + feedforward_deg + feedback_deg;
    float unrestricted_deg = requested_deg;
    float control_min_deg = BallBalance_Max(config->angle_min_deg,
        controller->zero_trim_deg - config->control_angle_limit_deg);
    float control_max_deg = BallBalance_Clamp(
        controller->zero_trim_deg + config->control_angle_limit_deg,
        config->angle_min_deg, config->angle_max_deg);
    requested_deg = BallBalance_Clamp(requested_deg, control_min_deg, control_max_deg);
    bool saturated = (requested_deg != unrestricted_deg) ||
                     (feedback_deg != raw_feedback_deg);

    float position_change = controller->have_previous_position
        ? fabsf(input->x_mm - controller->previous_position_mm) : 0.0f;
    bool stagnant = controller->have_previous_position && !input->moving &&
                    (position_change <= config->stuck_max_position_change_mm) &&
                    (abs_error >= config->stuck_min_error_mm) &&
                    (fabsf(requested_deg - controller->zero_trim_deg) >=
                     config->stuck_min_command_deg);
    if (stagnant){
        controller->stagnant_elapsed_s += input->dt_s;
        if (controller->stagnant_elapsed_s >= config->stuck_time_s){
            controller->intercepted = true;
        }
    } else{
        controller->stagnant_elapsed_s = 0.0f;
        controller->intercepted = false;
    }
    controller->previous_position_mm = input->x_mm;
    controller->have_previous_position = true;

    float rate_ratio = abs_error / config->angle_rate_transition_mm;
    float scheduled_rate_deg_s = config->angle_rate_far_deg_s +
        (config->angle_rate_near_deg_s - config->angle_rate_far_deg_s) /
        (1.0f + rate_ratio * rate_ratio);
    bool braking = (abs_speed > config->settled_speed_mm_s) &&
                   ((input->velocity_mm_s * acceleration_ref) < 0.0f);
    if (braking){ scheduled_rate_deg_s = config->angle_rate_brake_deg_s; }
    float max_step_deg = scheduled_rate_deg_s * input->dt_s;
    float change_deg = BallBalance_Clamp(requested_deg - controller->output_deg,
                                         -max_step_deg, max_step_deg);
    bool rate_limited = change_deg != (requested_deg - controller->output_deg);
    controller->output_deg += change_deg;

    if ((abs_error <= config->settled_position_mm) &&
        (abs_speed <= config->settled_speed_mm_s)){
        controller->settled_elapsed_s += input->dt_s;
        if (controller->settled_elapsed_s >= config->settled_time_s){
            controller->settled = true;
        }
    } else{
        controller->settled_elapsed_s = 0.0f;
        controller->settled = false;
    }

    float terminal_x;
    float terminal_v;
    float terminal_a;
    BallBalance_EvaluateProfile(&profile, profile.duration_s,
                                &terminal_x, &terminal_v, &terminal_a);

    output->angle_deg = controller->output_deg;
    output->x_ref_mm = x_ref;
    output->velocity_ref_mm_s = velocity_ref;
    output->acceleration_ref_mm_s2 = acceleration_ref;
    output->feedforward_deg = feedforward_deg;
    output->position_feedback_deg = position_feedback_deg;
    output->velocity_feedback_deg = velocity_feedback_deg;
    output->feedback_deg = feedback_deg;
    output->position_error_mm = position_error_mm;
    output->velocity_error_mm_s = velocity_error_mm_s;
    output->model_acceleration_mm_s2 = profile.a0_mm_s2;
    output->profile_duration_s = profile.duration_s;
    output->profile_lookahead_s = lookahead_s;
    output->profile_peak_velocity_mm_s = profile.peak_velocity_mm_s;
    output->profile_peak_acceleration_mm_s2 = profile.peak_acceleration_mm_s2;
    output->acceleration_integral_mm_s = terminal_v - profile.v0_mm_s;
    output->velocity_integral_mm = terminal_x - profile.x0_mm;
    output->terminal_position_residual_mm = terminal_x - controller->target_x_mm;
    output->terminal_velocity_residual_mm_s = terminal_v;
    output->terminal_acceleration_residual_mm_s2 = terminal_a;
    output->angle_rate_limit_deg_s = scheduled_rate_deg_s;
    output->zero_trim_deg = controller->zero_trim_deg;
    output->profile_limit_relaxed = profile.limit_relaxed;
    output->saturated = saturated;
    output->rate_limited = rate_limited;
    output->braking = braking;
    output->intercepted = controller->intercepted;
    output->settled = controller->settled;
    return true;
}
