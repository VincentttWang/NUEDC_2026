/**
 * @file  ball_scurve.c
 * @brief 纯五次 S 曲线滚球点到点控制的实现。
 */
#include "ball_scurve.h"

#include <math.h>

#define BALL_SCURVE_RAD_TO_DEG 57.29577951308232f
#define BALL_SCURVE_GRAVITY_MM_S2 9806.65f

/*
 * 静止到静止的五次剖面峰值加速度 = PEAK_ACC_FACTOR·Δx/T²。
 * 推导：s = t/T 时 a(s) ∝ 60s − 180s² + 120s³，极值在 s = (1−1/√3)/2 = 0.21132，
 * 代回得 5.7735 = 10/√3。
 */
#define BALL_SCURVE_PEAK_ACC_FACTOR 5.773502691896258f
/* 同一剖面的峰值速度 = 1.875·Δx/T（s = 0.5 处 30s²(1−s)² 的极值 15/8）。 */
#define BALL_SCURVE_PEAK_VEL_FACTOR 1.875f
/* 时长迭代次数：起点带初速时闭式时长只是近似，需按真实峰值加速度收紧。 */
#define BALL_SCURVE_DURATION_ITERATIONS 4U
/* 峰值加速度校核的采样点数（含端点）。 */
#define BALL_SCURVE_ACC_SAMPLES 21U

static float BallScurve_Clamp(float value, float low, float high){
    if (value < low){ return low; }
    if (value > high){ return high; }
    return value;
}

static float BallScurve_Abs(float value){
    return (value < 0.0f) ? -value : value;
}

static float BallScurve_SmoothStep(float value){
    value = BallScurve_Clamp(value, 0.0f, 1.0f);
    return value * value * (3.0f - 2.0f * value);
}

static float BallScurve_FilterToward(float current, float target,
                                     float dt_s, float tau_s){
    if (tau_s <= 0.0f){ return target; }
    float alpha = BallScurve_Clamp(dt_s / (tau_s + dt_s), 0.0f, 1.0f);
    return current + alpha * (target - current);
}

/**
 * @brief 由归一化时间求剖面的位置/速度/加速度。
 *
 * x(t) = x0 + v0·t + a0·t²/2 + c0·s³ + c1·s⁴ + c2·s⁵,  s = t/T
 *
 * 三个系数由末端 x=target、v=0、a=0 三个条件唯一确定（见 PlanTo）。
 */
static void BallScurve_Evaluate(const BALL_SCURVE_CONTROLLER *controller,
                                float time_s,
                                float *x_mm,
                                float *v_mm_s,
                                float *a_mm_s2){
    float duration = controller->duration_s;
    float s = time_s / duration;
    float s2 = s * s;
    float s3 = s2 * s;
    float s4 = s3 * s;
    float s5 = s4 * s;
    const float *c = controller->c;

    *x_mm = controller->x0_mm + controller->v0_mm_s * time_s +
            0.5f * controller->a0_mm_s2 * time_s * time_s +
            c[0] * s3 + c[1] * s4 + c[2] * s5;

    *v_mm_s = controller->v0_mm_s + controller->a0_mm_s2 * time_s +
              (3.0f * c[0] * s2 + 4.0f * c[1] * s3 + 5.0f * c[2] * s4) / duration;

    *a_mm_s2 = controller->a0_mm_s2 +
               (6.0f * c[0] * s + 12.0f * c[1] * s2 + 20.0f * c[2] * s3) /
               (duration * duration);
}

/** 解出给定时长下的三个归一化系数。 */
static void BallScurve_SolveCoefficients(BALL_SCURVE_CONTROLLER *controller,
                                         float target_mm,
                                         float duration_s){
    /*
     * 记 T = duration，剩余量为末端条件减去"惯性外推"的部分：
     *   dx = target − (x0 + v0·T + a0·T²/2)
     *   dv = (0 − (v0 + a0·T))·T          （已乘 T，与 s 的求导对齐）
     *   da = (0 − a0)·T²
     * 则 c0 = 10dx − 4dv + 0.5da, c1 = −15dx + 7dv − da, c2 = 6dx − 3dv + 0.5da。
     * 校核：c0+c1+c2 = dx，3c0+4c1+5c2 = dv，6c0+12c1+20c2 = da。
     */
    float t2 = duration_s * duration_s;
    float dx = target_mm - (controller->x0_mm +
                            controller->v0_mm_s * duration_s +
                            0.5f * controller->a0_mm_s2 * t2);
    float dv = -(controller->v0_mm_s + controller->a0_mm_s2 * duration_s) * duration_s;
    float da = -controller->a0_mm_s2 * t2;

    controller->duration_s = duration_s;
    controller->c[0] = 10.0f * dx - 4.0f * dv + 0.5f * da;
    controller->c[1] = -15.0f * dx + 7.0f * dv - da;
    controller->c[2] = 6.0f * dx - 3.0f * dv + 0.5f * da;
}

/** 采样求剖面全程的峰值 |a|，用于时长迭代校核。 */
static float BallScurve_PeakAcceleration(const BALL_SCURVE_CONTROLLER *controller){
    float peak = 0.0f;
    for (unsigned i = 0U; i < BALL_SCURVE_ACC_SAMPLES; i++){
        float time_s = controller->duration_s *
                       ((float)i / (float)(BALL_SCURVE_ACC_SAMPLES - 1U));
        float x;
        float v;
        float a;
        BallScurve_Evaluate(controller, time_s, &x, &v, &a);
        a = BallScurve_Abs(a);
        if (a > peak){ peak = a; }
    }
    return peak;
}

void BallScurve_Init(BALL_SCURVE_CONTROLLER *controller){
    if (controller == 0){ return; }
    controller->x0_mm = 0.0f;
    controller->v0_mm_s = 0.0f;
    controller->a0_mm_s2 = 0.0f;
    for (unsigned i = 0U; i < BALL_SCURVE_COEFFICIENTS; i++){
        controller->c[i] = 0.0f;
    }
    controller->duration_s = 1.0f;
    controller->elapsed_s = 0.0f;
    controller->target_mm = 0.0f;
    controller->active = false;
    controller->last_angle_deg = 0.0f;
    controller->have_last_angle = false;
    controller->settled_elapsed_s = 0.0f;
    controller->settled = false;
    controller->dither_phase_rad = 0.0f;
    controller->dither_stuck_elapsed_s = 0.0f;
    controller->dither_on = false;
    controller->hold_integral_deg = 0.0f;
    controller->hold_integral_on = false;
    controller->breakout_angle_deg = 0.0f;
    controller->breakout_stuck_elapsed_s = 0.0f;
    controller->breakout_release_elapsed_s = 0.0f;
    controller->breakout_on = false;
    controller->brake_blend = 0.0f;
    controller->hold_blend = 0.0f;
    controller->hold_enter_elapsed_s = 0.0f;
    controller->hold_mode = false;
    controller->capture_blend = 0.0f;
    controller->replan_error_elapsed_s = 0.0f;
    controller->replan_cooldown_elapsed_s = 0.0f;
}

void BallScurve_Reset(BALL_SCURVE_CONTROLLER *controller, float current_angle_deg){
    if (controller == 0){ return; }
    float target = controller->target_mm;
    BallScurve_Init(controller);
    controller->target_mm = target;
    controller->last_angle_deg = current_angle_deg;
    controller->have_last_angle = true;
}

float BallScurve_EstimateDuration(const BALL_SCURVE_CONFIG *config, float distance_mm){
    if ((config == 0) || (config->max_acceleration_mm_s2 <= 0.0f) ||
        (config->max_velocity_mm_s <= 0.0f)){
        return 0.0f;
    }
    float distance = BallScurve_Abs(distance_mm);
    float by_acceleration = sqrtf(BALL_SCURVE_PEAK_ACC_FACTOR * distance /
                                  config->max_acceleration_mm_s2);
    float by_velocity = BALL_SCURVE_PEAK_VEL_FACTOR * distance / config->max_velocity_mm_s;
    float duration = (by_acceleration > by_velocity) ? by_acceleration : by_velocity;
    if (duration < config->min_duration_s){ duration = config->min_duration_s; }
    if ((config->max_duration_s > 0.0f) && (duration > config->max_duration_s)){
        duration = config->max_duration_s;
    }
    return duration;
}

bool BallScurve_PlanTo(BALL_SCURVE_CONTROLLER *controller,
                       const BALL_SCURVE_CONFIG *config,
                       float x_now_mm,
                       float v_now_mm_s,
                       float target_mm){
    if ((controller == 0) || (config == 0)){ return false; }
    if ((config->rolling_acceleration_gain_mm_s2 <= 0.0f) ||
        (config->max_acceleration_mm_s2 <= 0.0f) ||
        (config->max_velocity_mm_s <= 0.0f) ||
        (config->min_duration_s <= 0.0f)){
        return false;
    }

    controller->x0_mm = x_now_mm;
    controller->v0_mm_s = v_now_mm_s;
    /*
     * 规划起点加速度取 0 而不是实测加速度：视觉只给位置，二阶差分的噪声
     * 远大于信号，把它塞进边界条件只会让剖面开头出现虚假的大加速度。
     * 代价是剖面首拍的 a_ref 与实际有阶跃，由输出斜率限制吸收。
     */
    controller->a0_mm_s2 = 0.0f;
    controller->target_mm = target_mm;
    controller->elapsed_s = 0.0f;

    float duration = BallScurve_EstimateDuration(config, target_mm - x_now_mm);
    if (duration <= 0.0f){ return false; }
    BallScurve_SolveCoefficients(controller, target_mm, duration);

    /*
     * 起点带初速时，闭式时长公式（按静止到静止导出）会低估峰值加速度，
     * 因此按实际采样峰值迭代收紧：T ← T·sqrt(peak/limit)。
     * a ∝ 1/T²，一次迭代即接近收敛，取 4 次留足余量。
     */
    for (unsigned i = 0U; i < BALL_SCURVE_DURATION_ITERATIONS; i++){
        float peak = BallScurve_PeakAcceleration(controller);
        if (peak <= config->max_acceleration_mm_s2){ break; }
        duration *= sqrtf(peak / config->max_acceleration_mm_s2);
        if ((config->max_duration_s > 0.0f) && (duration > config->max_duration_s)){
            duration = config->max_duration_s;
            BallScurve_SolveCoefficients(controller, target_mm, duration);
            break;
        }
        BallScurve_SolveCoefficients(controller, target_mm, duration);
    }

    controller->active = true;
    controller->settled_elapsed_s = 0.0f;
    controller->settled = false;
    /* 新航段不能继承上一目标的单向脱困方向或驻留计时。 */
    controller->dither_phase_rad = 0.0f;
    controller->dither_stuck_elapsed_s = 0.0f;
    controller->dither_on = false;
    controller->hold_integral_deg = 0.0f;
    controller->hold_integral_on = false;
    controller->breakout_angle_deg = 0.0f;
    controller->breakout_stuck_elapsed_s = 0.0f;
    controller->breakout_release_elapsed_s = 0.0f;
    controller->breakout_on = false;
    controller->hold_enter_elapsed_s = 0.0f;
    controller->hold_mode = false;
    controller->capture_blend = 0.0f;
    controller->replan_error_elapsed_s = 0.0f;
    controller->replan_cooldown_elapsed_s = 0.0f;
    return true;
}

bool BallScurve_IsProfileFinished(const BALL_SCURVE_CONTROLLER *controller){
    if (controller == 0){ return true; }
    return !controller->active;
}

bool BallScurve_Update(BALL_SCURVE_CONTROLLER *controller,
                       const BALL_SCURVE_CONFIG *config,
                       const BALL_SCURVE_INPUT *input,
                       BALL_SCURVE_OUTPUT *output){
    if ((controller == 0) || (config == 0) || (input == 0) || (output == 0)){
        return false;
    }
    if ((input->dt_s <= 0.0f) || (config->rolling_acceleration_gain_mm_s2 <= 0.0f)){
        return false;
    }

    /* --- 1. 推进剖面时钟并取参考量 --- */
    float x_ref;
    float v_ref;
    float a_ref;
    if (controller->active){
        controller->elapsed_s += input->dt_s;
        if (controller->elapsed_s >= controller->duration_s){
            controller->elapsed_s = controller->duration_s;
            controller->active = false;
        }
        BallScurve_Evaluate(controller, controller->elapsed_s, &x_ref, &v_ref, &a_ref);
    } else{
        /*
         * 剖面走完后参考量钉在目标点，控制律退化为绕目标的纯 PD。
         * 这里刻意不做任何低速捕获处理——本模块就是要暴露这个基线的落点误差。
         */
        x_ref = controller->target_mm;
        v_ref = 0.0f;
        a_ref = 0.0f;
    }

    /* --- 2. 受控重规划：误差需驻留且带冷却，绝不允许每拍重启剖面 --- */
    bool replanned = false;
    if (controller->active){
        controller->replan_cooldown_elapsed_s += input->dt_s;
        float abs_target_before_capture =
            BallScurve_Abs(controller->target_mm - input->x_mm);
        bool outside_capture = (config->capture_enter_error_mm <= 0.0f) ||
            (abs_target_before_capture > config->capture_enter_error_mm);
        bool position_bad = (config->replan_error_mm > 0.0f) &&
            (BallScurve_Abs(x_ref - input->x_mm) > config->replan_error_mm);
        bool velocity_bad = (config->replan_velocity_error_mm_s > 0.0f) &&
            (BallScurve_Abs(v_ref - input->velocity_mm_s) >
             config->replan_velocity_error_mm_s);
        bool fresh_velocity = input->velocity_trusted &&
            (input->measurement_age_ms <= config->velocity_floor_weight_age_ms);
        bool replan_bad = fresh_velocity && outside_capture &&
            (position_bad || velocity_bad);
        if (replan_bad){
            controller->replan_error_elapsed_s += input->dt_s;
        } else{
            controller->replan_error_elapsed_s = 0.0f;
        }
        if (replan_bad &&
            (controller->replan_error_elapsed_s >= config->replan_dwell_s) &&
            (controller->replan_cooldown_elapsed_s >= config->replan_cooldown_s)){
            float target = controller->target_mm;
            if (BallScurve_PlanTo(controller, config, input->x_mm,
                                  input->velocity_mm_s, target)){
                BallScurve_Evaluate(controller, 0.0f, &x_ref, &v_ref, &a_ref);
                replanned = true;
            }
        }
    } else{
        controller->replan_error_elapsed_s = 0.0f;
    }

    /* --- 3. 末端捕获：按实际目标误差把移动参考平滑拉向固定目标 --- */
    float target_error_for_capture = controller->target_mm - input->x_mm;
    float capture_target = controller->active ? 0.0f : 1.0f;
    float capture_span = config->capture_enter_error_mm - config->capture_full_error_mm;
    if (controller->active && (config->capture_enter_error_mm > 0.0f) &&
        (capture_span > 0.0f)){
        capture_target = BallScurve_SmoothStep(
            (config->capture_enter_error_mm -
             BallScurve_Abs(target_error_for_capture)) / capture_span);
    }
    controller->capture_blend = BallScurve_FilterToward(
        controller->capture_blend, capture_target, input->dt_s,
        config->capture_blend_tau_s);
    x_ref += controller->capture_blend * (controller->target_mm - x_ref);
    v_ref *= (1.0f - controller->capture_blend);
    a_ref *= (1.0f - controller->capture_blend);

    /* --- 4. 已知轨迹加速度预览：提前抵消步进位置环滞后 --- */
    float acceleration_preview = a_ref;
    if (controller->active && (config->acceleration_preview_s > 0.0f)){
        float preview_time = controller->elapsed_s + config->acceleration_preview_s;
        if (preview_time > controller->duration_s){
            preview_time = controller->duration_s;
        }
        float preview_x;
        float preview_v;
        BallScurve_Evaluate(controller, preview_time, &preview_x, &preview_v,
                            &acceleration_preview);
        acceleration_preview *= (1.0f - controller->capture_blend);
    }
    float nominal_ratio = BallScurve_Clamp(
        a_ref / config->rolling_acceleration_gain_mm_s2, -0.999f, 0.999f);
    float nominal_feedforward_deg =
        asinf(nominal_ratio) * BALL_SCURVE_RAD_TO_DEG;
    float preview_ratio = BallScurve_Clamp(
        acceleration_preview / config->rolling_acceleration_gain_mm_s2,
        -0.999f, 0.999f);
    float feedforward_deg = asinf(preview_ratio) * BALL_SCURVE_RAD_TO_DEG;
    float actuator_lead_deg = feedforward_deg - nominal_feedforward_deg;

    /* --- 5. 滚阻前馈，按参考速度方向施加 --- */
    float rolling_ff_deg = 0.0f;
    if ((config->rolling_resistance_deg != 0.0f) &&
        (BallScurve_Abs(v_ref) > config->rolling_ff_speed_deadband_mm_s)){
        rolling_ff_deg = (v_ref > 0.0f) ? config->rolling_resistance_deg
                                        : -config->rolling_resistance_deg;
    }

    /* --- 6. MOVE / BRAKE / HOLD 增益调度 --- */
    float position_error = x_ref - input->x_mm;
    float velocity_error = v_ref - input->velocity_mm_s;
    float target_error_now = controller->target_mm - input->x_mm;
    float abs_target_error = BallScurve_Abs(target_error_now);
    float closing_velocity = 0.0f;
    if (target_error_now > 0.0f){
        closing_velocity = input->velocity_mm_s;
    } else if (target_error_now < 0.0f){
        closing_velocity = -input->velocity_mm_s;
    }

    float velocity_weight = 1.0f;
    if (config->gain_schedule_enabled > 0.0f){
        float floor_weight = BallScurve_Clamp(config->velocity_untrusted_weight,
                                               0.0f, 1.0f);
        if (!input->velocity_trusted){
            velocity_weight = floor_weight;
        } else if (input->measurement_age_ms > config->velocity_full_weight_age_ms){
            float age_span = config->velocity_floor_weight_age_ms -
                             config->velocity_full_weight_age_ms;
            if (age_span <= 0.0f){
                velocity_weight = floor_weight;
            } else{
                float age_blend = (input->measurement_age_ms -
                                   config->velocity_full_weight_age_ms) / age_span;
                velocity_weight = 1.0f - BallScurve_Clamp(age_blend, 0.0f, 1.0f) *
                                  (1.0f - floor_weight);
            }
        }
    }
    if (!controller->active && !input->moving){
        float stationary_weight = BallScurve_Clamp(
            config->stationary_velocity_weight, 0.0f, 1.0f);
        if (velocity_weight > stationary_weight){
            velocity_weight = stationary_weight;
        }
    }

    float stopping_distance = 0.0f;
    float brake_target = 0.0f;
    if ((config->gain_schedule_enabled > 0.0f) && controller->active &&
        (closing_velocity > 0.0f) && (config->brake_acceleration_mm_s2 > 0.0f)){
        stopping_distance = closing_velocity * config->brake_delay_s +
            closing_velocity * closing_velocity /
            (2.0f * config->brake_acceleration_mm_s2);
        float error_scale = (abs_target_error > 0.1f) ? abs_target_error : 0.1f;
        float stop_ratio = stopping_distance / error_scale;
        float ratio_span = config->brake_blend_full_ratio -
                           config->brake_blend_start_ratio;
        if (ratio_span > 0.0f){
            brake_target = BallScurve_SmoothStep(
                (stop_ratio - config->brake_blend_start_ratio) / ratio_span);
        }
    }
    controller->brake_blend = BallScurve_FilterToward(
        controller->brake_blend, brake_target, input->dt_s,
        config->brake_blend_tau_s);

    if (config->gain_schedule_enabled > 0.0f){
        if (controller->hold_mode){
            if (controller->active ||
                (abs_target_error >= config->hold_exit_error_mm) ||
                (BallScurve_Abs(input->velocity_mm_s) >= config->hold_exit_speed_mm_s)){
                controller->hold_mode = false;
                controller->hold_enter_elapsed_s = 0.0f;
            }
        } else if (!controller->active && input->velocity_trusted &&
                   (input->measurement_age_ms <=
                    config->velocity_floor_weight_age_ms) &&
                   (abs_target_error <= config->hold_enter_error_mm) &&
                   (BallScurve_Abs(input->velocity_mm_s) <=
                    config->hold_enter_speed_mm_s)){
            controller->hold_enter_elapsed_s += input->dt_s;
            if (controller->hold_enter_elapsed_s >= config->hold_enter_dwell_s){
                controller->hold_mode = true;
            }
        } else{
            controller->hold_enter_elapsed_s = 0.0f;
        }
    } else{
        controller->brake_blend = 0.0f;
        controller->hold_blend = 0.0f;
        controller->hold_mode = false;
        controller->hold_enter_elapsed_s = 0.0f;
    }
    controller->hold_blend = BallScurve_FilterToward(
        controller->hold_blend, controller->hold_mode ? 1.0f : 0.0f,
        input->dt_s, config->hold_blend_tau_s);

    float move_brake_kp = config->kp_deg_per_mm + controller->brake_blend *
        (config->brake_kp_deg_per_mm - config->kp_deg_per_mm);
    float move_brake_kd = config->kd_deg_per_mm_s + controller->brake_blend *
        (config->brake_kd_deg_per_mm_s - config->kd_deg_per_mm_s);
    float effective_kp = move_brake_kp + controller->hold_blend *
        (config->hold_kp_deg_per_mm - move_brake_kp);
    float effective_kd = move_brake_kd + controller->hold_blend *
        (config->hold_kd_deg_per_mm_s - move_brake_kd);
    if (config->gain_schedule_enabled <= 0.0f){
        effective_kp = config->kp_deg_per_mm;
        effective_kd = config->kd_deg_per_mm_s;
        velocity_weight = 1.0f;
    }

    float raw_feedback_deg = effective_kp * position_error +
        velocity_weight * effective_kd * velocity_error;
    float feedback_deg = raw_feedback_deg;
    if (config->feedback_limit_deg > 0.0f){
        feedback_deg = BallScurve_Clamp(feedback_deg,
                                        -config->feedback_limit_deg,
                                        config->feedback_limit_deg);
    }
    bool feedback_clipped = (feedback_deg != raw_feedback_deg);

    /* --- 6. 抖动：只在球确实被静摩擦钉住时注入 --- */
    /*
     * 触发条件三选三：剖面已走完（HOLD 段）、误差仍超判据、球基本不动。
     * 前两条保证"确实需要"，第三条保证"确实是卡住而不是还在滚"。
     * 误差进入 dither_min_error_mm 后自动停振，让静摩擦把球钉在那里——
     * 这同时提供了捕获：抖动破摩擦让球爬过去，停振让摩擦把它锁住。
     */
    float dither_deg = 0.0f;
    if ((config->dither_amplitude_deg > 0.0f) && !controller->active){
        bool needed =
            (BallScurve_Abs(target_error_now) > config->dither_min_error_mm) &&
            (BallScurve_Abs(input->velocity_mm_s) < config->dither_max_speed_mm_s);
        if (needed){
            controller->dither_stuck_elapsed_s += input->dt_s;
        } else{
            controller->dither_stuck_elapsed_s = 0.0f;
        }
        controller->dither_on =
            (controller->dither_stuck_elapsed_s >= config->dither_dwell_s);
    } else{
        controller->dither_stuck_elapsed_s = 0.0f;
        controller->dither_on = false;
    }
    if (controller->dither_on){
        controller->dither_phase_rad +=
            2.0f * 3.14159265358979f * config->dither_frequency_hz * input->dt_s;
        /* 相位取模，避免长时间累加后单精度失去分辨率。 */
        while (controller->dither_phase_rad > 2.0f * 3.14159265358979f){
            controller->dither_phase_rad -= 2.0f * 3.14159265358979f;
        }
        dither_deg = config->dither_amplitude_deg * sinf(controller->dither_phase_rad);
    } else{
        /* 从零相位重新起振，保证第一拍不产生角度阶跃。 */
        controller->dither_phase_rad = 0.0f;
    }

    /* --- 6b. 小误差静止积分：不动时补力，一动就快速撤销 --- */
    float hold_integral_deg = 0.0f;
    bool hold_integral_enabled =
        (config->hold_integral_ki_deg_per_mm_s > 0.0f) &&
        (config->hold_integral_max_error_mm > config->hold_integral_min_error_mm);
    if (hold_integral_enabled){
        float abs_err = BallScurve_Abs(target_error_now);
        float abs_v = BallScurve_Abs(input->velocity_mm_s);
        bool fresh_velocity = input->velocity_trusted &&
            (input->measurement_age_ms <= config->velocity_floor_weight_age_ms);
        bool in_small_error_band =
            (abs_err > config->hold_integral_min_error_mm) &&
            (abs_err <= config->hold_integral_max_error_mm);
        bool stationary = fresh_velocity && !input->moving &&
            (abs_v <= config->hold_integral_max_speed_mm_s);
        bool motion_release = input->moving ||
            (abs_v >= config->hold_integral_release_speed_mm_s);
        bool release = controller->active || !fresh_velocity || motion_release ||
            !in_small_error_band;

        controller->hold_integral_on = !controller->active &&
            in_small_error_band && stationary;
        if (controller->hold_integral_on){
            controller->hold_integral_deg +=
                config->hold_integral_ki_deg_per_mm_s * target_error_now * input->dt_s;
        } else if (release){
            /*
             * 球一动，累计倾角就从“克服静摩擦”变成额外加速误差。除固定退积分外，
             * 再按实测球速追加反向补偿；速度越大，撤销越快。速度不可信时只用
             * 固定速率，避免噪声把积分瞬间抽空。
             */
            float release_rate = config->hold_integral_release_rate_deg_s;
            if (motion_release && fresh_velocity){
                release_rate += config->hold_integral_motion_comp_deg_per_mm * abs_v;
            }
            float step = release_rate * input->dt_s;
            if (controller->hold_integral_deg > step){
                controller->hold_integral_deg -= step;
            } else if (controller->hold_integral_deg < -step){
                controller->hold_integral_deg += step;
            } else{
                controller->hold_integral_deg = 0.0f;
            }
        }
        hold_integral_deg = controller->hold_integral_deg;
    } else{
        controller->hold_integral_deg = 0.0f;
        controller->hold_integral_on = false;
    }

    /* --- 6c. 单向渐增脱困：只朝目标方向加倾角，球一动就撤 --- */
    /*
     * 与抖动**互斥**（见下方 breakout_enabled 判断），因为两者解决同一个问题
     * 但代价不同：
     *
     *   抖动   —— 双向往复。能脱困，但向系统注入往复能量，末端不安静；
     *              且经步进位置环衰减后（τ=1/SERVO_KP=0.333 s @ KP=3），
     *              2 Hz 时管身只剩指令的 23%，实际破不了 0.62° 的脱离角。
     *   单向脱困 —— 只朝目标方向加倾角。是直流量，**不受位置环频率衰减**，
     *              指令 1.0° 就是管身 1.0°，权限真实可用。
     *
     * 小误差由上面的有界积分处理；这里只接管更大的误差。两个机构不叠加。
     */
    float breakout_deg = 0.0f;
    bool breakout_enabled = (config->breakout_max_angle_deg > 0.0f);
    if (breakout_enabled){
        /* 互斥：脱困接管后抖动必须闭嘴，否则两个机构互相打架。 */
        dither_deg = 0.0f;
        controller->dither_on = false;
        controller->dither_phase_rad = 0.0f;

        float abs_err = BallScurve_Abs(target_error_now);
        float abs_v = BallScurve_Abs(input->velocity_mm_s);

        if (!controller->breakout_on){
            /*
             * 触发三条同时满足并持续 dwell：剖面已结束、误差仍大、球基本不动。
             * 必须看速度——只看误差会在移动末段（球高速穿过目标附近）误触发。
             */
            float breakout_error_floor = config->breakout_min_error_mm;
            if (hold_integral_enabled &&
                (config->hold_integral_max_error_mm > breakout_error_floor)){
                breakout_error_floor = config->hold_integral_max_error_mm;
            }
            bool stuck = !controller->active &&
                         (abs_err > breakout_error_floor) &&
                         (BallScurve_Abs(controller->hold_integral_deg) < 0.001f) &&
                         (abs_v < config->breakout_max_speed_mm_s);
            if (stuck){
                controller->breakout_stuck_elapsed_s += input->dt_s;
                if (controller->breakout_stuck_elapsed_s >= config->breakout_dwell_s){
                    controller->breakout_on = true;
                    controller->breakout_release_elapsed_s = 0.0f;
                }
            } else{
                controller->breakout_stuck_elapsed_s = 0.0f;
            }
        }

        if (controller->breakout_on){
            /*
             * 释放判据不能用 v != 0：视觉速度有量化（1 mm/s 台阶）和延迟，
             * 噪声就能满足。要求「速度方向确实朝向目标」**且**「超过可靠阈值」，
             * 再持续 release_dwell 才认定脱困成功。
             */
            bool moving_to_target =
                ((input->velocity_mm_s * target_error_now) > 0.0f) &&
                (abs_v > config->breakout_release_speed_mm_s);
            if (moving_to_target){
                controller->breakout_release_elapsed_s += input->dt_s;
            } else{
                controller->breakout_release_elapsed_s = 0.0f;
            }

            /*
             * ⚠ 必须带上 moving_to_target 这一项。少了它，当
             *   breakout_release_dwell_s = 0 时 `elapsed >= 0` 恒真，release
             *   首拍就成立，倾角永远爬不起来——脱困功能**静默失效**，而遥测里
             *   brk 一直是 0，看不出是被这个判据卡死的。
             */
            bool release = (moving_to_target &&
                            (controller->breakout_release_elapsed_s >=
                             config->breakout_release_dwell_s)) ||
                           (abs_err <= config->breakout_min_error_mm);
            if (release){
                /* 快速斜坡归零而非瞬间置 0——瞬间撤销就是一个角度阶跃。 */
                float step = config->breakout_release_rate_deg_s * input->dt_s;
                if (controller->breakout_angle_deg > step){
                    controller->breakout_angle_deg -= step;
                } else if (controller->breakout_angle_deg < -step){
                    controller->breakout_angle_deg += step;
                } else{
                    controller->breakout_angle_deg = 0.0f;
                    controller->breakout_on = false;
                    controller->breakout_stuck_elapsed_s = 0.0f;
                    controller->breakout_release_elapsed_s = 0.0f;
                }
            } else{
                /* 朝目标方向按斜坡加倾角，钳在上限。 */
                float dir = (target_error_now >= 0.0f) ? 1.0f : -1.0f;
                controller->breakout_angle_deg +=
                    dir * config->breakout_ramp_rate_deg_s * input->dt_s;
                controller->breakout_angle_deg =
                    BallScurve_Clamp(controller->breakout_angle_deg,
                                     -config->breakout_max_angle_deg,
                                     config->breakout_max_angle_deg);
            }
        }
        breakout_deg = controller->breakout_angle_deg;
    } else{
        controller->breakout_angle_deg = 0.0f;
        controller->breakout_stuck_elapsed_s = 0.0f;
        controller->breakout_release_elapsed_s = 0.0f;
        controller->breakout_on = false;
    }

    /*
     * --- 7. 车辆纵向加速度前馈 ---
     * 车体系内 (7/5)x_ddot = g*sin(theta) - a_vehicle*cos(theta)。
     * 令相对加速度为 0 得 tan(theta_ff)=a_vehicle/g；滚动系数在等式两侧约掉，
     * 因而这里必须除以 g，而不是滚球增益 (5/7)g。
     */
    float vehicle_acceleration_ff_deg =
        atanf(input->vehicle_acceleration_mm_s2 / BALL_SCURVE_GRAVITY_MM_S2) *
        BALL_SCURVE_RAD_TO_DEG;

    /* --- 8. 合成、限幅、斜率限制 --- */
    float command = config->level_bias_deg + feedforward_deg + rolling_ff_deg +
                    feedback_deg + vehicle_acceleration_ff_deg +
                    hold_integral_deg + dither_deg + breakout_deg;
    /*
     * 积分器自身不设角度上限，但总指令若已撞物理限位，就撤回本拍继续朝
     * 饱和方向的积分增量。它是抗饱和回算，不是额外倾角钳位：离开饱和后
     * 积分仍可继续增长，已有积分量也不会被截断。
     */
    bool integral_worsens_saturation = controller->hold_integral_on &&
        (((command > config->angle_max_deg) && (target_error_now > 0.0f)) ||
         ((command < config->angle_min_deg) && (target_error_now < 0.0f)));
    if (integral_worsens_saturation){
        float rejected_increment = config->hold_integral_ki_deg_per_mm_s *
            target_error_now * input->dt_s;
        controller->hold_integral_deg -= rejected_increment;
        hold_integral_deg = controller->hold_integral_deg;
        command -= rejected_increment;
    }
    float limited = BallScurve_Clamp(command, config->angle_min_deg, config->angle_max_deg);
    bool saturated = (limited != command);

    bool rate_limited = false;
    if (config->angle_rate_limit_deg_s > 0.0f){
        /*
         * 斜率限制的基准是**上一拍下发的指令**，不是编码器实测角。用实测角会
         * 把步进的跟踪滞后叠进来，形成"指令追不上、滞后越积越多"的正反馈。
         * 实测角只在 Reset 时用来续接一次。
         */
        if (!controller->have_last_angle){
            controller->last_angle_deg = input->actual_angle_deg;
            controller->have_last_angle = true;
        }
        float max_step = config->angle_rate_limit_deg_s * input->dt_s;
        float delta = limited - controller->last_angle_deg;
        if (delta > max_step){
            limited = controller->last_angle_deg + max_step;
            rate_limited = true;
        } else if (delta < -max_step){
            limited = controller->last_angle_deg - max_step;
            rate_limited = true;
        }
    }
    controller->last_angle_deg = limited;
    controller->have_last_angle = true;

    /* --- 9. 稳定判据（纯遥测）--- */
    float target_error = target_error_now;
    if (!controller->active &&
        (BallScurve_Abs(target_error) <= config->settled_position_mm) &&
        (BallScurve_Abs(input->velocity_mm_s) <= config->settled_speed_mm_s)){
        controller->settled_elapsed_s += input->dt_s;
        if (controller->settled_elapsed_s >= config->settled_time_s){
            controller->settled = true;
        }
    } else{
        controller->settled_elapsed_s = 0.0f;
        controller->settled = false;
    }

    output->angle_deg = limited;
    output->x_ref_mm = x_ref;
    output->v_ref_mm_s = v_ref;
    output->a_ref_mm_s2 = a_ref;
    output->acceleration_preview_mm_s2 = acceleration_preview;
    output->feedforward_deg = feedforward_deg;
    output->actuator_lead_deg = actuator_lead_deg;
    output->rolling_ff_deg = rolling_ff_deg;
    output->feedback_deg = feedback_deg;
    output->dither_deg = dither_deg;
    output->hold_integral_deg = hold_integral_deg;
    output->hold_integral_on = controller->hold_integral_on;
    output->breakout_deg = breakout_deg;
    output->breakout_on = controller->breakout_on;
    output->breakout_stuck_s = controller->breakout_stuck_elapsed_s;
    output->breakout_release_s = controller->breakout_release_elapsed_s;
    output->position_error_mm = position_error;
    output->velocity_error_mm_s = velocity_error;
    output->effective_kp_deg_per_mm = effective_kp;
    output->effective_kd_deg_per_mm_s = effective_kd;
    output->brake_blend = controller->brake_blend;
    output->hold_blend = controller->hold_blend;
    output->capture_blend = controller->capture_blend;
    output->stopping_distance_mm = stopping_distance;
    output->closing_velocity_mm_s = closing_velocity;
    output->velocity_weight = velocity_weight;
    output->profile_time_s = controller->elapsed_s;
    output->profile_duration_s = controller->duration_s;
    output->profile_active = controller->active;
    output->saturated = saturated;
    output->feedback_clipped = feedback_clipped;
    output->rate_limited = rate_limited;
    output->dither_on = controller->dither_on;
    output->replanned = replanned;
    output->settled = controller->settled;
    output->gain_mode = (config->gain_schedule_enabled <= 0.0f)
        ? BALL_SCURVE_GAIN_MOVE
        : ((controller->hold_blend >= 0.5f)
        ? BALL_SCURVE_GAIN_HOLD
        : ((controller->brake_blend >= 0.5f)
            ? BALL_SCURVE_GAIN_BRAKE : BALL_SCURVE_GAIN_MOVE));
    return true;
}
