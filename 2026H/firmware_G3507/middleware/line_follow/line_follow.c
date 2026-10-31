/**
 * @file  line_follow.c
 * @brief Middleware 层巡线闭环控制器实现。
 */
#include "line_follow.h"

#include "chassis.h"
#include "filter/filter.h"
#include "kinematics/kinematics.h"
#include "wit_sdk.h"

#include <stddef.h>

static const float sensor_position[LINE_FOLLOW_SENSOR_COUNT] = {
    -40.0000f, -28.5714f, -17.1429f, -5.7143f,
      5.7143f,  17.1429f,  28.5714f, 40.0000f,
};

static bool LineFollow_IsSensorEnabled(uint8_t index){
    return ((LINE_FOLLOW_ACTIVE_SENSOR_MASK & (1U << index)) != 0U);
}

static LINE_FOLLOW_CONFIG line_follow_config;
static LINE_FOLLOW_OUTPUT line_follow_output;
static PID_CONTROLLER line_follow_pid;
static bool line_follow_initialized;
static float line_follow_filtered_error;

LINE_FOLLOW_CONFIG LineFollow_GetDefaultConfig(void){
    /* 默认使用陀螺串级；PID_CONFIG 仅供关闭增稳后的退化路径使用。 */
    LINE_FOLLOW_CONFIG config = {
        .base_duty = LINE_FOLLOW_DEFAULT_BASE_DUTY,
        .sensor_position_scale = LINE_FOLLOW_DEFAULT_POSITION_SCALE,
        .output_limit = LINE_FOLLOW_DEFAULT_OUTPUT_LIMIT,
        .differential_limit = LINE_FOLLOW_DEFAULT_DIFFERENTIAL_LIMIT,
        .pid_config = {
            .kp = LINE_FOLLOW_DEFAULT_PID_KP,
            .ki = LINE_FOLLOW_DEFAULT_PID_KI,
            .kd = LINE_FOLLOW_DEFAULT_PID_KD,
            .integral_limit = LINE_FOLLOW_DEFAULT_PID_INTEGRAL_LIMIT,
            .output_limit = LINE_FOLLOW_DEFAULT_PID_OUTPUT_LIMIT,
            .mode = LINE_FOLLOW_DEFAULT_PID_MODE,
        },
        .gyro_stab_enabled = LINE_FOLLOW_DEFAULT_GYRO_STAB_ENABLED,
        .gyro_line_kp = LINE_FOLLOW_DEFAULT_GYRO_LINE_KP,
        .gyro_stab_kp = LINE_FOLLOW_DEFAULT_GYRO_STAB_KP,
        .omega_ref_limit = LINE_FOLLOW_DEFAULT_OMEGA_REF_LIMIT,
        .omega_line_limit = LINE_FOLLOW_DEFAULT_OMEGA_LINE_LIMIT,
        .gyro_z_sign = LINE_FOLLOW_DEFAULT_GYRO_Z_SIGN,
    };

    return config;
}

static void LineFollow_EnsureInitialized(void){
    if (!line_follow_initialized){
        LineFollow_Init(NULL);
    }
}

void LineFollow_Init(const LINE_FOLLOW_CONFIG *config){
    if (config == NULL){
        line_follow_config = LineFollow_GetDefaultConfig();
    } else{
        line_follow_config = *config;
    }

    PID_Init(&line_follow_pid, &line_follow_config.pid_config);
    LineFollow_Reset();
    line_follow_initialized = true;
}

void LineFollow_Reset(void){
    PID_Reset(&line_follow_pid);

    line_follow_filtered_error = 0.0f;
    line_follow_output.level_mask = 0U;
    line_follow_output.black_count = 0U;
    line_follow_output.error = 0.0f;
    line_follow_output.correction = 0.0f;
    line_follow_output.omega_ref_deg_s = 0.0f;
    line_follow_output.omega_measured_deg_s = 0.0f;
    line_follow_output.left_duty = 0.0f;
    line_follow_output.right_duty = 0.0f;
    line_follow_output.line_lost = true;
}

BSP_STATUS LineFollow_EvaluateDetectedMask(uint8_t detected_mask, float dt_s,
                                           LINE_FOLLOW_OUTPUT *out){
    return LineFollow_EvaluateDetectedMaskWithOmegaFeedforward(
        detected_mask, dt_s, 0.0f, out);
}

BSP_STATUS LineFollow_EvaluateDetectedMaskWithOmegaFeedforward(
    uint8_t detected_mask, float dt_s, float omega_feedforward_deg_s,
    LINE_FOLLOW_OUTPUT *out){
    if (out == NULL){
        return BSP_STATUS_NULL;
    }

    LineFollow_EnsureInitialized();

    LINE_FOLLOW_INPUT input;
    for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        input.level[i] = ((detected_mask & (1U << i)) != 0U) ? 0U : 1U;
    }

    float omega_deg_s = 0.0f;
    if (line_follow_config.gyro_stab_enabled){
        JY61P_I2C_SAMPLE sample;
        if (JY61P_I2C_GetSnapshot(&sample)){
            omega_deg_s = line_follow_config.gyro_z_sign *
                          sample.data.gyro_deg_s.z;
        }
    }

    BSP_STATUS status = LineFollow_ComputeWithOmegaFeedforward(
        &input, dt_s, omega_deg_s, omega_feedforward_deg_s, out);
    if (status != BSP_STATUS_OK){
        return status;
    }
    line_follow_output = *out;
    if (out->line_lost){
        return BSP_STATUS_NOT_READY;
    }

    return BSP_STATUS_OK;
}

BSP_STATUS LineFollow_EvaluateOmegaFeedforwardOnly(
    float omega_feedforward_deg_s, LINE_FOLLOW_OUTPUT *out){
    if (out == NULL){
        return BSP_STATUS_NULL;
    }

    LineFollow_EnsureInitialized();
    if (!line_follow_config.gyro_stab_enabled){
        return BSP_STATUS_NOT_READY;
    }

    JY61P_I2C_SAMPLE sample;
    if (!JY61P_I2C_GetSnapshot(&sample)){
        return BSP_STATUS_NOT_READY;
    }

    float omega_deg_s = line_follow_config.gyro_z_sign *
                        sample.data.gyro_deg_s.z;
    float omega_ref = Kinematics_Clamp(
        omega_feedforward_deg_s,
        -line_follow_config.omega_ref_limit,
        line_follow_config.omega_ref_limit);
    float correction = line_follow_config.gyro_stab_kp *
                       (omega_ref - omega_deg_s);
    if (line_follow_config.differential_limit > 0.0f){
        float correction_limit = line_follow_config.differential_limit * 0.5f;
        correction = Kinematics_Clamp(correction,
                                      -correction_limit,
                                      correction_limit);
    }

    KINEMATICS_DIFFERENTIAL_OUTPUT duty =
        Kinematics_DifferentialMix(line_follow_config.base_duty,
                                   correction,
                                   line_follow_config.output_limit);
    out->level_mask = 0xFFU;
    out->black_count = 0U;
    out->error = 0.0f;
    out->correction = correction;
    out->omega_ref_deg_s = omega_ref;
    out->omega_measured_deg_s = omega_deg_s;
    out->left_duty = duty.left;
    out->right_duty = duty.right;
    out->line_lost = true;
    line_follow_output = *out;
    return BSP_STATUS_OK;
}

BSP_STATUS LineFollow_UpdateDetectedMask(uint8_t detected_mask, float dt_s){
    BSP_STATUS status = LineFollow_EvaluateDetectedMask(
        detected_mask, dt_s, &line_follow_output);
    if (status != BSP_STATUS_OK){
        return status;
    }

    return Chassis_SetDuty(line_follow_output.left_duty,
                           line_follow_output.right_duty);
}

BSP_STATUS LineFollow_Observe(const LINE_FOLLOW_INPUT *input,
                              float position_scale,
                              LINE_FOLLOW_OBSERVATION *out){
    if ((input == NULL) || (out == NULL)){
        return BSP_STATUS_NULL;
    }

    float position_sum = 0.0f;
    uint8_t black_count = 0U;
    uint8_t level_mask = 0U;
    uint8_t black_mask = 0U;

    for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        if (input->level[i] != 0U){
            level_mask |= (uint8_t)(1U << i);
        } else if (LineFollow_IsSensorEnabled(i)){
            black_mask |= (uint8_t)(1U << i);
            position_sum += sensor_position[i];
            black_count++;
        }
    }

    out->level_mask = level_mask;
    out->black_mask = black_mask;
    out->black_count = black_count;
    out->line_lost = (black_count == 0U);
    out->error = out->line_lost
                     ? 0.0f
                     : (position_sum / (float)black_count) * position_scale;
    return BSP_STATUS_OK;
}

BSP_STATUS LineFollow_ObserveDetectedMask(uint8_t detected_mask,
                                          float position_scale,
                                          LINE_FOLLOW_OBSERVATION *out){
    if (out == NULL){
        return BSP_STATUS_NULL;
    }

    LINE_FOLLOW_INPUT input;
    for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        input.level[i] = ((detected_mask & (1U << i)) != 0U) ? 0U : 1U;
    }
    return LineFollow_Observe(&input, position_scale, out);
}

BSP_STATUS LineFollow_Compute(const LINE_FOLLOW_INPUT *input,
                              float dt_s,
                              float omega_deg_s,
                              LINE_FOLLOW_OUTPUT *out){
    return LineFollow_ComputeWithOmegaFeedforward(
        input, dt_s, omega_deg_s, 0.0f, out);
}

BSP_STATUS LineFollow_ComputeWithOmegaFeedforward(
    const LINE_FOLLOW_INPUT *input, float dt_s, float omega_deg_s,
    float omega_feedforward_deg_s, LINE_FOLLOW_OUTPUT *out){
    if ((input == NULL) || (out == NULL)){
        return BSP_STATUS_NULL;
    }

    LineFollow_EnsureInitialized();

    LINE_FOLLOW_OBSERVATION observation;
    BSP_STATUS observe_status = LineFollow_Observe(
        input, line_follow_config.sensor_position_scale, &observation);
    if (observe_status != BSP_STATUS_OK){
        return observe_status;
    }

    out->level_mask = observation.level_mask;
    out->black_count = observation.black_count;
    out->line_lost = observation.line_lost;

    if (out->line_lost){
        out->error = 0.0f;
        out->correction = 0.0f;
        out->omega_ref_deg_s = 0.0f;
        out->omega_measured_deg_s = omega_deg_s;
        out->left_duty = 0.0f;
        out->right_duty = 0.0f;
        return BSP_STATUS_OK;
    }

    /* 多路命中时取横向位置均值，单位 mm：负值表示线偏左，正值表示线偏右。 */
    out->error = observation.error;

    /* 原始误差保存在 out；滤波和死区后的 control_error 进入控制律。 */
    line_follow_filtered_error = Filter_LowpassEma(line_follow_filtered_error,
        out->error, LINE_FOLLOW_ERROR_LPF_ALPHA);
    float control_error = Filter_Deadband(line_follow_filtered_error,
        LINE_FOLLOW_ERROR_DEADBAND);

    if (line_follow_config.gyro_stab_enabled){
        float omega_line = Kinematics_Clamp(
            line_follow_config.gyro_line_kp * control_error,
            -line_follow_config.omega_line_limit,
            line_follow_config.omega_line_limit);
        float omega_ref = Kinematics_Clamp(
            omega_feedforward_deg_s + omega_line,
            -line_follow_config.omega_ref_limit,
            line_follow_config.omega_ref_limit);
        out->omega_ref_deg_s = omega_ref;
        out->omega_measured_deg_s = omega_deg_s;
        out->correction = line_follow_config.gyro_stab_kp *
                          (omega_ref - omega_deg_s);
    } else{
        out->omega_ref_deg_s = 0.0f;
        out->omega_measured_deg_s = omega_deg_s;
        out->correction = PID_Update(&line_follow_pid, control_error, 0.0f, dt_s);
    }

    /* left-right=2*correction，因此按差值上限的一半限制修正量。 */
    if (line_follow_config.differential_limit > 0.0f){
        float correction_limit = line_follow_config.differential_limit * 0.5f;
        out->correction = Kinematics_Clamp(out->correction,
                                           -correction_limit,
                                           correction_limit);
    }

    KINEMATICS_DIFFERENTIAL_OUTPUT duty =
        Kinematics_DifferentialMix(line_follow_config.base_duty,
                                   out->correction,
                                   line_follow_config.output_limit);

    out->left_duty = duty.left;
    out->right_duty = duty.right;

    return BSP_STATUS_OK;
}

LINE_FOLLOW_OUTPUT LineFollow_GetOutput(void){
    return line_follow_output;
}
