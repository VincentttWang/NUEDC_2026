#include "line_tracking.h"

#include "chassis.h"
#include "kinematics/kinematics.h"

#include <stddef.h>

static const float s_sensor_position[LINE_FOLLOW_SENSOR_COUNT] = {
    -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f,
};

static bool LineTracking_IsSensorEnabled(uint8_t index){
    return ((LINE_TRACKING_ACTIVE_SENSOR_MASK & (1U << index)) != 0U);
}

static LINE_TRACKING_CONFIG s_line_tracking_config;
static LINE_TRACKING_OUTPUT s_line_tracking_output;
static PID_CONTROLLER s_line_tracking_pid;
static bool s_line_tracking_initialized;

static LINE_TRACKING_CONFIG LineTracking_DefaultConfig(void){
    /*
     * 默认参数偏向低速稳定巡线：
     * base_duty 负责持续前进，PID 输出只作为左右轮差速修正。
     */
    LINE_TRACKING_CONFIG config = {
        .base_duty = LINE_TRACKING_DEFAULT_BASE_DUTY,
        .sensor_position_scale = LINE_TRACKING_DEFAULT_POSITION_SCALE,
        .output_limit = LINE_TRACKING_DEFAULT_OUTPUT_LIMIT,
        .differential_limit = LINE_TRACKING_DEFAULT_DIFFERENTIAL_LIMIT,
        .pid_config = {
            .kp = 1.0f,
            .ki = 0.0f,
            .kd = 1.5f,
            .integral_limit = 500.0f,
            .output_limit = LINE_TRACKING_DEFAULT_CORRECTION_LIMIT,
            .mode = PID_MODE_POSITION,
        },
    };

    return config;
}

static void LineTracking_EnsureInitialized(void){
    /* 允许上层忘记显式 Init 时仍能使用默认参数运行。 */
    if (!s_line_tracking_initialized){
        LineTracking_Init(NULL);
    }
}

void LineTracking_Init(const LINE_TRACKING_CONFIG *config){
    if (config == NULL){
        s_line_tracking_config = LineTracking_DefaultConfig();
    } else{
        s_line_tracking_config = *config;
    }

    PID_Init(&s_line_tracking_pid, &s_line_tracking_config.pid_config);
    LineTracking_Reset();
    s_line_tracking_initialized = true;
}

void LineTracking_Reset(void){
    PID_Reset(&s_line_tracking_pid);

    s_line_tracking_output.error = 0.0f;
    s_line_tracking_output.correction = 0.0f;
    s_line_tracking_output.left_duty = 0.0f;
    s_line_tracking_output.right_duty = 0.0f;
    s_line_tracking_output.active_count = 0U;
    s_line_tracking_output.line_lost = true;
}

BSP_STATUS LineTracking_Update(float dt_s){
    LineTracking_EnsureInitialized();

    /* 完整闭环入口：先刷新 8 路灰度，再计算差速，最后下发到底盘。 */
    BSP_STATUS status = LineFollow_Update();
    if (status != BSP_STATUS_OK){
        return status;
    }

    LINE_FOLLOW_SENSOR_STATE sensor;
    status = LineFollow_GetSensor(&sensor);
    if (status != BSP_STATUS_OK){
        return status;
    }

    status = LineTracking_Compute(&sensor, dt_s, &s_line_tracking_output);
    if (status != BSP_STATUS_OK){
        return status;
    }

    if (s_line_tracking_output.line_lost){
        /*
         * 丢线时不在 core 层自行搜索或刹车。不同题目流程可能有不同恢复策略，
         * 因此只返回 NOT_READY，由 app 决定是否刹车、等待或切换状态。
         */
        return BSP_STATUS_NOT_READY;
    }

    return Chassis_SetDuty(s_line_tracking_output.left_duty,
                           s_line_tracking_output.right_duty);
}

BSP_STATUS LineTracking_Compute(const LINE_FOLLOW_SENSOR_STATE *sensor,
                                float dt_s,
                                LINE_TRACKING_OUTPUT *out){
    if ((sensor == NULL) || (out == NULL)){
        return BSP_STATUS_NULL;
    }

    LineTracking_EnsureInitialized();

    float position_sum = 0.0f;
    uint8_t active_count = 0U;

    /*
     * 用启用的灰度传感器横向位置建立线中心估计。
     * 当前巡线逻辑沿用旧 Digital[] 语义：value == 0 表示检测到黑线。
     */
    for (uint8_t i = 0; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        if (LineTracking_IsSensorEnabled(i) && sensor->value[i] == 0U){
            position_sum += s_sensor_position[i];
            active_count++;
        }
    }

    out->active_count = active_count;
    out->line_lost = (active_count == 0U);

    if (out->line_lost){
        out->error = 0.0f;
        out->correction = 0.0f;
        out->left_duty = 0.0f;
        out->right_duty = 0.0f;
        return BSP_STATUS_OK;
    }

    /*
     * 多路同时触发时取位置均值，得到线相对车体中心的横向偏差。
     * 偏差为负说明线更靠左，偏差为正说明线更靠右。
     */
    float average_position = position_sum / (float)active_count;
    out->error = average_position * s_line_tracking_config.sensor_position_scale;
    out->correction = PID_Update(&s_line_tracking_pid, out->error, 0.0f, dt_s);

    /*
     * 巡线时左右轮差值为 2 * correction。这里单独限制 correction，
     * 只收敛循迹动作的差速强度，不影响 Motion 的直行、倒车和原地转向。
     */
    if (s_line_tracking_config.differential_limit > 0.0f){
        float correction_limit = s_line_tracking_config.differential_limit * 0.5f;
        out->correction = Kinematics_Clamp(out->correction,
                                           -correction_limit,
                                           correction_limit);
    }

    /*
     * forward 为基础前进占空比，turn 为 PID 转向修正量。
     * Kinematics_DifferentialMix() 负责把二者混成左右轮占空比并统一限幅。
     */
    KINEMATICS_DIFFERENTIAL_OUTPUT duty =
        Kinematics_DifferentialMix(s_line_tracking_config.base_duty,
                                   out->correction,
                                   s_line_tracking_config.output_limit);

    out->left_duty = duty.left;
    out->right_duty = duty.right;

    return BSP_STATUS_OK;
}

LINE_TRACKING_OUTPUT LineTracking_GetOutput(void){
    return s_line_tracking_output;
}
