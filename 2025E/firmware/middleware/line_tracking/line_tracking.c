#include "line_tracking.h"

#include "chassis.h"
#include "kinematics/kinematics.h"
#include "wit_sdk.h"

#include <stddef.h>

static const float s_sensor_position[LINE_FOLLOW_SENSOR_COUNT] = {
    -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f,
};

/*
 * 抑制数字灰度"量化跳变→bang-bang→蛇形"的两个补丁 (不改车速/限幅):
 *   - error 一阶低通(EMA): 平滑离散跳变, 直接掐掉 kd 在跳变上的尖峰;
 *   - 中心死区: 线在中心两传感器内(|error|<=5, 即偏心 <=0.5 格)不修, 消除绕中心的极限环。
 * alpha 越小越平滑但越滞后; deadband 越大越不抖但容许的偏心越大。
 */
#ifndef LINE_TRACKING_ERROR_LPF_ALPHA
#define LINE_TRACKING_ERROR_LPF_ALPHA 0.5f
#endif
/* 中心死区: 线大致居中(|error|<=该值)就直行、不做微修正, 让车"提交直线运动", 消掉数字灰度
 * 量化噪声引起的高频微修蛇形。scale=10 时 error 10 ≈ 偏 1 个传感器, 故 10 ≈ 容许 ±1 格偏心。
 * 放宽 -> 更不抖但容许偏心更大(过大可能贴近传感器阵列边缘才纠、转弯入口判断变晚);
 * 收窄 -> 循迹更贴线但微修蛇形回来。现场按"直线不抖 & 不跑偏丢线"折中(可试 6~15)。 */
#ifndef LINE_TRACKING_ERROR_DEADBAND
#define LINE_TRACKING_ERROR_DEADBAND 10.0f
#endif
static float s_line_tracking_filtered_error;

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
            .kd = 0.0f,   /* 不对量化质心求导(会出尖峰); 靠宽死区+低通抑制微修蛇形 */
            .integral_limit = 500.0f,
            .output_limit = LINE_TRACKING_DEFAULT_CORRECTION_LIMIT,
            .mode = PID_MODE_POSITION,
        },
        .gyro_stab_enabled = LINE_TRACKING_DEFAULT_GYRO_STAB_ENABLED,
        .gyro_line_kp = LINE_TRACKING_DEFAULT_GYRO_LINE_KP,
        .gyro_stab_kp = LINE_TRACKING_DEFAULT_GYRO_STAB_KP,
        .omega_ref_limit = LINE_TRACKING_DEFAULT_OMEGA_REF_LIMIT,
        .gyro_z_sign = LINE_TRACKING_DEFAULT_GYRO_Z_SIGN,
    };

    return config;
}

static void LineTracking_EnsureInitialized(void){
    /* 允许上层忘记显式 Init 时仍能使用默认参数运行。 */
    if (!s_line_tracking_initialized){
        LineTracking_Init(NULL);
    }
}

LINE_TRACKING_CONFIG LineTracking_GetDefaultConfig(void){
    return LineTracking_DefaultConfig();
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

    s_line_tracking_filtered_error = 0.0f;
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

    /*
     * 读车体航向角速度 gz。JY61P 挂 I2C0、由 SysTick ISR 全局轮询, WitGetData 随时
     * 可取新鲜值, 所以【所有巡线任务(E1/F1/F2)都自动接入】陀螺增稳, 无需各任务改动。
     * 增稳关闭或 IMU 读失败 → omega=0, Compute 退化为纯位置控制 (无阻尼, 但不崩)。
     */
    float omega_deg_s = 0.0f;
    if (s_line_tracking_config.gyro_stab_enabled){
        WIT_IMU_DATA imu;
        if (WitGetData(&imu) == WIT_HAL_OK){
            omega_deg_s = s_line_tracking_config.gyro_z_sign * imu.gyro_deg_s.z;
        }
    }

    status = LineTracking_Compute(&sensor, dt_s, omega_deg_s, &s_line_tracking_output);
    if (status != BSP_STATUS_OK){
        return status;
    }

    if (s_line_tracking_output.line_lost){
        /*
         * 丢线时不在本层自行搜索或刹车。不同题目流程可能有不同恢复策略，
         * 因此只返回 NOT_READY，由 app 决定是否刹车、等待或切换状态。
         */
        return BSP_STATUS_NOT_READY;
    }

    return Chassis_SetDuty(s_line_tracking_output.left_duty,
                           s_line_tracking_output.right_duty);
}

BSP_STATUS LineTracking_Compute(const LINE_FOLLOW_SENSOR_STATE *sensor,
                                float dt_s,
                                float omega_deg_s,
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

    /*
     * error 一阶低通 + 中心死区: 把喂给 PID 的误差平滑并对小偏心置零,
     * 消除数字灰度量化跳变导致的转向尖峰/极限环蛇形。out->error 仍保留原始值。
     */
    s_line_tracking_filtered_error += LINE_TRACKING_ERROR_LPF_ALPHA *
        (out->error - s_line_tracking_filtered_error);
    float pid_error = s_line_tracking_filtered_error;
    if ((pid_error > -LINE_TRACKING_ERROR_DEADBAND) &&
        (pid_error < LINE_TRACKING_ERROR_DEADBAND)){
        pid_error = 0.0f;
    }

    /*
     * 陀螺增稳串级 (补偿"等占空比不直行"的硬件不对称 + 阻尼蛇形):
     *   外环: 灰度偏差 pid_error → 期望航向角速度 ω_ref (限幅);
     *   内环: correction = gyro_stab_kp · (ω_ref − gz), gz=omega_deg_s (已带符号)。
     * 居中时 ω_ref=0 → 内环把实际 gz 压向 0 → 自动补偿不对称直行, 且 gz 是航向阻尼项。
     * 关闭增稳(或无 IMU 传 0)则退回纯位置 PID 旧行为。
     */
    if (s_line_tracking_config.gyro_stab_enabled){
        float omega_ref = Kinematics_Clamp(
            s_line_tracking_config.gyro_line_kp * pid_error,
            -s_line_tracking_config.omega_ref_limit,
            s_line_tracking_config.omega_ref_limit);
        out->correction = s_line_tracking_config.gyro_stab_kp *
                          (omega_ref - omega_deg_s);
    } else{
        out->correction = PID_Update(&s_line_tracking_pid, pid_error, 0.0f, dt_s);
    }

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
