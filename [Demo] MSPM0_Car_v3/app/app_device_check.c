#include "app_device_check.h"

#include "bsp_time.h"
#include "canmv_uart.h"
#include "chassis.h"
#include "gimbal.h"
#include "hall_encoder.h"
#include "key.h"
#include "line_follow.h"
#include "pid.h"
#include "step_motor.h"
#include "ui.h"
#include "wit_sdk.h"
#include "mpu6050.h"
#include "hc_sr04.h"
#include "f32c_bldc.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define APP_DEVICE_IMU_FRAME_SIZE 11U
#define APP_DEVICE_LOOP_DELAY_MS 20U
#define APP_DEVICE_RENDER_PERIOD_MS 100U
#define APP_DEVICE_YAW_HOLD_IDLE_DELAY_MS 1U
#define APP_DEVICE_YAW_HOLD_RENDER_PERIOD_MS 250U
#define APP_DEVICE_MOTOR_DUTY_PERCENT 30.0f
#define APP_DEVICE_STEP_SPEED_DEG_S 120.0f
#define APP_DEVICE_STEP_RUN_MS 1000U
#define APP_DEVICE_YAW_HOLD_KP 4.0f
#define APP_DEVICE_YAW_HOLD_KI 0.0f
#define APP_DEVICE_YAW_HOLD_KD 0.15f
#define APP_DEVICE_YAW_HOLD_FEEDFORWARD_GAIN 1.2f
#define APP_DEVICE_YAW_HOLD_OUTPUT_LIMIT_DEG_S 240.0f
#define APP_DEVICE_YAW_HOLD_RATE_LIMIT_DEG_S2 1800.0f
#define APP_DEVICE_YAW_HOLD_MIN_START_SPEED_DEG_S 65.0f
#define APP_DEVICE_YAW_HOLD_DEADBAND_DEG 0.2f
#define APP_DEVICE_YAW_HOLD_GYRO_DEADBAND_DEG_S 1.5f

typedef enum {
    APP_DEVICE_MODULE_MOTOR = 0,
    APP_DEVICE_MODULE_YAW,
    APP_DEVICE_MODULE_PITCH,
    APP_DEVICE_MODULE_YAW_HOLD,
    APP_DEVICE_MODULE_LINE,
    APP_DEVICE_MODULE_IMU,
    APP_DEVICE_MODULE_K230,
    APP_DEVICE_MODULE_ULTRASONIC,
    APP_DEVICE_MODULE_BLDC,
    APP_DEVICE_MODULE_COUNT
} APP_DEVICE_MODULE;

typedef enum {
    APP_DEVICE_MOTOR_STOP = 0,
    APP_DEVICE_MOTOR_LEFT,
    APP_DEVICE_MOTOR_RIGHT,
    APP_DEVICE_MOTOR_BOTH,
    APP_DEVICE_MOTOR_COUNT
} APP_DEVICE_MOTOR_TEST;

typedef enum {
    APP_DEVICE_IMU_ATTITUDE = 0,
    APP_DEVICE_IMU_GYRO,
    APP_DEVICE_IMU_STATUS,
    APP_DEVICE_IMU_MPU6050,
    APP_DEVICE_IMU_COUNT
} APP_DEVICE_IMU_TEST;

/* BLDC 测试预设: 速度(RPM) + 位置(0.1°) 合并循环 */
typedef enum {
    APP_DEVICE_BLDC_SPD_0 = 0,
    APP_DEVICE_BLDC_SPD_30,
    APP_DEVICE_BLDC_SPD_60,
    APP_DEVICE_BLDC_SPD_90,
    APP_DEVICE_BLDC_SPD_120,
    APP_DEVICE_BLDC_SPD_M30,
    APP_DEVICE_BLDC_SPD_M60,
    APP_DEVICE_BLDC_POS_90,
    APP_DEVICE_BLDC_POS_180,
    APP_DEVICE_BLDC_POS_270,
    APP_DEVICE_BLDC_POS_360,
    APP_DEVICE_BLDC_POS_M90,
    APP_DEVICE_BLDC_COUNT
} APP_DEVICE_BLDC_TEST;

static uint8_t s_imu_rx_buffer[APP_DEVICE_IMU_FRAME_SIZE];
static uint8_t s_imu_rx_index;
static uint32_t s_imu_frame_count;
static volatile uint32_t s_imu_update_count;

volatile uint32_t g_app_device_imu_rx_byte_count;
volatile uint32_t g_app_device_imu_valid_frame_count;
volatile uint32_t g_app_device_imu_invalid_frame_count;
volatile uint8_t g_app_device_imu_last_type;
volatile uint8_t g_app_device_imu_last_byte;

extern volatile uint32_t g_app_debug_uart_irq_count;
extern volatile uint32_t g_app_debug_uart_rx_irq_count;
extern volatile uint32_t g_app_debug_uart_drained_byte_count;
extern volatile uint32_t g_app_debug_uart_empty_rx_irq_count;
extern volatile uint32_t g_app_debug_uart_unhandled_irq_count;
extern volatile uint8_t g_app_debug_uart_last_iidx;

static APP_DEVICE_MOTOR_TEST s_motor_test;
static APP_DEVICE_IMU_TEST s_imu_test;
static APP_DEVICE_BLDC_TEST s_bldc_test;
static PID_CONTROLLER s_yaw_hold_pid;
static float s_yaw_hold_target_deg;
static float s_yaw_hold_error_deg;
static float s_yaw_hold_gyro_z_deg_s;
static float s_yaw_hold_pid_output_deg_s;
static float s_yaw_hold_raw_output_deg_s;
static float s_yaw_hold_output_deg_s;
static bool s_yaw_hold_active;
static bool s_yaw_hold_pid_initialized;
static uint32_t s_yaw_hold_last_update_count;
static bool s_bldc_initialized;

static float AppDeviceCheck_AbsFloat(float value){
    return (value < 0.0f) ? -value : value;
}

static float AppDeviceCheck_SignFloat(float value){
    if (value > 0.0f){
        return 1.0f;
    }

    if (value < 0.0f){
        return -1.0f;
    }

    return 0.0f;
}

static float AppDeviceCheck_LimitFloat(float value, float limit){
    if (limit <= 0.0f){
        return value;
    }

    if (value > limit){
        return limit;
    }

    if (value < -limit){
        return -limit;
    }

    return value;
}

static float AppDeviceCheck_NormalizeAngleError(float target_deg, float feedback_deg){
    float error_deg = target_deg - feedback_deg;

    while (error_deg > 180.0f){
        error_deg -= 360.0f;
    }

    while (error_deg < -180.0f){
        error_deg += 360.0f;
    }

    return error_deg;
}

static void AppDeviceCheck_InitYawHoldPid(void){
    PID_CONFIG config = {
        .kp = APP_DEVICE_YAW_HOLD_KP,
        .ki = APP_DEVICE_YAW_HOLD_KI,
        .kd = APP_DEVICE_YAW_HOLD_KD,
        .integral_limit = 0.0f,
        .output_limit = APP_DEVICE_YAW_HOLD_OUTPUT_LIMIT_DEG_S,
        .mode = PID_MODE_POSITION,
    };

    PID_Init(&s_yaw_hold_pid, &config);
    s_yaw_hold_pid_initialized = true;
}

static bool AppDeviceCheck_IsImuFrameValid(const uint8_t *frame){
    uint16_t sum = 0U;

    if (frame[0] != 0x55U){
        return false;
    }

    for (uint8_t i = 0U; i < 10U; i++){
        sum += frame[i];
    }

    return frame[10] == (uint8_t)(sum & 0xFFU);
}

static bool AppDeviceCheck_IsImuFrameType(uint8_t type){
    return (type == 0x51U) || (type == 0x52U) || (type == 0x53U);
}

static void AppDeviceCheck_ResyncImuFrame(void){
    for (uint8_t i = 1U; i < APP_DEVICE_IMU_FRAME_SIZE; i++){
        if (s_imu_rx_buffer[i] == 0x55U){
            uint8_t remain = (uint8_t)(APP_DEVICE_IMU_FRAME_SIZE - i);
            memmove(s_imu_rx_buffer, &s_imu_rx_buffer[i], remain);
            s_imu_rx_index = remain;
            return;
        }
    }

    s_imu_rx_index = 0U;
}

void AppDeviceCheck_ProcessImuByte(uint8_t byte){
    g_app_device_imu_rx_byte_count++;
    g_app_device_imu_last_byte = byte;

    if ((s_imu_rx_index == 0U) && (byte != 0x55U)){
        return;
    }

    s_imu_rx_buffer[s_imu_rx_index++] = byte;

    if ((s_imu_rx_index == 2U) && !AppDeviceCheck_IsImuFrameType(s_imu_rx_buffer[1])){
        AppDeviceCheck_ResyncImuFrame();
        return;
    }

    if (s_imu_rx_index < APP_DEVICE_IMU_FRAME_SIZE){
        return;
    }

    if (AppDeviceCheck_IsImuFrameValid(s_imu_rx_buffer)){
        GYROSCOPE_DATA_Decoder(s_imu_rx_buffer);
        s_imu_frame_count++;
        s_imu_update_count++;
        g_app_device_imu_valid_frame_count++;
        g_app_device_imu_last_type = s_imu_rx_buffer[1];
        s_imu_rx_index = 0U;
        return;
    }

    g_app_device_imu_invalid_frame_count++;
    AppDeviceCheck_ResyncImuFrame();
}

static const char *AppDeviceCheck_ModuleTitle(APP_DEVICE_MODULE module){
    static const char *titles[APP_DEVICE_MODULE_COUNT] = {
        "Motor",
        "Yaw Step",
        "Pitch Step",
        "Yaw Hold",
        "Line Sensor",
        "IMU",
        "K230",
        "Ultrasonic",
        "BLDC",
    };

    return (module < APP_DEVICE_MODULE_COUNT) ? titles[module] : "Device";
}

static uint8_t AppDeviceCheck_TestCount(APP_DEVICE_MODULE module){
    switch (module){
        case APP_DEVICE_MODULE_MOTOR:
            return (uint8_t)APP_DEVICE_MOTOR_COUNT;
        case APP_DEVICE_MODULE_IMU:
            return (uint8_t)APP_DEVICE_IMU_COUNT;
        case APP_DEVICE_MODULE_BLDC:
            return (uint8_t)APP_DEVICE_BLDC_COUNT;
        case APP_DEVICE_MODULE_YAW:
        case APP_DEVICE_MODULE_PITCH:
        case APP_DEVICE_MODULE_YAW_HOLD:
        case APP_DEVICE_MODULE_LINE:
        case APP_DEVICE_MODULE_K230:
        default:
            return 1U;
    }
}

static const char *AppDeviceCheck_MotorTestName(APP_DEVICE_MOTOR_TEST test){
    static const char *names[APP_DEVICE_MOTOR_COUNT] = {
        "Stop",
        "Left",
        "Right",
        "Both",
    };

    return (test < APP_DEVICE_MOTOR_COUNT) ? names[test] : "Stop";
}

static void AppDeviceCheck_FormatBinary8(uint8_t value, char *out, size_t out_size){
    if (out == NULL || out_size < 9U){
        return;
    }

    for (uint8_t i = 0U; i < 8U; i++){
        out[i] = ((value & (uint8_t)(1U << i)) != 0U) ? '1' : '0';
    }
    out[8] = '\0';
}

static void AppDeviceCheck_StopOutputs(void){
    (void)Chassis_Stop(CHASSIS_STOP_MODE_BRAKE);
    (void)Gimbal_Stop();
    s_yaw_hold_active = false;
    s_yaw_hold_output_deg_s = 0.0f;
    s_yaw_hold_raw_output_deg_s = 0.0f;
    if (s_bldc_initialized){
        BLDC_Disable(BLDC_ADDR_1);
        BLDC_Disable(BLDC_ADDR_2);
    }
}

static void AppDeviceCheck_ResetMotorTest(void){
    s_motor_test = APP_DEVICE_MOTOR_STOP;
    HallEncoder_Reset();
    (void)Chassis_Stop(CHASSIS_STOP_MODE_BRAKE);
}

static void AppDeviceCheck_ApplyMotorTest(void){
    HallEncoder_Reset();

    switch (s_motor_test){
        case APP_DEVICE_MOTOR_LEFT:
            (void)Chassis_SetDuty(APP_DEVICE_MOTOR_DUTY_PERCENT, 0.0f);
            break;
        case APP_DEVICE_MOTOR_RIGHT:
            (void)Chassis_SetDuty(0.0f, APP_DEVICE_MOTOR_DUTY_PERCENT);
            break;
        case APP_DEVICE_MOTOR_BOTH:
            (void)Chassis_SetDuty(APP_DEVICE_MOTOR_DUTY_PERCENT, APP_DEVICE_MOTOR_DUTY_PERCENT);
            break;
        case APP_DEVICE_MOTOR_STOP:
        default:
            (void)Chassis_Stop(CHASSIS_STOP_MODE_BRAKE);
            break;
    }
}

static void AppDeviceCheck_RunStepPulse(STEP_MOTOR_CHANNEL channel){
    (void)StepMotor_RunFor(channel, APP_DEVICE_STEP_SPEED_DEG_S, APP_DEVICE_STEP_RUN_MS);
    (void)StepMotor_RunFor(channel, -APP_DEVICE_STEP_SPEED_DEG_S, APP_DEVICE_STEP_RUN_MS);
}

static void AppDeviceCheck_StartYawHold(void){
    WIT_IMU_DATA data = {0};

    if (!s_yaw_hold_pid_initialized){
        AppDeviceCheck_InitYawHoldPid();
    }

    (void)WitGetData(&data);
    s_yaw_hold_target_deg = data.attitude_deg.yaw;
    s_yaw_hold_error_deg = 0.0f;
    s_yaw_hold_gyro_z_deg_s = 0.0f;
    s_yaw_hold_pid_output_deg_s = 0.0f;
    s_yaw_hold_raw_output_deg_s = 0.0f;
    s_yaw_hold_output_deg_s = 0.0f;
    PID_Reset(&s_yaw_hold_pid);
    s_yaw_hold_last_update_count = JY61P_I2C_GetPollCount();
    s_yaw_hold_active = true;
}

static void AppDeviceCheck_StopYawHold(void){
    s_yaw_hold_active = false;
    s_yaw_hold_gyro_z_deg_s = 0.0f;
    s_yaw_hold_pid_output_deg_s = 0.0f;
    s_yaw_hold_raw_output_deg_s = 0.0f;
    s_yaw_hold_output_deg_s = 0.0f;
    PID_Reset(&s_yaw_hold_pid);
    s_yaw_hold_last_update_count = JY61P_I2C_GetPollCount();
    (void)Gimbal_SetSpeed(0.0f, 0.0f);
}

static void AppDeviceCheck_UpdateYawHold(float dt_s){
    WIT_IMU_DATA data = {0};

    if (!s_yaw_hold_active){
        return;
    }

    if (dt_s <= 0.0f){
        dt_s = 0.001f;
    }

    (void)WitGetData(&data);
    s_yaw_hold_gyro_z_deg_s = data.gyro_deg_s.z;
    s_yaw_hold_error_deg = AppDeviceCheck_NormalizeAngleError(s_yaw_hold_target_deg,
                                                              data.attitude_deg.yaw);

    if ((AppDeviceCheck_AbsFloat(s_yaw_hold_error_deg) <= APP_DEVICE_YAW_HOLD_DEADBAND_DEG) &&
        (AppDeviceCheck_AbsFloat(s_yaw_hold_gyro_z_deg_s) <= APP_DEVICE_YAW_HOLD_GYRO_DEADBAND_DEG_S)){
        s_yaw_hold_pid_output_deg_s = 0.0f;
        s_yaw_hold_raw_output_deg_s = 0.0f;
        s_yaw_hold_output_deg_s = 0.0f;
        PID_Reset(&s_yaw_hold_pid);
    } else{
        float gyro_z_deg_s = s_yaw_hold_gyro_z_deg_s;
        if (AppDeviceCheck_AbsFloat(gyro_z_deg_s) <= APP_DEVICE_YAW_HOLD_GYRO_DEADBAND_DEG_S){
            gyro_z_deg_s = 0.0f;
        }

        s_yaw_hold_pid_output_deg_s = PID_Update(&s_yaw_hold_pid,
                                                 s_yaw_hold_error_deg,
                                                 0.0f,
                                                 dt_s);

        s_yaw_hold_raw_output_deg_s =
            s_yaw_hold_pid_output_deg_s -
            APP_DEVICE_YAW_HOLD_FEEDFORWARD_GAIN * gyro_z_deg_s;
        s_yaw_hold_raw_output_deg_s =
            AppDeviceCheck_LimitFloat(s_yaw_hold_raw_output_deg_s,
                                      APP_DEVICE_YAW_HOLD_OUTPUT_LIMIT_DEG_S);

        if ((AppDeviceCheck_AbsFloat(s_yaw_hold_raw_output_deg_s) <
             APP_DEVICE_YAW_HOLD_MIN_START_SPEED_DEG_S) &&
            ((AppDeviceCheck_AbsFloat(s_yaw_hold_error_deg) > APP_DEVICE_YAW_HOLD_DEADBAND_DEG) ||
             (AppDeviceCheck_AbsFloat(gyro_z_deg_s) > APP_DEVICE_YAW_HOLD_GYRO_DEADBAND_DEG_S))){
            s_yaw_hold_raw_output_deg_s =
                AppDeviceCheck_SignFloat(s_yaw_hold_raw_output_deg_s) *
                APP_DEVICE_YAW_HOLD_MIN_START_SPEED_DEG_S;
        }

        float max_delta_deg_s = APP_DEVICE_YAW_HOLD_RATE_LIMIT_DEG_S2 * dt_s;
        float delta_deg_s = s_yaw_hold_raw_output_deg_s - s_yaw_hold_output_deg_s;
        delta_deg_s = AppDeviceCheck_LimitFloat(delta_deg_s, max_delta_deg_s);
        s_yaw_hold_output_deg_s =
            AppDeviceCheck_LimitFloat(s_yaw_hold_output_deg_s + delta_deg_s,
                                      APP_DEVICE_YAW_HOLD_OUTPUT_LIMIT_DEG_S);
    }

    (void)Gimbal_SetSpeed(s_yaw_hold_output_deg_s, 0.0f);
}

static void AppDeviceCheck_UpdateYawHoldOnImuFrame(float dt_s){
    uint32_t update_count = JY61P_I2C_GetPollCount();

    if (!s_yaw_hold_active || (update_count == s_yaw_hold_last_update_count)){
        return;
    }

    s_yaw_hold_last_update_count = update_count;
    AppDeviceCheck_UpdateYawHold(dt_s);
}

static void AppDeviceCheck_EnterModule(APP_DEVICE_MODULE module){
    AppDeviceCheck_StopOutputs();

    switch (module){
        case APP_DEVICE_MODULE_MOTOR:
            AppDeviceCheck_ResetMotorTest();
            break;
        case APP_DEVICE_MODULE_IMU:
            s_imu_test = APP_DEVICE_IMU_ATTITUDE;
            break;
        case APP_DEVICE_MODULE_YAW:
            StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_YAW);
            break;
        case APP_DEVICE_MODULE_PITCH:
            StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_PITCH);
            break;
        case APP_DEVICE_MODULE_YAW_HOLD:
            if (!s_yaw_hold_pid_initialized){
                AppDeviceCheck_InitYawHoldPid();
            }
            AppDeviceCheck_StopYawHold();
            break;
        case APP_DEVICE_MODULE_BLDC:
            if (!s_bldc_initialized){
                BLDC_Init();
                s_bldc_initialized = true;
            }
            s_bldc_test = APP_DEVICE_BLDC_SPD_0;
            BLDC_Motor1.data_ready = 0U;
            BLDC_Motor2.data_ready = 0U;
            BLDC_Enable(BLDC_ADDR_1);
            BLDC_Enable(BLDC_ADDR_2);
            BLDC_SetMode(BLDC_ADDR_1, MODE_SPEED);
            BLDC_SetMode(BLDC_ADDR_2, MODE_SPEED);
            break;
        case APP_DEVICE_MODULE_LINE:
        case APP_DEVICE_MODULE_K230:
        default:
            break;
    }
}

static void AppDeviceCheck_AdvanceTest(APP_DEVICE_MODULE module){
    switch (module){
        case APP_DEVICE_MODULE_MOTOR:
            s_motor_test = (APP_DEVICE_MOTOR_TEST)(((uint8_t)s_motor_test + 1U) %
                (uint8_t)APP_DEVICE_MOTOR_COUNT);
            AppDeviceCheck_ApplyMotorTest();
            break;
        case APP_DEVICE_MODULE_YAW:
            AppDeviceCheck_RunStepPulse(STEP_MOTOR_CHANNEL_YAW);
            break;
        case APP_DEVICE_MODULE_PITCH:
            AppDeviceCheck_RunStepPulse(STEP_MOTOR_CHANNEL_PITCH);
            break;
        case APP_DEVICE_MODULE_YAW_HOLD:
            if (s_yaw_hold_active){
                AppDeviceCheck_StopYawHold();
            } else{
                AppDeviceCheck_StartYawHold();
            }
            break;
        case APP_DEVICE_MODULE_IMU:
            s_imu_test = (APP_DEVICE_IMU_TEST)(((uint8_t)s_imu_test + 1U) %
                (uint8_t)APP_DEVICE_IMU_COUNT);
            break;
        case APP_DEVICE_MODULE_BLDC:{
            APP_DEVICE_BLDC_TEST prev = s_bldc_test;

            s_bldc_test = (APP_DEVICE_BLDC_TEST)(((uint8_t)s_bldc_test + 1U) %
                (uint8_t)APP_DEVICE_BLDC_COUNT);

            /* 检测速度 → 位置 或 位置 → 速度 的模式切换 */
            bool prev_is_pos = (prev >= APP_DEVICE_BLDC_POS_90);
            bool curr_is_pos = (s_bldc_test >= APP_DEVICE_BLDC_POS_90);

            if (curr_is_pos && !prev_is_pos){
                /* 切换到位置模式: 位置模式必须设定运动速度, 否则不动 */
                BLDC_SetMode(BLDC_ADDR_1, MODE_MULTI_POS);
                BLDC_SetMode(BLDC_ADDR_2, MODE_MULTI_POS);
                BLDC_SetSpeed(BLDC_ADDR_1, BLDC_PITCH_SPEED);
                BLDC_SetSpeed(BLDC_ADDR_2, BLDC_PITCH_SPEED);
                BLDC_Motor1.data_ready = 0U;
                BLDC_Motor2.data_ready = 0U;
            } else if (!curr_is_pos && prev_is_pos){
                /* 切换回速度模式 */
                BLDC_SetMode(BLDC_ADDR_1, MODE_SPEED);
                BLDC_SetMode(BLDC_ADDR_2, MODE_SPEED);
                BLDC_Motor1.data_ready = 0U;
                BLDC_Motor2.data_ready = 0U;
            }
            break;
        }
        case APP_DEVICE_MODULE_LINE:
        case APP_DEVICE_MODULE_K230:
        default:
            break;
    }
}

static bool AppDeviceCheck_IsNextModuleEvent(void){
    return Key_IsShortPress(KEY_ID_DOWN);
}

static bool AppDeviceCheck_IsPrevModuleEvent(void){
    return Key_IsShortPress(KEY_ID_UP);
}

static bool AppDeviceCheck_IsActionEvent(void){
    return Key_IsShortPress(KEY_ID_ENTER);
}

static bool AppDeviceCheck_IsBackEvent(void){
    return Key_IsShortPress(KEY_ID_CALIB) || Key_IsLongPress(KEY_ID_CALIB);
}

static void AppDeviceCheck_Render(APP_DEVICE_MODULE module){
    char line0[24];
    char line1[24];
    char line2[24];
    char line3[24];

    switch (module){
        case APP_DEVICE_MODULE_MOTOR:{
            CHASSIS_DUTY duty = Chassis_GetDuty();
            snprintf(line0, sizeof(line0), "Mode:%s", AppDeviceCheck_MotorTestName(s_motor_test));
            snprintf(line1, sizeof(line1), "Duty:%0.0f,%0.0f", duty.left_percent, duty.right_percent);
            snprintf(line2, sizeof(line2), "Cnt:%ld", (long)HallEncoder_GetCount());
            snprintf(line3, sizeof(line3), "Spd:%0.2f Dis:%0.2f", Chassis_GetSpeed(), Chassis_GetDistance());
            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1, line2, line3, "K3:mode K1/4:mod", NULL);
            break;
        }
        case APP_DEVICE_MODULE_YAW:{
            (void)StepMotor_UpdateState(STEP_MOTOR_CHANNEL_YAW, BSP_Time_GetMs());
            snprintf(line0, sizeof(line0), "Single:start");
            snprintf(line1, sizeof(line1), "Speed:%0.0f", StepMotor_GetSpeed(STEP_MOTOR_CHANNEL_YAW));
            snprintf(line2, sizeof(line2), "Pos:%0.1f", StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_YAW));
            snprintf(line3, sizeof(line3), "+1s then -1s");
            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1, line2, line3, "K3:start K1/4:mod", NULL);
            break;
        }
        case APP_DEVICE_MODULE_PITCH:{
            (void)StepMotor_UpdateState(STEP_MOTOR_CHANNEL_PITCH, BSP_Time_GetMs());
            snprintf(line0, sizeof(line0), "Single:start");
            snprintf(line1, sizeof(line1), "Speed:%0.0f", StepMotor_GetSpeed(STEP_MOTOR_CHANNEL_PITCH));
            snprintf(line2, sizeof(line2), "Pos:%0.1f", StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_PITCH));
            snprintf(line3, sizeof(line3), "+1s then -1s");
            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1, line2, line3, "K3:start K1/4:mod", NULL);
            break;
        }
        case APP_DEVICE_MODULE_YAW_HOLD:{
            WIT_IMU_DATA data = {0};
            (void)WitGetData(&data);

            snprintf(line0, sizeof(line0), "Mode:%s", s_yaw_hold_active ? "ON" : "OFF");
            snprintf(line1, sizeof(line1), "Yaw:%0.1f", data.attitude_deg.yaw);
            snprintf(line2, sizeof(line2), "Tar:%0.1f Err:%0.1f",
                     s_yaw_hold_target_deg,
                     s_yaw_hold_error_deg);
            snprintf(line3, sizeof(line3), "Gz:%0.0f Out:%0.0f",
                     s_yaw_hold_gyro_z_deg_s,
                     s_yaw_hold_output_deg_s);
            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1, line2, line3, "K3:on/off K1/4", NULL);
            break;
        }
        case APP_DEVICE_MODULE_LINE:{
            char binary[9];
            (void)LineFollow_UpdateSensor();
            AppDeviceCheck_FormatBinary8(LineFollow_GetSensorMask(), binary, sizeof(binary));
            snprintf(line0, sizeof(line0), "BIN:%s", binary);
            snprintf(line1, sizeof(line1), "HEX:0x%02X", LineFollow_GetSensorMask());
            snprintf(line2, sizeof(line2), "Active:%u", LineFollow_GetActiveCount());
            snprintf(line3, sizeof(line3), "Realtime");
            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1, line2, line3, "K1/K4:module", NULL);
            break;
        }
        case APP_DEVICE_MODULE_K230:{
            const CANMV_TARGET_DATA *laser_data = CanMvUart_GetTargetData(CANMV_TARGET_LASER);

            snprintf(line0, sizeof(line0), "Rx:%lu F:%lu",
                     (unsigned long)g_canmv_uart_rx_byte_count,
                     (unsigned long)g_canmv_uart_valid_frame_count);
            snprintf(line1, sizeof(line1), "Drop:%lu B:%02X",
                     (unsigned long)g_canmv_uart_drop_count,
                     g_canmv_uart_last_byte);

            if (laser_data != NULL && laser_data->count >= 4U){
                snprintf(line2, sizeof(line2), "T:%u,%u",
                         laser_data->value[0],
                         laser_data->value[1]);
                snprintf(line3, sizeof(line3), "L:%u,%u S:%d",
                         laser_data->value[2],
                         laser_data->value[3],
                         (int)laser_data->status);
            } else{
                snprintf(line2, sizeof(line2), "T:--,--");
                snprintf(line3, sizeof(line3), "L:--,-- S:%d",
                         (int)CanMvUart_GetStatus(CANMV_TARGET_LASER));
            }

            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1, line2, line3, "K1/K4:module", NULL);
            break;
        }
        case APP_DEVICE_MODULE_ULTRASONIC:{
            float dist = SR04_GetDistance();
            snprintf(line0, sizeof(line0), "Dist:%0.1f cm", (double)dist);
            snprintf(line1, sizeof(line1), "Trig:PA27 Echo:PA26");
            snprintf(line2, sizeof(line2), "Time:%0.1f ms", (double)(dist * 58.0f / 1000.0f));
            snprintf(line3, sizeof(line3), "%s", (dist > 0.0f) ? "Valid" : "Timeout");
            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1, line2, line3, "K1/K4:module", NULL);
            break;
        }
        case APP_DEVICE_MODULE_BLDC:{
            /* 预设值查找表, 与 APP_DEVICE_BLDC_TEST 枚举一一对应 */
            static const int16_t speed_vals[] = {0, 30, 60, 90, 120, -30, -60};
            static const int32_t pos_vals[]  = {900, 1800, 2700, 3600, -900};
            static const char *speed_names[] = {"0RPM","30RPM","60RPM","90RPM","120RPM","-30RPM","-60RPM"};
            static const char *pos_names[]   = {"90deg","180deg","270deg","360deg","-90deg"};

            if (s_bldc_test >= APP_DEVICE_BLDC_POS_90){
                /* 位置模式 */
                uint8_t idx = (uint8_t)(s_bldc_test - APP_DEVICE_BLDC_POS_90);
                int32_t angle_x10 = pos_vals[idx];

                BLDC_SetMultiAngle(BLDC_ADDR_1, angle_x10);
                BLDC_SetMultiAngle(BLDC_ADDR_2, angle_x10);
                BLDC_ReqFeedback(BLDC_ADDR_1, FB_MULTI_ANGLE);
                BLDC_ReqFeedback(BLDC_ADDR_2, FB_MULTI_ANGLE);

                snprintf(line0, sizeof(line0), "Mode:Position");
                snprintf(line1, sizeof(line1), "Tgt:%s", pos_names[idx]);
                snprintf(line2, sizeof(line2), "M1:%0.1f M2:%0.1f",
                         (double)BLDC_Motor1.multi_angle / 10.0,
                         (double)BLDC_Motor2.multi_angle / 10.0);
                snprintf(line3, sizeof(line3), "V:%0.1fV %s",
                         (double)BLDC_Motor1.voltage / 100.0,
                         (BLDC_Motor1.data_ready != 0U) ? "OK" : "---");
            } else{
                /* 速度模式 */
                int16_t rpm = speed_vals[(uint8_t)s_bldc_test];

                BLDC_SetSpeed(BLDC_ADDR_1, rpm);
                BLDC_SetSpeed(BLDC_ADDR_2, rpm);
                BLDC_ReqFeedback(BLDC_ADDR_1, FB_SPEED);
                BLDC_ReqFeedback(BLDC_ADDR_2, FB_SPEED);

                snprintf(line0, sizeof(line0), "Mode:Speed");
                snprintf(line1, sizeof(line1), "Tgt:%s", speed_names[(uint8_t)s_bldc_test]);
                snprintf(line2, sizeof(line2), "M1:%d M2:%d RPM",
                         (int)BLDC_Motor1.speed, (int)BLDC_Motor2.speed);
                snprintf(line3, sizeof(line3), "V:%0.1fV %s",
                         (double)BLDC_Motor1.voltage / 100.0,
                         (BLDC_Motor1.data_ready != 0U) ? "OK" : "---");
            }
            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1,
                           line2, line3, "K3:next K1/4:mod", NULL);
            break;
        }
        case APP_DEVICE_MODULE_IMU:
        default:{
            WIT_IMU_DATA data = {0};
            (void)WitGetData(&data);
            if (s_imu_test == APP_DEVICE_IMU_GYRO){
                snprintf(line0, sizeof(line0), "Gx:%0.1f", data.gyro_deg_s.x);
                snprintf(line1, sizeof(line1), "Gy:%0.1f", data.gyro_deg_s.y);
                snprintf(line2, sizeof(line2), "Gz:%0.1f", data.gyro_deg_s.z);
                snprintf(line3, sizeof(line3), "Poll:%lu", (unsigned long)JY61P_I2C_GetPollCount());
                Ui_RenderLines("IMU Gyro", line0, line1, line2, line3, "K3:item K1/4", NULL);
            } else if (s_imu_test == APP_DEVICE_IMU_STATUS){
                snprintf(line0, sizeof(line0), "Poll:%lu Err:%lu",
                         (unsigned long)JY61P_I2C_GetPollCount(),
                         (unsigned long)JY61P_I2C_GetErrorCount());
                snprintf(line1, sizeof(line1), "Nack:%lu Time:%lu",
                         (unsigned long)JY61P_I2C_GetNackCount(),
                         (unsigned long)JY61P_I2C_GetTimeoutCount());
                snprintf(line2, sizeof(line2), "UART Irq:%lu",
                         (unsigned long)g_app_debug_uart_irq_count);
                snprintf(line3, sizeof(line3), "UART Byte:%lu",
                         (unsigned long)g_app_debug_uart_drained_byte_count);
                Ui_RenderLines("IMU Status", line0, line1, line2, line3, "K3:item K1/4", NULL);
            } else if (s_imu_test == APP_DEVICE_IMU_MPU6050){
                short gx, gy, gz, ax, ay, az;
                MPU6050_ReadGyro(&gx, &gy, &gz);
                MPU6050_ReadAccel(&ax, &ay, &az);
                snprintf(line0, sizeof(line0), "G:%d,%d,%d", (int)gx, (int)gy, (int)gz);
                snprintf(line1, sizeof(line1), "A:%d,%d,%d", (int)ax, (int)ay, (int)az);
                snprintf(line2, sizeof(line2), "Temp:%0.1fC", (double)MPU6050_ReadTemp());
                snprintf(line3, sizeof(line3), "%s", MPU6050_IsPresent() ? "Detected" : "Not Found");
                Ui_RenderLines("IMU MPU6050", line0, line1, line2, line3, "K3:item K1/4", NULL);
            } else{
                snprintf(line0, sizeof(line0), "Roll:%0.1f", data.attitude_deg.roll);
                snprintf(line1, sizeof(line1), "Pitch:%0.1f", data.attitude_deg.pitch);
                snprintf(line2, sizeof(line2), "Yaw:%0.1f", data.attitude_deg.yaw);
                snprintf(line3, sizeof(line3), "Poll:%lu", (unsigned long)JY61P_I2C_GetPollCount());
                Ui_RenderLines("IMU Att", line0, line1, line2, line3, "K3:item K1/4", NULL);
            }
            break;
        }
    }
}

void AppDeviceCheck_Run(void){
    APP_DEVICE_MODULE module = APP_DEVICE_MODULE_MOTOR;
    uint32_t last_render_ms = BSP_Time_GetMs();
    uint32_t last_update_ms = last_render_ms;

    Key_ClearAllEvents();
    AppDeviceCheck_EnterModule(module);
    AppDeviceCheck_Render(module);

    while (1){
        if (AppDeviceCheck_IsNextModuleEvent()){
            AppDeviceCheck_StopYawHold();
            module = (APP_DEVICE_MODULE)(((uint8_t)module + 1U) % (uint8_t)APP_DEVICE_MODULE_COUNT);
            AppDeviceCheck_EnterModule(module);
            Key_ClearAllEvents();
            AppDeviceCheck_Render(module);
            last_render_ms = BSP_Time_GetMs();
        }

        if (AppDeviceCheck_IsPrevModuleEvent()){
            AppDeviceCheck_StopYawHold();
            module = (APP_DEVICE_MODULE)(((uint8_t)module + (uint8_t)APP_DEVICE_MODULE_COUNT - 1U) %
                (uint8_t)APP_DEVICE_MODULE_COUNT);
            AppDeviceCheck_EnterModule(module);
            Key_ClearAllEvents();
            AppDeviceCheck_Render(module);
            last_render_ms = BSP_Time_GetMs();
        }

        if (AppDeviceCheck_IsActionEvent()){
            if (AppDeviceCheck_TestCount(module) > 0U){
                AppDeviceCheck_AdvanceTest(module);
            }
            Key_ClearAllEvents();
            AppDeviceCheck_Render(module);
            last_render_ms = BSP_Time_GetMs();
        }

        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_update_ms) / 1000.0f;
        last_update_ms = now_ms;
        AppDeviceCheck_UpdateYawHoldOnImuFrame(dt_s);

        if (AppDeviceCheck_IsBackEvent()){
            Key_ClearAllEvents();
            AppDeviceCheck_StopOutputs();
            return;
        }

        uint32_t render_period_ms = s_yaw_hold_active ?
            APP_DEVICE_YAW_HOLD_RENDER_PERIOD_MS : APP_DEVICE_RENDER_PERIOD_MS;

        if ((BSP_Time_GetMs() - last_render_ms) >= render_period_ms){
            AppDeviceCheck_Render(module);
            last_render_ms = BSP_Time_GetMs();
        }

        BSP_DelayMs(s_yaw_hold_active ?
                    APP_DEVICE_YAW_HOLD_IDLE_DELAY_MS :
                    APP_DEVICE_LOOP_DELAY_MS);
    }
}
