#include "app_device_check.h"

#include "app_debug_cmd.h"
#include "bsp_time.h"
#include "canmv_uart.h"
#include "chassis.h"
#include "gimbal.h"
#include "gimbal_tracking.h"
#include "hall_encoder.h"
#include "key.h"
#include "line_follow.h"
#include "ui.h"
#include "wit_sdk.h"
#include "bsp/bldc/f32c_bldc.h"
#include "ti_msp_dl_config.h"

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
/*
 * Yaw Hold 采用开环补偿, 不做角度闭环:
 * 陀螺仪(JY61P)装在车体上而非云台, 无法测云台指向, 故不能用姿态角闭环。
 * 车体以角速度 gyro_z 转动时, 云台以相反角速度补偿即可维持世界指向:
 *   云台指令速度 = -COMP_GAIN * gyro_z
 * COMP_GAIN=1.0 表示 1:1 完全抵消; 若云台与车体转向传动比不同或需过/欠补偿,
 * 调此增益。GYRO_DEADBAND 抑制静止零漂, OUTPUT_LIMIT 限幅, RATE_LIMIT 平滑指令。
 */
#define APP_DEVICE_YAW_HOLD_COMP_GAIN 1.0f
#define APP_DEVICE_YAW_HOLD_OUTPUT_LIMIT_DEG_S 240.0f
#define APP_DEVICE_YAW_HOLD_RATE_LIMIT_DEG_S2 1800.0f
#define APP_DEVICE_YAW_HOLD_GYRO_DEADBAND_DEG_S 1.5f
/* Yaw Hold 调试打印: 经 debug 串口 (BlueTooth_INST) 低频输出控制内部量。
 * 控制更新为 200Hz, 打印按下述周期节流, 避免刷屏并降低阻塞发送占用。 */
#define APP_DEVICE_YAW_HOLD_DEBUG_ENABLE 1
#define APP_DEVICE_YAW_HOLD_DEBUG_PERIOD_MS 50U

typedef enum {
    APP_DEVICE_MODULE_MOTOR = 0,
    APP_DEVICE_MODULE_YAW_HOLD,
    APP_DEVICE_MODULE_LINE,
    APP_DEVICE_MODULE_IMU,
    APP_DEVICE_MODULE_K230,
    APP_DEVICE_MODULE_BLDC,
    APP_DEVICE_MODULE_PID,
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
static float s_yaw_hold_gyro_z_deg_s;
static float s_yaw_hold_output_deg_s;
static bool s_yaw_hold_active;
static uint32_t s_yaw_hold_last_update_count;
static uint32_t s_yaw_hold_last_update_ms;
static bool s_bldc_initialized;

static float AppDeviceCheck_AbsFloat(float value){
    return (value < 0.0f) ? -value : value;
}

#if APP_DEVICE_YAW_HOLD_DEBUG_ENABLE
/* 经 debug 串口阻塞发送一段字符串 (仅用于低频调试打印)。 */
static void AppDeviceCheck_DebugUartSend(const char *text){
    while (*text != '\0'){
        while (DL_UART_isTXFIFOFull(BlueTooth_INST)){
            /* 等待 TX FIFO 有空位 */
        }
        DL_UART_Main_transmitData(BlueTooth_INST, (uint8_t)*text);
        text++;
    }
}
#endif

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
        "Yaw Hold",
        "Line Sensor",
        "IMU",
        "K230",
        "BLDC",
        "PID Comm",
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

static void AppDeviceCheck_StartYawHold(void){
    s_yaw_hold_gyro_z_deg_s = 0.0f;
    s_yaw_hold_output_deg_s = 0.0f;
    s_yaw_hold_last_update_count = JY61P_I2C_GetPollCount();
    s_yaw_hold_last_update_ms = BSP_Time_GetMs();
    s_yaw_hold_active = true;
}

static void AppDeviceCheck_StopYawHold(void){
    s_yaw_hold_active = false;
    s_yaw_hold_gyro_z_deg_s = 0.0f;
    s_yaw_hold_output_deg_s = 0.0f;
    s_yaw_hold_last_update_count = JY61P_I2C_GetPollCount();
    s_yaw_hold_last_update_ms = BSP_Time_GetMs();
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

    /*
     * 开环补偿: 陀螺仪在车体上, 只能测车体角速度 gyro_z。
     * 令云台以相反角速度旋转, 抵消车体转动, 维持云台世界指向。
     * 死区内(静止零漂)不动, 输出归零。
     */
    {
        float gyro_z_deg_s = s_yaw_hold_gyro_z_deg_s;
        float target_output_deg_s;

        if (AppDeviceCheck_AbsFloat(gyro_z_deg_s) <= APP_DEVICE_YAW_HOLD_GYRO_DEADBAND_DEG_S){
            gyro_z_deg_s = 0.0f;
        }

        target_output_deg_s =
            AppDeviceCheck_LimitFloat(-APP_DEVICE_YAW_HOLD_COMP_GAIN * gyro_z_deg_s,
                                      APP_DEVICE_YAW_HOLD_OUTPUT_LIMIT_DEG_S);

        /* 斜率限幅平滑指令, 抑制陀螺瞬时尖峰造成的速度阶跃。 */
        float max_delta_deg_s = APP_DEVICE_YAW_HOLD_RATE_LIMIT_DEG_S2 * dt_s;
        float delta_deg_s = target_output_deg_s - s_yaw_hold_output_deg_s;
        delta_deg_s = AppDeviceCheck_LimitFloat(delta_deg_s, max_delta_deg_s);
        s_yaw_hold_output_deg_s =
            AppDeviceCheck_LimitFloat(s_yaw_hold_output_deg_s + delta_deg_s,
                                      APP_DEVICE_YAW_HOLD_OUTPUT_LIMIT_DEG_S);
    }

    (void)Gimbal_SetSpeed(s_yaw_hold_output_deg_s, 0.0f);

#if APP_DEVICE_YAW_HOLD_DEBUG_ENABLE
    /* 低频调试打印: 车体航向/车体角速度/云台补偿指令/控制周期 dt。
     * 按时间节流到 APP_DEVICE_YAW_HOLD_DEBUG_PERIOD_MS, 阻塞发送但占用很小。 */
    {
        static uint32_t s_yaw_hold_debug_last_ms;
        uint32_t now_ms = BSP_Time_GetMs();

        if ((now_ms - s_yaw_hold_debug_last_ms) >= APP_DEVICE_YAW_HOLD_DEBUG_PERIOD_MS){
            char dbg[80];

            s_yaw_hold_debug_last_ms = now_ms;
            (void)snprintf(dbg, sizeof(dbg),
                           "YH yaw=%.1f gz=%.1f out=%.1f dt=%.1fms\r\n",
                           (double)data.attitude_deg.yaw,
                           (double)s_yaw_hold_gyro_z_deg_s,
                           (double)s_yaw_hold_output_deg_s,
                           (double)(dt_s * 1000.0f));
            AppDeviceCheck_DebugUartSend(dbg);
        }
    }
#endif
}

static void AppDeviceCheck_UpdateYawHoldOnImuFrame(void){
    uint32_t update_count = JY61P_I2C_GetPollCount();
    uint32_t now_ms;
    float dt_s;

    if (!s_yaw_hold_active || (update_count == s_yaw_hold_last_update_count)){
        return;
    }

    /*
     * dt 必须取"上一次控制更新到本次"的真实间隔 (帧到帧, 约 5 ms),
     * 而不是主循环单次迭代的间隔 (约 1 ms)。后者会让 PID 的 D 项/积分/
     * 斜率限幅所用 dt 偏小约 5 倍, 放大微分噪声并削弱限幅斜率。
     */
    now_ms = BSP_Time_GetMs();
    dt_s = (float)(now_ms - s_yaw_hold_last_update_ms) / 1000.0f;
    s_yaw_hold_last_update_ms = now_ms;
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
        case APP_DEVICE_MODULE_YAW_HOLD:
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

static void AppDeviceCheck_RunPidCommTest(void);

static void AppDeviceCheck_AdvanceTest(APP_DEVICE_MODULE module){
    switch (module){
        case APP_DEVICE_MODULE_MOTOR:
            s_motor_test = (APP_DEVICE_MOTOR_TEST)(((uint8_t)s_motor_test + 1U) %
                (uint8_t)APP_DEVICE_MOTOR_COUNT);
            AppDeviceCheck_ApplyMotorTest();
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
        case APP_DEVICE_MODULE_PID:
            AppDeviceCheck_RunPidCommTest();   /* K3 启动 PID 通讯测试 (阻塞至返回) */
            break;
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

/*
 * PID 通讯测试: 验证上位机能否经 debug 串口(Debug_Ex)实时修改跟踪 PID。
 * 进入后以宏默认为基线并回显; 循环轮询上位机命令(AppDebugCmd_Poll)并把当前 PID
 * 显示在 OLED 供核对; 长按返回退出, 退出时用 GimbalTracking_Init(NULL) 恢复宏默认。
 * 仅测试通讯与设参机制, 不驱动云台电机。
 */
static void AppDeviceCheck_RunPidCommTest(void){
    char line0[24];
    char line1[24];
    char line2[24];
    char line3[24];
    uint32_t last_render_ms = BSP_Time_GetMs();

    AppDeviceCheck_StopOutputs();
    GimbalTracking_Init(NULL);          /* 基线: 宏定义默认 PID */
    AppDebugCmd_EmitConfig();           /* 让上位机同步当前值 */
    Key_ClearAllEvents();

    while (!AppDeviceCheck_IsBackEvent()){
        AppDebugCmd_Poll();             /* 应用上位机 "s <key> <val>" 命令 */

        if ((BSP_Time_GetMs() - last_render_ms) >= APP_DEVICE_RENDER_PERIOD_MS){
            GIMBAL_TRACKING_CONFIG c = GimbalTracking_GetConfig();
            last_render_ms = BSP_Time_GetMs();
            snprintf(line0, sizeof(line0), "Ykp%.2f Yki%.2f",
                     (double)c.yaw_pid.kp, (double)c.yaw_pid.ki);
            snprintf(line1, sizeof(line1), "Ykd%.2f Ol%.0f",
                     (double)c.yaw_pid.kd, (double)c.yaw_pid.output_limit);
            snprintf(line2, sizeof(line2), "Pkp%.2f Pki%.2f",
                     (double)c.pitch_pid.kp, (double)c.pitch_pid.ki);
            snprintf(line3, sizeof(line3), "Pkd%.2f", (double)c.pitch_pid.kd);
            Ui_RenderLines("PID Test", line0, line1, line2, line3,
                           "PC:s/g  K2long:back", NULL);
        }
        BSP_DelayMs(APP_DEVICE_LOOP_DELAY_MS);
    }

    GimbalTracking_Init(NULL);          /* 退出: 恢复宏定义默认 PID */
    AppDebugCmd_EmitConfig();
    Key_ClearAllEvents();
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
        case APP_DEVICE_MODULE_YAW_HOLD:{
            WIT_IMU_DATA data = {0};
            (void)WitGetData(&data);

            snprintf(line0, sizeof(line0), "Mode:%s", s_yaw_hold_active ? "ON" : "OFF");
            snprintf(line1, sizeof(line1), "Yaw:%0.1f", data.attitude_deg.yaw);
            snprintf(line2, sizeof(line2), "Gz:%0.1f", s_yaw_hold_gyro_z_deg_s);
            snprintf(line3, sizeof(line3), "Out:%0.1f", s_yaw_hold_output_deg_s);
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
            const CANMV_TARGET_DATA *angle_data = CanMvUart_GetTargetData(CANMV_TARGET_ANGLE);

            snprintf(line0, sizeof(line0), "Rx:%lu F:%lu",
                     (unsigned long)g_canmv_uart_rx_byte_count,
                     (unsigned long)g_canmv_uart_valid_frame_count);
            snprintf(line1, sizeof(line1), "Drop:%lu A:%lu",
                     (unsigned long)g_canmv_uart_drop_count,
                     (unsigned long)g_canmv_uart_angle_frame_count);

            if (angle_data != NULL && angle_data->count >= 2U){
                float yaw_error_deg = (float)(int16_t)angle_data->value[0] /
                                      CANMV_ANGLE_SCALE;
                float pitch_error_deg = (float)(int16_t)angle_data->value[1] /
                                        CANMV_ANGLE_SCALE;
                snprintf(line2, sizeof(line2), "Yaw:%0.2f", yaw_error_deg);
                snprintf(line3, sizeof(line3), "Pit:%0.2f S:%d",
                         pitch_error_deg, (int)angle_data->status);
            } else{
                snprintf(line2, sizeof(line2), "Yaw:--.--");
                snprintf(line3, sizeof(line3), "Pit:--.-- S:%d",
                         (int)CanMvUart_GetStatus(CANMV_TARGET_ANGLE));
            }

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
        case APP_DEVICE_MODULE_PID:{
            GIMBAL_TRACKING_CONFIG c = GimbalTracking_GetConfig();
            snprintf(line0, sizeof(line0), "Ykp%.2f Pkp%.2f",
                     (double)c.yaw_pid.kp, (double)c.pitch_pid.kp);
            snprintf(line1, sizeof(line1), "PID comm test");
            snprintf(line2, sizeof(line2), "K3: start test");
            snprintf(line3, sizeof(line3), "PC send s/g");
            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1,
                           line2, line3, "K3:test K1/4:mod", NULL);
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

        AppDeviceCheck_UpdateYawHoldOnImuFrame();

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
