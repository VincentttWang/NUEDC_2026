#include "app_e_task.h"

#include "app_debug_cmd.h"
#include "bsp_time.h"
#include "canmv_uart.h"
#include "chassis.h"
#include "debug_uart.h"
#include "gimbal.h"
#include "gimbal_tracking/gimbal_tracking.h"
#include "key.h"
#include "line_follow.h"
#include "line_tracking/line_tracking.h"
#include "motion/motion.h"
#include "ui.h"
#include "wit_sdk.h"

#include <stdbool.h>
#include <stdio.h>

#define APP_E_MAX_LAPS 5U
#define APP_E_EDGES_PER_LAP 4U
#define APP_E_LINE_TIMEOUT_MS 20000U
#define APP_E_RECT_SCAN_SPEED_DEG_S 60.0f
#define APP_E_RECT_SCAN_ANGLE_DEG 360.0f
#define APP_E_RECT_SCAN_TIMEOUT_MS 8000U
#define APP_E_RECT_FOUND_SETTLE_MS 200U
#define APP_E_LINE_LOST_GRACE_MS 1000U
#define APP_E_LINE_EDGE_MIN_DISTANCE_M 0.15f
#define APP_E_CORNER_TURN_LEFT_DUTY_PERCENT -12.0f
#define APP_E_CORNER_TURN_RIGHT_DUTY_PERCENT 12.0f
#define APP_E_CORNER_FORWARD_DISTANCE_M 0.09f
#define APP_E_CORNER_FORWARD_DUTY_PERCENT 15.0f
#define APP_E_CORNER_ENTER_CONFIRM_COUNT 2U
#define APP_E_CORNER_EXIT_CONFIRM_COUNT 3U
#define APP_E_LOOP_DELAY_MS 10U
/* 更新同步遥测抽取: 每 N 次传感器更新记一条 (带 SysTick 时间戳还原真实时序)。 */
#define APP_E_AIM_LOG_EVERY_FRAMES 1U   /* Aim: 每 N 个 CanMV 视觉帧 */
#define APP_E_LINE_LOG_EVERY_SCANS 10U  /* E1: 每 N 次灰度扫描 */
/* 复用设备检查页已实测的车体转向开环 Yaw Hold 参数。 */
#define APP_E_TURN_YAW_HOLD_COMP_GAIN 1.0f
#define APP_E_TURN_YAW_HOLD_OUTPUT_LIMIT_DEG_S 240.0f
#define APP_E_TURN_YAW_HOLD_RATE_LIMIT_DEG_S2 1800.0f
#define APP_E_TURN_YAW_HOLD_GYRO_DEADBAND_DEG_S 1.5f

typedef enum {
    APP_E_LINE_STATE_FOLLOW = 0,
    APP_E_LINE_STATE_CORNER_FORWARD,
    APP_E_LINE_STATE_CORNER_ARC
} APP_E_LINE_STATE;

typedef enum {
    APP_E_CORNER_NONE = 0,
    APP_E_CORNER_LEFT,
    APP_E_CORNER_RIGHT
} APP_E_CORNER_DIR;

typedef enum {
    APP_E_RECT_SCAN_YAW_POSITIVE = 0,
    APP_E_RECT_SCAN_YAW_NEGATIVE
} APP_E_RECT_SCAN_DIR;

typedef enum {
    APP_E_AIM_MODE_NONE = 0,
    APP_E_AIM_MODE_CENTER,
} APP_E_AIM_MODE;

static float s_app_e_turn_yaw_hold_deg_s;
static uint32_t s_app_e_turn_yaw_hold_last_poll_count;
static uint32_t s_app_e_turn_yaw_hold_last_update_ms;

static float AppE_AbsFloat(float value){
    return (value < 0.0f) ? -value : value;
}

static float AppE_LimitFloat(float value, float limit){
    if (value > limit){
        return limit;
    }
    if (value < -limit){
        return -limit;
    }
    return value;
}

/* K230 UART1 接收此命令后直接控制激光 GPIO；命令只从 MSPM0 发往 K230。 */
static void AppE_SetLaserEnabled(bool enabled){
    (void)CanMvUart_SendString(enabled ? "LASER=1\n" : "LASER=0\n");
}

/*
 * 车体转弯时，IMU 的 gyro_z 表示车体相对世界的角速度；云台以反向速度补偿。
 * 视觉控制仍是主控制量，Yaw Hold 仅叠加在 yaw 指令上，因而不会中断靶心追踪。
 */
static void AppE_ApplyTurnYawHold(bool turning,
                                  float tracking_yaw_deg_s,
                                  float tracking_pitch_deg_s){
    WIT_IMU_DATA imu = {0};
    uint32_t poll_count;
    uint32_t now_ms;
    float dt_s;
    float gyro_z_deg_s;
    float target_hold_deg_s;
    float max_delta_deg_s;
    float combined_yaw_deg_s;

    if (!turning){
        s_app_e_turn_yaw_hold_deg_s = 0.0f;
        return;
    }

    poll_count = JY61P_I2C_GetPollCount();
    if (poll_count != s_app_e_turn_yaw_hold_last_poll_count){
        now_ms = BSP_Time_GetMs();
        dt_s = (float)(now_ms - s_app_e_turn_yaw_hold_last_update_ms) / 1000.0f;
        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }
        s_app_e_turn_yaw_hold_last_poll_count = poll_count;
        s_app_e_turn_yaw_hold_last_update_ms = now_ms;

        (void)WitGetData(&imu);
        gyro_z_deg_s = imu.gyro_deg_s.z;
        if (AppE_AbsFloat(gyro_z_deg_s) <= APP_E_TURN_YAW_HOLD_GYRO_DEADBAND_DEG_S){
            gyro_z_deg_s = 0.0f;
        }

        target_hold_deg_s = AppE_LimitFloat(-APP_E_TURN_YAW_HOLD_COMP_GAIN * gyro_z_deg_s,
                                             APP_E_TURN_YAW_HOLD_OUTPUT_LIMIT_DEG_S);
        max_delta_deg_s = APP_E_TURN_YAW_HOLD_RATE_LIMIT_DEG_S2 * dt_s;
        s_app_e_turn_yaw_hold_deg_s += AppE_LimitFloat(target_hold_deg_s - s_app_e_turn_yaw_hold_deg_s,
                                                        max_delta_deg_s);
        s_app_e_turn_yaw_hold_deg_s = AppE_LimitFloat(s_app_e_turn_yaw_hold_deg_s,
                                                       APP_E_TURN_YAW_HOLD_OUTPUT_LIMIT_DEG_S);
    }

    combined_yaw_deg_s = AppE_LimitFloat(tracking_yaw_deg_s + s_app_e_turn_yaw_hold_deg_s,
                                         APP_E_TURN_YAW_HOLD_OUTPUT_LIMIT_DEG_S);
    (void)Gimbal_SetSpeed(combined_yaw_deg_s, tracking_pitch_deg_s);
}

static BSP_STATUS AppE_ApplyCornerTurn(APP_E_CORNER_DIR corner_dir){
    if (corner_dir == APP_E_CORNER_LEFT){
        return Chassis_SetDuty(APP_E_CORNER_TURN_LEFT_DUTY_PERCENT,
                               APP_E_CORNER_TURN_RIGHT_DUTY_PERCENT);
    }

    if (corner_dir == APP_E_CORNER_RIGHT){
        return Chassis_SetDuty(APP_E_CORNER_TURN_RIGHT_DUTY_PERCENT,
                               APP_E_CORNER_TURN_LEFT_DUTY_PERCENT);
    }

    return BSP_STATUS_INVALID_ARG;
}

static uint32_t AppE_ElapsedMs(uint32_t start_ms){
    return BSP_Time_GetMs() - start_ms;
}

static bool AppE_IsBackEvent(void){
    return Key_IsLongPress(KEY_ID_CALIB);
}

static void AppE_WaitBack(void){
    Key_ClearAllEvents();

    while (!AppE_IsBackEvent()){
        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    Key_ClearAllEvents();
}

static void AppE_PrepareTaskInput(void){
    Key_ClearAllEvents();

    while (Key_IsPressed(KEY_ID_ENTER)){
        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    Key_ClearAllEvents();
}

static uint8_t AppE_NormalizeLapCount(uint8_t lap_count){
    if (lap_count == 0U){
        return 1U;
    }

    if (lap_count > APP_E_MAX_LAPS){
        return APP_E_MAX_LAPS;
    }

    return lap_count;
}

static bool AppE_IsLineActive(const LINE_FOLLOW_SENSOR_STATE *sensor,
                              uint8_t index){
    if ((sensor == NULL) || (index >= LINE_FOLLOW_SENSOR_COUNT)){
        return false;
    }

    if ((LINE_TRACKING_ACTIVE_SENSOR_MASK & (1U << index)) == 0U){
        return false;
    }

    return sensor->value[index] == 0U;
}

static bool AppE_IsLineInnerActive(const LINE_FOLLOW_SENSOR_STATE *sensor){
    return AppE_IsLineActive(sensor, 3U) ||
           AppE_IsLineActive(sensor, 4U);
}

static bool AppE_HasEnabledLineActive(const LINE_FOLLOW_SENSOR_STATE *sensor){
    if (sensor == NULL){
        return false;
    }

    for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        if (AppE_IsLineActive(sensor, i)){
            return true;
        }
    }

    return false;
}

static void AppE_RunLineTask(uint8_t lap_count, APP_E_AIM_MODE aim_mode){
    char line0[24];
    char line1[24];
    char line2[24];
    uint8_t target_laps = AppE_NormalizeLapCount(lap_count);
    int32_t target_edges = (int32_t)(target_laps * APP_E_EDGES_PER_LAP);
    uint32_t task_timeout_ms = APP_E_LINE_TIMEOUT_MS * target_laps;
    uint32_t start_ms = BSP_Time_GetMs();
    uint32_t last_ms = start_ms;
    uint8_t gs_div = 0U;
    uint32_t line_lost_start_ms = start_ms;
    float corner_forward_start_distance = 0.0f;
    uint32_t corner_count = 0U;
    bool line_lost_pending = false;
    APP_E_LINE_STATE line_state = APP_E_LINE_STATE_FOLLOW;
    APP_E_CORNER_DIR corner_dir = APP_E_CORNER_NONE;
    uint8_t corner_enter_count = 0U;
    MOTION_COMMAND line_follow_command = Motion_CommandLineFollow();

    s_app_e_turn_yaw_hold_deg_s = 0.0f;
    s_app_e_turn_yaw_hold_last_poll_count = JY61P_I2C_GetPollCount();
    s_app_e_turn_yaw_hold_last_update_ms = BSP_Time_GetMs();

    /*
     * 基本要求(1)只使用底盘巡线；发挥任务在同一主循环中并行运行云台视觉控制。
     * 圈数换算为边线数：当前场地按每圈 4 条边线估计。
     */
    if (aim_mode == APP_E_AIM_MODE_NONE){
        AppE_SetLaserEnabled(false);
        (void)GimbalTracking_Stop();
    } else{
        AppE_SetLaserEnabled(true);
        Gimbal_EnsurePitchReady();
        GimbalTracking_Init(NULL);
        GimbalTracking_Reset();
    }
    LineFollow_Reset();
    LineTracking_Init(NULL);
    Motion_Init();
    Chassis_ResetDistance();
    Ui_RenderLines("E1 Line",
                   "Running...",
                   "Lap:0",
                   "Edge:0",
                   "Time:0.0",
                   "K2 long:stop",
                   NULL);
    DebugUart_Printf("[E1] start laps=%u edges=%ld\r\n",
                     (unsigned)target_laps, (long)target_edges);

    while (AppE_ElapsedMs(start_ms) < task_timeout_ms){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;

        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        last_ms = now_ms;

        if (aim_mode == APP_E_AIM_MODE_CENTER){
            (void)GimbalTracking_UpdateLaserCenter(dt_s);
            {
                GIMBAL_TRACKING_STATE tracking = GimbalTracking_GetState();

                AppE_ApplyTurnYawHold(line_state == APP_E_LINE_STATE_CORNER_ARC,
                                      tracking.yaw_speed,
                                      tracking.pitch_speed);
            }
        }

        BSP_STATUS status = BSP_STATUS_OK;
        LINE_FOLLOW_SENSOR_STATE sensor = {0};

        if (LineFollow_GetSensor(&sensor) != BSP_STATUS_OK){
            (void)LineFollow_UpdateSensor();
            (void)LineFollow_GetSensor(&sensor);
        }

        if (line_state == APP_E_LINE_STATE_FOLLOW){
            /* Motion_Apply() 只执行巡线运动原语，圈数和丢线仍由 E1 任务状态机判断。 */
            status = Motion_Apply(&line_follow_command, dt_s);
            (void)LineFollow_GetSensor(&sensor);

            /*
             * 当前测试策略：8 路灰度全部未检测到轨道时认为到达拐角入口。
             * 该条件不携带左右方向，E1 按逆时针行驶需求默认执行左转。
             */
            if (AppE_HasEnabledLineActive(&sensor)){
                corner_enter_count = 0U;
            } else if (corner_enter_count < APP_E_CORNER_ENTER_CONFIRM_COUNT){
                corner_enter_count++;
            }

            if (corner_enter_count >= APP_E_CORNER_ENTER_CONFIRM_COUNT){
                corner_dir = APP_E_CORNER_LEFT;
                line_state = APP_E_LINE_STATE_CORNER_FORWARD;
                corner_forward_start_distance = Chassis_GetDistance();
                corner_enter_count = 0U;
                corner_count++;
                status = Chassis_SetDuty(APP_E_CORNER_FORWARD_DUTY_PERCENT, APP_E_CORNER_FORWARD_DUTY_PERCENT);
                DebugUart_Printf("[E1] corner#%lu enter fwd t=%lu\r\n",
                                 (unsigned long)corner_count,
                                 (unsigned long)AppE_ElapsedMs(start_ms));
            }
        } else if (line_state == APP_E_LINE_STATE_CORNER_FORWARD){
            /*
             * 转角确认后按设定占空比前行驶出一定距离，
             * 直到编码器累加距离达到设定阈值后再开始差速转弯。
             */
            status = Chassis_SetDuty(APP_E_CORNER_FORWARD_DUTY_PERCENT, APP_E_CORNER_FORWARD_DUTY_PERCENT);
            (void)LineFollow_UpdateSensor();
            (void)LineFollow_GetSensor(&sensor);

            float current_distance = Chassis_GetDistance();
            float distance_diff = current_distance - corner_forward_start_distance;
            if ((distance_diff >= APP_E_CORNER_FORWARD_DISTANCE_M) || (distance_diff <= -APP_E_CORNER_FORWARD_DISTANCE_M)){
                line_state = APP_E_LINE_STATE_CORNER_ARC;
                status = AppE_ApplyCornerTurn(corner_dir);
            }
        } else{
            status = AppE_ApplyCornerTurn(corner_dir);
            (void)LineFollow_UpdateSensor();
            (void)LineFollow_GetSensor(&sensor);

            /* 只要 3/4 号传感器检测到黑线就结束转弯。 */
            if (AppE_IsLineInnerActive(&sensor)){
                (void)Chassis_Brake();
                line_state = APP_E_LINE_STATE_FOLLOW;
                corner_dir = APP_E_CORNER_NONE;
                line_lost_pending = false;
                LineTracking_Reset();
                LineFollow_IncrementEdge();
                {
                    int32_t ec = LineFollow_GetEdgeCount();
                    DebugUart_Printf("[E1] edge=%ld lap=%u t=%lu\r\n",
                                     (long)ec, (unsigned)(ec / APP_E_EDGES_PER_LAP),
                                     (unsigned long)AppE_ElapsedMs(start_ms));
                }
            }
        }

        bool line_missing = (line_state == APP_E_LINE_STATE_FOLLOW) &&
                            !AppE_HasEnabledLineActive(&sensor);

        if (line_missing){
            if (!line_lost_pending){
                line_lost_start_ms = now_ms;
                line_lost_pending = true;
            }
        } else{
            line_lost_pending = false;
        }

        if (line_missing &&
            ((now_ms - line_lost_start_ms) >= APP_E_LINE_LOST_GRACE_MS)){
            char turn_line[24];

            /* 丢线属于题目流程故障，在 app 层刹车并等待用户长按返回。 */
            (void)Motion_Stop();
            DebugUart_Printf("[E1] LINE LOST turn=%lu t=%lu\r\n",
                             (unsigned long)corner_count,
                             (unsigned long)AppE_ElapsedMs(start_ms));
            snprintf(turn_line, sizeof(turn_line), "Turn:%lu", (unsigned long)corner_count);
            Ui_RenderLines("E1 Line",
                           "[WARN]",
                           "Line lost",
                           turn_line,
                           "K2 long:back",
                           NULL,
                           NULL);
            AppE_WaitBack();
            AppE_SetLaserEnabled(false);
            (void)GimbalTracking_Stop();
            return;
        }

        int32_t edge_count = LineFollow_GetEdgeCount();
        uint8_t completed_laps = (uint8_t)(edge_count / APP_E_EDGES_PER_LAP);

        snprintf(line0, sizeof(line0), "Lap:%u/%u", completed_laps, target_laps);
        snprintf(line1, sizeof(line1), "Edge:%ld/%ld", (long)edge_count, (long)target_edges);
        snprintf(line2, sizeof(line2), "Time:%lu.%01lu",
                 (unsigned long)(AppE_ElapsedMs(start_ms) / 1000U),
                 (unsigned long)((AppE_ElapsedMs(start_ms) / 100U) % 10U));
        Ui_UpdateContentLine(1U, line0);
        Ui_UpdateContentLine(2U, line1);
        Ui_UpdateContentLine(3U, line2);

        /* 更新同步遥测: 每 N 次灰度扫描记一条 (带 SysTick 时间戳)。 */
        if (++gs_div >= APP_E_LINE_LOG_EVERY_SCANS){
            uint16_t gs = 0U;
            gs_div = 0U;
            for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
                if (AppE_IsLineActive(&sensor, i)){
                    gs |= (uint16_t)(1U << i);
                }
            }
            DebugUart_Printf("[E1] t=%lu gs=0x%02X st=%d lap=%u edge=%ld\r\n",
                             (unsigned long)now_ms, (unsigned)gs, (int)line_state,
                             (unsigned)completed_laps, (long)edge_count);
        }

        if (edge_count >= target_edges){
            (void)Motion_Stop();
            DebugUart_Printf("[E1] FINISH laps=%u t=%lu\r\n",
                             (unsigned)completed_laps,
                             (unsigned long)AppE_ElapsedMs(start_ms));
            Ui_RenderStatusPage("E1 Line", UI_STATUS_OK, "Finished", "K2 long:back");
            AppE_WaitBack();
            AppE_SetLaserEnabled(false);
            (void)GimbalTracking_Stop();
            return;
        }

        if (AppE_IsBackEvent()){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    (void)Motion_Stop();
    AppE_SetLaserEnabled(false);
    (void)GimbalTracking_Stop();
    Ui_RenderStatusPage("E1 Line", UI_STATUS_WARN, "Stopped/timeout", "K2 long:back");
    AppE_WaitBack();
}

void AppE_RunLineFollow(uint8_t lap_count){
    AppE_RunLineTask(lap_count, APP_E_AIM_MODE_NONE);
}

void AppE_RunLineAim(uint8_t lap_count){
    AppE_RunLineTask(lap_count, APP_E_AIM_MODE_CENTER);
}

void AppE_RunContinuousAim(void){
    char line0[24];
    char line1[24];
    char line2[24];
    uint32_t start_ms = BSP_Time_GetMs();
    uint32_t last_ms = start_ms;
    uint32_t last_vf = g_canmv_uart_valid_frame_count;
    uint8_t vf_div = 0U;
    bool prev_locked = false;

    /*
     * 持续视觉瞄准: 不设超时, 一直把 K230 目标中心闭环驱到图像中心(=激光瞄准),
     * 丢失目标时 GimbalTracking 内部会停云台, 目标重现自动恢复, 直到长按返回。
     * 与 E2 共用 GimbalTracking_UpdateLaserCenter, 但不在链路超时时退出。
     */
    AppE_PrepareTaskInput();
    (void)Chassis_Brake();
    /* Aim track 用于持续跟踪调试：进入后激光保持常亮，直到用户退出。 */
    AppE_SetLaserEnabled(true);
    Gimbal_EnsurePitchReady();
    GimbalTracking_Init(NULL);
    GimbalTracking_Reset();
    Ui_RenderLines("Aim track",
                   "Tracking...",
                   "Lock:WAIT",
                   "Err:0,0",
                   "Spd:0,0",
                   "K2 long:stop",
                   NULL);
    DebugUart_Puts("[AIM] start\r\n");
    AppDebugCmd_EmitConfig();   /* 进入即回显当前 PID, 供上位机滑块同步 */

    while (1){
        uint32_t now_ms = BSP_Time_GetMs();

        AppDebugCmd_Poll();     /* 处理上位机实时调参命令 */
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;

        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        last_ms = now_ms;
        BSP_STATUS status = GimbalTracking_UpdateLaserCenter(dt_s);
        GIMBAL_TRACKING_STATE state = GimbalTracking_GetState();

        bool locked = (status == BSP_STATUS_OK);
        const char *lock = locked ? "LOCK" : (state.link_timeout ? "LOST" : "WAIT");

        snprintf(line0, sizeof(line0), "Lock:%s", lock);
        snprintf(line1, sizeof(line1), "Err:%0.0f,%0.0f", state.error.x, state.error.y);
        snprintf(line2, sizeof(line2), "Spd:%0.0f,%0.0f", state.yaw_speed, state.pitch_speed);
        Ui_UpdateContentLine(1U, line0);
        Ui_UpdateContentLine(2U, line1);
        Ui_UpdateContentLine(3U, line2);

        /* 事件: Lock/Lost 切换即时打印。 */
        if (locked != prev_locked){
            prev_locked = locked;
            DebugUart_Printf("[AIM] %s ex=%d ey=%d\r\n", locked ? "LOCK" : "LOST",
                             (int)state.error.x, (int)state.error.y);
        }
        /* 更新同步遥测: 每收到 N 个 CanMV 视觉帧记一条 (带 SysTick 时间戳)。 */
        {
            uint32_t vf = g_canmv_uart_valid_frame_count;
            if (vf != last_vf){
                last_vf = vf;
                if (++vf_div >= APP_E_AIM_LOG_EVERY_FRAMES){
                    GIMBAL_ANGLE ang = Gimbal_GetAngle();
                    vf_div = 0U;
                    DebugUart_Printf(
                        "[AIM] t=%lu st=%d ex=%d ey=%d cy=%d cp=%d pit=%d yaw=%d vf=%lu drp=%lu\r\n",
                        (unsigned long)now_ms,
                        locked ? 1 : (state.link_timeout ? 2 : 0),
                        (int)state.error.x, (int)state.error.y,
                        (int)state.yaw_speed, (int)state.pitch_speed,
                        (int)ang.pitch_deg, (int)ang.yaw_deg,
                        (unsigned long)vf,
                        (unsigned long)g_canmv_uart_drop_count);
                }
            }
        }

        if (AppE_IsBackEvent()){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    (void)GimbalTracking_Stop();
    (void)Gimbal_Stop();
    AppE_SetLaserEnabled(false);
    DebugUart_Puts("[AIM] stop\r\n");
    Ui_RenderStatusPage("Aim track", UI_STATUS_OK, "Stopped", "K2 long:back");
    AppE_WaitBack();
}

static void AppE_RunAimCenter(uint32_t aim_duration_ms,
                              const char *title,
                              bool enable_laser_after_aim){
    char line0[24];
    char line1[24];
    char line2[24];
    uint32_t start_ms = BSP_Time_GetMs();
    uint32_t last_ms = start_ms;

    AppE_PrepareTaskInput();
    (void)Chassis_Brake();
    AppE_SetLaserEnabled(false);
    /* 瞄准前确保 pitch 已抬升到位并处于速度模式 (开机为非阻塞抬升)。 */
    Gimbal_EnsurePitchReady();
    GimbalTracking_Init(NULL);
    Ui_RenderLines(title,
                   "Aiming center",
                   "Laser:WAIT",
                   "Err:0,0",
                   "Time:0.0",
                   "K2 long:stop",
                   NULL);

    while ((aim_duration_ms == 0U) || (AppE_ElapsedMs(start_ms) < aim_duration_ms)){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;

        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        last_ms = now_ms;
        BSP_STATUS status = GimbalTracking_UpdateLaserCenter(dt_s);
        GIMBAL_TRACKING_STATE state = GimbalTracking_GetState();

        snprintf(line0, sizeof(line0), "Laser:%s", status == BSP_STATUS_OK ? "OK" : "WAIT");
        snprintf(line1, sizeof(line1), "Err:%0.0f,%0.0f", state.error.x, state.error.y);
        snprintf(line2, sizeof(line2), "Time:%lu.%01lu",
                 (unsigned long)(AppE_ElapsedMs(start_ms) / 1000U),
                 (unsigned long)((AppE_ElapsedMs(start_ms) / 100U) % 10U));
        Ui_UpdateContentLine(1U, line0);
        Ui_UpdateContentLine(2U, line1);
        Ui_UpdateContentLine(3U, line2);

        if (AppE_IsBackEvent()){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    if ((aim_duration_ms != 0U) &&
        (AppE_ElapsedMs(start_ms) >= aim_duration_ms) &&
        enable_laser_after_aim){
        AppE_SetLaserEnabled(true);
    } else{
        AppE_SetLaserEnabled(false);
    }
    (void)GimbalTracking_Stop();
    Ui_RenderStatusPage(title, UI_STATUS_OK, "Aim finished", "K2 long:back");
    AppE_WaitBack();
    AppE_SetLaserEnabled(false);
}

void AppE_RunAimCenter2s(void){
    AppE_RunAimCenter(2000U, "E2 Aim", true);
}

static float AppE_GetRectScanSpeed(APP_E_RECT_SCAN_DIR scan_dir){
    if (scan_dir == APP_E_RECT_SCAN_YAW_NEGATIVE){
        return -APP_E_RECT_SCAN_SPEED_DEG_S;
    }

    return APP_E_RECT_SCAN_SPEED_DEG_S;
}

static const char *AppE_GetRectScanTitle(APP_E_RECT_SCAN_DIR scan_dir){
    if (scan_dir == APP_E_RECT_SCAN_YAW_NEGATIVE){
        return "E3 Yaw-";
    }

    return "E3 Yaw+";
}

static bool AppE_IsRectScanAngleReached(APP_E_RECT_SCAN_DIR scan_dir){
    float yaw_deg = Gimbal_GetAngle().yaw_deg;

    if (scan_dir == APP_E_RECT_SCAN_YAW_NEGATIVE){
        return yaw_deg <= -APP_E_RECT_SCAN_ANGLE_DEG;
    }

    return yaw_deg >= APP_E_RECT_SCAN_ANGLE_DEG;
}

static bool AppE_ScanForRect(APP_E_RECT_SCAN_DIR scan_dir){
    char line0[24];
    char line1[24];
    uint32_t start_ms = BSP_Time_GetMs();
    const char *title = AppE_GetRectScanTitle(scan_dir);

    AppE_PrepareTaskInput();
    GimbalTracking_Init(NULL);
    Gimbal_ResetAxisPosition(GIMBAL_AXIS_YAW);
    (void)Gimbal_SetSpeed(AppE_GetRectScanSpeed(scan_dir), 0.0f);
    Ui_RenderLines(title,
                   "Scanning...",
                   "Yaw:0",
                   "Rect:WAIT",
                   "K2 long:stop",
                   NULL,
                   NULL);

    while (AppE_ElapsedMs(start_ms) < APP_E_RECT_SCAN_TIMEOUT_MS){
        (void)Gimbal_Update();

        if (GimbalTracking_IsRectValid()){
            (void)Gimbal_Stop();
            Ui_RenderLines(title,
                           "Rect found",
                           "Stop 200ms",
                           "Then E2 aim",
                           "K2 long:stop",
                           NULL,
                           NULL);
            BSP_DelayMs(APP_E_RECT_FOUND_SETTLE_MS);
            return true;
        }

        if (AppE_IsRectScanAngleReached(scan_dir)){
            break;
        }

        snprintf(line0, sizeof(line0), "Yaw:%0.0f", Gimbal_GetAngle().yaw_deg);
        snprintf(line1, sizeof(line1), "Time:%lu.%01lu",
                 (unsigned long)(AppE_ElapsedMs(start_ms) / 1000U),
                 (unsigned long)((AppE_ElapsedMs(start_ms) / 100U) % 10U));
        Ui_UpdateContentLine(1U, line0);
        Ui_UpdateContentLine(3U, line1);

        if (AppE_IsBackEvent()){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    (void)Gimbal_Stop();
    return false;
}

static void AppE_RunRectScanAim(APP_E_RECT_SCAN_DIR scan_dir){
    const char *title = AppE_GetRectScanTitle(scan_dir);

    (void)Chassis_Brake();

    if (!AppE_ScanForRect(scan_dir)){
        Ui_RenderStatusPage(title, UI_STATUS_WARN, "Rect not found", "K2 long:back");
        AppE_WaitBack();
        return;
    }

    /*
     * E3 只负责按指定方向扫描并确认矩形；确认后停 200ms，
     * 随后的云台闭环直接复用 E2 的激光点瞄准逻辑。
     */
    AppE_RunAimCenter(4000U, title, true);
}

void AppE_RunRectScanYawPositive(void){
    AppE_RunRectScanAim(APP_E_RECT_SCAN_YAW_POSITIVE);
}

void AppE_RunRectScanYawNegative(void){
    AppE_RunRectScanAim(APP_E_RECT_SCAN_YAW_NEGATIVE);
}

void AppE_RunAutoScanAim(void){
    (void)Chassis_Brake();
    AppE_SetLaserEnabled(false);

    /* 不要求操作者预判初始朝向：完整扫描一个方向失败后回到原方向并反向搜索。 */
    if (!AppE_ScanForRect(APP_E_RECT_SCAN_YAW_POSITIVE) &&
        !AppE_ScanForRect(APP_E_RECT_SCAN_YAW_NEGATIVE)){
        Ui_RenderStatusPage("E3 Auto", UI_STATUS_WARN, "Rect not found", "K2 long:back");
        AppE_WaitBack();
        return;
    }

    AppE_RunAimCenter(4000U, "E3 Auto", true);
}
