#include "app_e_task.h"

#include "app_debug_cmd.h"
#include "middleware/auto_aim/auto_aim.h"
#include "bsp_time.h"
#include "canmv_uart.h"
#include "chassis.h"
#include "bsp/debug_uart/debug_uart.h"
#include "gimbal.h"
#include "gimbal_tracking/gimbal_tracking.h"
#include "key.h"
#include "line_follow.h"
#include "line_tracking/line_tracking.h"
#include "motion/motion.h"
#include "ui.h"

#include <stdbool.h>
#include <stdio.h>

#define APP_E_MAX_LAPS 5U
#define APP_E_EDGES_PER_LAP 4U
#define APP_E_LINE_TIMEOUT_MS 20000U
#define APP_E_LINE_LOST_GRACE_MS 1000U
#define APP_E_LINE_EDGE_MIN_DISTANCE_M 0.15f
#define APP_E_CORNER_TURN_LEFT_DUTY_PERCENT -12.0f
#define APP_E_CORNER_TURN_RIGHT_DUTY_PERCENT 12.0f
#define APP_E_CORNER_FORWARD_DISTANCE_M 0.09f
#define APP_E_CORNER_FORWARD_DUTY_PERCENT 15.0f
#define APP_E_CORNER_ENTER_CONFIRM_COUNT 2U
#define APP_E_CORNER_EXIT_CONFIRM_COUNT 3U
#define APP_E_LOOP_DELAY_MS 10U
#define APP_E_AIM_LOCK_YAW_DEG 0.8f
#define APP_E_AIM_LOCK_PITCH_DEG 0.8f
#define APP_E_AIM_LOCK_CONFIRM_FRAMES 8U
#define APP_E_AIM_FORCE_SHOT_MS 1800U
#define APP_E_LASER_PULSE_MS 20U
#define APP_E_TRACK_SIDE_M 1.0f
#define APP_E_AUTO_AIM_IMU_FAIL_LIMIT 10U
/* 更新同步遥测抽取: 每 N 次传感器更新记一条 (带 SysTick 时间戳还原真实时序)。 */
#define APP_E_AIM_LOG_EVERY_FRAMES 1U   /* Aim: 每 N 个 CanMV 视觉帧 */
#define APP_E_LINE_LOG_EVERY_SCANS 10U  /* E1: 每 N 次灰度扫描 */

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
    APP_E_AIM_MODE_NONE = 0,
    APP_E_AIM_MODE_CENTER_FEEDFORWARD,
    APP_E_AIM_MODE_CIRCLE_FEEDFORWARD,
} APP_E_AIM_MODE;

/* 默认从 A 点出发并始终左转：A -> C -> D -> B -> A。 */
static const LOCALIZATION_CORNER s_app_e_corner_sequence[APP_E_EDGES_PER_LAP] = {
    LOCALIZATION_CORNER_C,
    LOCALIZATION_CORNER_D,
    LOCALIZATION_CORNER_B,
    LOCALIZATION_CORNER_A,
};

static float AppE_AbsFloat(float value){
    return (value < 0.0f) ? -value : value;
}

/* K230 UART1 接收此命令后直接控制激光 GPIO；命令只从 MSPM0 发往 K230。 */
static void AppE_SetLaserEnabled(bool enabled){
    (void)CanMvUart_SendString(enabled ? "LASER=1\n" : "LASER=0\n");
}

static void AppE_PulseLaser(void){
    AppE_SetLaserEnabled(true);
    BSP_DelayMs(APP_E_LASER_PULSE_MS);
    AppE_SetLaserEnabled(false);
}

static void AppE_StopLineAim(APP_E_AIM_MODE aim_mode){
    AppE_SetLaserEnabled(false);
    if (aim_mode != APP_E_AIM_MODE_NONE){
        (void)AutoAim_Stop();
    }
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
    uint8_t auto_aim_imu_fail_count = 0U;
    MOTION_COMMAND line_follow_command = Motion_CommandLineFollow();

    /*
     * 基本要求(1)只使用底盘巡线；发挥任务在同一主循环中并行运行云台视觉控制。
     * 圈数换算为边线数：当前场地按每圈 4 条边线估计。
     */
    Chassis_ResetDistance();
    if (aim_mode == APP_E_AIM_MODE_NONE){
        AppE_SetLaserEnabled(false);
    } else{
        Gimbal_EnsurePitchReady();
        AutoAim_Init(NULL);
        if (AutoAim_Start(aim_mode == APP_E_AIM_MODE_CIRCLE_FEEDFORWARD
                              ? AUTO_AIM_MODE_CIRCLE
                              : AUTO_AIM_MODE_CENTER) != BSP_STATUS_OK){
            (void)Motion_Stop();
            AppE_SetLaserEnabled(false);
            Ui_RenderStatusPage("Line aim", UI_STATUS_WARN, "IMU not ready", "K2 long:back");
            AppE_WaitBack();
            return;
        }
        /* 发挥任务要求运动期间连续照射：前馈就绪后先开激光，再启动底盘。 */
        AppE_SetLaserEnabled(true);
    }
    LineFollow_Reset();
    LineTracking_Init(NULL);
    Motion_Init();
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

        if (aim_mode != APP_E_AIM_MODE_NONE){
            if (AutoAim_Update(dt_s) == BSP_STATUS_OK){
                auto_aim_imu_fail_count = 0U;
            } else if (++auto_aim_imu_fail_count >= APP_E_AUTO_AIM_IMU_FAIL_LIMIT){
                (void)Motion_Stop();
                AppE_StopLineAim(aim_mode);
                Ui_RenderStatusPage("Line aim", UI_STATUS_WARN, "IMU lost", "K2 long:back");
                AppE_WaitBack();
                return;
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
                    if ((aim_mode != APP_E_AIM_MODE_NONE) && (ec > 0)){
                        uint32_t edge_index = (uint32_t)(ec - 1) % APP_E_EDGES_PER_LAP;
                        AutoAim_AnchorCorner(s_app_e_corner_sequence[edge_index],
                                             (float)ec * APP_E_TRACK_SIDE_M);
                    }
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
            AppE_StopLineAim(aim_mode);
            AppE_WaitBack();
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
            AppE_StopLineAim(aim_mode);
            Ui_RenderStatusPage("E1 Line", UI_STATUS_OK, "Finished", "K2 long:back");
            AppE_WaitBack();
            return;
        }

        if (AppE_IsBackEvent()){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    (void)Motion_Stop();
    AppE_StopLineAim(aim_mode);
    Ui_RenderStatusPage("E1 Line", UI_STATUS_WARN, "Stopped/timeout", "K2 long:back");
    AppE_WaitBack();
}

void AppE_RunLineFollow(uint8_t lap_count){
    AppE_RunLineTask(lap_count, APP_E_AIM_MODE_NONE);
}

void AppE_RunLineAim(uint8_t lap_count){
    AppE_RunLineTask(lap_count, APP_E_AIM_MODE_CENTER_FEEDFORWARD);
}

void AppE_RunLineCircle(uint8_t lap_count){
    AppE_RunLineTask(lap_count, APP_E_AIM_MODE_CIRCLE_FEEDFORWARD);
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

static bool AppE_IsAimLocked(float yaw_error_deg, float pitch_error_deg){
    return (AppE_AbsFloat(yaw_error_deg) <= APP_E_AIM_LOCK_YAW_DEG) &&
           (AppE_AbsFloat(pitch_error_deg) <= APP_E_AIM_LOCK_PITCH_DEG);
}

static void AppE_RunVisualAim2s(void){
    char line0[24];
    char line1[24];
    char line2[24];
    uint32_t start_ms = BSP_Time_GetMs();
    uint32_t last_ms = start_ms;
    uint8_t lock_frames = 0U;
    bool shot = false;

    AppE_PrepareTaskInput();
    (void)Chassis_Brake();
    AppE_SetLaserEnabled(false);
    Gimbal_EnsurePitchReady();
    GimbalTracking_Init(NULL);
    GimbalTracking_Reset();
    Ui_RenderLines("E2 Aim",
                   "Aiming center",
                   "Laser:WAIT",
                   "Err:0,0",
                   "Time:0.0",
                   "K2 long:stop",
                   NULL);

    while (AppE_ElapsedMs(start_ms) < 2000U){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;

        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        last_ms = now_ms;
        BSP_STATUS status = GimbalTracking_UpdateLaserCenter(dt_s);
        GIMBAL_TRACKING_STATE state = GimbalTracking_GetState();

        if ((status == BSP_STATUS_OK) &&
            AppE_IsAimLocked(state.error.x, state.error.y)){
            if (lock_frames < APP_E_AIM_LOCK_CONFIRM_FRAMES){
                lock_frames++;
            }
        } else{
            lock_frames = 0U;
        }

        if (!shot && ((lock_frames >= APP_E_AIM_LOCK_CONFIRM_FRAMES) ||
                      (AppE_ElapsedMs(start_ms) >= APP_E_AIM_FORCE_SHOT_MS))){
            AppE_PulseLaser();
            shot = true;
        }

        snprintf(line0, sizeof(line0), "Laser:%s", shot ? "SHOT" : "WAIT");
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

    AppE_SetLaserEnabled(false);
    (void)GimbalTracking_Stop();
    Ui_RenderStatusPage("E2 Aim", shot ? UI_STATUS_OK : UI_STATUS_WARN,
                        shot ? "Shot finished" : "Stopped", "K2 long:back");
    AppE_WaitBack();
}

void AppE_RunAimCenter2s(void){
    AppE_RunVisualAim2s();
}

void AppE_RunKnownPoseAim4s(void){
    char line0[24];
    char line1[24];
    char line2[24];
    uint32_t start_ms = BSP_Time_GetMs();
    uint32_t last_ms = start_ms;
    uint8_t lock_frames = 0U;
    bool shot = false;

    AppE_PrepareTaskInput();
    (void)Chassis_Brake();
    AppE_SetLaserEnabled(false);
    Gimbal_EnsurePitchReady();
    AutoAim_Init(NULL);
    if (AutoAim_Start(AUTO_AIM_MODE_CENTER) != BSP_STATUS_OK){
        Ui_RenderStatusPage("E3 Geo aim", UI_STATUS_WARN, "IMU not ready", "K2 long:back");
        AppE_WaitBack();
        return;
    }
    Ui_RenderLines("E3 Geo aim",
                   "Feedforward",
                   "Laser:WAIT",
                   "Err:0.0,0.0",
                   "Time:0.0",
                   "K2 long:stop",
                   NULL);

    while (AppE_ElapsedMs(start_ms) < 4000U){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;
        last_ms = now_ms;
        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        if (AutoAim_Update(dt_s) != BSP_STATUS_OK){
            (void)AutoAim_Stop();
            AppE_SetLaserEnabled(false);
            Ui_RenderStatusPage("E3 Geo aim", UI_STATUS_WARN, "IMU lost", "K2 long:back");
            AppE_WaitBack();
            return;
        }
        AUTO_AIM_STATE state = AutoAim_GetState();

        if (state.vision_valid &&
            AppE_IsAimLocked(state.vision_yaw_error_deg,
                             state.vision_pitch_error_deg)){
            if (lock_frames < APP_E_AIM_LOCK_CONFIRM_FRAMES){
                lock_frames++;
            }
        } else if (!state.vision_valid){
            /* 没有新视觉帧时保留连续帧计数；明确收到偏差超限才清零。 */
        } else{
            lock_frames = 0U;
        }

        if (!shot && ((lock_frames >= APP_E_AIM_LOCK_CONFIRM_FRAMES) ||
                      (AppE_ElapsedMs(start_ms) >= 3800U))){
            AppE_PulseLaser();
            shot = true;
        }

        snprintf(line0, sizeof(line0), "Laser:%s", shot ? "SHOT" : "WAIT");
        snprintf(line1, sizeof(line1), "Err:%0.1f,%0.1f",
                 state.vision_yaw_error_deg, state.vision_pitch_error_deg);
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

    AppE_SetLaserEnabled(false);
    (void)AutoAim_Stop();
    Ui_RenderStatusPage("E3 Geo aim", shot ? UI_STATUS_OK : UI_STATUS_WARN,
                        shot ? "Shot finished" : "Stopped", "K2 long:back");
    AppE_WaitBack();
}
