#include "app_e_task.h"

#include "bsp_time.h"
#include "chassis.h"
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

static void AppE_WaitBack(void){
    Key_ClearAllEvents();

    while (!Key_IsLongPress(KEY_ID_1)){
        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    Key_ClearAllEvents();
}

static void AppE_PrepareTaskInput(void){
    Key_ClearAllEvents();

    while (Key_IsPressed(KEY_ID_1)){
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

void AppE_RunLineFollow(uint8_t lap_count){
    char line0[24];
    char line1[24];
    char line2[24];
    uint8_t target_laps = AppE_NormalizeLapCount(lap_count);
    int32_t target_edges = (int32_t)(target_laps * APP_E_EDGES_PER_LAP);
    uint32_t task_timeout_ms = APP_E_LINE_TIMEOUT_MS * target_laps;
    uint32_t start_ms = BSP_Time_GetMs();
    uint32_t last_ms = start_ms;
    uint32_t line_lost_start_ms = start_ms;
    float corner_forward_start_distance = 0.0f;
    uint32_t corner_count = 0U;
    bool line_lost_pending = false;
    APP_E_LINE_STATE line_state = APP_E_LINE_STATE_FOLLOW;
    APP_E_CORNER_DIR corner_dir = APP_E_CORNER_NONE;
    uint8_t corner_enter_count = 0U;
    MOTION_COMMAND line_follow_command = Motion_CommandLineFollow();

    /*
     * E1 只使用底盘巡线，进入任务前停止可能残留的云台视觉跟踪。
     * 圈数换算为边线数：当前场地按每圈 4 条边线估计。
     */
    (void)GimbalTracking_Stop();
    LineFollow_Reset();
    LineTracking_Init(NULL);
    Motion_Init();
    Chassis_ResetDistance();
    Ui_RenderLines("E1 Line",
                   "Running...",
                   "Lap:0",
                   "Edge:0",
                   "Time:0.0",
                   "Long:stop",
                   NULL);

    while (AppE_ElapsedMs(start_ms) < task_timeout_ms){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;

        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        last_ms = now_ms;

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
            snprintf(turn_line, sizeof(turn_line), "Turn:%lu", (unsigned long)corner_count);
            Ui_RenderLines("E1 Line",
                           "[WARN]",
                           "Line lost",
                           turn_line,
                           "Long:back",
                           NULL,
                           NULL);
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

        if (edge_count >= target_edges){
            (void)Motion_Stop();
            Ui_RenderStatusPage("E1 Line", UI_STATUS_OK, "Finished", "Long:back");
            AppE_WaitBack();
            return;
        }

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    (void)Motion_Stop();
    Ui_RenderStatusPage("E1 Line", UI_STATUS_WARN, "Stopped/timeout", "Long:back");
    AppE_WaitBack();
}

void AppE_RunYawSpeedTest(float yaw_deg_s){
    char line0[24];
    char line1[24];

    /*
     * 此测试用于使用给定的速度参数旋转二维云台 Yaw 轴，
     * 以帮助找到电机的最小驱动边界值。
     */
    (void)Chassis_Brake();
    GimbalTracking_Stop();
    Gimbal_Init();
    Gimbal_ResetAxisPosition(GIMBAL_AXIS_YAW);
    
    /* 先下发驱动速度给电机底层（立刻生效开始发PWM），
     * 避免因为 OLED 刷屏较慢带来的肉眼可见的延迟感。
     */
    (void)Gimbal_SetSpeed(yaw_deg_s, 0.0f);

    Ui_RenderLines("Yaw spd test",
                   "Starting...",
                   NULL,
                   NULL,
                   "Long:stop",
                   NULL,
                   NULL);

    while (1){
        (void)Gimbal_Update();

        snprintf(line0, sizeof(line0), "Spd:%0.1f", yaw_deg_s);
        snprintf(line1, sizeof(line1), "Yaw:%0.1f", Gimbal_GetAngle().yaw_deg);
        Ui_UpdateContentLine(0U, line0);
        Ui_UpdateContentLine(1U, line1);

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    (void)Gimbal_Stop();
    Ui_RenderStatusPage("Yaw spd test", UI_STATUS_OK, "Wait return", "Long:back");
    AppE_WaitBack();
}

static void AppE_RunAimCenter(uint32_t timeout_ms, const char *title){
    char line0[24];
    char line1[24];
    char line2[24];
    uint32_t start_ms = BSP_Time_GetMs();
    uint32_t last_ms = start_ms;

    AppE_PrepareTaskInput();
    (void)Chassis_Brake();
    GimbalTracking_Init(NULL);
    Ui_RenderLines(title,
                   "Aiming center",
                   "Laser:WAIT",
                   "Err:0,0",
                   "Time:0.0",
                   "Long:stop",
                   NULL);

    while ((timeout_ms == 0U) || (AppE_ElapsedMs(start_ms) < timeout_ms)){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;

        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        last_ms = now_ms;
        BSP_STATUS status = GimbalTracking_UpdateLaserCenter(dt_s);
        GIMBAL_TRACKING_STATE state = GimbalTracking_GetState();

        if (state.link_timeout){
            break;
        }

        snprintf(line0, sizeof(line0), "Laser:%s", status == BSP_STATUS_OK ? "OK" : "WAIT");
        snprintf(line1, sizeof(line1), "Err:%0.0f,%0.0f", state.error.x, state.error.y);
        snprintf(line2, sizeof(line2), "Time:%lu.%01lu",
                 (unsigned long)(AppE_ElapsedMs(start_ms) / 1000U),
                 (unsigned long)((AppE_ElapsedMs(start_ms) / 100U) % 10U));
        Ui_UpdateContentLine(1U, line0);
        Ui_UpdateContentLine(2U, line1);
        Ui_UpdateContentLine(3U, line2);

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    (void)GimbalTracking_Stop();
    Ui_RenderStatusPage(title, UI_STATUS_OK, "Aim finished", "Long:back");
    AppE_WaitBack();
}

void AppE_RunAimCenter2s(void){
    AppE_RunAimCenter(0U, "E2 Aim");
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
                   "Long:stop",
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
                           "Long:stop",
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

        if (Key_IsLongPress(KEY_ID_1)){
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
        Ui_RenderStatusPage(title, UI_STATUS_WARN, "Rect not found", "Long:back");
        AppE_WaitBack();
        return;
    }

    /*
     * E3 只负责按指定方向扫描并确认矩形；确认后停 200ms，
     * 随后的云台闭环直接复用 E2 的激光点瞄准逻辑。
     */
    AppE_RunAimCenter(0U, title);
}

void AppE_RunRectScanYawPositive(void){
    AppE_RunRectScanAim(APP_E_RECT_SCAN_YAW_POSITIVE);
}

void AppE_RunRectScanYawNegative(void){
    AppE_RunRectScanAim(APP_E_RECT_SCAN_YAW_NEGATIVE);
}
