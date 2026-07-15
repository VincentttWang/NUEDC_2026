#include "app_e_task.h"

#include "app_e_calibration.h"
#include "middleware/auto_aim/auto_aim.h"
#include "bsp_time.h"
#include "canmv_uart.h"
#include "chassis.h"
#include "bsp/debug_uart/debug_uart.h"
#include "bsp/bldc/f32c_bldc.h"
#include "gimbal.h"
#include "gimbal_tracking.h"
#include "key.h"
#include "line_follow.h"
#include "line_tracking.h"
#include "motion.h"
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
/* UI 刷新周期(ms): OLED 每行"清屏+写字"是 I2C 慢操作, 每圈刷 3 行会把控制循环拖到 ~14Hz。
 * 圈数/时间显示无需高刷, 降到 ~400ms 一次, 把控制率还给循迹/云台 (实测循环 72ms→~15-25ms)。 */
#define APP_E_UI_REFRESH_MS 400U
/* CSV 调试遥测周期(ms): 供 aim_tune_viz.py 整定用; 每条 printf ~5ms 占循环, 比赛可加大或设 0 关。 */
#define APP_E_TELEMETRY_INTERVAL_MS 20U
/* 起步前倒计时秒数: 车刹住、云台已瞄准、激光已亮, 让用户摆好车/退开再走。 */
#define APP_E_START_COUNTDOWN_S 3U
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

static void AppE_PulseLaser(uint32_t pulse_ms){
    AppE_SetLaserEnabled(true);
    BSP_DelayMs(pulse_ms);
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
    uint32_t ui_last_ms = 0U;   /* UI 刷新降频时间戳 (0 => 首圈立即刷一次) */
    uint32_t line_lost_start_ms = start_ms;
    float corner_forward_start_distance = 0.0f;
    uint32_t corner_count = 0U;
    bool line_lost_pending = false;
    APP_E_LINE_STATE line_state = APP_E_LINE_STATE_FOLLOW;
    APP_E_CORNER_DIR corner_dir = APP_E_CORNER_NONE;
    uint8_t corner_enter_count = 0U;
    uint8_t auto_aim_imu_fail_count = 0U;
    APP_E_CALIBRATION_CONFIG calibration = AppE_GetCalibrationConfig();
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
        AutoAim_Init(&calibration.auto_aim);
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

    /*
     * 起步前倒计时: 车保持刹车、云台已在瞄准起点位姿、激光已亮 (aim 模式),
     * 便于摆好车/退开再走。长按 K2(CALIB) 可中途放弃。
     * 倒计时同时是"起步静态对齐"窗口: 车静止、位姿已知, 视觉以大量程校正器
     * 一次性吸收静态总偏差 (零位/摆车/安装); 视觉没锁上则零副作用纯前馈起跑。
     */
    if (aim_mode != APP_E_AIM_MODE_NONE){
        AutoAim_SetStartupAlign(true);
    }
    for (uint8_t cd = APP_E_START_COUNTDOWN_S; cd > 0U; cd--){
        char cd_line[24];
        uint32_t cd_end_ms = BSP_Time_GetMs() + 1000U;
        snprintf(cd_line, sizeof(cd_line), "Start in %u...", (unsigned)cd);
        Ui_UpdateContentLine(1U, cd_line);
        while ((int32_t)(cd_end_ms - BSP_Time_GetMs()) > 0){
            if (AppE_IsBackEvent()){
                (void)Motion_Stop();
                AppE_StopLineAim(aim_mode);
                Key_ClearAllEvents();
                return;
            }
            if (aim_mode != APP_E_AIM_MODE_NONE){
                (void)AutoAim_Update(0.02f);   /* 云台瞄准起点位姿 (车静止) */
            }
            (void)Chassis_Brake();
            BSP_DelayMs(APP_E_LOOP_DELAY_MS);
        }
    }
    /* 倒计时结束: 关闭对齐窗口(冻结起步校正), 重置计时基准(超时不含倒计时)。 */
    if (aim_mode != APP_E_AIM_MODE_NONE){
        AutoAim_SetStartupAlign(false);
    }
    start_ms = BSP_Time_GetMs();
    last_ms = start_ms;

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
            } else if (++auto_aim_imu_fail_count >= calibration.imu_fail_limit){
                (void)Motion_Stop();
                AppE_StopLineAim(aim_mode);
                Ui_RenderStatusPage("Line aim", UI_STATUS_WARN, "IMU lost", "K2 long:back");
                AppE_WaitBack();
                return;
            }
        }

        /*
         * 转弯滞后调试遥测: 每 ~20ms 经 Debug_Ex(UART1@115200) 输出一条 CSV:
         *   A,t_ms,line_state,gyroZ,vCenter,heading,yawCmd,yawRateFF,yawActX10,yawBias,visErrY
         *   line_state: 0=FOLLOW 1=CORNER_FWD 2=CORNER_ARC
         *   yawActX10 : F32C 上报 yaw 多圈角(0.1°); 逻辑角=-yawActX10/10 (GIMBAL_YAW_DIR=-1)
         *   yawBias   : 视觉总校正 = 起步对齐 + 慢跟踪 bias (deg)
         *   visErrY   : 最近一个有效视觉帧的 yaw 误差 (deg, 原始符号; 无帧时保持旧值)
         *   注: yawAct 因请求-响应异步约滞后一个采样(~20ms)。
         */
        if ((aim_mode != APP_E_AIM_MODE_NONE) && (APP_E_TELEMETRY_INTERVAL_MS > 0U)){
            static uint32_t s_dbg_last_ms;
            if ((uint32_t)(now_ms - s_dbg_last_ms) >= APP_E_TELEMETRY_INTERVAL_MS){
                s_dbg_last_ms = now_ms;
                AUTO_AIM_STATE aim_st = AutoAim_GetState();
                DebugUart_Printf("A,%lu,%d,%.1f,%.2f,%.1f,%.1f,%.1f,%ld,%.2f,%.2f\r\n",
                                 (unsigned long)now_ms, (int)line_state,
                                 aim_st.gyro_z_deg_s, aim_st.v_center_mps,
                                 aim_st.pose.heading_deg, aim_st.yaw_command_deg,
                                 aim_st.yaw_rate_ff_deg_s,
                                 (long)BLDC_Motor1.multi_angle,
                                 aim_st.startup_bias.yaw_bias_deg +
                                     aim_st.vision_bias.yaw_bias_deg,
                                 aim_st.vision_yaw_error_deg);
                BLDC_ReqFeedback(BLDC_ADDR_1, FB_MULTI_ANGLE);   /* 请求下次 yaw 实际角 */
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
                                             (float)ec * calibration.auto_aim.track_side_m);
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

        /* UI 刷新降频: 每圈刷 3 行 OLED(I2C 慢写)会把控制循环拖到 ~14Hz。
         * 圈数/时间无需高刷, 每 APP_E_UI_REFRESH_MS 刷一次, 释放控制率给循迹/云台。 */
        if ((uint32_t)(now_ms - ui_last_ms) >= APP_E_UI_REFRESH_MS){
            ui_last_ms = now_ms;
            snprintf(line0, sizeof(line0), "Lap:%u/%u", completed_laps, target_laps);
            snprintf(line1, sizeof(line1), "Edge:%ld/%ld", (long)edge_count, (long)target_edges);
            snprintf(line2, sizeof(line2), "Time:%lu.%01lu",
                     (unsigned long)(AppE_ElapsedMs(start_ms) / 1000U),
                     (unsigned long)((AppE_ElapsedMs(start_ms) / 100U) % 10U));
            Ui_UpdateContentLine(1U, line0);
            Ui_UpdateContentLine(2U, line1);
            Ui_UpdateContentLine(3U, line2);
        }

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
    APP_E_CALIBRATION_CONFIG calibration = AppE_GetCalibrationConfig();
    uint32_t start_ms;
    uint32_t last_ms;
    uint32_t last_display_ms;
    uint32_t last_vf = g_canmv_uart_angle_frame_count;
    uint8_t vf_div = 0U;
    uint8_t imu_fail_count = 0U;
    bool startup_align = true;

    /*
     * F1 静止等价入口: 完整复用 AppE_GetCalibrationConfig + AutoAim 控制链。
     * 前 APP_E_START_COUNTDOWN_S 秒与 F1 一样启用大范围 startup bias，随后冻结
     * 起步校正并切入逐轴小增益视觉 bias。没有视觉帧时仍由规定起点的几何前馈
     * 保持靶心，不再退回旧 GimbalTracking 像素 PID/速度控制路线。
     */
    AppE_PrepareTaskInput();
    (void)Chassis_Brake();
    /* Aim track 用于持续跟踪调试：进入后激光保持常亮，直到用户退出。 */
    AppE_SetLaserEnabled(true);
    Gimbal_EnsurePitchReady();
    AutoAim_Init(&calibration.auto_aim);
    if (AutoAim_Start(AUTO_AIM_MODE_CENTER) != BSP_STATUS_OK){
        AppE_SetLaserEnabled(false);
        Ui_RenderStatusPage("Aim track", UI_STATUS_WARN, "IMU not ready", "K2 long:back");
        AppE_WaitBack();
        return;
    }
    AutoAim_SetStartupAlign(true);
    start_ms = BSP_Time_GetMs();
    last_ms = start_ms;
    last_display_ms = start_ms;
    Ui_RenderLines("Aim track",
                   "F1 params: ALIGN",
                   "Vision:WAIT",
                   "Err:0.0,0.0",
                   "Bias:0.0,0.0",
                   "K2 long:stop",
                   NULL);
    DebugUart_Puts("[AIM] start mode=F1_PARAMS\r\n");

    while (1){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;

        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        last_ms = now_ms;
        if (startup_align &&
            ((uint32_t)(now_ms - start_ms) >= APP_E_START_COUNTDOWN_S * 1000U)){
            AutoAim_SetStartupAlign(false);
            startup_align = false;
            DebugUart_Puts("[AIM] startup-align frozen\r\n");
        }

        BSP_STATUS status = AutoAim_Update(dt_s);
        AUTO_AIM_STATE state = AutoAim_GetState();
        if (status == BSP_STATUS_OK){
            imu_fail_count = 0U;
        } else if (++imu_fail_count >= calibration.imu_fail_limit){
            DebugUart_Puts("[AIM] stop: IMU lost\r\n");
            break;
        }

        if ((uint32_t)(now_ms - last_display_ms) >= APP_E_UI_REFRESH_MS){
            last_display_ms = now_ms;
            snprintf(line0, sizeof(line0), "Vision:%s",
                     CanMvUart_GetStatus(CANMV_TARGET_ANGLE) == CANMV_STATUS_OK
                         ? "VALID" : "WAIT");
            snprintf(line1, sizeof(line1), "Err:%0.1f,%0.1f",
                     state.vision_yaw_error_deg, state.vision_pitch_error_deg);
            snprintf(line2, sizeof(line2), "Bias:%0.1f,%0.1f",
                     state.startup_bias.yaw_bias_deg + state.vision_bias.yaw_bias_deg,
                     state.startup_bias.pitch_bias_deg + state.vision_bias.pitch_bias_deg);
            Ui_UpdateContentLine(0U, startup_align ? "F1 params: ALIGN" : "F1 params: TRACK");
            Ui_UpdateContentLine(1U, line0);
            Ui_UpdateContentLine(2U, line1);
            Ui_UpdateContentLine(3U, line2);
        }

        /* 每收到 N 个 K230 角度帧记录一次与 F1 同口径的角度域状态。 */
        {
            uint32_t vf = g_canmv_uart_angle_frame_count;
            if (vf != last_vf){
                last_vf = vf;
                if (++vf_div >= APP_E_AIM_LOG_EVERY_FRAMES){
                    vf_div = 0U;
                    DebugUart_Printf(
                        "[AIM] t=%lu al=%u vv=%u ey=%0.2f ep=%0.2f cy=%0.2f cp=%0.2f by=%0.2f bp=%0.2f vf=%lu\r\n",
                        (unsigned long)now_ms,
                        startup_align ? 1U : 0U,
                        state.vision_valid ? 1U : 0U,
                        state.vision_yaw_error_deg, state.vision_pitch_error_deg,
                        state.yaw_command_deg, state.pitch_command_deg,
                        state.startup_bias.yaw_bias_deg + state.vision_bias.yaw_bias_deg,
                        state.startup_bias.pitch_bias_deg + state.vision_bias.pitch_bias_deg,
                        (unsigned long)vf);
                }
            }
        }

        if (AppE_IsBackEvent()){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    (void)AutoAim_Stop();
    AppE_SetLaserEnabled(false);
    DebugUart_Puts("[AIM] stop\r\n");
    Ui_RenderStatusPage("Aim track", UI_STATUS_OK, "Stopped", "K2 long:back");
    AppE_WaitBack();
}

static bool AppE_IsAimLocked(float yaw_error_deg,
                             float pitch_error_deg,
                             const APP_E_CALIBRATION_CONFIG *calibration){
    return (AppE_AbsFloat(yaw_error_deg) <= calibration->aim_lock_yaw_deg) &&
           (AppE_AbsFloat(pitch_error_deg) <= calibration->aim_lock_pitch_deg);
}

static void AppE_RunVisualAim2s(void){
    char line0[24];
    char line1[24];
    char line2[24];
    uint32_t start_ms = BSP_Time_GetMs();
    uint32_t last_ms = start_ms;
    uint8_t lock_frames = 0U;
    bool shot = false;
    APP_E_CALIBRATION_CONFIG calibration = AppE_GetCalibrationConfig();

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
            AppE_IsAimLocked(state.error.x, state.error.y, &calibration)){
            if (lock_frames < calibration.aim_lock_confirm_frames){
                lock_frames++;
            }
        } else{
            lock_frames = 0U;
        }

        if (!shot && ((lock_frames >= calibration.aim_lock_confirm_frames) ||
                      (AppE_ElapsedMs(start_ms) >= calibration.e2_force_shot_ms))){
            AppE_PulseLaser(calibration.laser_pulse_ms);
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
    APP_E_CALIBRATION_CONFIG calibration = AppE_GetCalibrationConfig();

    AppE_PrepareTaskInput();
    (void)Chassis_Brake();
    AppE_SetLaserEnabled(false);
    Gimbal_EnsurePitchReady();
    AutoAim_Init(&calibration.auto_aim);
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
                             state.vision_pitch_error_deg,
                             &calibration)){
            if (lock_frames < calibration.aim_lock_confirm_frames){
                lock_frames++;
            }
        } else if (!state.vision_valid){
            /* 没有新视觉帧时保留连续帧计数；明确收到偏差超限才清零。 */
        } else{
            lock_frames = 0U;
        }

        if (!shot && ((lock_frames >= calibration.aim_lock_confirm_frames) ||
                      (AppE_ElapsedMs(start_ms) >= calibration.e3_force_shot_ms))){
            AppE_PulseLaser(calibration.laser_pulse_ms);
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
