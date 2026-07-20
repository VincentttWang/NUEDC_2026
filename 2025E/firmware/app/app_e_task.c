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
#include "kinematics/kinematics.h"
#include "motion.h"
#include "ui.h"

#include <stdbool.h>
#include <stdio.h>

#define APP_E_MAX_LAPS 5U
#define APP_E_EDGES_PER_LAP 4U
#define APP_E_LINE_TIMEOUT_MS 20000U
#define APP_E_LINE_LOST_GRACE_MS 1000U
#define APP_E_LINE_EDGE_MIN_DISTANCE_M 0.15f
/* CORNER_ARC 原地转弯占空比: ±9→±11 适当提速防超时。±9 转弯慢约一倍(CORNER_ARC 帧数翻倍);
 * 现有陀螺增稳(压航向蛇形) + differential_limit 16(出弯刹车权限) + anchor 位姿渐变(消 snap
 * 阶跃) 三重保护已把出弯云台过冲压到 ~5.7°, 可承受更快转弯。±11 提速明显但不满打(原 ±12)。
 * 若出弯过冲又变大就回调 ±10; 若仍超时可试 ±12。 */
#define APP_E_CORNER_TURN_LEFT_DUTY_PERCENT -11.0f
#define APP_E_CORNER_TURN_RIGHT_DUTY_PERCENT 11.0f
#define APP_E_CORNER_FORWARD_DISTANCE_M 0.09f
#define APP_E_CORNER_FORWARD_DUTY_PERCENT 15.0f
/* F1 慢速稳定版参数: 全面降速 + 无总超时, 追求"稳"(不冲出/不脱靶)而非圈速。 */
#define APP_E_SLOW_BASE_DUTY 22.0f            /* 直线循迹基础占空比 (常规 34) */
#define APP_E_SLOW_CORNER_DUTY 8.0f           /* CORNER_ARC 原地转弯 ± (常规 11) */
#define APP_E_SLOW_FORWARD_DUTY 12.0f         /* CORNER_FORWARD 前进 (常规 15) */
#define APP_E_CORNER_ENTER_CONFIRM_COUNT 2U
#define APP_E_CORNER_EXIT_CONFIRM_COUNT 3U
#define APP_E_LOOP_DELAY_MS 10U
/* UI 刷新周期(ms): OLED 每行"清屏+写字"是 I2C 慢操作, 每圈刷 3 行会把控制循环拖到 ~14Hz。
 * 圈数/时间显示无需高刷, 降到 ~400ms 一次, 把控制率还给循迹/云台 (实测循环 72ms→~15-25ms)。 */
#define APP_E_UI_REFRESH_MS 400U
/* CSV 调试遥测周期(ms): 供 aim_tune_viz.py 整定用; 每条 printf ~5ms 占循环, 比赛可加大或设 0 关。 */
#define APP_E_TELEMETRY_INTERVAL_MS 20U
/* 瞄准任务起步前的最短几何预定位时间；此后按视觉锁定事件起步。 */
#define APP_E_AIM_GEOMETRY_SETTLE_MS 1000U
/* F1/F2/F3 进入视觉对准前, 把 pitch 在几何角基础上抬升该角(deg), 让相机先看到标靶再锁定;
 * 锁定后由视觉 servo 回靶心。+ = 抬高电机角(视觉往上看); 若方向反了(相机反而朝下)改负值。
 * 0 = 不抬升。受 pitch 软件限位约束。 */
#define APP_E_AIM_ENTRY_PITCH_UP_DEG 5.0f
/* 视觉对准阶段总时限(ms): 超时未锁定则纯前馈降级起步(无起步对齐, 慢 bias 照常,
 * 视觉恢复后自动接管)。竞赛兜底: 视觉故障时"跑起来拿部分分"优于永不起步。
 * 0 = 无限等待(调试时用, 保留"按状态起步"原语义)。 */
#define APP_E_AIM_ALIGN_TIMEOUT_MS 15000U
/* 更新同步遥测抽取: 每 N 次传感器更新记一条 (带 SysTick 时间戳还原真实时序)。 */
#define APP_E_AIM_LOG_EVERY_FRAMES 1U   /* Aim: 每 N 个 CanMV 视觉帧 */
#define APP_E_LINE_LOG_EVERY_SCANS 10U  /* E1: 每 N 次灰度扫描 */
/* E3 扫描瞄准: yaw 单向扫描角速度(deg/s)。越大越快找到靶标但越易冲过靶面,
 * 需与 K230 出帧率折中 (可试 45~90)。方向由子菜单 Scan +/- 选定。 */
#define APP_E_SCAN_YAW_SPEED_DEG_S 60.0f

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

/** Aim Track/F1 共用的视觉连续锁定状态。 */
typedef struct {
    uint8_t confirm_frames;
    bool locked;
} APP_E_AIM_TRACK_LOCK;

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

static bool AppE_IsAimLocked(float yaw_error_deg,
                             float pitch_error_deg,
                             const APP_E_CALIBRATION_CONFIG *calibration);

static void AppE_AimTrackLock_Reset(APP_E_AIM_TRACK_LOCK *lock){
    if (lock == NULL){
        return;
    }
    lock->confirm_frames = 0U;
    lock->locked = false;
}

/*
 * Aim Track/F1 共用锁定判据：只在 K230 新有效角度帧到达时累计；控制环中间的
 * “无新帧”不破坏连续性，K230 明确 LOST/NOT_FOUND 或误差越界才清零。
 */
static bool AppE_AimTrackLock_Update(APP_E_AIM_TRACK_LOCK *lock,
                                     bool new_valid_frame,
                                     CANMV_STATUS vision_status,
                                     float yaw_error_deg,
                                     float pitch_error_deg,
                                     const APP_E_CALIBRATION_CONFIG *calibration){
    if ((lock == NULL) || (calibration == NULL)){
        return false;
    }

    if (new_valid_frame){
        if (AppE_IsAimLocked(yaw_error_deg, pitch_error_deg, calibration)){
            if (lock->confirm_frames < calibration->aim_lock_confirm_frames){
                lock->confirm_frames++;
            }
        } else{
            lock->confirm_frames = 0U;
        }
    } else if (vision_status != CANMV_STATUS_OK){
        lock->confirm_frames = 0U;
    }

    lock->locked = lock->confirm_frames >= calibration->aim_lock_confirm_frames;
    return lock->locked;
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

/* 巡线任务速度/超时档位: 正常版与慢速稳定版共用同一逻辑, 仅参数不同。 */
typedef struct {
    float line_base_duty;        /* 直线循迹基础占空比 (%) */
    float corner_turn_duty;      /* CORNER_ARC 原地转弯 ± 占空比 (%) */
    float corner_forward_duty;   /* CORNER_FORWARD 前进占空比 (%) */
    float corner_forward_dist_m; /* CORNER_FORWARD 前进距离 (m) */
    bool no_timeout;             /* 无总超时 (只按圈数完成/长按退出) */
} APP_E_LINE_PROFILE;

static APP_E_LINE_PROFILE AppE_LineProfileNormal(void){
    APP_E_LINE_PROFILE p = {
        .line_base_duty = LINE_TRACKING_DEFAULT_BASE_DUTY,
        .corner_turn_duty = APP_E_CORNER_TURN_RIGHT_DUTY_PERCENT,
        .corner_forward_duty = APP_E_CORNER_FORWARD_DUTY_PERCENT,
        .corner_forward_dist_m = APP_E_CORNER_FORWARD_DISTANCE_M,
        .no_timeout = false,
    };
    return p;
}

static APP_E_LINE_PROFILE AppE_LineProfileSlow(void){
    APP_E_LINE_PROFILE p = {
        .line_base_duty = APP_E_SLOW_BASE_DUTY,
        .corner_turn_duty = APP_E_SLOW_CORNER_DUTY,
        .corner_forward_duty = APP_E_SLOW_FORWARD_DUTY,
        .corner_forward_dist_m = APP_E_CORNER_FORWARD_DISTANCE_M,
        .no_timeout = true,
    };
    return p;
}

/* 原地转弯: 逆时针左转 = 左轮反、右轮正 (turn_duty 为幅值)。 */
static BSP_STATUS AppE_ApplyCornerTurn(APP_E_CORNER_DIR corner_dir, float turn_duty){
    if (corner_dir == APP_E_CORNER_LEFT){
        return Chassis_SetDuty(-turn_duty, turn_duty);
    }

    if (corner_dir == APP_E_CORNER_RIGHT){
        return Chassis_SetDuty(turn_duty, -turn_duty);
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

static void AppE_RunLineTask(uint8_t lap_count, APP_E_AIM_MODE aim_mode,
                             const APP_E_LINE_PROFILE *profile){
    char line0[24];
    char line1[24];
    char line2[24];
    APP_E_LINE_PROFILE prof = (profile != NULL) ? *profile : AppE_LineProfileNormal();
    uint8_t target_laps = AppE_NormalizeLapCount(lap_count);
    int32_t target_edges = (int32_t)(target_laps * APP_E_EDGES_PER_LAP);
    uint32_t task_timeout_ms = prof.no_timeout ? UINT32_MAX
                                               : (APP_E_LINE_TIMEOUT_MS * target_laps);
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
    /*
     * 起步拐角 (仅发挥任务 F1/F2/F3): 规定摆位为"车身在 AB 段、车头前沿压 AC 线、车头朝 -X"
     * (见 app_e_calibration.c start_pose)。启动后检测到的【第一个】拐角是在 A 处左拐进入
     * A->C 首边——它不是一条已巡完的边, 不能计边数/锚定, 否则会把 A 误锚成序列首元素 C(位置
     * 错 1m) 且整圈少巡一条边。跳过后 C/D/B/A 正常计为 4 条边、锚定 [C,D,B,A] 不变。
     * E1(aim_mode==NONE, 摆位自定、车头朝 -Y 直接直行 A->C)不受影响。
     */
    bool initial_start_turn = (aim_mode != APP_E_AIM_MODE_NONE);
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
        AutoAim_SetVisionCorrectionEnabled(false);
        /* 发挥任务要求运动期间连续照射：前馈就绪后先开激光，再启动底盘。 */
        AppE_SetLaserEnabled(true);
    }
    LineFollow_Reset();
    LINE_TRACKING_CONFIG lt_cfg = LineTracking_GetDefaultConfig();
    lt_cfg.base_duty = prof.line_base_duty;   /* 按档位覆盖直线基础速度 */
    LineTracking_Init(&lt_cfg);
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
     * 瞄准任务按“状态”起步，不再按固定 3s 强制起跑：
     *   1) 最短 1s 纯几何预定位，禁止视觉积分，避开云台初始大角度过渡；
     *   2) 停止几何控制，复用 Aim Track 的纯视觉角度速度闭环瞄准；
     *   3) 连续锁定后，用“当前已对准角 - 起点几何角”生成 startup bias；
     *   4) 重启 AutoAim，并携带该基准无跳变切入运动前馈与视觉慢校正。
     * 没有视觉/始终未锁定时车辆持续刹车，无总超时；长按 K2 可安全退出。
     */
    if (aim_mode != APP_E_AIM_MODE_NONE){
        uint32_t settle_start_ms = BSP_Time_GetMs();
        uint32_t align_last_ms = settle_start_ms;
        uint32_t align_ui_ms = settle_start_ms;
        APP_E_AIM_TRACK_LOCK align_lock;
        uint8_t align_imu_fail_count = 0U;

        AppE_AimTrackLock_Reset(&align_lock);

        Ui_UpdateContentLine(0U, "Geometry settle");
        Ui_UpdateContentLine(1U, "Vision bias:OFF");
        while ((uint32_t)(BSP_Time_GetMs() - settle_start_ms) <
               APP_E_AIM_GEOMETRY_SETTLE_MS){
            uint32_t now_ms = BSP_Time_GetMs();
            float dt_s = (float)(now_ms - align_last_ms) / 1000.0f;
            align_last_ms = now_ms;
            if (dt_s <= 0.0f){
                dt_s = 0.001f;
            }
            if (AutoAim_Update(dt_s) == BSP_STATUS_OK){
                align_imu_fail_count = 0U;
            } else if (++align_imu_fail_count >= calibration.imu_fail_limit){
                AppE_StopLineAim(aim_mode);
                Ui_RenderStatusPage("Line aim", UI_STATUS_WARN,
                                    "IMU lost", "K2 long:back");
                AppE_WaitBack();
                return;
            }
            (void)Chassis_Brake();
            if (AppE_IsBackEvent()){
                AppE_StopLineAim(aim_mode);
                Key_ClearAllEvents();
                return;
            }
            BSP_DelayMs(APP_E_LOOP_DELAY_MS);
        }

        /* 几何预定位结束后释放 AutoAim，真正复用 Aim Track 纯视觉控制器。 */
        (void)AutoAim_Stop();
        GimbalTracking_Init(NULL);
        GimbalTracking_Reset();
        /* 进入视觉对准前抬升 pitch: 在几何角基础上加 look-up 偏置, 让相机先看到标靶;
         * GimbalTracking 无目标时保持该抬升角, 锁定后经视觉 servo 回靶心。 */
        if (APP_E_AIM_ENTRY_PITCH_UP_DEG != 0.0f){
            GIMBAL_ANGLE entry_ang = Gimbal_GetAngle();
            (void)Gimbal_SetPitchDeg(entry_ang.pitch_deg +
                                     APP_E_AIM_ENTRY_PITCH_UP_DEG);
        }
        align_last_ms = BSP_Time_GetMs();
        align_ui_ms = align_last_ms;
        uint32_t align_last_vf = g_canmv_uart_angle_frame_count;
        uint32_t align_begin_ms = BSP_Time_GetMs();
        DebugUart_Puts("[E1] AimTrack visual-align begin\r\n");
        Ui_UpdateContentLine(0U, "Vision aligning");

        while (!align_lock.locked){
            if ((APP_E_AIM_ALIGN_TIMEOUT_MS != 0U) &&
                ((uint32_t)(BSP_Time_GetMs() - align_begin_ms) >=
                 APP_E_AIM_ALIGN_TIMEOUT_MS)){
                break;   /* 超时: 走纯前馈降级起步 */
            }
            uint32_t now_ms = BSP_Time_GetMs();
            float dt_s = (float)(now_ms - align_last_ms) / 1000.0f;
            align_last_ms = now_ms;
            if (dt_s <= 0.0f){
                dt_s = 0.001f;
            }
            (void)GimbalTracking_UpdateAngle(dt_s);
            GIMBAL_TRACKING_STATE align_state = GimbalTracking_GetState();
            uint32_t current_vf = g_canmv_uart_angle_frame_count;
            bool new_angle_frame = current_vf != align_last_vf;
            if (new_angle_frame){
                align_last_vf = current_vf;
            }
            bool vision_ready = align_state.target_valid && !align_state.link_timeout;
            (void)AppE_AimTrackLock_Update(
                &align_lock,
                new_angle_frame && vision_ready,
                CanMvUart_GetStatus(CANMV_TARGET_ANGLE),
                align_state.error.x,
                align_state.error.y,
                &calibration);

            if ((uint32_t)(now_ms - align_ui_ms) >= APP_E_UI_REFRESH_MS){
                char align_line[24];
                align_ui_ms = now_ms;
                snprintf(align_line, sizeof(align_line), "Err:%0.1f,%0.1f",
                         align_state.error.x, align_state.error.y);
                Ui_UpdateContentLine(1U, align_line);
                snprintf(align_line, sizeof(align_line), "Lock:%u/%u",
                         (unsigned)align_lock.confirm_frames,
                         (unsigned)calibration.aim_lock_confirm_frames);
                Ui_UpdateContentLine(2U, align_line);
                DebugUart_Printf("[E1] align ey=%.2f ep=%.2f lock=%u/%u\r\n",
                                 align_state.error.x, align_state.error.y,
                                 (unsigned)align_lock.confirm_frames,
                                 (unsigned)calibration.aim_lock_confirm_frames);
            }

            (void)Chassis_Brake();
            if (AppE_IsBackEvent()){
                (void)GimbalTracking_Stop();
                AppE_SetLaserEnabled(false);
                Key_ClearAllEvents();
                return;
            }
            BSP_DelayMs(APP_E_LOOP_DELAY_MS);
        }

        /*
         * 纯视觉已将光轴对准靶心。yaw 对准角必须读【编码器反馈的逻辑角】:
         * 开环估计 Gimbal_GetAngle().yaw_deg 在视觉伺服(SetSpeed)期间按下发值
         * 积分, 符号约定与 aim_solver 逻辑系差一个 GIMBAL_YAW_DIR, 跨路径相减
         * 会得到反号 bias(起步误差翻倍而非清零)。阻塞刷新反馈同时也让交接后
         * velff 首拍的就近对齐/位置外环吃到新鲜角, 消除起步甩动瞬态。
         * pitch 无此问题: pitch_deg 即真实下发的位置设定点(电机角)。
         */
        float startup_yaw_bias = 0.0f;
        float startup_pitch_bias = 0.0f;
        if (align_lock.locked){
            float aligned_yaw_deg = Gimbal_ReadYawFeedbackDeg();
            GIMBAL_ANGLE aligned_angle = Gimbal_GetAngle();
            AIM_SOLVER_RESULT start_ff =
                (aim_mode == APP_E_AIM_MODE_CIRCLE_FEEDFORWARD)
                    ? AimSolver_SolveCircle(calibration.auto_aim.start_pose,
                                            calibration.auto_aim.circle_phase0_deg)
                    : AimSolver_SolveCenter(calibration.auto_aim.start_pose);
            startup_yaw_bias = Kinematics_AngleDiffDeg(aligned_yaw_deg,
                                                       start_ff.yaw_cmd_deg);
            startup_pitch_bias = aligned_angle.pitch_deg - start_ff.pitch_cmd_deg;
            DebugUart_Printf(
                "[E1] vision LOCK aligned=%.2f,%.2f ff=%.2f,%.2f bias=%.2f,%.2f\r\n",
                aligned_yaw_deg, aligned_angle.pitch_deg,
                start_ff.yaw_cmd_deg, start_ff.pitch_cmd_deg,
                startup_yaw_bias, startup_pitch_bias);
        }

        (void)GimbalTracking_Stop();
        AutoAim_Init(&calibration.auto_aim);
        if (AutoAim_Start(aim_mode == APP_E_AIM_MODE_CIRCLE_FEEDFORWARD
                              ? AUTO_AIM_MODE_CIRCLE
                              : AUTO_AIM_MODE_CENTER) != BSP_STATUS_OK){
            AppE_StopLineAim(aim_mode);
            Ui_RenderStatusPage("Line aim", UI_STATUS_WARN,
                                "IMU not ready", "K2 long:back");
            AppE_WaitBack();
            return;
        }
        if (align_lock.locked){
            AutoAim_SetStartupAlign(true);
            AutoAim_SetStartupBias(startup_yaw_bias, startup_pitch_bias);
            AutoAim_SetStartupAlign(false);
        } else{
            /* 视觉未锁定超时: 纯前馈降级起步 (无起步对齐基准);
             * 视觉恢复后慢 bias 自动接管修残差, 不阻塞任务。 */
            DebugUart_Puts("[E1] align TIMEOUT -> pure feedforward start\r\n");
        }
        AutoAim_SetVisionCorrectionEnabled(true);
        Ui_UpdateContentLine(0U, align_lock.locked ? "Running..." : "Run (no lock)");
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
         *   A,t_ms,lineState,gz,vCenter,heading,yawCmd,yawRateFF,yawActX10,
         *     yawBias,visErrY,startupBias,steadyBias,visValid,visStatus,
         *     visionFrame,bldcAngleFrame,yawOmegaCmd,yawRpmCmd,pitchBias,visErrP,
         *     poseX,poseY,bodyPitch,bodyRoll,
         *     pitchCmd,pitchActX10,pitchRateFF,dutyL,dutyR
         *   line_state: 0=FOLLOW 1=CORNER_FWD 2=CORNER_ARC
         *   yawActX10 : F32C 上报 yaw 多圈角(0.1°); 逻辑角=-yawActX10/10 (GIMBAL_YAW_DIR=-1)
         *   yawBias   : 视觉总校正 = startupBias + steadyBias (deg)
         *   visErrY   : 最近一个有效视觉帧 yaw 误差；无有效帧时保持旧值
         *   visValid  : 本控制周期是否刚消费一帧有效视觉角度 (脉冲量)
         *   visStatus : CanMV 当前状态 (0=OK, 其余见 CANMV_STATUS)
         *   visionFrame/bldcAngleFrame: 两条接收链路的累计帧数；不增长即链路冻结
         *   yawOmegaCmd/yawRpmCmd: MCU 外环最终角速度与实际下发 F32C 的 RPM
         *   pitchBias : pitch 视觉总校正 = 起步对齐 + 慢跟踪 (deg, 电机角)
         *   visErrP   : 最近有效视觉帧 pitch 误差 (deg, 光束角; 验 pitch 符号用)
         *   poseX/Y   : 定位位姿 (m, 世界系; 供离线回归确定性残差的几何来源)
         *   bodyPitch/Roll: 车体姿态 (deg, IMU 融合; 诊断加减速俯仰/弯中侧倾
         *                   对光束竖直方向的耦合 —— 云台 yaw≈90° 时 roll≈光束pitch)
         *   pitchCmd    : pitch 位置指令 (deg, 电机角; 供 pitch 环数字孪生/辨识)
         *   pitchActX10 : F32C 上报 pitch 多圈角(0.1°); 电机实际角 = pitchActX10/10
         *   pitchRateFF : pitch 解析速率前馈 (deg/s)
         *   dutyL/dutyR : 底盘左右轮当前占空比(%); 转弯段用于建模航向过冲(治本用)
         *   注: yawAct/pitchAct 因请求-响应异步约滞后一个采样(~20ms)。
         */
        if ((aim_mode != APP_E_AIM_MODE_NONE) && (APP_E_TELEMETRY_INTERVAL_MS > 0U)){
            static uint32_t s_dbg_last_ms;
            if ((uint32_t)(now_ms - s_dbg_last_ms) >= APP_E_TELEMETRY_INTERVAL_MS){
                s_dbg_last_ms = now_ms;
                AUTO_AIM_STATE aim_st = AutoAim_GetState();
                GIMBAL_SPEED gimbal_speed = Gimbal_GetSpeed();
                CHASSIS_DUTY tlm_duty = Chassis_GetDuty();
                DebugUart_Printf(
                    "A,%lu,%d,%.1f,%.2f,%.1f,%.1f,%.1f,%ld,%.2f,%.2f,"
                    "%.2f,%.2f,%u,%d,%lu,%lu,%.1f,%d,%.2f,%.2f,%.3f,%.3f,"
                    "%.2f,%.2f,%.2f,%ld,%.1f,%.0f,%.0f\r\n",
                                 (unsigned long)now_ms, (int)line_state,
                                 aim_st.gyro_z_deg_s, aim_st.v_center_mps,
                                 aim_st.pose.heading_deg, aim_st.yaw_command_deg,
                                 aim_st.yaw_rate_ff_deg_s,
                                 (long)BLDC_Motor1.multi_angle,
                                 aim_st.startup_bias.yaw_bias_deg +
                                     aim_st.vision_bias.yaw_bias_deg,
                                 aim_st.vision_yaw_error_deg,
                                 aim_st.startup_bias.yaw_bias_deg,
                                 aim_st.vision_bias.yaw_bias_deg,
                                 aim_st.vision_valid ? 1U : 0U,
                                 (int)CanMvUart_GetStatus(CANMV_TARGET_ANGLE),
                                 (unsigned long)g_canmv_uart_angle_frame_count,
                                 (unsigned long)BLDC_Motor1.multi_angle_frame_count,
                                 gimbal_speed.yaw_deg_s,
                                 (int)Gimbal_GetYawCommandRpm(),
                                 aim_st.startup_bias.pitch_bias_deg +
                                     aim_st.vision_bias.pitch_bias_deg,
                                 aim_st.vision_pitch_error_deg,
                                 aim_st.pose.x_m, aim_st.pose.y_m,
                                 aim_st.body_pitch_deg, aim_st.body_roll_deg,
                                 aim_st.pitch_command_deg,
                                 (long)BLDC_Motor2.multi_angle,
                                 aim_st.pitch_rate_ff_deg_s,
                                 tlm_duty.left_percent, tlm_duty.right_percent);
                BLDC_ReqFeedback(BLDC_ADDR_1, FB_MULTI_ANGLE);   /* 请求下次 yaw 实际角 */
                BLDC_ReqFeedback(BLDC_ADDR_2, FB_MULTI_ANGLE);   /* 请求下次 pitch 实际角 */
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
            } else{
                if (corner_enter_count < APP_E_CORNER_ENTER_CONFIRM_COUNT){
                    corner_enter_count++;
                }
                /*
                 * 丢线即降到入弯前进速度: 直线提速(base_duty 34)后, 90° 弯是"线突然全丢",
                 * 若在"全丢→2帧确认"窗口继续全速, 动量会把车顶过拐点直接冲出轨道。丢线瞬间
                 * 降速(并让车减速)给检测留余量、限制冲过拐点的距离; 与随后 CORNER_FORWARD 同速无缝。
                 */
                status = Chassis_SetDuty(prof.corner_forward_duty,
                                         prof.corner_forward_duty);
            }

            if (corner_enter_count >= APP_E_CORNER_ENTER_CONFIRM_COUNT){
                corner_dir = APP_E_CORNER_LEFT;
                line_state = APP_E_LINE_STATE_CORNER_FORWARD;
                corner_forward_start_distance = Chassis_GetDistance();
                corner_enter_count = 0U;
                corner_count++;
                status = Chassis_SetDuty(prof.corner_forward_duty, prof.corner_forward_duty);
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
            if ((distance_diff >= prof.corner_forward_dist_m) || (distance_diff <= -prof.corner_forward_dist_m)){
                line_state = APP_E_LINE_STATE_CORNER_ARC;
                status = AppE_ApplyCornerTurn(corner_dir, prof.corner_turn_duty);
            }
        } else{
            status = AppE_ApplyCornerTurn(corner_dir, prof.corner_turn_duty);
            (void)LineFollow_UpdateSensor();
            (void)LineFollow_GetSensor(&sensor);

            /* 只要 3/4 号传感器检测到黑线就结束转弯。 */
            if (AppE_IsLineInnerActive(&sensor)){
                (void)Chassis_Brake();
                line_state = APP_E_LINE_STATE_FOLLOW;
                corner_dir = APP_E_CORNER_NONE;
                line_lost_pending = false;
                LineTracking_Reset();
                if (initial_start_turn){
                    /* 起步在 A 处的左拐: 非巡完一条边, 不计边/不锚定 (见 initial_start_turn
                     * 声明处注释)。清标志后 A->C 起为首条正常边。 */
                    initial_start_turn = false;
                    DebugUart_Printf("[E1] start-turn @A done (no edge) t=%lu\r\n",
                                     (unsigned long)AppE_ElapsedMs(start_ms));
                } else{
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
    AppE_RunLineTask(lap_count, APP_E_AIM_MODE_NONE, NULL);
}

void AppE_RunLineAim(uint8_t lap_count){
    AppE_RunLineTask(lap_count, APP_E_AIM_MODE_CENTER_FEEDFORWARD, NULL);
}

void AppE_RunLineAimSlow(uint8_t lap_count){
    /* F1 慢速稳定版: 与 F1 同逻辑, 全面降速 + 无总超时, 追求"稳"。 */
    APP_E_LINE_PROFILE slow = AppE_LineProfileSlow();
    AppE_RunLineTask(lap_count, APP_E_AIM_MODE_CENTER_FEEDFORWARD, &slow);
}

void AppE_RunLineCircle(uint8_t lap_count){
    AppE_RunLineTask(lap_count, APP_E_AIM_MODE_CIRCLE_FEEDFORWARD, NULL);
}

void AppE_RunContinuousAim(void){
    char line0[24];
    char line1[24];
    char line2[24];
    uint32_t last_ms = BSP_Time_GetMs();
    uint32_t last_display_ms = last_ms;
    uint32_t last_vf = g_canmv_uart_angle_frame_count;
    uint32_t lock_last_vf = last_vf;
    uint8_t vf_div = 0U;
    APP_E_CALIBRATION_CONFIG calibration = AppE_GetCalibrationConfig();
    APP_E_AIM_TRACK_LOCK vision_lock;

    /*
     * 纯视觉角度闭环入口：只消费 K230 的 yaw/pitch 角度误差。
     * 不初始化 AutoAim，因此不使用世界坐标、规定起点、IMU、编码器、靶心几何
     * 或运动速率前馈。GimbalTracking 将角度误差经两轴独立 PID 转成速度指令；
     * K230 丢失目标时停止 yaw 并保持 pitch，重获目标后自动恢复。
     */
    AppE_PrepareTaskInput();
    (void)Chassis_Brake();
    AppE_SetLaserEnabled(true);
    Gimbal_EnsurePitchReady();
    GimbalTracking_Init(NULL);
    GimbalTracking_Reset();
    AppE_AimTrackLock_Reset(&vision_lock);
    Ui_RenderLines("Aim track",
                   "K230 angle only",
                   "Vision:WAIT",
                   "Err:0.0,0.0",
                   "Speed:0.0,0.0",
                   "K2 long:stop",
                   NULL);
    DebugUart_Puts("[AIM] start mode=K230_ANGLE_ONLY\r\n");

    while (1){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;

        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        last_ms = now_ms;
        (void)GimbalTracking_UpdateAngle(dt_s);
        GIMBAL_TRACKING_STATE state = GimbalTracking_GetState();
        bool vision_ready = state.target_valid && !state.link_timeout;
        uint32_t current_vf = g_canmv_uart_angle_frame_count;
        bool new_angle_frame = current_vf != lock_last_vf;
        if (new_angle_frame){
            lock_last_vf = current_vf;
        }
        (void)AppE_AimTrackLock_Update(&vision_lock,
                                      new_angle_frame && vision_ready,
                                      CanMvUart_GetStatus(CANMV_TARGET_ANGLE),
                                      state.error.x, state.error.y,
                                      &calibration);

        if ((uint32_t)(now_ms - last_display_ms) >= APP_E_UI_REFRESH_MS){
            GIMBAL_ANGLE ui_ang = Gimbal_GetAngle();
            last_display_ms = now_ms;
            snprintf(line0, sizeof(line0), "Vision:%s",
                     vision_lock.locked ? "LOCK" : (vision_ready ? "TRACK" : "WAIT"));
            snprintf(line1, sizeof(line1), "Err:%0.1f,%0.1f",
                     state.error.x, state.error.y);
            /* Pit = pitch 位置设定点(电机角): 静态两距离标定 k 的读数来源
             * (锁定后设定点≈实际角, 记 0.5m/1.5m 两处读数求 k)。 */
            snprintf(line2, sizeof(line2), "Pit:%0.2f Y:%0.0f",
                     ui_ang.pitch_deg, state.yaw_speed);
            Ui_UpdateContentLine(1U, line0);
            Ui_UpdateContentLine(2U, line1);
            Ui_UpdateContentLine(3U, line2);
        }

        /* 每收到 N 个 K230 角度帧记录角度误差、PID 速度输出和 yaw 实际角。 */
        {
            uint32_t vf = g_canmv_uart_angle_frame_count;
            if (vf != last_vf){
                last_vf = vf;
                if (++vf_div >= APP_E_AIM_LOG_EVERY_FRAMES){
                    GIMBAL_ANGLE log_ang = Gimbal_GetAngle();
                    vf_div = 0U;
                    DebugUart_Printf(
                        "[AIM] t=%lu vv=%u lk=%u ey=%0.2f ep=%0.2f vy=%0.2f vp=%0.2f "
                        "ya=%ld pit=%0.2f vf=%lu\r\n",
                        (unsigned long)now_ms,
                        vision_ready ? 1U : 0U,
                        (unsigned)vision_lock.confirm_frames,
                        state.error.x, state.error.y,
                        state.yaw_speed, state.pitch_speed,
                        (long)BLDC_Motor1.multi_angle,
                        log_ang.pitch_deg,
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

    (void)GimbalTracking_Stop();
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
    uint32_t last_ui_ms = start_ms;
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

        /* UI 降频到 400ms: OLED 每轮刷 3 行 I2C 慢写会把控制环拖到 ~14Hz, 拖慢瞄准响应;
         * 与 Aim Track 一致节流, 把控制率还给视觉闭环。激光发射由 lock_frames 决定, 与刷屏无关。 */
        if ((uint32_t)(now_ms - last_ui_ms) >= APP_E_UI_REFRESH_MS){
            last_ui_ms = now_ms;
            snprintf(line0, sizeof(line0), "Laser:%s", shot ? "SHOT" : "WAIT");
            snprintf(line1, sizeof(line1), "Err:%0.0f,%0.0f", state.error.x, state.error.y);
            snprintf(line2, sizeof(line2), "Time:%lu.%01lu",
                     (unsigned long)(AppE_ElapsedMs(start_ms) / 1000U),
                     (unsigned long)((AppE_ElapsedMs(start_ms) / 100U) % 10U));
            Ui_UpdateContentLine(1U, line0);
            Ui_UpdateContentLine(2U, line1);
            Ui_UpdateContentLine(3U, line2);
        }

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

void AppE_RunScanAim(APP_E_SCAN_DIR scan_dir){
    char line0[24];
    char line1[24];
    uint32_t last_ms = BSP_Time_GetMs();
    uint32_t last_ui_ms = last_ms;
    bool shot = false;
    APP_E_CALIBRATION_CONFIG calibration = AppE_GetCalibrationConfig();
    APP_E_AIM_TRACK_LOCK vision_lock;
    /* 方向由子菜单选定: 负向取反, 正向不变。yaw 无角度限位, 可连续多圈搜索。 */
    float scan_speed = (scan_dir == APP_E_SCAN_DIR_NEGATIVE)
                           ? -APP_E_SCAN_YAW_SPEED_DEG_S
                           : APP_E_SCAN_YAW_SPEED_DEG_S;
    const char *dir_text = (scan_dir == APP_E_SCAN_DIR_NEGATIVE) ? "Scan:-" : "Scan:+";

    /*
     * E3 扫描瞄准: 靶标初始位置未知。
     *   阶段1: 按选定方向单向匀速扫描 yaw, 直到 K230 角度协议报告发现靶标;
     *   阶段2: 停止扫描, 切入两轴角度速度闭环追踪; 连续锁定确认后发射一次,
     *          之后激光常亮并继续追踪, 长按 K2 退出。
     * 不设总超时(扫到为止); pitch 扫描期间传 0 保持开机抬升的工作仰角。
     */
    AppE_PrepareTaskInput();
    (void)Chassis_Brake();
    AppE_SetLaserEnabled(false);
    Gimbal_EnsurePitchReady();
    GimbalTracking_Init(NULL);
    GimbalTracking_Reset();
    AppE_AimTrackLock_Reset(&vision_lock);
    Ui_RenderLines("E3 Scan aim",
                   dir_text,
                   "Target:SEARCH",
                   "Err:0.0,0.0",
                   "K2 long:stop",
                   NULL,
                   NULL);
    DebugUart_Printf("[E3] scan start dir=%s\r\n", dir_text);

    /* 阶段1: 单向连续扫描, 直到有【新角度帧且 status OK】才判为发现靶标(避免陈旧 OK 误判)。 */
    uint32_t scan_last_vf = g_canmv_uart_angle_frame_count;
    bool found = false;
    while (!found){
        (void)Gimbal_SetSpeed(scan_speed, 0.0f);
        uint32_t vf = g_canmv_uart_angle_frame_count;
        if (vf != scan_last_vf){
            scan_last_vf = vf;
            if (CanMvUart_GetStatus(CANMV_TARGET_ANGLE) == CANMV_STATUS_OK){
                found = true;
            }
        }
        if (AppE_IsBackEvent()){
            (void)GimbalTracking_Stop();
            AppE_SetLaserEnabled(false);
            Key_ClearAllEvents();
            return;
        }
        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    /* 发现靶标: 停扫描并复位追踪 PID, 无跳变切入角度闭环。 */
    (void)Gimbal_Stop();
    GimbalTracking_Reset();
    last_ms = BSP_Time_GetMs();
    last_ui_ms = last_ms;
    uint32_t lock_last_vf = g_canmv_uart_angle_frame_count;
    DebugUart_Puts("[E3] target FOUND -> track\r\n");
    Ui_UpdateContentLine(1U, "Target:TRACK");

    /* 阶段2: 角度闭环追踪 + 锁定发射(仅一次) + 之后常亮继续追踪。锁定判据复用 Aim Track:
     * 只在新有效角度帧到达时累计确认帧, 明确 LOST/越界才清零, 抗帧间抖动。 */
    while (1){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;
        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }
        last_ms = now_ms;

        (void)GimbalTracking_UpdateAngle(dt_s);
        GIMBAL_TRACKING_STATE state = GimbalTracking_GetState();
        bool vision_ready = state.target_valid && !state.link_timeout;
        uint32_t current_vf = g_canmv_uart_angle_frame_count;
        bool new_angle_frame = current_vf != lock_last_vf;
        if (new_angle_frame){
            lock_last_vf = current_vf;
        }
        bool locked = AppE_AimTrackLock_Update(&vision_lock,
                                               new_angle_frame && vision_ready,
                                               CanMvUart_GetStatus(CANMV_TARGET_ANGLE),
                                               state.error.x, state.error.y,
                                               &calibration);

        if (!shot && locked){
            AppE_PulseLaser(calibration.laser_pulse_ms);   /* 发射一次 */
            AppE_SetLaserEnabled(true);                     /* 之后常亮继续追踪 */
            shot = true;
            DebugUart_Puts("[E3] SHOT -> keep tracking\r\n");
        }

        /* UI 降频到 400ms: 避免每轮 OLED I2C 慢写拖低控制率、拖慢瞄准响应。 */
        if ((uint32_t)(now_ms - last_ui_ms) >= APP_E_UI_REFRESH_MS){
            last_ui_ms = now_ms;
            snprintf(line0, sizeof(line0), "Target:%s",
                     shot ? "SHOT" : (vision_ready ? "TRACK" : "LOST"));
            snprintf(line1, sizeof(line1), "Err:%0.1f,%0.1f", state.error.x, state.error.y);
            Ui_UpdateContentLine(1U, line0);
            Ui_UpdateContentLine(2U, line1);
        }

        if (AppE_IsBackEvent()){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    AppE_SetLaserEnabled(false);
    (void)GimbalTracking_Stop();
    Ui_RenderStatusPage("E3 Scan aim", shot ? UI_STATUS_OK : UI_STATUS_WARN,
                        shot ? "Shot done" : "Stopped", "K2 long:back");
    AppE_WaitBack();
}
