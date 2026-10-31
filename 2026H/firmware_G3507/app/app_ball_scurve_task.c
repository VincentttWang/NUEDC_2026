/**
 * @file  app_ball_scurve_task.c
 * @brief 纯 S 曲线滚球点到点任务：O → +5cm → −5cm（要求 3 序列）。
 *
 * ================== 关于"物理换算不准" ==================
 *
 * 本任务**不假设**下面任何一个换算是准的：
 *
 *   1. 滚球增益 K_G = (5/7)g —— 管身扭转、球非实心、接触点偏移都会让它偏小；
 *   2. 曲柄滑槽查表 cnt→deg —— 尺寸测量误差直接进入这张表；
 *   3. **水平位计数** —— 上电参考位是"断电自重靠住那一端"，其重复性、机构装配
 *      公差和水管自身弯曲都会让真实动力学水平点偏离查表的 0°（当前表为 180 cnt）。
 *
 * 第 3 条最要命：它是一个**常值倾角偏置**，在 T 秒的移动里造成
 * 0.5·K_G·Δθ·T² 的落点偏差——Δθ = 0.45°（15 cnt）、T = 1.2 s 时就是 40 mm。
 *
 * 应对方式是**不去猜，而是让它可辨识**：
 *
 *   - 所有换算都集中在下面的 H3S_* 宏和 SCURVE_CONFIG 里，一处可改；
 *   - `H3S_LEVEL_BIAS_DEG` 默认 0，即"承认没标定"，其真值会以单向漂移暴露出来；
 *   - 遥测同时输出**实际水管角 beam**（编码器经查表）和**球加速度估计 aest**，
 *     二者做一次线性回归 aest = K·(beam − θ0) 就同时得到真实增益 K 和真实
 *     水平角 θ0。S 曲线移动本身在 ±3° 内扫过倾角，**这趟移动就是辨识实验**，
 *     不需要另做标定动作。
 *
 * 因此本任务里的所有"精确公式"都只当**近似控制关系**用：前馈负责大致把球送过去，
 * 剩下的偏差由反馈吃掉，并由遥测暴露出来供离线辨识。
 *
 * 低速阶段使用单向渐增脱困：误差仍大且球持续低速时只朝目标增加倾角，
 * 球开始向目标移动后快速撤销，避免静摩擦残差和积分蓄力过冲。
 */
#include "app_ball_scurve_task.h"

#include "app_ball_config.h"
#include "app_ball_tune.h"
#include "app_fmt.h"
#include "ball_scurve.h"
#include "bsp_time.h"
#include "chassis.h"
#include "debug_uart.h"
#include "key.h"
#include "rpi_uart.h"
#include "step_motor.h"
#include "ui.h"
#include "wit_sdk.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define H3S_UI_PERIOD_MS         100U
/*
 * 遥测周期取 80 ms，把 640 B 单行的链路占用从约 93% 降至约 70%，为第三题
 * 控制拍和偶发状态日志留出余量。球的固有周期约 2.6 s，仍有约 32 点/周期。
 */
#define H3S_TELEMETRY_PERIOD_MS   80U
#define H3S_VISION_RANGE_MM      120.0f
#define H3S_START_POSITION_MM     10.0f
#define H3S_START_SPEED_MM_S       5.0f
/*
 * 步进转速上限。底层 STEP 时钟已提高到 500 kHz，×32 细分下 960 deg/s 需要约 17067 Hz。
 * 进入任务时同时临时提高位置环增益，避免执行器速度上限提高后仍被慢位置环拖住。
 */
#define H3S_ACTUATOR_SPEED_DEG_S 960.0f
#define H3S_ACTUATOR_SERVO_KP_S_INV 12.0f
/* 独立 Hold 调试入口的视觉接管确认时间；第三题由任务内 ENTER 显式启动。 */
#define H3S_HOLD_ARM_DWELL_MS     100U
#define H3S_FINISH_POSITION_MM       2.0f
#define H3S_FINISH_SPEED_MM_S        8.0f
#define H3S_FINISH_DWELL_S            0.20f
#define H3S_CHALLENGE_LIMIT_MS    5000U

/*
 * 动力学水平点相对查表 0° 的偏置，deg。**默认 0 = 承认未标定**。
 * 标定方法见文件头：跑一趟本任务，对遥测的 (beam, aest) 做线性回归，
 * 零交点即真实水平角，填到这里。1 cnt ≈ 0.03°，15 cnt ≈ 0.45°。
 */
#define H3S_LEVEL_BIAS_DEG 0.0f
#define H3S_LEVEL_BIAS_MIN_DEG (-2.0f)
#define H3S_LEVEL_BIAS_MAX_DEG (+2.0f)

/*
 * 独立 H3 Hold 的车体直线加速度前馈。
 * 实车遥测确认 JY61P X 轴对应弯道横向加速度，车体纵向使用 Y 轴。
 * 当前安装下 Y 正向朝车尾，因此统一取反，使“车体向前加速”为控制正方向。
 * 用户确认传感器读数稳定，因此不做进入任务时的零偏标定，只保留轻量低通和限幅。
 */
#define H3S_IMU_FORWARD_AXIS_SIGN          (-1.0f)
#define H3S_IMU_ACCEL_GAIN_DEFAULT          (-1.0f)
#define H3S_IMU_ACCEL_FILTER_TAU_S            0.05f
#define H3S_IMU_ACCEL_LIMIT_M_S2              4.0f
#define H3S_IMU_DATA_MAX_AGE_MS             100U
#define H3S_GRAVITY_M_S2                      9.80665f
#define H3S_RAD_TO_DEG                       57.2957795f

/* O 是按键前的放球起点，不是一次运动航点；计时后只执行 +5cm → −5cm。 */
static const float H3S_WAYPOINT_MM[] = { 50.0f, -50.0f };
#define H3S_WAYPOINT_COUNT ((uint8_t)(sizeof(H3S_WAYPOINT_MM) / sizeof(H3S_WAYPOINT_MM[0])))

/* ===== 任务模式 =====
 *
 *   SCURVE —— 第三题正式业务：进入后先按 HOLD 控制律守 0 mm；任务内 ENTER
 *             才开始计时并执行 +50→−50 mm，完成后持续守住 −50 mm。
 *
 *   HOLD   —— **无剖面**，直接把目标钉在 0 mm 让控制律持续运行。
 *             进入后跳过航点规划，control_law 始终以 x_ref=0、v_ref=0 为目标，
 *             抖动在球被静摩擦钉住时自动激活，是最简单的单一状态验证入口。
 *             适合单独验证：静摩擦能否被突破、抖动触发时机、扰动恢复能力。
 */
typedef enum { H3S_MODE_SCURVE = 0, H3S_MODE_HOLD } H3S_MODE;
static H3S_MODE h3s_mode;
static bool h3s_standalone;

static const char *H3S_ModeName(H3S_MODE m){
    return m == H3S_MODE_HOLD ? "hold" : "scurve";
}

/*
 * 热更支持：去掉 const，允许 app_ball_tune 运行时写入。
 * 参数表在 H3S_TUNE_TABLE 里定义，进入任务时绑定。
 */
static BALL_SCURVE_CONFIG H3S_SCURVE_CONFIG = {
    /*
     * K_G 名义值 (5/7)·g = 7004.75 mm/s² per rad。**这是近似值**：只用来把
     * a_ref 换成前馈倾角，偏 25% 也只是让前馈少给/多给同比例的驱动，
     * 由剖面跟踪反馈补上（仿真 gain 场景：巡航 RMS 1.63 mm、末端 0.03 mm）。
     */
    .rolling_acceleration_gain_mm_s2 = 7004.75f,

    /* 峰值 280 mm/s² 对应峰值倾角 asin(280/7004.75) ≈ 2.29°，仍在权限内。 */
    .max_acceleration_mm_s2 = 280.0f,
    .max_velocity_mm_s = 120.0f,
    .min_duration_s = 0.40f,
    .max_duration_s = 3.00f,
    /* 第三题位置环 KP=12 s^-1 对应约 83 ms；100 ms 预览覆盖少量通信/机构滞后。 */
    .acceleration_preview_s = 0.10f,

    /* wn = 2.4 rad/s、ζ = 0.9：Kp = wn²/K_G、Kd = 2ζwn/K_G，再由 rad/m 转 deg/mm。 */
    .kp_deg_per_mm = 0.04711f,
    .kd_deg_per_mm_s = 0.03533f,
    .feedback_limit_deg = 2.0f,

    /* MOVE/BRAKE/HOLD 连续增益调度。sched=0 可严格回退到上面的固定增益。 */
    .gain_schedule_enabled = 1.0f,
    .brake_kp_deg_per_mm = 0.00500f,
    .brake_kd_deg_per_mm_s = 0.03533f,
    .hold_kp_deg_per_mm = 0.04711f,
    .hold_kd_deg_per_mm_s = 0.01550f,
    .brake_delay_s = 0.22f,
    .brake_acceleration_mm_s2 = 120.0f,
    .brake_blend_start_ratio = 0.793f,
    .brake_blend_full_ratio = 1.00f,
    .brake_blend_tau_s = 0.08f,
    .hold_enter_error_mm = 15.0f,
    .hold_enter_speed_mm_s = 20.0f,
    .hold_enter_dwell_s = 0.30f,
    .hold_exit_error_mm = 30.0f,
    .hold_exit_speed_mm_s = 35.0f,
    .hold_blend_tau_s = 0.25f,
    .velocity_full_weight_age_ms = 60.0f,
    .velocity_floor_weight_age_ms = 120.0f,
    .velocity_untrusted_weight = 0.30f,
    .stationary_velocity_weight = 0.10f,

    /* 末端 20 mm 开始转入真实目标捕获，5 mm 内固定目标完全接管。 */
    .capture_enter_error_mm = 20.0f,
    .capture_full_error_mm = 5.0f,
    .capture_blend_tau_s = 0.08f,

    /*
     * 滚阻前馈默认 0：先跑一趟测出实际欠冲量 d，再按 θ_roll = 2d/(K_G·T²) 反解填回。
     * 拍脑袋填值会把一个可测量的系统性偏差换成一个不可测的。
     */
    .rolling_resistance_deg = 0.0f,
    .rolling_ff_speed_deadband_mm_s = 3.0f,
    .level_bias_deg = H3S_LEVEL_BIAS_DEG,

    /* 角度限位已取消，完全交由 BSP 层 cnt 软限位（SOFT_MIN/MAX 20..430 cnt）。
     * 此处的 min/max 设到物理不可达值，确保本层永不截断控制律输出。 */
    .angle_min_deg = -15.0f,
    .angle_max_deg = 15.0f,
    .angle_rate_limit_deg_s = 960.0f,

    /*
     * 抖动：**默认关闭**，让位给单向脱困（见下方 breakout_*）。
     *
     * 关闭的理由不是"不需要破静摩擦"，而是**这版基线上它破不了**：
     * 抖动是 2 Hz 交流量，经步进位置环（一阶滞后，τ=1/SERVO_KP）衰减。
     * 源码原注释按 τ=0.1 s（SERVO_KP=10）算出"衰减 0.62 倍、管身抖 ±0.6°"，
     * 但那个运行期增益覆盖已在回退到本基线时移除，现在 SERVO_KP=3 → τ=0.333 s：
     *
     *     |H(2Hz)| = 1/sqrt(1+(2π·2·0.333)²) = 0.232
     *     0.97° 指令 → 管身只有 0.225°，而破静摩擦需要 0.62°，**缺口 64%**
     *
     * 要在 2 Hz 下达到 0.62° 需指令 2.67°，超出 dith_amp 上界 2.0°。
     * 降频可行（1 Hz 需 1.44°），但纹波按 A/f² 涨到 1.92 mm。
     * 单向脱困没有这个衰减问题——它是准直流量，1.4° 指令就是 1.4°。
     *
     * 想 A/B 对照时：设 dith_amp>0 且 breakout_max=0，两者互斥。
     */
    .dither_amplitude_deg = 0.0f,
    .dither_frequency_hz = 2.0f,
    .dither_min_error_mm = 3.0f,   /* 与 settled_position_mm 对齐：进判据即停振 */
    .dither_max_speed_mm_s = 8.0f,
    .dither_dwell_s = 0.5f,

    /* 稳定至 0：3..8 mm 小误差缓慢积分，球速超过 15.45 mm/s 后以 0.3 deg/s 释放。 */
    .hold_integral_ki_deg_per_mm_s = 0.0025f,
    .hold_integral_min_error_mm = 3.0f,
    .hold_integral_max_error_mm = 8.0f,
    .hold_integral_max_speed_mm_s = 1.0f,
    .hold_integral_release_speed_mm_s = 15.45f,
    .hold_integral_release_rate_deg_s = 0.3f,
    .hold_integral_motion_comp_deg_per_mm = 0.40f,

    /*
     * 单向脱困：只朝目标方向渐增倾角，球一动就撤。
     *
     * 相对抖动的两点结构性优势：
     *   ① 不注入往复能量——抖动每周期都把球往两边推一次，末端必然带纹波；
     *   ② 不受位置环低通衰减——抖动是 2 Hz 交流量，经 τ=1/SERVO_KP=0.333 s
     *      的位置环只剩 23%（实测 SERVO_KP=3），0.97° 指令到管身只有 0.225°，
     *      而破静摩擦需要 0.62°，缺口 64%。**默认抖动参数在这版基线上根本破不了**。
     *      单向脱困是准直流量，位置环几乎不衰减，1.4° 指令就是 1.4°。
     *
     * 上限 1.4°：实测 θ_stick ≈ 0.62°，留约 126% 余量；仍避免继续大幅增加，
     * 因为突破瞬间净倾角越大，冲出去越远。
     */
    .breakout_max_angle_deg = 1.4f,
    .breakout_ramp_rate_deg_s = 1.2f,    /* 0.62° 阈值约 0.52 s 爬到 */
    .breakout_release_rate_deg_s = 8.0f, /* 撤销比建立快约 6.7 倍：动了就立刻松手 */
    /* 误差仍大于 2 mm 且球持续低速时即可介入；进入 2 mm 内后快速撤销。 */
    .breakout_min_error_mm = 8.0f,
    .breakout_max_speed_mm_s = 5.0f,
    .breakout_dwell_s = 0.25f,
    .breakout_release_speed_mm_s = 6.0f, /* 须高于视觉速度量化噪声 */
    .breakout_release_dwell_s = 0.10f,

    .settled_position_mm = 3.0f,
    .settled_speed_mm_s = 5.0f,
    .settled_time_s = 0.5f,

    /* 只按位置偏差触发：连续 120 ms 超 10 mm，两次至少间隔 500 ms。 */
    .replan_error_mm = 10.0f,
    .replan_velocity_error_mm_s = 0.0f,
    .replan_dwell_s = 0.12f,
    .replan_cooldown_s = 0.50f,
};

/* 0 = 关闭并严格回退原 H3 Hold；允许负值用于上板确认 JY61P 安装方向。 */
static float h3s_imu_gain = H3S_IMU_ACCEL_GAIN_DEFAULT;
static const float h3s_imu_gain_h4_start = -1.4f;
static const float h3s_imu_gain_h4_stop = -1.4f;
static const float h3s_imu_gain_h56_start = -1.5f;
static const float h3s_imu_gain_h56_stop = -1.5f;
/* 阶段切入首拍的同步步进预置角；四项由实车试验独立填写。 */
static float h3s_imu_initial_angle_h4_start_deg = 0.0f;
static float h3s_imu_initial_angle_h4_stop_deg = 0.0f;
static float h3s_imu_initial_angle_h56_start_deg = 0.0f;
static float h3s_imu_initial_angle_h56_stop_deg = 0.0f;
static APP_BALL_IMU_GAIN_PROFILE h3s_imu_gain_profile = APP_BALL_IMU_GAIN_H3;

/* 抖动的角速率需求必须留在输出斜率限制之内，否则会被削顶成三角波。 */
#if 0 /* 编译期无法算 2πfA，这里以注释留下判据 */
2π × dither_frequency_hz × dither_amplitude_deg = 12.2 deg/s < angle_rate_limit_deg_s(60)
#endif

/*
 * ============ 参数热更表 ============
 *
 * 绑定到 H3S_SCURVE_CONFIG 的关键字段，允许运行时通过串口调整。
 * 进入任务时调用 AppBallTune_Init 注册，Tick 里调用 AppBallTune_Poll 收命令。
 *
 * 协议：
 *   ?              列出全部参数
 *   kp             读 Kp
 *   kp=0.05        写 Kp
 *
 * 每项都有物理约束的上下界，越界拒绝（不静默钳位）。
 * 掉电恢复编译期默认值——整定出的值必须回填源码才算数。
 */
static const APP_BALL_TUNE_ENTRY H3S_TUNE_TABLE[] = {
    /* 反馈增益 */
    {
        .name = "kp",
        .value = &H3S_SCURVE_CONFIG.kp_deg_per_mm,
        .min_value = 0.01f,   /* 下界：静摩擦残差 θ_stick/Kp < 20mm → Kp > 0.62/20 */
        .max_value = 0.15f,   /* 上界：环路裕度，过高震荡 */
        .unit = "deg/mm",
    },
    {
        .name = "kd",
        .value = &H3S_SCURVE_CONFIG.kd_deg_per_mm_s,
        .min_value = 0.005f,  /* 下界：欠阻尼会震荡 */
        .max_value = 0.10f,   /* 上界：过阻尼响应慢 */
        .unit = "deg/(mm/s)",
    },
    {
        .name = "sched",
        .value = &H3S_SCURVE_CONFIG.gain_schedule_enabled,
        .min_value = 0.0f,
        .max_value = 1.0f,
        .unit = "bool",
    },
    {
        .name = "bkp",
        .value = &H3S_SCURVE_CONFIG.brake_kp_deg_per_mm,
        .min_value = 0.005f,
        .max_value = 0.15f,
        .unit = "deg/mm",
    },
    {
        .name = "bkd",
        .value = &H3S_SCURVE_CONFIG.brake_kd_deg_per_mm_s,
        .min_value = 0.005f,
        .max_value = 0.10f,
        .unit = "deg/(mm/s)",
    },
    {
        .name = "hkp",
        .value = &H3S_SCURVE_CONFIG.hold_kp_deg_per_mm,
        .min_value = 0.01f,
        .max_value = 0.15f,
        .unit = "deg/mm",
    },
    {
        .name = "hkd",
        .value = &H3S_SCURVE_CONFIG.hold_kd_deg_per_mm_s,
        .min_value = 0.0f,
        .max_value = 0.10f,
        .unit = "deg/(mm/s)",
    },
    {
        .name = "bdelay",
        .value = &H3S_SCURVE_CONFIG.brake_delay_s,
        .min_value = 0.0f,
        .max_value = 0.60f,
        .unit = "s",
    },
    {
        .name = "bacc",
        .value = &H3S_SCURVE_CONFIG.brake_acceleration_mm_s2,
        .min_value = 20.0f,
        .max_value = 500.0f,
        .unit = "mm/s2",
    },
    {
        .name = "bstart",
        .value = &H3S_SCURVE_CONFIG.brake_blend_start_ratio,
        .min_value = 0.10f,
        .max_value = 1.50f,
        .unit = "ratio",
    },
    {
        .name = "bfull",
        .value = &H3S_SCURVE_CONFIG.brake_blend_full_ratio,
        .min_value = 0.20f,
        .max_value = 2.50f,
        .unit = "ratio",
    },
    {
        .name = "btau",
        .value = &H3S_SCURVE_CONFIG.brake_blend_tau_s,
        .min_value = 0.0f,
        .max_value = 1.0f,
        .unit = "s",
    },
    {
        .name = "hent_e",
        .value = &H3S_SCURVE_CONFIG.hold_enter_error_mm,
        .min_value = 1.0f,
        .max_value = 50.0f,
        .unit = "mm",
    },
    {
        .name = "hent_v",
        .value = &H3S_SCURVE_CONFIG.hold_enter_speed_mm_s,
        .min_value = 1.0f,
        .max_value = 80.0f,
        .unit = "mm/s",
    },
    {
        .name = "hent_dw",
        .value = &H3S_SCURVE_CONFIG.hold_enter_dwell_s,
        .min_value = 0.0f,
        .max_value = 2.0f,
        .unit = "s",
    },
    {
        .name = "hext_e",
        .value = &H3S_SCURVE_CONFIG.hold_exit_error_mm,
        .min_value = 2.0f,
        .max_value = 100.0f,
        .unit = "mm",
    },
    {
        .name = "hext_v",
        .value = &H3S_SCURVE_CONFIG.hold_exit_speed_mm_s,
        .min_value = 2.0f,
        .max_value = 150.0f,
        .unit = "mm/s",
    },
    {
        .name = "htau",
        .value = &H3S_SCURVE_CONFIG.hold_blend_tau_s,
        .min_value = 0.0f,
        .max_value = 2.0f,
        .unit = "s",
    },
    {
        .name = "vage1",
        .value = &H3S_SCURVE_CONFIG.velocity_full_weight_age_ms,
        .min_value = 0.0f,
        .max_value = 180.0f,
        .unit = "ms",
    },
    {
        .name = "vage0",
        .value = &H3S_SCURVE_CONFIG.velocity_floor_weight_age_ms,
        .min_value = 1.0f,
        .max_value = 200.0f,
        .unit = "ms",
    },
    {
        .name = "vfloor",
        .value = &H3S_SCURVE_CONFIG.velocity_untrusted_weight,
        .min_value = 0.0f,
        .max_value = 1.0f,
        .unit = "ratio",
    },
    {
        .name = "vstill",
        .value = &H3S_SCURVE_CONFIG.stationary_velocity_weight,
        .min_value = 0.0f,
        .max_value = 1.0f,
        .unit = "ratio",
    },
    {
        .name = "apreview",
        .value = &H3S_SCURVE_CONFIG.acceleration_preview_s,
        .min_value = 0.0f,
        .max_value = 0.30f,
        .unit = "s",
    },
    {
        .name = "cap_enter",
        .value = &H3S_SCURVE_CONFIG.capture_enter_error_mm,
        .min_value = 0.0f,
        .max_value = 50.0f,
        .unit = "mm",
    },
    {
        .name = "cap_full",
        .value = &H3S_SCURVE_CONFIG.capture_full_error_mm,
        .min_value = 0.0f,
        .max_value = 20.0f,
        .unit = "mm",
    },
    {
        .name = "cap_tau",
        .value = &H3S_SCURVE_CONFIG.capture_blend_tau_s,
        .min_value = 0.0f,
        .max_value = 0.50f,
        .unit = "s",
    },
    {
        .name = "rp_err",
        .value = &H3S_SCURVE_CONFIG.replan_error_mm,
        .min_value = 0.0f,
        .max_value = 30.0f,
        .unit = "mm",
    },
    {
        .name = "rp_vel",
        .value = &H3S_SCURVE_CONFIG.replan_velocity_error_mm_s,
        .min_value = 0.0f,
        .max_value = 100.0f,
        .unit = "mm/s",
    },
    {
        .name = "rp_dwell",
        .value = &H3S_SCURVE_CONFIG.replan_dwell_s,
        .min_value = 0.0f,
        .max_value = 1.0f,
        .unit = "s",
    },
    {
        .name = "rp_cool",
        .value = &H3S_SCURVE_CONFIG.replan_cooldown_s,
        .min_value = 0.02f,
        .max_value = 2.0f,
        .unit = "s",
    },
    {
        .name = "fblim",
        .value = &H3S_SCURVE_CONFIG.feedback_limit_deg,
        .min_value = 1.0f,    /* 最少留 1° 才有意义 */
        .max_value = 5.0f,    /* 已对齐 BSP 软限位 ±5.345°/±5.236° */
        .unit = "deg",
    },

    /* 抖动参数 */
    {
        .name = "dith_amp",
        .value = &H3S_SCURVE_CONFIG.dither_amplitude_deg,
        .min_value = 0.0f,    /* 0 = 关闭抖动 */
        .max_value = 2.0f,    /* 上界：2πfA < angle_rate_limit，2π×2×2 = 25 deg/s < 960 */
        .unit = "deg",
    },
    {
        .name = "dith_freq",
        .value = &H3S_SCURVE_CONFIG.dither_frequency_hz,
        .min_value = 0.5f,    /* 太慢破不了静摩擦 */
        .max_value = 5.0f,    /* 太快角速率超限 */
        .unit = "Hz",
    },
    {
        .name = "dith_minerr",
        .value = &H3S_SCURVE_CONFIG.dither_min_error_mm,
        .min_value = 0.0f,    /* 0 = 始终抖 */
        .max_value = 20.0f,   /* 超过这个误差说明不在目标附近 */
        .unit = "mm",
    },
    {
        .name = "dith_maxspd",
        .value = &H3S_SCURVE_CONFIG.dither_max_speed_mm_s,
        .min_value = 0.0f,    /* 0 = 不管球速 */
        .max_value = 50.0f,   /* 球在快速运动时不抖 */
        .unit = "mm/s",
    },
    {
        .name = "dith_dwell",
        .value = &H3S_SCURVE_CONFIG.dither_dwell_s,
        .min_value = 0.0f,    /* 0 = 立即起振 */
        .max_value = 5.0f,    /* 等太久没意义 */
        .unit = "s",
    },

    /* 小误差静止积分 */
    { .name = "i_ki", .value = &H3S_SCURVE_CONFIG.hold_integral_ki_deg_per_mm_s,
      .min_value = 0.0f, .max_value = 0.5f, .unit = "deg/mm/s" },
    { .name = "i_minerr", .value = &H3S_SCURVE_CONFIG.hold_integral_min_error_mm,
      .min_value = 0.0f, .max_value = 10.0f, .unit = "mm" },
    { .name = "i_maxerr", .value = &H3S_SCURVE_CONFIG.hold_integral_max_error_mm,
      .min_value = 2.0f, .max_value = 30.0f, .unit = "mm" },
    { .name = "i_maxspd", .value = &H3S_SCURVE_CONFIG.hold_integral_max_speed_mm_s,
      .min_value = 0.0f, .max_value = 20.0f, .unit = "mm/s" },
    { .name = "i_relspd", .value = &H3S_SCURVE_CONFIG.hold_integral_release_speed_mm_s,
      .min_value = 0.0f, .max_value = 30.0f, .unit = "mm/s" },
    { .name = "i_relrate", .value = &H3S_SCURVE_CONFIG.hold_integral_release_rate_deg_s,
      .min_value = 0.0f, .max_value = 60.0f, .unit = "deg/s" },
    { .name = "i_mcomp", .value = &H3S_SCURVE_CONFIG.hold_integral_motion_comp_deg_per_mm,
      .min_value = 0.0f, .max_value = 1.0f, .unit = "deg/mm" },

    /*
     * 单向脱困。与抖动互斥：brk_amp > 0 时抖动被强制归零。
     * 想 A/B 对比就改这一项——brk_amp=0 回到抖动，>0 切到单向脱困。
     */
    {
        .name = "brk_amp",
        .value = &H3S_SCURVE_CONFIG.breakout_max_angle_deg,
        .min_value = 0.0f,    /* 0 = 关闭单向脱困，回退到抖动 */
        .max_value = 2.0f,    /* 上界：θ_stick≈0.62°，2.0 已是 3 倍余量，再大突破即飞 */
        .unit = "deg",
    },
    {
        .name = "brk_ramp",
        .value = &H3S_SCURVE_CONFIG.breakout_ramp_rate_deg_s,
        .min_value = 0.05f,   /* 下界：太慢等不到突破 */
        .max_value = 5.0f,    /* 上界：太快等于阶跃，失去"渐增"意义 */
        .unit = "deg/s",
    },
    {
        .name = "brk_rel",
        .value = &H3S_SCURVE_CONFIG.breakout_release_rate_deg_s,
        .min_value = 0.5f,    /* 下界：撤太慢会把球推过头 */
        .max_value = 60.0f,   /* 上界受 angle_rate_limit_deg_s(960) 约束 */
        .unit = "deg/s",
    },
    {
        .name = "brk_minerr",
        .value = &H3S_SCURVE_CONFIG.breakout_min_error_mm,
        .min_value = 0.0f,    /* 0 = 不看误差（不推荐，会在判据内推球） */
        .max_value = 30.0f,
        .unit = "mm",
    },
    {
        .name = "brk_maxspd",
        .value = &H3S_SCURVE_CONFIG.breakout_max_speed_mm_s,
        .min_value = 0.0f,    /* 触发所需的"静止"判据 */
        .max_value = 30.0f,   /* 太大会在球还在滚时误触发 */
        .unit = "mm/s",
    },
    {
        .name = "brk_dwell",
        .value = &H3S_SCURVE_CONFIG.breakout_dwell_s,
        .min_value = 0.0f,    /* 0 = 立即起爬 */
        .max_value = 3.0f,    /* 等太久浪费时间 */
        .unit = "s",
    },
    {
        .name = "brk_relspd",
        .value = &H3S_SCURVE_CONFIG.breakout_release_speed_mm_s,
        .min_value = 1.0f,    /* 下界：必须高于视觉速度量化噪声，否则误判已脱困 */
        .max_value = 30.0f,   /* 上界：太高等不到释放，脱困角会顶到上限 */
        .unit = "mm/s",
    },
    {
        .name = "brk_reldw",
        .value = &H3S_SCURVE_CONFIG.breakout_release_dwell_s,
        .min_value = 0.0f,    /* 0 = 一满足就撤 */
        .max_value = 1.0f,    /* 太长会推过头 */
        .unit = "s",
    },

    /* 滚阻前馈 */
    {
        .name = "roll_ff",
        .value = &H3S_SCURVE_CONFIG.rolling_resistance_deg,
        .min_value = -1.0f,   /* 反向坡度补偿 */
        .max_value = 1.0f,    /* 正向坡度补偿 */
        .unit = "deg",
    },
    {
        .name = "roll_db",
        .value = &H3S_SCURVE_CONFIG.rolling_ff_speed_deadband_mm_s,
        .min_value = 0.0f,    /* 0 = 始终施加 */
        .max_value = 10.0f,   /* 高速时不施加 */
        .unit = "mm/s",
    },

    /* 动力学水平点偏置 */
    {
        .name = "lvl_bias",
        .value = &H3S_SCURVE_CONFIG.level_bias_deg,
        .min_value = H3S_LEVEL_BIAS_MIN_DEG,   /* 标定误差范围 */
        .max_value = H3S_LEVEL_BIAS_MAX_DEG,
        .unit = "deg",
    },

    /* 独立 H3 Hold 的网页调参项；H4/H5-H6 使用上方固件常量。 */
    {
        .name = "imu_gain",
        .value = &h3s_imu_gain,
        .min_value = -2.0f,
        .max_value = 0.0f,
        .unit = "ratio",
    },
    /* 轨迹参数 */
    {
        .name = "amax",
        .value = &H3S_SCURVE_CONFIG.max_acceleration_mm_s2,
        .min_value = 50.0f,   /* 太慢影响时长 */
        .max_value = 500.0f,  /* 太快超出倾角范围 */
        .unit = "mm/s²",
    },
    {
        .name = "vmax",
        .value = &H3S_SCURVE_CONFIG.max_velocity_mm_s,
        .min_value = 30.0f,
        .max_value = 200.0f,
        .unit = "mm/s",
    },
};

#define H3S_TUNE_COUNT ((uint8_t)(sizeof(H3S_TUNE_TABLE) / sizeof(H3S_TUNE_TABLE[0])))

/** 精简状态：只回答"能不能控"，标志位回答"控到哪了"。 */
typedef enum {
    H3S_STATE_IDLE = 0,   /* 无有效视觉 / 球出界：保持当前水管角不动 */
    H3S_STATE_ACTIVE,     /* 视觉有效，控制律运行中，行为全由标志位驱动 */
    H3S_STATE_FAULT       /* 执行器故障 */
} H3S_STATE;

static H3S_STATE h3s_state;
static BALL_SCURVE_CONTROLLER h3s_controller;
static BALL_SCURVE_OUTPUT h3s_output;
static RPI_UART_PREDICTION h3s_prediction;
static bool h3s_have_prediction;
static uint8_t h3s_waypoint;
static int32_t h3s_target_count;
static uint32_t h3s_start_ms;
static uint32_t h3s_phase_ms;
static uint32_t h3s_finish_ms;
static uint32_t h3s_last_ui;
static uint32_t h3s_last_telemetry;
static H3S_STATE h3s_rendered_state;
static float h3s_target_dwell_s;
static bool h3s_challenge_started;
static bool h3s_armed;
static float h3s_hold_target_mm;
/* Device Check 捕获状态只在本次上电的 RAM 中保留，不改编码器坐标或写 Flash。 */
static bool h3s_level_runtime_calibrated;
/* 组合任务提供规划加速度；独立 H3 Hold 改由 JY61P 实测纵向加速度提供。 */
static float h3s_vehicle_acceleration_mm_s2;
static float h3s_imu_accel_raw_m_s2;
static float h3s_imu_accel_filtered_m_s2;
static float h3s_imu_ff_deg;
static bool  h3s_imu_valid;
static bool  h3s_imu_filter_initialized;
static bool  h3s_imu_initial_angle_pending;
static APP_BALL_IMU_ASSIST_PHASE h3s_imu_assist_phase;

/* 球加速度估计：只用于遥测与离线辨识，**不进入控制律**。 */
static float h3s_prev_velocity_mm_s;
static float h3s_acceleration_est_mm_s2;
static bool  h3s_have_prev_velocity;

typedef struct {
    int16_t count;
    float angle_deg;
} H3S_LINKAGE_POINT;

/*
 * 由 pygame_linkage_visualizer.py 的槽约束模型离线生成，与 app_ball_task.c 同一张表。
 * ⚠ 表的 0°（180 cnt）是**几何名义水平**，不是动力学水平点。二者之差由
 *   H3S_LEVEL_BIAS_DEG 承担，未标定时为 0。
 */
static const H3S_LINKAGE_POINT H3S_LINKAGE_TABLE[] = {
    {  20, -5.345027f }, {  40, -4.635128f }, {  60, -3.933998f },
    {  80, -3.243158f }, { 100, -2.564173f }, { 120, -1.898660f },
    { 140, -1.248293f }, { 160, -0.614805f }, { 180, +0.000000f },
    { 200, +0.594243f }, { 220, +1.165966f }, { 240, +1.713122f },
    { 260, +2.233576f }, { 280, +2.725092f }, { 300, +3.185334f },
    { 320, +3.611860f }, { 340, +4.002121f }, { 360, +4.353460f },
    { 380, +4.663114f }, { 400, +4.928215f }, { 420, +5.145804f },
    { 430, +5.235833f },
};
#define H3S_LINKAGE_POINT_COUNT \
    ((uint8_t)(sizeof(H3S_LINKAGE_TABLE) / sizeof(H3S_LINKAGE_TABLE[0])))

static uint8_t H3S_PutStr(char *dst, const char *src){
    uint8_t n = 0U;
    while (src[n] != '\0'){
        dst[n] = src[n];
        n++;
    }
    return n;
}

static int32_t H3S_RoundToCount(float value){
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

static int32_t H3S_ClampCount(int32_t count){
    if (count < STEP_MOTOR_ENC_SOFT_MIN_COUNTS){ return STEP_MOTOR_ENC_SOFT_MIN_COUNTS; }
    if (count > STEP_MOTOR_ENC_SOFT_MAX_COUNTS){ return STEP_MOTOR_ENC_SOFT_MAX_COUNTS; }
    return count;
}

static float H3S_LinkageAngleFromCount(int32_t count){
    count = H3S_ClampCount(count);
    for (uint8_t i = 1U; i < H3S_LINKAGE_POINT_COUNT; i++){
        const H3S_LINKAGE_POINT *low = &H3S_LINKAGE_TABLE[i - 1U];
        const H3S_LINKAGE_POINT *high = &H3S_LINKAGE_TABLE[i];
        if (count <= high->count){
            float fraction = (float)(count - low->count) /
                             (float)(high->count - low->count);
            return low->angle_deg + fraction * (high->angle_deg - low->angle_deg);
        }
    }
    return H3S_LINKAGE_TABLE[H3S_LINKAGE_POINT_COUNT - 1U].angle_deg;
}

static int32_t H3S_LinkageCountFromAngle(float angle_deg){
    if (angle_deg <= H3S_LINKAGE_TABLE[0].angle_deg){ return H3S_LINKAGE_TABLE[0].count; }
    for (uint8_t i = 1U; i < H3S_LINKAGE_POINT_COUNT; i++){
        const H3S_LINKAGE_POINT *low = &H3S_LINKAGE_TABLE[i - 1U];
        const H3S_LINKAGE_POINT *high = &H3S_LINKAGE_TABLE[i];
        if (angle_deg <= high->angle_deg){
            float fraction = (angle_deg - low->angle_deg) /
                             (high->angle_deg - low->angle_deg);
            float count = (float)low->count + fraction * (float)(high->count - low->count);
            return H3S_ClampCount(H3S_RoundToCount(count));
        }
    }
    return H3S_LINKAGE_TABLE[H3S_LINKAGE_POINT_COUNT - 1U].count;
}

bool AppBallLevel_SetFromEncoderCount(int32_t encoder_count){
    if ((encoder_count < STEP_MOTOR_ENC_SOFT_MIN_COUNTS) ||
        (encoder_count > STEP_MOTOR_ENC_SOFT_MAX_COUNTS)){
        return false;
    }

    float bias_deg = H3S_LinkageAngleFromCount(encoder_count);
    if ((bias_deg < H3S_LEVEL_BIAS_MIN_DEG) ||
        (bias_deg > H3S_LEVEL_BIAS_MAX_DEG)){
        return false;
    }

    H3S_SCURVE_CONFIG.level_bias_deg = bias_deg;
    h3s_level_runtime_calibrated = true;
    return true;
}

float AppBallLevel_GetBiasDeg(void){
    return H3S_SCURVE_CONFIG.level_bias_deg;
}

bool AppBallLevel_IsRuntimeCalibrated(void){
    return h3s_level_runtime_calibrated;
}

static float H3S_ActualBeamAngleDeg(void){
    return H3S_LinkageAngleFromCount(StepMotor_GetEncoderCount());
}

static BSP_STATUS H3S_CommandAngle(float angle_deg){
    int32_t requested = H3S_LinkageCountFromAngle(angle_deg);
    if (requested == h3s_target_count){ return BSP_STATUS_OK; }
    h3s_target_count = requested;
    return StepMotor_MoveToCount(requested);
}

static float H3S_TargetMm(void){
    if (h3s_mode == H3S_MODE_HOLD){ return h3s_hold_target_mm; }
    return h3s_challenge_started ? H3S_WAYPOINT_MM[h3s_waypoint] : 0.0f;
}

static uint32_t H3S_ElapsedMs(uint32_t now){
    if ((h3s_mode == H3S_MODE_SCURVE) && !h3s_challenge_started){ return 0U; }
    uint32_t end_ms = (h3s_finish_ms != 0U) ? h3s_finish_ms : now;
    return end_ms - h3s_start_ms;
}

static bool H3S_StartReady(void){
    return h3s_have_prediction && h3s_prediction.velocity_trusted &&
           (fabsf(h3s_prediction.x_mm) <= H3S_START_POSITION_MM) &&
           (fabsf(h3s_prediction.velocity_mm_s) <= H3S_START_SPEED_MM_S);
}

static const char *H3S_StateName(void){
    if (h3s_state == H3S_STATE_IDLE){ return "NO VISION"; }
    if (h3s_state == H3S_STATE_FAULT){ return "ACTUATOR ERR"; }
    /* ACTIVE: 行为由标志位区分 */
    if (h3s_mode == H3S_MODE_HOLD){
        return h3s_armed ? "HOLD 0cm" : "ARMING...";
    }
    if (h3s_finish_ms != 0U){
        return ((h3s_finish_ms - h3s_start_ms) <= H3S_CHALLENGE_LIMIT_MS)
            ? "DONE <=5s" : "DONE OVERTIME";
    }
    if (!h3s_challenge_started){
        return H3S_StartReady() ? "ENTER TO START" : "HOLDING 0mm";
    }
    if (h3s_output.profile_active){
        return (h3s_waypoint == 0U) ? "MOVE +5cm" : "MOVE -5cm";
    }
    return (h3s_waypoint == 0U) ? "TURN" : "STABILIZE";
}

static void H3S_Render(void){
    char l1[20];
    char l2[20];
    char l3[20];
    char l4[20];
    char l5[20];
    uint8_t n;

    n = H3S_PutStr(l1, H3S_StateName());
    l1[n] = '\0';

    n = H3S_PutStr(l2, "x ");
    AppFmt_Fixed(&l2[n], h3s_have_prediction ? h3s_prediction.x_mm : 0.0f, 1U);
    while (l2[n] != '\0'){ n++; }
    n += H3S_PutStr(&l2[n], " > ");
    AppFmt_Fixed(&l2[n], H3S_TargetMm(), 0U);

    n = H3S_PutStr(l3, "time ");
    uint32_t shown_ms = H3S_ElapsedMs(BSP_Time_GetMs());
    AppFmt_Fixed(&l3[n], (float)shown_ms * 0.001f, 2U);
    while (l3[n] != '\0'){ n++; }
    n += H3S_PutStr(&l3[n], " /5.00s");

    n = H3S_PutStr(l4, "u ");
    AppFmt_Fixed(&l4[n], h3s_output.angle_deg, 2U);
    while (l4[n] != '\0'){ n++; }
    n += H3S_PutStr(&l4[n], " b ");
    AppFmt_Fixed(&l4[n], H3S_ActualBeamAngleDeg(), 2U);

    n = H3S_PutStr(l5, "cnt ");
    AppFmt_I32(&l5[n], StepMotor_GetEncoderCount());
    while (l5[n] != '\0'){ n++; }
    n += H3S_PutStr(&l5[n], " >");
    AppFmt_I32(&l5[n], h3s_target_count);

    bool show_enter = (h3s_mode == H3S_MODE_SCURVE) && !h3s_challenge_started;
    const char *footer = show_enter ? "ENTER:start BACK:exit" : "BACK: safe exit";
    Ui_RenderLines((h3s_mode == H3S_MODE_HOLD) ? "H3 Hold 0cm" : "H3 Challenge",
                   l1, l2, l3, l4, l5, footer);
}

static bool H3S_ReadPrediction(void){
    h3s_have_prediction = RpiUart_Observe(&h3s_prediction);
    return h3s_have_prediction;
}

/** 由视觉速度差分估计球加速度。仅供遥测与离线辨识，不进入控制律。 */
static void H3S_UpdateAccelerationEstimate(float velocity_mm_s, float dt){
    if (dt <= 0.0f){ return; }
    if (h3s_have_prev_velocity){
        float raw = (velocity_mm_s - h3s_prev_velocity_mm_s) / dt;
        /* 0.15 的一阶低通只为让读数可看；辨识时应当用原始 v 序列在主机侧重算。 */
        h3s_acceleration_est_mm_s2 += 0.15f * (raw - h3s_acceleration_est_mm_s2);
    } else{
        h3s_have_prev_velocity = true;
    }
    h3s_prev_velocity_mm_s = velocity_mm_s;
}

static float H3S_ActiveImuGain(void){
    if (h3s_standalone){
        return h3s_imu_gain;
    }
    if (h3s_imu_gain_profile == APP_BALL_IMU_GAIN_H4){
        return (h3s_imu_assist_phase == APP_BALL_IMU_ASSIST_STOP_DECEL)
                   ? h3s_imu_gain_h4_stop
                   : h3s_imu_gain_h4_start;
    }
    if (h3s_imu_gain_profile == APP_BALL_IMU_GAIN_H56){
        return (h3s_imu_assist_phase == APP_BALL_IMU_ASSIST_STOP_DECEL)
                   ? h3s_imu_gain_h56_stop
                   : h3s_imu_gain_h56_start;
    }
    return h3s_imu_gain;
}

static float H3S_InitialImuAngleDeg(void){
    if (h3s_standalone){
        return 0.0f;
    }
    if (h3s_imu_gain_profile == APP_BALL_IMU_GAIN_H4){
        return (h3s_imu_assist_phase == APP_BALL_IMU_ASSIST_STOP_DECEL)
                   ? h3s_imu_initial_angle_h4_stop_deg
                   : h3s_imu_initial_angle_h4_start_deg;
    }
    if (h3s_imu_gain_profile == APP_BALL_IMU_GAIN_H56){
        return (h3s_imu_assist_phase == APP_BALL_IMU_ASSIST_STOP_DECEL)
                   ? h3s_imu_initial_angle_h56_stop_deg
                   : h3s_imu_initial_angle_h56_start_deg;
    }
    return 0.0f;
}

/**
 * 把 JY61P Y 轴车体纵向加速度转换成滚球加速度前馈。
 *
 * 对独立 H3 Hold 与 H4/H5/H6 载球循迹同等生效：
 *   - H3 Hold：补偿桌面扰动/手推等外部加速度；
 *   - H4/H5/H6：实测车体加速度比规划值更准（含电机响应滞后、路面坡度），
 *     会覆盖 AppBallHold_SetVehicleAcceleration 此前写入的规划值。
 *
 * ⚠ 本车 JY61P Y 轴与车体纵向对齐，符号由 H3S_IMU_FORWARD_AXIS_SIGN 统一反号。
 */
static void H3S_UpdateImuAcceleration(float dt){
    if (h3s_mode != H3S_MODE_HOLD){
        return;
    }
    /* 组合任务只在严格标记的起步/停车阶段启用；独立 Hold 忽略此门控。 */
    if (!h3s_standalone &&
        (h3s_imu_assist_phase == APP_BALL_IMU_ASSIST_OFF)){
        h3s_vehicle_acceleration_mm_s2 = 0.0f;
        return;
    }

    /*
     * 阶段切入首拍先发送预置角，使步进目标与底盘加速/减速同步开始；
     * 再把等效物理加速度播种给低通，后续实测 IMU 从该初值平滑接管。
     */
    if (h3s_imu_initial_angle_pending){
        h3s_imu_initial_angle_pending = false;
        float initial_deg = H3S_InitialImuAngleDeg();
        if (!isfinite(initial_deg)){
            initial_deg = 0.0f;
        }
        if (initial_deg > H3S_SCURVE_CONFIG.angle_max_deg){
            initial_deg = H3S_SCURVE_CONFIG.angle_max_deg;
        } else if (initial_deg < -H3S_SCURVE_CONFIG.angle_max_deg){
            initial_deg = -H3S_SCURVE_CONFIG.angle_max_deg;
        }

        float initial_accel_m_s2 =
            tanf(initial_deg / H3S_RAD_TO_DEG) * H3S_GRAVITY_M_S2;
        h3s_vehicle_acceleration_mm_s2 = initial_accel_m_s2 * 1000.0f;
        h3s_imu_ff_deg = initial_deg;
        h3s_imu_accel_raw_m_s2 = 0.0f;
        h3s_imu_valid = false;

        float active_gain = H3S_ActiveImuGain();
        if (fabsf(active_gain) > 0.0001f){
            float filter_seed_m_s2 = initial_accel_m_s2 / active_gain;
            if (filter_seed_m_s2 > H3S_IMU_ACCEL_LIMIT_M_S2){
                filter_seed_m_s2 = H3S_IMU_ACCEL_LIMIT_M_S2;
            } else if (filter_seed_m_s2 < -H3S_IMU_ACCEL_LIMIT_M_S2){
                filter_seed_m_s2 = -H3S_IMU_ACCEL_LIMIT_M_S2;
            }
            h3s_imu_accel_filtered_m_s2 = filter_seed_m_s2;
            h3s_imu_filter_initialized = true;
        }
        return;
    }

    JY61P_I2C_Poll();

    JY61P_I2C_SAMPLE sample;
    bool valid = JY61P_I2C_IsDataFresh(H3S_IMU_DATA_MAX_AGE_MS) &&
                 JY61P_I2C_GetSnapshot(&sample);
    float raw_m_s2 = 0.0f;
    if (valid){
        raw_m_s2 = H3S_IMU_FORWARD_AXIS_SIGN * sample.data.acc_g.y *
                   H3S_GRAVITY_M_S2;
        valid = isfinite(raw_m_s2);
    }

    if (valid){
        if (raw_m_s2 > H3S_IMU_ACCEL_LIMIT_M_S2){
            raw_m_s2 = H3S_IMU_ACCEL_LIMIT_M_S2;
        } else if (raw_m_s2 < -H3S_IMU_ACCEL_LIMIT_M_S2){
            raw_m_s2 = -H3S_IMU_ACCEL_LIMIT_M_S2;
        }
        h3s_imu_accel_raw_m_s2 = raw_m_s2;
    } else{
        /* 数据失效时把辅助平滑退回 0，避免水管指令阶跃。 */
        h3s_imu_accel_raw_m_s2 = 0.0f;
    }

    float safe_dt = (dt > 0.0f) ? dt : 0.02f;
    float alpha = safe_dt / (H3S_IMU_ACCEL_FILTER_TAU_S + safe_dt);
    if (!h3s_imu_filter_initialized && valid){
        h3s_imu_accel_filtered_m_s2 = h3s_imu_accel_raw_m_s2;
        h3s_imu_filter_initialized = true;
    } else{
        h3s_imu_accel_filtered_m_s2 += alpha *
            (h3s_imu_accel_raw_m_s2 - h3s_imu_accel_filtered_m_s2);
    }

    h3s_imu_valid = valid;
    float phase_accel_m_s2 = h3s_imu_accel_filtered_m_s2;
    if (!h3s_standalone){
        /*
         * 先按车体物理加速度方向拒绝阶段外分量，再乘可正可负的 imu_gain。
         * 这样负增益可以反转最终倾角，但不会让起步阶段采纳减速样本，或让停车
         * 阶段采纳加速样本。
         */
        if ((h3s_imu_assist_phase == APP_BALL_IMU_ASSIST_START_ACCEL) &&
            (phase_accel_m_s2 < 0.0f)){
            phase_accel_m_s2 = 0.0f;
        } else if ((h3s_imu_assist_phase == APP_BALL_IMU_ASSIST_STOP_DECEL) &&
                   (phase_accel_m_s2 > 0.0f)){
            phase_accel_m_s2 = 0.0f;
        }
    }
    float active_gain = H3S_ActiveImuGain();
    float assisted_accel_m_s2 = active_gain * phase_accel_m_s2;
    h3s_vehicle_acceleration_mm_s2 = assisted_accel_m_s2 * 1000.0f;
    h3s_imu_ff_deg = atanf(h3s_vehicle_acceleration_mm_s2 /
                           (H3S_GRAVITY_M_S2 * 1000.0f)) * H3S_RAD_TO_DEG;
}

static void H3S_PlanCurrentWaypoint(void){
    BallScurve_PlanTo(&h3s_controller, &H3S_SCURVE_CONFIG,
                      h3s_prediction.x_mm, h3s_prediction.velocity_mm_s,
                      H3S_WAYPOINT_MM[h3s_waypoint]);
    h3s_phase_ms = BSP_Time_GetMs();
    DebugUart_Printf("[SCV] plan wp=%u target=%.1f from x=%.2f v=%.2f T=%.3f\r\n",
                     (unsigned)h3s_waypoint,
                     (double)H3S_WAYPOINT_MM[h3s_waypoint],
                     (double)h3s_prediction.x_mm,
                     (double)h3s_prediction.velocity_mm_s,
                     (double)h3s_controller.duration_s);
}

static void H3S_Enter(bool standalone){
    h3s_standalone = standalone;
    if (h3s_standalone){
        (void)Chassis_Brake();
    }
    StepMotor_AbortStartup();
    (void)StepMotor_Stop();
    (void)StepMotor_SetSpeedLimit(H3S_ACTUATOR_SPEED_DEG_S);
    (void)StepMotor_SetServoGain(H3S_ACTUATOR_SERVO_KP_S_INV);

    BallScurve_Init(&h3s_controller);
    h3s_output = (BALL_SCURVE_OUTPUT){0};

    h3s_state = H3S_STATE_IDLE;
    h3s_waypoint = 0U;
    h3s_target_count = StepMotor_GetEncoderCount();
    h3s_have_prediction = false;
    h3s_prev_velocity_mm_s = 0.0f;
    h3s_acceleration_est_mm_s2 = 0.0f;
    h3s_have_prev_velocity = false;
    h3s_start_ms = BSP_Time_GetMs();
    h3s_phase_ms = h3s_start_ms;
    h3s_finish_ms = 0U;
    h3s_last_ui = 0U;
    h3s_last_telemetry = 0U;
    h3s_rendered_state = (H3S_STATE)0xFF;
    h3s_target_dwell_s = 0.0f;
    h3s_challenge_started = false;
    h3s_armed = false;
    h3s_hold_target_mm = 0.0f;
    h3s_vehicle_acceleration_mm_s2 = 0.0f;
    h3s_imu_accel_raw_m_s2 = 0.0f;
    h3s_imu_accel_filtered_m_s2 = 0.0f;
    h3s_imu_ff_deg = 0.0f;
    h3s_imu_valid = false;
    h3s_imu_filter_initialized = false;
    h3s_imu_initial_angle_pending = false;
    h3s_imu_assist_phase = APP_BALL_IMU_ASSIST_OFF;
    if ((h3s_mode == H3S_MODE_HOLD) && h3s_standalone){
        JY61P_I2C_Init();
    }
    RpiUart_ResetStats();

    /* Hold 调试入口打印完整整定上下文；正式比赛入口避免启动突发占满串口。 */
    if ((h3s_mode == H3S_MODE_HOLD) && h3s_standalone){
    DebugUart_Printf(
        "[SCVCFG] mode=%s linkage=pygame-lut lut_level=180 "
        "lut_zero_is_nominal=1 level_bias=%.3fdeg interp_err=%.4fdeg\r\n",
        H3S_ModeName(h3s_mode),
        (double)H3S_SCURVE_CONFIG.level_bias_deg,
        (double)APP_BALL_LINKAGE_MODEL_MAX_INTERP_ERROR_DEG);
    DebugUart_Printf(
        "[SCVCFG] kg=%.1f amax=%.1f vmax=%.1f tmin=%.2f tmax=%.2f\r\n",
        (double)H3S_SCURVE_CONFIG.rolling_acceleration_gain_mm_s2,
        (double)H3S_SCURVE_CONFIG.max_acceleration_mm_s2,
        (double)H3S_SCURVE_CONFIG.max_velocity_mm_s,
        (double)H3S_SCURVE_CONFIG.min_duration_s,
        (double)H3S_SCURVE_CONFIG.max_duration_s);
    DebugUart_Printf(
        "[SCVCFG] kp=%.5f kd=%.5f fblim=%.2f roll=%.3f rolldb=%.1f replan=%.1f\r\n",
        (double)H3S_SCURVE_CONFIG.kp_deg_per_mm,
        (double)H3S_SCURVE_CONFIG.kd_deg_per_mm_s,
        (double)H3S_SCURVE_CONFIG.feedback_limit_deg,
        (double)H3S_SCURVE_CONFIG.rolling_resistance_deg,
        (double)H3S_SCURVE_CONFIG.rolling_ff_speed_deadband_mm_s,
        (double)H3S_SCURVE_CONFIG.replan_error_mm);
    DebugUart_Printf(
        "[SCVCFG] imu axis=y sign=%.0f gain=%.2f tau=%.3fs limit=%.1fmps2 age=%ums\r\n",
        (double)H3S_IMU_FORWARD_AXIS_SIGN, (double)h3s_imu_gain,
        (double)H3S_IMU_ACCEL_FILTER_TAU_S,
        (double)H3S_IMU_ACCEL_LIMIT_M_S2,
        (unsigned)H3S_IMU_DATA_MAX_AGE_MS);
    DebugUart_Printf(
        "[SCVCFG] sched=%.0f move=%.5f/%.5f brake=%.5f/%.5f hold=%.5f/%.5f "
        "bd=%.2f ba=%.1f br=%.2f..%.2f bt=%.2f ht=%.2f\r\n",
        (double)H3S_SCURVE_CONFIG.gain_schedule_enabled,
        (double)H3S_SCURVE_CONFIG.kp_deg_per_mm,
        (double)H3S_SCURVE_CONFIG.kd_deg_per_mm_s,
        (double)H3S_SCURVE_CONFIG.brake_kp_deg_per_mm,
        (double)H3S_SCURVE_CONFIG.brake_kd_deg_per_mm_s,
        (double)H3S_SCURVE_CONFIG.hold_kp_deg_per_mm,
        (double)H3S_SCURVE_CONFIG.hold_kd_deg_per_mm_s,
        (double)H3S_SCURVE_CONFIG.brake_delay_s,
        (double)H3S_SCURVE_CONFIG.brake_acceleration_mm_s2,
        (double)H3S_SCURVE_CONFIG.brake_blend_start_ratio,
        (double)H3S_SCURVE_CONFIG.brake_blend_full_ratio,
        (double)H3S_SCURVE_CONFIG.brake_blend_tau_s,
        (double)H3S_SCURVE_CONFIG.hold_blend_tau_s);
    DebugUart_Printf(
        "[SCVCFG] hold enter=%.1fmm/%.1fmmps/%.2fs exit=%.1fmm/%.1fmmps "
        "vweight=%.0f..%.0fms floor=%.2f still=%.2f\r\n",
        (double)H3S_SCURVE_CONFIG.hold_enter_error_mm,
        (double)H3S_SCURVE_CONFIG.hold_enter_speed_mm_s,
        (double)H3S_SCURVE_CONFIG.hold_enter_dwell_s,
        (double)H3S_SCURVE_CONFIG.hold_exit_error_mm,
        (double)H3S_SCURVE_CONFIG.hold_exit_speed_mm_s,
        (double)H3S_SCURVE_CONFIG.velocity_full_weight_age_ms,
        (double)H3S_SCURVE_CONFIG.velocity_floor_weight_age_ms,
        (double)H3S_SCURVE_CONFIG.velocity_untrusted_weight,
        (double)H3S_SCURVE_CONFIG.stationary_velocity_weight);
    DebugUart_Printf(
        "[SCVCFG] preview=%.3fs capture=%.1f..%.1fmm/tau%.2fs "
        "replan=%.1fmm/%.1fmmps dwell=%.2fs cool=%.2fs\r\n",
        (double)H3S_SCURVE_CONFIG.acceleration_preview_s,
        (double)H3S_SCURVE_CONFIG.capture_enter_error_mm,
        (double)H3S_SCURVE_CONFIG.capture_full_error_mm,
        (double)H3S_SCURVE_CONFIG.capture_blend_tau_s,
        (double)H3S_SCURVE_CONFIG.replan_error_mm,
        (double)H3S_SCURVE_CONFIG.replan_velocity_error_mm_s,
        (double)H3S_SCURVE_CONFIG.replan_dwell_s,
        (double)H3S_SCURVE_CONFIG.replan_cooldown_s);
    DebugUart_Printf(
        "[SCVCFG] alim=%.2f..%.2f arate=%.1f speed=%.1f servo_kp=%.1f "
        "tol=%d resume=%d minspd=%.1f tick=%ums\r\n",
        (double)H3S_SCURVE_CONFIG.angle_min_deg,
        (double)H3S_SCURVE_CONFIG.angle_max_deg,
        (double)H3S_SCURVE_CONFIG.angle_rate_limit_deg_s,
        (double)H3S_ACTUATOR_SPEED_DEG_S,
        (double)H3S_ACTUATOR_SERVO_KP_S_INV,
        (int)STEP_MOTOR_POSITION_TOLERANCE_COUNTS,
        (int)STEP_MOTOR_SERVO_RESUME_COUNTS,
        (double)STEP_MOTOR_SERVO_MIN_SPEED_DEG_S,
        (unsigned)STEP_MOTOR_TICK_PERIOD_MS);
    DebugUart_Printf(
        "[SCVCFG] dither amp=%.2fdeg f=%.1fHz minerr=%.1fmm maxspd=%.1f "
        "dwell=%.2fs rate_need=%.1fdeg/s\r\n",
        (double)H3S_SCURVE_CONFIG.dither_amplitude_deg,
        (double)H3S_SCURVE_CONFIG.dither_frequency_hz,
        (double)H3S_SCURVE_CONFIG.dither_min_error_mm,
        (double)H3S_SCURVE_CONFIG.dither_max_speed_mm_s,
        (double)H3S_SCURVE_CONFIG.dither_dwell_s,
        (double)(6.2831853f * H3S_SCURVE_CONFIG.dither_frequency_hz *
                 H3S_SCURVE_CONFIG.dither_amplitude_deg));
    DebugUart_Printf(
        "[SCVCFG] small-I ki=%.3f err=%.1f..%.1fmm "
        "speed=%.1f/%.1f rel=%.1fdeg/s mcomp=%.3fdeg/mm\r\n",
        (double)H3S_SCURVE_CONFIG.hold_integral_ki_deg_per_mm_s,
        (double)H3S_SCURVE_CONFIG.hold_integral_min_error_mm,
        (double)H3S_SCURVE_CONFIG.hold_integral_max_error_mm,
        (double)H3S_SCURVE_CONFIG.hold_integral_max_speed_mm_s,
        (double)H3S_SCURVE_CONFIG.hold_integral_release_speed_mm_s,
        (double)H3S_SCURVE_CONFIG.hold_integral_release_rate_deg_s,
        (double)H3S_SCURVE_CONFIG.hold_integral_motion_comp_deg_per_mm);
    DebugUart_Printf(
        "[SCVCFG] breakout max=%.2fdeg ramp=%.2fdeg/s rel=%.1fdeg/s "
        "minerr=%.1fmm maxspd=%.1f dwell=%.2fs relspd=%.1f reldwell=%.2fs %s\r\n",
        (double)H3S_SCURVE_CONFIG.breakout_max_angle_deg,
        (double)H3S_SCURVE_CONFIG.breakout_ramp_rate_deg_s,
        (double)H3S_SCURVE_CONFIG.breakout_release_rate_deg_s,
        (double)H3S_SCURVE_CONFIG.breakout_min_error_mm,
        (double)H3S_SCURVE_CONFIG.breakout_max_speed_mm_s,
        (double)H3S_SCURVE_CONFIG.breakout_dwell_s,
        (double)H3S_SCURVE_CONFIG.breakout_release_speed_mm_s,
        (double)H3S_SCURVE_CONFIG.breakout_release_dwell_s,
        (H3S_SCURVE_CONFIG.breakout_max_angle_deg > 0.0f)
            ? "ACTIVE(dither forced off)" : "off(dither active)");
    }
    if (h3s_mode == H3S_MODE_SCURVE){
        DebugUart_Printf("[H3CFG] hold0->ENTER->%.0f->%.0f turn=no-verify "
                         "finish=%.1fmm/%.1fmmps/%.2fs limit=%ums\r\n",
                         (double)H3S_WAYPOINT_MM[0], (double)H3S_WAYPOINT_MM[1],
                         (double)H3S_FINISH_POSITION_MM,
                         (double)H3S_FINISH_SPEED_MM_S, (double)H3S_FINISH_DWELL_S,
                         (unsigned)H3S_CHALLENGE_LIMIT_MS);
    }

    /* 两种入口都允许热更；只有 Hold 调试入口主动打印整张参数表。 */
    AppBallTune_Init(H3S_TUNE_TABLE, H3S_TUNE_COUNT);
    if ((h3s_mode == H3S_MODE_HOLD) && h3s_standalone){ AppBallTune_PrintAll(); }
}

static void H3S_Telemetry(uint32_t now, bool usable, STEP_MOTOR_GUARD_STATE guard){
    RPI_UART_STATS stats;
    RpiUart_GetStats(&stats);
    int32_t encoder = StepMotor_GetEncoderCount();
    float beam = H3S_LinkageAngleFromCount(encoder);
    float target_mm = H3S_TargetMm();

    /*
     * 一行输出全部中间量，key=value 便于主机侧按名解析。分组顺序 =
     * 数据流方向：视觉 → 剖面 → 控制分量 → 执行器 → 链路健康。
     *
     * 辨识用的两个关键量是 beam（实际水管角）和 aest（球加速度估计）：
     * 对 (beam, aest) 做线性回归，斜率 = 真实 K_G，零交点 = 真实水平角。
     */
    DebugUart_Printf(
        /* --- 时基与状态 --- */
        "[SCV] t=%lu st=%u wp=%u ok=%u "
        /* --- 视觉层：原始 / 使用值 / 龄期 / 质量 --- */
        "xr=%.2f x=%.2f vr=%.2f v=%.2f aest=%.1f age=%.1f q=%u "
        /* --- 剖面层 --- */
        "tgt=%.1f xref=%.2f vref=%.2f aref=%.1f apv=%.1f tp=%.3f tpd=%.3f act=%u "
        "gm=%u bb=%.2f hb=%.2f cb=%.2f kpe=%.4f kde=%.4f ds=%.1f vc=%.1f vw=%.2f "
        /* --- 控制分量：合成前每一项 --- */
        "bias=%.3f ff=%.3f lead=%.3f rff=%.3f fb=%.3f iacc=%.3f dith=%.3f brka=%.3f u=%.3f "
        /* --- JY61P 车体纵向加速度辅助 --- */
        "araw=%.3f aflt=%.3f aff=%.3f imuok=%u "
        /* --- 跟踪误差 + 单向脱困计时（判误触发 / 判释放是否太晚）--- */
        "ex=%.2f ev=%.2f etgt=%.2f brkst=%.2f brkrel=%.2f "
        /* --- 执行器层：指令角 vs 实际角是滞后的直接指标 --- */
        "beam=%.3f lag=%.3f cnt=%ld cmd=%ld perr=%ld spd=%.1f frq=%lu at=%u "
        /*
         * --- 标志与链路健康 ---
         * fbc = PD 分量被 feedback_limit_deg 夹住（sat 只反映物理角度范围，
         * 早期版本缺这一位，45% 的反馈限幅在遥测里是隐形的）。
         * dth = 抖动正在注入；brk = 单向脱困正在介入（两者互斥）。
         */
        "sat=%u fbc=%u rl=%u rp=%u iact=%u dth=%u brk=%u set=%u mv=%u vv=%u px=%u edge=%u deg=%u hold=%u "
        "guard=%u fps=%.1f gap=%lu inv=%lu crc=%lu drop=%lu\r\n",
        (unsigned long)H3S_ElapsedMs(now), (unsigned)h3s_state,
        (unsigned)h3s_waypoint, (unsigned)(usable ? 1U : 0U),

        (double)(h3s_have_prediction ? h3s_prediction.measured_x_mm : 0.0f),
        (double)(h3s_have_prediction ? h3s_prediction.x_mm : 0.0f),
        (double)(h3s_have_prediction ? h3s_prediction.measured_velocity_mm_s : 0.0f),
        (double)(h3s_have_prediction ? h3s_prediction.velocity_mm_s : 0.0f),
        (double)h3s_acceleration_est_mm_s2,
        (double)(h3s_have_prediction ? h3s_prediction.age_ms : 0.0f),
        (unsigned)(h3s_have_prediction ? h3s_prediction.quality : 0U),

        (double)target_mm,
        (double)h3s_output.x_ref_mm, (double)h3s_output.v_ref_mm_s,
        (double)h3s_output.a_ref_mm_s2,
        (double)h3s_output.acceleration_preview_mm_s2,
        (double)h3s_output.profile_time_s, (double)h3s_output.profile_duration_s,
        (unsigned)(h3s_output.profile_active ? 1U : 0U),
        (unsigned)h3s_output.gain_mode,
        (double)h3s_output.brake_blend, (double)h3s_output.hold_blend,
        (double)h3s_output.capture_blend,
        (double)h3s_output.effective_kp_deg_per_mm,
        (double)h3s_output.effective_kd_deg_per_mm_s,
        (double)h3s_output.stopping_distance_mm,
        (double)h3s_output.closing_velocity_mm_s,
        (double)h3s_output.velocity_weight,

        (double)H3S_SCURVE_CONFIG.level_bias_deg,
        (double)h3s_output.feedforward_deg, (double)h3s_output.actuator_lead_deg,
        (double)h3s_output.rolling_ff_deg,
        (double)h3s_output.feedback_deg, (double)h3s_output.hold_integral_deg,
        (double)h3s_output.dither_deg,
        (double)h3s_output.breakout_deg,
        (double)h3s_output.angle_deg,

        (double)h3s_imu_accel_raw_m_s2,
        (double)h3s_imu_accel_filtered_m_s2,
        (double)h3s_imu_ff_deg,
        (unsigned)(h3s_imu_valid ? 1U : 0U),

        (double)h3s_output.position_error_mm, (double)h3s_output.velocity_error_mm_s,
        (double)(target_mm - (h3s_have_prediction ? h3s_prediction.x_mm : 0.0f)),
        (double)h3s_output.breakout_stuck_s, (double)h3s_output.breakout_release_s,

        (double)beam, (double)(h3s_output.angle_deg - beam),
        (long)encoder, (long)h3s_target_count,
        (long)StepMotor_GetPositionErrorCount(),
        (double)StepMotor_GetSpeed(),
        (unsigned long)StepMotor_GetStepFrequencyHz(),
        (unsigned)(StepMotor_IsAtTarget() ? 1U : 0U),

        (unsigned)(h3s_output.saturated ? 1U : 0U),
        (unsigned)(h3s_output.feedback_clipped ? 1U : 0U),
        (unsigned)(h3s_output.rate_limited ? 1U : 0U),
        (unsigned)(h3s_output.replanned ? 1U : 0U),
        (unsigned)(h3s_output.hold_integral_on ? 1U : 0U),
        (unsigned)(h3s_output.dither_on ? 1U : 0U),
        (unsigned)(h3s_output.breakout_on ? 1U : 0U),
        (unsigned)(h3s_output.settled ? 1U : 0U),
        (unsigned)(h3s_have_prediction && h3s_prediction.moving),
        (unsigned)(h3s_have_prediction && h3s_prediction.velocity_trusted),
        (unsigned)(h3s_have_prediction && h3s_prediction.position_extrapolated),
        (unsigned)(h3s_have_prediction && h3s_prediction.edge),
        (unsigned)(h3s_have_prediction && h3s_prediction.degraded),
        (unsigned)(h3s_have_prediction && h3s_prediction.hold_output),
        (unsigned)guard, (double)stats.frame_rate_x10 * 0.1,
        (unsigned long)stats.seq_gap, (unsigned long)stats.invalid,
        (unsigned long)stats.crc_fail,
        (unsigned long)DebugUart_GetDroppedBytes());
}

static APP_TASK_STATUS H3S_Tick(float dt){
    uint32_t now = BSP_Time_GetMs();
    STEP_MOTOR_GUARD_STATE guard = StepMotor_GetGuardState();
    bool usable = H3S_ReadPrediction();

    /* 热更轮询：收串口命令并执行（非阻塞，单拍最多一行）。 */
    AppBallTune_Poll();

    /* 独立 H3 Hold 使用实测纵向加速度；其他模式继续沿用原有输入来源。 */
    H3S_UpdateImuAcceleration(dt);

    if (guard == STEP_MOTOR_GUARD_FAULT){
        h3s_state = H3S_STATE_FAULT;
        return APP_TASK_FAULT;
    }

    if (usable){
        H3S_UpdateAccelerationEstimate(h3s_prediction.velocity_mm_s, dt);
    }

    /* IDLE: 无有效视觉，保持当前水管角，等待视觉恢复。 */
    if (h3s_state == H3S_STATE_IDLE){
        if (usable && (fabsf(h3s_prediction.x_mm) <= H3S_VISION_RANGE_MM)){
            BallScurve_Reset(&h3s_controller, H3S_ActualBeamAngleDeg());
            h3s_state = H3S_STATE_ACTIVE;
            h3s_phase_ms = now;
            h3s_target_dwell_s = 0.0f;
            DebugUart_Printf("[SCV] armed x=%.2f v=%.2f age=%.1f beam=%.3f\r\n",
                             (double)h3s_prediction.x_mm,
                             (double)h3s_prediction.velocity_mm_s,
                             (double)h3s_prediction.age_ms,
                             (double)H3S_ActualBeamAngleDeg());
        }
        goto render_and_return;
    }

    /* FAULT: 执行器故障，终止任务。 */
    if (h3s_state == H3S_STATE_FAULT){
        return APP_TASK_FAULT;
    }

    /* ===== ACTIVE：视觉有效，统一跑控制律，行为全由标志位驱动 ===== */

    /* 视觉丢失 → 回到 IDLE，保持当前水管角不动。 */
    if (!usable || (fabsf(h3s_prediction.x_mm) > H3S_VISION_RANGE_MM)){
        (void)StepMotor_Stop();
        h3s_target_count = StepMotor_GetEncoderCount();
        h3s_target_dwell_s = 0.0f;
        h3s_state = H3S_STATE_IDLE;
        goto render_and_return;
    }

    /* HOLD 模式：100ms 视觉武装确认。 */
    if ((h3s_mode == H3S_MODE_HOLD) && !h3s_armed){
        if ((now - h3s_phase_ms) >= H3S_HOLD_ARM_DWELL_MS){
            h3s_armed = true;
            DebugUart_Printf("[SCV] hold0 armed x=%.2f beam=%.3f\r\n",
                             (double)h3s_prediction.x_mm,
                             (double)H3S_ActualBeamAngleDeg());
        }
    }

    /* SCURVE 模式：未发车时守 0mm，检测 ENTER。 */
    if ((h3s_mode == H3S_MODE_SCURVE) && !h3s_challenge_started){
        if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
            if (H3S_StartReady()){
                h3s_challenge_started = true;
                h3s_start_ms = now;
                h3s_finish_ms = 0U;
                h3s_waypoint = 0U;
                h3s_target_dwell_s = 0.0f;
                DebugUart_Printf("[H3] start x=%.2f v=%.2f\r\n",
                                 (double)h3s_prediction.x_mm,
                                 (double)h3s_prediction.velocity_mm_s);
                H3S_PlanCurrentWaypoint();
            } else{
                DebugUart_Printf("[H3] start rejected x=%.2f v=%.2f vv=%u\r\n",
                                 (double)h3s_prediction.x_mm,
                                 (double)h3s_prediction.velocity_mm_s,
                                 (unsigned)(h3s_prediction.velocity_trusted ? 1U : 0U));
            }
        }
    }

    /* 统一控制律：所有 ACTIVE 路径共享这一个调用点。 */
    {
        BALL_SCURVE_INPUT input = {
            .x_mm = h3s_prediction.x_mm,
            .velocity_mm_s = h3s_prediction.velocity_mm_s,
            .vehicle_acceleration_mm_s2 = h3s_vehicle_acceleration_mm_s2,
            .actual_angle_deg = H3S_ActualBeamAngleDeg(),
            .velocity_trusted = h3s_prediction.velocity_trusted,
            .moving = h3s_prediction.moving,
            .measurement_age_ms = h3s_prediction.age_ms,
            .dt_s = dt,
        };
        if (!BallScurve_Update(&h3s_controller, &H3S_SCURVE_CONFIG, &input,
                               &h3s_output) ||
            (H3S_CommandAngle(h3s_output.angle_deg) != BSP_STATUS_OK)){
            h3s_state = H3S_STATE_FAULT;
            return APP_TASK_FAULT;
        }
    }

    /* SCURVE: 剖面结束 → 切下一航点（+50→−50 no-verify）或进验证（−50 末点）。 */
    if ((h3s_mode == H3S_MODE_SCURVE) && h3s_challenge_started &&
        (h3s_finish_ms == 0U) && !h3s_output.profile_active){
        DebugUart_Printf("[SCV] arrive wp=%u target=%.1f x=%.2f err=%.2f\r\n",
                         (unsigned)h3s_waypoint,
                         (double)H3S_WAYPOINT_MM[h3s_waypoint],
                         (double)h3s_prediction.x_mm,
                         (double)(h3s_prediction.x_mm -
                                  H3S_WAYPOINT_MM[h3s_waypoint]));
        if (h3s_waypoint == 0U){
            DebugUart_Printf("[H3] turn +5cm no-verify ms=%lu x=%.2f v=%.2f\r\n",
                             (unsigned long)(now - h3s_start_ms),
                             (double)h3s_prediction.x_mm,
                             (double)h3s_prediction.velocity_mm_s);
            h3s_waypoint++;
            h3s_target_dwell_s = 0.0f;
            H3S_PlanCurrentWaypoint();
        } else{
            h3s_target_dwell_s = 0.0f;
        }
    }

    /* SCURVE: 末点（−50mm）稳定验证。 */
    if ((h3s_mode == H3S_MODE_SCURVE) && h3s_challenge_started &&
        (h3s_finish_ms == 0U) && (h3s_waypoint >= 1U) &&
        !h3s_output.profile_active){
        float target_error = H3S_TargetMm() - h3s_prediction.x_mm;
        bool qualified = h3s_prediction.velocity_trusted &&
            !h3s_prediction.moving &&
            (fabsf(target_error) <= H3S_FINISH_POSITION_MM) &&
            (fabsf(h3s_prediction.velocity_mm_s) <= H3S_FINISH_SPEED_MM_S);
        h3s_target_dwell_s = qualified ? (h3s_target_dwell_s + dt) : 0.0f;
        if (h3s_target_dwell_s >= H3S_FINISH_DWELL_S){
            h3s_finish_ms = now;
            DebugUart_Printf("[H3] done ms=%lu pass=%u x=%.2f v=%.2f\r\n",
                             (unsigned long)(h3s_finish_ms - h3s_start_ms),
                             (unsigned)(((h3s_finish_ms - h3s_start_ms) <=
                                         H3S_CHALLENGE_LIMIT_MS) ? 1U : 0U),
                             (double)h3s_prediction.x_mm,
                             (double)h3s_prediction.velocity_mm_s);
        }
    }

render_and_return:

    /* 组合模式由循迹任务独占调试串口，避免两路周期遥测挤满 115200 UART。 */
    if (h3s_standalone && !AppBallTune_IsListing() &&
        ((now - h3s_last_telemetry) >= H3S_TELEMETRY_PERIOD_MS)){
        h3s_last_telemetry = now;
        H3S_Telemetry(now, usable, guard);
    }

    if (h3s_standalone){
        bool render_changed = (h3s_state != h3s_rendered_state);
        bool render_periodic = (now - h3s_last_ui) >= H3S_UI_PERIOD_MS;
        if ((render_changed || render_periodic) && !Ui_IsFlushBusy()){
            h3s_last_ui = now;
            H3S_Render();
            h3s_rendered_state = h3s_state;
        }
    }
    return APP_TASK_RUNNING;
}

static void H3S_Exit(void){
    h3s_vehicle_acceleration_mm_s2 = 0.0f;
    (void)StepMotor_Stop();
    (void)StepMotor_SetServoGain(STEP_MOTOR_SERVO_KP);
    h3s_target_count = StepMotor_GetEncoderCount();
    DebugUart_Printf("[SCV] exit wp=%u hold-count=%ld beam=%.3f\r\n",
                     (unsigned)h3s_waypoint, (long)h3s_target_count,
                     (double)H3S_ActualBeamAngleDeg());
}

/*
 * 两个入口共用 Tick / Exit，只有 Enter 的**模式赋值**不同。
 * 拆成两个 Enter 包装而不是两份任务文件：控制律只有一处，任何修改
 * 自动对两个入口一致生效，A/B 对比的差异才只可能来自行为，不可能
 * 来自代码漂移。
 */
static void H3S_EnterChallenge(void){ h3s_mode = H3S_MODE_SCURVE; H3S_Enter(true); }
static void H3S_EnterHold(void)  { h3s_mode = H3S_MODE_HOLD;   H3S_Enter(true); }
/* Hold -5cm：进入后自动以 -50mm 为稳定目标，供设备检查页直接观测末点行为。 */
static void H3S_EnterHold_m5(void){
    h3s_mode = H3S_MODE_HOLD;
    H3S_Enter(true);
    AppBallHold_SetTargetMm(-50.0f);
}

void AppBallHold_Enter(void){
    h3s_mode = H3S_MODE_HOLD;
    H3S_Enter(false);
}

bool AppBallHold_SetTargetMm(float target_mm){
    if (!isfinite(target_mm) || (fabsf(target_mm) > H3S_VISION_RANGE_MM)){
        return false;
    }
    h3s_hold_target_mm = target_mm;
    h3s_controller.target_mm = target_mm;
    BallScurve_Reset(&h3s_controller, H3S_ActualBeamAngleDeg());
    if (h3s_state == H3S_STATE_IDLE){
        h3s_state = H3S_STATE_ACTIVE;
    }
    h3s_target_dwell_s = 0.0f;
    DebugUart_Printf("[SCV] hold target=%.2fmm beam=%.3f\r\n",
                     (double)target_mm, (double)H3S_ActualBeamAngleDeg());
    return true;
}

void AppBallHold_SetVehicleAcceleration(float acceleration_mps2){
    h3s_vehicle_acceleration_mm_s2 = isfinite(acceleration_mps2)
        ? acceleration_mps2 * 1000.0f : 0.0f;
}

void AppBallHold_SetImuAssistPhase(APP_BALL_IMU_ASSIST_PHASE phase){
    if ((phase != APP_BALL_IMU_ASSIST_OFF) &&
        (phase != APP_BALL_IMU_ASSIST_START_ACCEL) &&
        (phase != APP_BALL_IMU_ASSIST_STOP_DECEL)){
        phase = APP_BALL_IMU_ASSIST_OFF;
    }
    if (h3s_imu_assist_phase == phase){
        return;
    }

    /*
     * 巡航关闭期间不更新本地低通；阶段切换必须丢弃上一阶段残留，
     * 否则停车重新开启时会短暂继承起步正加速度并给出反向倾角。
     */
    h3s_imu_assist_phase = phase;
    h3s_vehicle_acceleration_mm_s2 = 0.0f;
    h3s_imu_accel_raw_m_s2 = 0.0f;
    h3s_imu_accel_filtered_m_s2 = 0.0f;
    h3s_imu_ff_deg = 0.0f;
    h3s_imu_valid = false;
    h3s_imu_filter_initialized = false;
    h3s_imu_initial_angle_pending =
        !h3s_standalone &&
        ((phase == APP_BALL_IMU_ASSIST_START_ACCEL) ||
         (phase == APP_BALL_IMU_ASSIST_STOP_DECEL)) &&
        (fabsf(H3S_InitialImuAngleDeg()) > 0.0001f);
}

void AppBallHold_SelectImuGainProfile(APP_BALL_IMU_GAIN_PROFILE profile){
    if ((profile != APP_BALL_IMU_GAIN_H3) &&
        (profile != APP_BALL_IMU_GAIN_H4) &&
        (profile != APP_BALL_IMU_GAIN_H56)){
        profile = APP_BALL_IMU_GAIN_H3;
    }
    h3s_imu_gain_profile = profile;
}

APP_TASK_STATUS AppBallHold_Tick(float dt){
    return H3S_Tick(dt);
}

void AppBallHold_Exit(void){
    H3S_Exit();
}

void AppBallHold_GetSnapshot(APP_BALL_HOLD_SNAPSHOT *out){
    if (out == NULL){ return; }
    out->ball_x_mm    = h3s_prediction.x_mm;
    out->ball_v_mm_s  = h3s_prediction.velocity_mm_s;
    out->target_mm    = H3S_TargetMm();
    out->error_mm     = h3s_output.position_error_mm;
    out->angle_deg    = h3s_output.angle_deg;
    out->veh_ff_deg   = h3s_imu_ff_deg;
    out->feedback_deg = h3s_output.feedback_deg;
    out->breakout_deg = h3s_output.breakout_deg;
    out->integral_deg = h3s_output.hold_integral_deg;
    out->breakout_on  = h3s_output.breakout_on;
    out->integral_on  = h3s_output.hold_integral_on;
    out->saturated    = h3s_output.saturated;
    out->feedback_clipped = h3s_output.feedback_clipped;
}

/** 第三题正式业务：O → +50 → −50 mm，完成后持续守住末点并显示用时。 */
const APP_TASK_DESC APP_H3_CHALLENGE = {
    "H3 Challenge", H3S_EnterChallenge, H3S_Tick, H3S_Exit
};

/**
 * 单点保持 0mm。无剖面，控制律以 x_ref=0 / v_ref=0 持续运行。
 *
 * 目的：单独验证「静摩擦能否被突破」「抖动触发时机是否正确」
 * 以及「扰动后恢复能力」，不受 S 曲线运动本身的影响。
 * 遥测格式与 H3 Challenge 完全相同，配套 ball_hold_monitor.html 观测。
 */
const APP_TASK_DESC APP_H3_BALL_HOLD = {
    "H3 Hold 0cm", H3S_EnterHold, H3S_Tick, H3S_Exit
};
/**
 * 单点保持 −50mm(−5cm)。与 APP_H3_BALL_HOLD 共用控制律，仅目标不同。
 * 供设备检查页直接观测末点保持行为（脱困、积分、扰动恢复）。
 */
const APP_TASK_DESC APP_H3_BALL_HOLD_M5 = {
    "H3 Hold -5cm", H3S_EnterHold_m5, H3S_Tick, H3S_Exit
};
