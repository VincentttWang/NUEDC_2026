/**
 * @file  app_ball_task.c
 * @brief 小车静止时的滚球 0 cm 闭环任务。
 */
#include "app_ball_task.h"

#include "app_ball_config.h"
#include "app_fmt.h"
#include "ball_balance.h"
#include "bsp_time.h"
#include "chassis.h"
#include "debug_uart.h"
#include "rpi_uart.h"
#include "step_motor.h"
#include "ui.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define H3_UI_PERIOD_MS             100U
#define H3_TELEMETRY_PERIOD_MS      60U
#define H3_GOOD_SAMPLE_TICKS        1U
#define H3_ARM_POSITION_MM          120.0f
#define H3_MOTOR_SHAFT_SPEED_DEG_S  960.0f
#define H3_MOTOR_SERVO_KP_S_INV     6.0f

static const BALL_BALANCE_CONFIG H3_BALANCE_CONFIG = {
    .rolling_acceleration_gain_mm_s2 = APP_BALL_ROLLING_GAIN_MM_S2_PER_RAD,
    .profile_max_velocity_mm_s = APP_BALL_PROFILE_MAX_VELOCITY_MM_S,
    .profile_max_acceleration_mm_s2 = APP_BALL_PROFILE_MAX_ACCELERATION_MM_S2,
    .profile_min_duration_s = APP_BALL_PROFILE_MIN_DURATION_S,
    .profile_max_duration_s = APP_BALL_PROFILE_MAX_DURATION_S,
    /* 每 20 ms 重规划，但执行 120 ms 预瞄点，避免每拍都停留在零加速度起点。 */
    .profile_lookahead_s = APP_BALL_PROFILE_LOOKAHEAD_S,
    .position_feedback_gain_deg_per_mm = 0.025f,
    .velocity_feedback_gain_deg_per_mm_s = 0.018f,
    .feedback_limit_deg = 1.5f,
    /* 只在中心低速且无扰动时学习；窗口外以 180 s 时间常数缓慢泄漏。 */
    .zero_trim_ki_deg_per_mm_s = 0.004f,
    .zero_trim_leak_tau_s = 180.0f,
    .zero_trim_limit_deg = 0.6f,
    /* 180 cnt 水平相位下软限位约为 -5.345°~+5.236°，两端留约 0.1°。 */
    .angle_min_deg = -5.2f * APP_BALL_LINKAGE_ANGLE_SCALE +
                     APP_BALL_LINKAGE_ANGLE_OFFSET_DEG,
    .angle_max_deg = 5.1f * APP_BALL_LINKAGE_ANGLE_SCALE +
                     APP_BALL_LINKAGE_ANGLE_OFFSET_DEG,
    .control_angle_limit_deg = 4.0f,
    .angle_rate_near_deg_s = 320.0f,
    .angle_rate_far_deg_s = 180.0f,
    .angle_rate_brake_deg_s = 480.0f,
    .angle_rate_transition_mm = 30.0f,
    .stuck_min_error_mm = 20.0f,
    .stuck_max_position_change_mm = 0.5f,
    .stuck_min_command_deg = 1.2f,
    .stuck_time_s = 0.8f,
    .trim_min_position_mm = 2.0f,
    .trim_max_position_mm = 20.0f,
    .trim_min_speed_mm_s = 0.0f,
    .trim_max_speed_mm_s = 8.0f,
    .settled_position_mm = 2.0f,
    .settled_speed_mm_s = 5.0f,
    .settled_time_s = 1.0f,
};

typedef enum {
    H3_STATE_WAIT_VISION = 0,
    H3_STATE_ACTIVE,
    H3_STATE_DEGRADED,
    H3_STATE_ACTUATOR_FAULT
} H3_STATE;

static H3_STATE h3_state;
static BALL_BALANCE_CONTROLLER h3_controller;
static BALL_BALANCE_OUTPUT h3_output;
static RPI_UART_PREDICTION h3_prediction;
static bool h3_have_prediction;
static uint8_t h3_good_ticks;
static int32_t h3_target_count;
static uint32_t h3_start_ms;
static uint32_t h3_last_ui;
static uint32_t h3_last_telemetry;
static H3_STATE h3_rendered_state;
static bool h3_rendered_settled;

typedef struct {
    int16_t count;
    float angle_deg;
} H3_LINKAGE_POINT;

/*
 * 由 pygame_linkage_visualizer.py 的自包含槽约束模型在编码器软限位内离线生成。
 * 20-count 线性插值的全区间最大误差约 0.006127°，远小于机构回差和视觉闭环
 * 的实际分辨率。180 cnt 是当前水平标定，但仍不用于 H3 safe return。
 */
static const H3_LINKAGE_POINT H3_LINKAGE_TABLE[] = {
    {  20, -5.345027f }, {  40, -4.635128f }, {  60, -3.933998f },
    {  80, -3.243158f }, { 100, -2.564173f }, { 120, -1.898660f },
    { 140, -1.248293f }, { 160, -0.614805f }, { 180, +0.000000f },
    { 200, +0.594243f }, { 220, +1.165966f }, { 240, +1.713122f },
    { 260, +2.233576f }, { 280, +2.725092f }, { 300, +3.185334f },
    { 320, +3.611860f }, { 340, +4.002121f }, { 360, +4.353460f },
    { 380, +4.663114f }, { 400, +4.928215f }, { 420, +5.145804f },
    { 430, +5.235833f },
};

#define H3_LINKAGE_POINT_COUNT \
    ((uint8_t)(sizeof(H3_LINKAGE_TABLE) / sizeof(H3_LINKAGE_TABLE[0])))

static uint8_t H3_PutStr(char *dst, const char *src){
    uint8_t n = 0U;
    while (src[n] != '\0'){
        dst[n] = src[n];
        n++;
    }
    return n;
}

static int32_t H3_RoundToCount(float value){
    return (value >= 0.0f) ? (int32_t)(value + 0.5f)
                           : (int32_t)(value - 0.5f);
}

static int32_t H3_ClampCount(int32_t count){
    if (count < STEP_MOTOR_ENC_SOFT_MIN_COUNTS){
        return STEP_MOTOR_ENC_SOFT_MIN_COUNTS;
    }
    if (count > STEP_MOTOR_ENC_SOFT_MAX_COUNTS){
        return STEP_MOTOR_ENC_SOFT_MAX_COUNTS;
    }
    return count;
}

static float H3_LinkageAngleFromCount(int32_t count){
    count = H3_ClampCount(count);
    for (uint8_t i = 1U; i < H3_LINKAGE_POINT_COUNT; i++){
        const H3_LINKAGE_POINT *low = &H3_LINKAGE_TABLE[i - 1U];
        const H3_LINKAGE_POINT *high = &H3_LINKAGE_TABLE[i];
        if (count <= high->count){
            float fraction = (float)(count - low->count) /
                             (float)(high->count - low->count);
            float nominal_deg = low->angle_deg +
                fraction * (high->angle_deg - low->angle_deg);
            return nominal_deg * APP_BALL_LINKAGE_ANGLE_SCALE +
                   APP_BALL_LINKAGE_ANGLE_OFFSET_DEG;
        }
    }
    return H3_LINKAGE_TABLE[H3_LINKAGE_POINT_COUNT - 1U].angle_deg *
           APP_BALL_LINKAGE_ANGLE_SCALE + APP_BALL_LINKAGE_ANGLE_OFFSET_DEG;
}

static int32_t H3_LinkageCountFromAngle(float angle_deg){
    float nominal_angle_deg = (angle_deg - APP_BALL_LINKAGE_ANGLE_OFFSET_DEG) /
                              APP_BALL_LINKAGE_ANGLE_SCALE;
    if (nominal_angle_deg <= H3_LINKAGE_TABLE[0].angle_deg){
        return H3_LINKAGE_TABLE[0].count;
    }
    for (uint8_t i = 1U; i < H3_LINKAGE_POINT_COUNT; i++){
        const H3_LINKAGE_POINT *low = &H3_LINKAGE_TABLE[i - 1U];
        const H3_LINKAGE_POINT *high = &H3_LINKAGE_TABLE[i];
        if (nominal_angle_deg <= high->angle_deg){
            float fraction = (nominal_angle_deg - low->angle_deg) /
                             (high->angle_deg - low->angle_deg);
            float count = (float)low->count +
                          fraction * (float)(high->count - low->count);
            return H3_ClampCount(H3_RoundToCount(count));
        }
    }
    return H3_LINKAGE_TABLE[H3_LINKAGE_POINT_COUNT - 1U].count;
}

static float H3_ActualBeamAngleDeg(void){
    return H3_LinkageAngleFromCount(StepMotor_GetEncoderCount());
}

static BSP_STATUS H3_CommandAngle(float angle_deg){
    int32_t requested = H3_LinkageCountFromAngle(angle_deg);
    if (requested == h3_target_count){
        return BSP_STATUS_OK;
    }
    h3_target_count = requested;
    return StepMotor_MoveToCount(requested);
}

static bool H3_ReadUsablePrediction(void){
    h3_have_prediction = RpiUart_Observe(&h3_prediction);
    /*
     * H3 直接信任树莓派最近一次有效位置。QUALITY、V_VALID、短时降级和单帧
     * INVALID 只保留作遥测诊断，不再打断闭环；RpiUart_Observe 的 200 ms
     * 超时仍是通信彻底中断时的最后保护。Observe 不用倾角模型制造速度。
     */
    return h3_have_prediction;
}

static const char *H3_StateName(void){
    switch (h3_state){
        case H3_STATE_WAIT_VISION:    return "WAIT VISION";
        case H3_STATE_ACTIVE:
            if (h3_output.intercepted){ return "STUCK CHECK"; }
            return h3_output.settled ? "STABLE 0cm" : "BALANCING";
        case H3_STATE_DEGRADED:       return "HOLD ANGLE";
        case H3_STATE_ACTUATOR_FAULT: return "ACTUATOR ERR";
        default:                      return "UNKNOWN";
    }
}

static void H3_Render(void){
    char l1[20];
    char l2[20];
    char l3[20];
    char l4[20];
    char l5[20];
    uint8_t n;

    n = H3_PutStr(l1, H3_StateName());
    l1[n] = '\0';

    n = H3_PutStr(l2, "x ");
    AppFmt_Fixed(&l2[n], h3_have_prediction ? h3_prediction.x_mm : 0.0f, 1U);
    while (l2[n] != '\0'){ n++; }
    n += H3_PutStr(&l2[n], " v ");
    AppFmt_I32(&l2[n], h3_have_prediction ?
               (int32_t)h3_prediction.velocity_mm_s : 0);

    n = H3_PutStr(l3, "cnt ");
    AppFmt_I32(&l3[n], StepMotor_GetEncoderCount());
    while (l3[n] != '\0'){ n++; }
    n += H3_PutStr(&l3[n], " >");
    AppFmt_I32(&l3[n], h3_target_count);

    n = H3_PutStr(l4, "u ");
    AppFmt_Fixed(&l4[n], h3_output.angle_deg, 2U);
    while (l4[n] != '\0'){ n++; }
    n += H3_PutStr(&l4[n], " b ");
    AppFmt_Fixed(&l4[n], h3_output.zero_trim_deg, 2U);

    n = H3_PutStr(l5, "age ");
    AppFmt_Fixed(&l5[n], h3_have_prediction ? h3_prediction.age_ms : 0.0f, 0U);
    while (l5[n] != '\0'){ n++; }
    n += H3_PutStr(&l5[n], " Q");
    AppFmt_I32(&l5[n], h3_have_prediction ? (int32_t)h3_prediction.quality : 0);

    Ui_RenderLines("H3 Hold 0cm", l1, l2, l3, l4, l5, "BACK: safe exit");
}

static void H3_Enter(void){
    /* H3 的定义就是车体静止；显式退出可能遗留的速度闭环并主动制动两轮。 */
    (void)Chassis_Brake();
    /*
     * 机械水平位只是粗标定，进入 H3 时不得先向它运动。放弃可能尚未结束的上电
     * 抬升，并把步进目标钉在当前实测位置，等待视觉闭环从当前姿态接管。
     */
    StepMotor_AbortStartup();
    (void)StepMotor_Stop();
    (void)StepMotor_SetSpeedLimit(H3_MOTOR_SHAFT_SPEED_DEG_S);
    (void)StepMotor_SetServoGain(H3_MOTOR_SERVO_KP_S_INV);
    BallBalance_Init(&h3_controller);
    BallBalance_SetTarget(&h3_controller, 0.0f);
    BallBalance_SetZeroTrim(&h3_controller, APP_BALL_INITIAL_ZERO_TRIM_DEG);
    h3_output = (BALL_BALANCE_OUTPUT){0};
    h3_output.angle_rate_limit_deg_s = H3_BALANCE_CONFIG.angle_rate_near_deg_s;
    h3_output.zero_trim_deg = APP_BALL_INITIAL_ZERO_TRIM_DEG;
    h3_state = H3_STATE_WAIT_VISION;
    h3_good_ticks = 0U;
    h3_target_count = StepMotor_GetEncoderCount();
    h3_have_prediction = false;
    h3_start_ms = BSP_Time_GetMs();
    h3_last_ui = 0U;
    h3_last_telemetry = 0U;
    h3_rendered_state = (H3_STATE)0xFF;
    h3_rendered_settled = false;
    RpiUart_ResetStats();
    DebugUart_Printf(
        "[BALL] enter hold0 linkage=pygame-lut level=%.1f range=%.0f..%.0f "
        "interp=%.4fdeg scale=%.5f offset=%.4fdeg geom=%u phase=nominal\r\n",
        (double)APP_BALL_LINKAGE_MODEL_LEVEL_COUNTS,
        (double)APP_BALL_LINKAGE_MODEL_LEVEL_MIN_COUNTS,
        (double)APP_BALL_LINKAGE_MODEL_LEVEL_MAX_COUNTS,
        (double)APP_BALL_LINKAGE_MODEL_MAX_INTERP_ERROR_DEG,
        (double)APP_BALL_LINKAGE_ANGLE_SCALE,
        (double)APP_BALL_LINKAGE_ANGLE_OFFSET_DEG,
        (unsigned)APP_BALL_LINKAGE_MODEL_GEOMETRY_VALIDATED);
    DebugUart_Printf(
        "[BALLCFG] mode=receding-quintic kg=%.2f vmax=%.1f amax=%.1f "
        "T=%.2f..%.2f look=%.0fms motor=%.1f mkp=%.1f initbias=%.2f\r\n",
        (double)H3_BALANCE_CONFIG.rolling_acceleration_gain_mm_s2,
        (double)H3_BALANCE_CONFIG.profile_max_velocity_mm_s,
        (double)H3_BALANCE_CONFIG.profile_max_acceleration_mm_s2,
        (double)H3_BALANCE_CONFIG.profile_min_duration_s,
        (double)H3_BALANCE_CONFIG.profile_max_duration_s,
        (double)(H3_BALANCE_CONFIG.profile_lookahead_s * 1000.0f),
        (double)H3_MOTOR_SHAFT_SPEED_DEG_S,
        (double)H3_MOTOR_SERVO_KP_S_INV,
        (double)APP_BALL_INITIAL_ZERO_TRIM_DEG
    );
    DebugUart_Printf(
        "[BALLCFG] kx=%.4f kv=%.4f fblim=%.2f ki=%.4f leak=%.1f blim=%.2f\r\n",
        (double)H3_BALANCE_CONFIG.position_feedback_gain_deg_per_mm,
        (double)H3_BALANCE_CONFIG.velocity_feedback_gain_deg_per_mm_s,
        (double)H3_BALANCE_CONFIG.feedback_limit_deg,
        (double)H3_BALANCE_CONFIG.zero_trim_ki_deg_per_mm_s,
        (double)H3_BALANCE_CONFIG.zero_trim_leak_tau_s,
        (double)H3_BALANCE_CONFIG.zero_trim_limit_deg);
    DebugUart_Printf(
        "[BALLCFG] ctrl=%.2f physical=%.2f..%.2f rates=%.1f/%.1f/%.1f "
        "rscale=%.1f stuck=%.1fmm,%.2fs,diag arm=%.1f timeout=%u period=%u\r\n",
        (double)H3_BALANCE_CONFIG.control_angle_limit_deg,
        (double)H3_BALANCE_CONFIG.angle_min_deg,
        (double)H3_BALANCE_CONFIG.angle_max_deg,
        (double)H3_BALANCE_CONFIG.angle_rate_near_deg_s,
        (double)H3_BALANCE_CONFIG.angle_rate_far_deg_s,
        (double)H3_BALANCE_CONFIG.angle_rate_brake_deg_s,
        (double)H3_BALANCE_CONFIG.angle_rate_transition_mm,
        (double)H3_BALANCE_CONFIG.stuck_min_error_mm,
        (double)H3_BALANCE_CONFIG.stuck_time_s,
        (double)H3_ARM_POSITION_MM, (unsigned)RPI_UART_TIMEOUT_MS,
        (unsigned)H3_TELEMETRY_PERIOD_MS);
}

static APP_TASK_STATUS H3_Tick(float dt){
    uint32_t now = BSP_Time_GetMs();
    STEP_MOTOR_GUARD_STATE guard = StepMotor_GetGuardState();
    bool usable = H3_ReadUsablePrediction();

    if (guard == STEP_MOTOR_GUARD_FAULT){
        h3_state = H3_STATE_ACTUATOR_FAULT;
        return APP_TASK_FAULT;
    }

    switch (h3_state){
        case H3_STATE_WAIT_VISION:
            if (usable && (fabsf(h3_prediction.x_mm) <= H3_ARM_POSITION_MM)){
                if (h3_good_ticks < H3_GOOD_SAMPLE_TICKS){
                    h3_good_ticks++;
                }
                if (h3_good_ticks >= H3_GOOD_SAMPLE_TICKS){
                    BallBalance_Reset(&h3_controller, H3_ActualBeamAngleDeg());
                    BallBalance_SetTarget(&h3_controller, 0.0f);
                    BallBalance_SetZeroTrim(&h3_controller,
                                            APP_BALL_INITIAL_ZERO_TRIM_DEG);
                    h3_state = H3_STATE_ACTIVE;
                    DebugUart_Printf("[BALL] armed x=%.1f v=%.1f age=%.1f\r\n",
                                     (double)h3_prediction.x_mm,
                                     (double)h3_prediction.velocity_mm_s,
                                     (double)h3_prediction.age_ms);
                }
            } else{
                h3_good_ticks = 0U;
            }
            break;

        case H3_STATE_ACTIVE:
            if (!usable){
                /* 失去视觉时就地保持，不向可能错误的标定水平位回归。 */
                (void)StepMotor_Stop();
                h3_target_count = StepMotor_GetEncoderCount();
                h3_state = H3_STATE_DEGRADED;
                h3_good_ticks = 0U;
                break;
            }
            {
                bool trim_health = !h3_output.braking &&
                    !h3_output.intercepted &&
                    !h3_prediction.edge && !h3_prediction.degraded &&
                    h3_prediction.velocity_trusted &&
                    !h3_output.profile_limit_relaxed;
                BALL_BALANCE_INPUT input = {
                    .x_mm = h3_prediction.x_mm,
                    .velocity_mm_s = h3_prediction.velocity_mm_s,
                    .actual_angle_deg = H3_ActualBeamAngleDeg(),
                    .dt_s = dt,
                    .moving = h3_prediction.moving,
                    .allow_zero_trim = trim_health,
                };
                if (!BallBalance_Update(&h3_controller, &H3_BALANCE_CONFIG,
                                        &input, &h3_output) ||
                    (H3_CommandAngle(h3_output.angle_deg) != BSP_STATUS_OK)){
                    h3_state = H3_STATE_ACTUATOR_FAULT;
                    return APP_TASK_FAULT;
                }
            }
            break;

        case H3_STATE_DEGRADED:
            /* 保持失链瞬间水管角；只等待视觉恢复，绝不执行盲目 safe return。 */
            if (usable && (fabsf(h3_prediction.x_mm) <= H3_ARM_POSITION_MM)){
                if (h3_good_ticks < H3_GOOD_SAMPLE_TICKS){
                    h3_good_ticks++;
                }
                if (h3_good_ticks >= H3_GOOD_SAMPLE_TICKS){
                    BallBalance_Reset(&h3_controller, H3_ActualBeamAngleDeg());
                    BallBalance_SetTarget(&h3_controller, 0.0f);
                    BallBalance_SetZeroTrim(&h3_controller,
                                            APP_BALL_INITIAL_ZERO_TRIM_DEG);
                    h3_state = H3_STATE_ACTIVE;
                }
            } else{
                h3_good_ticks = 0U;
            }
            break;

        case H3_STATE_ACTUATOR_FAULT:
        default:
            return APP_TASK_FAULT;
    }

    if ((now - h3_last_telemetry) >= H3_TELEMETRY_PERIOD_MS){
        RPI_UART_STATS rpi_stats;
        RpiUart_GetStats(&rpi_stats);
        int32_t encoder_count = StepMotor_GetEncoderCount();
        float motor_angle_deg =
            ((float)encoder_count - APP_BALL_LINKAGE_MODEL_LEVEL_COUNTS) *
            360.0f / STEP_MOTOR_ENCODER_COUNTS_PER_REV;
        h3_last_telemetry = now;
        DebugUart_Printf(
            "[BALL] t=%lu st=%u ok=%u xr=%.2f x=%.2f v=%.2f age=%.1f "
            "et=%.2f xref=%.2f vref=%.2f aref=%.1f T=%.3f look=%.3f "
            "pv=%.1f pa=%.1f a0=%.1f ia=%.2f iv=%.2f rx=%.4f rv=%.4f ra=%.3f "
            "ex=%.2f ev=%.2f aff=%.3f fbp=%.3f fbv=%.3f fb=%.3f "
            "rate=%.1f u=%.3f bias=%.3f beam=%.3f motor=%.3f "
            "cnt=%ld tgt=%ld mspd=%.1f mkp=%.1f sf=%lu "
            "arr=%lu cc=%lu ctr=%lu sdt=%lu smax=%lu "
            "lim=%u sat=%u rl=%u brk=%u stuck=%u set=%u mv=%u vv=%u px=%u edge=%u "
            "deg=%u hold=%u q=%u guard=%u fps=%.1f "
            "gap=%lu inv=%lu crc=%lu drop=%lu\r\n",
            (unsigned long)(now - h3_start_ms), (unsigned)h3_state,
            (unsigned)(usable ? 1U : 0U),
            (double)(h3_have_prediction ? h3_prediction.measured_x_mm : 0.0f),
            (double)(h3_have_prediction ? h3_prediction.x_mm : 0.0f),
            (double)(h3_have_prediction ? h3_prediction.velocity_mm_s : 0.0f),
            (double)(h3_have_prediction ? h3_prediction.age_ms : 0.0f),
            (double)(h3_have_prediction ? -h3_prediction.x_mm : 0.0f),
            (double)h3_output.x_ref_mm,
            (double)h3_output.velocity_ref_mm_s,
            (double)h3_output.acceleration_ref_mm_s2,
            (double)h3_output.profile_duration_s,
            (double)h3_output.profile_lookahead_s,
            (double)h3_output.profile_peak_velocity_mm_s,
            (double)h3_output.profile_peak_acceleration_mm_s2,
            (double)h3_output.model_acceleration_mm_s2,
            (double)h3_output.acceleration_integral_mm_s,
            (double)h3_output.velocity_integral_mm,
            (double)h3_output.terminal_position_residual_mm,
            (double)h3_output.terminal_velocity_residual_mm_s,
            (double)h3_output.terminal_acceleration_residual_mm_s2,
            (double)h3_output.position_error_mm,
            (double)h3_output.velocity_error_mm_s,
            (double)h3_output.feedforward_deg,
            (double)h3_output.position_feedback_deg,
            (double)h3_output.velocity_feedback_deg,
            (double)h3_output.feedback_deg,
            (double)h3_output.angle_rate_limit_deg_s,
            (double)h3_output.angle_deg,
            (double)h3_output.zero_trim_deg,
            (double)H3_ActualBeamAngleDeg(), (double)motor_angle_deg,
            (long)encoder_count, (long)h3_target_count,
            (double)StepMotor_GetSpeed(), (double)StepMotor_GetServoGain(),
            (unsigned long)StepMotor_GetStepFrequencyHz(),
            (unsigned long)StepMotor_GetPwmLoadValue(),
            (unsigned long)StepMotor_GetPwmCompareValue(),
            (unsigned long)StepMotor_GetPwmCounterValue(),
            (unsigned long)StepMotor_GetTickIntervalMs(),
            (unsigned long)StepMotor_GetMaxTickIntervalMs(),
            (unsigned)(h3_output.profile_limit_relaxed ? 1U : 0U),
            (unsigned)(h3_output.saturated ? 1U : 0U),
            (unsigned)(h3_output.rate_limited ? 1U : 0U),
            (unsigned)(h3_output.braking ? 1U : 0U),
            (unsigned)(h3_output.intercepted ? 1U : 0U),
            (unsigned)(h3_output.settled ? 1U : 0U),
            (unsigned)(h3_have_prediction && h3_prediction.moving),
            (unsigned)(h3_have_prediction && h3_prediction.velocity_trusted),
            (unsigned)(h3_have_prediction && h3_prediction.position_extrapolated),
            (unsigned)(h3_have_prediction && h3_prediction.edge),
            (unsigned)(h3_have_prediction && h3_prediction.degraded),
            (unsigned)(h3_have_prediction && h3_prediction.hold_output),
            (unsigned)(h3_have_prediction ? h3_prediction.quality : 0U),
            (unsigned)guard, (double)rpi_stats.frame_rate_x10 * 0.1,
            (unsigned long)rpi_stats.seq_gap,
            (unsigned long)rpi_stats.invalid,
            (unsigned long)rpi_stats.crc_fail,
            (unsigned long)DebugUart_GetDroppedBytes());
    }
    /*
     * OLED 整帧已改为约 23 ms 的后台 DMA，ACTIVE 也可按 10 Hz 更新数据。若上一帧
     * 仍在发送则直接跳过本拍，绝不在 20 ms 控制回调里等待；状态/稳定变化因
     * rendered 快照未更新，会在后续控制拍持续重试直至成功上屏。
     */
    bool render_changed = (h3_state != h3_rendered_state) ||
                          (h3_output.settled != h3_rendered_settled);
    bool render_periodic = (now - h3_last_ui) >= H3_UI_PERIOD_MS;
    if ((render_changed || render_periodic) && !Ui_IsFlushBusy()){
        h3_last_ui = now;
        H3_Render();
        h3_rendered_state = h3_state;
        h3_rendered_settled = h3_output.settled;
    }
    return APP_TASK_RUNNING;
}

static void H3_Exit(void){
    (void)StepMotor_Stop();
    (void)StepMotor_SetServoGain(STEP_MOTOR_SERVO_KP);
    h3_target_count = StepMotor_GetEncoderCount();
    DebugUart_Printf("[BALL] exit hold-count=%ld\r\n", (long)h3_target_count);
}

const APP_TASK_DESC APP_H3_BALL_STATIC = {
    "H3 Static PID", H3_Enter, H3_Tick, H3_Exit
};
