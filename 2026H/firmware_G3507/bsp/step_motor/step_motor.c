/**
 * @file  step_motor.c
 * @brief BSP 摆杆步进电机位置式控制 + QEI 反馈实现。
 *
 * 分层:
 *   位置指令 MoveToCount ──► 限幅 ──► target_counts
 *                                        │
 *   Tick ─► 采样编码器 ─► 抬升状态机 ─► 限位守护 ─► 位置伺服 ─► 出脉冲
 *
 * 脉冲由定时器 PWM 产生:伺服算出的速度(deg/s)先换算成步进频率,再由定时器时钟推出
 * 重载值 ARR,占空比固定取 ARR/2(方波)。方向与使能是普通 GPIO。
 *
 * 速度下发是内部实现细节,外部拿不到——位置限幅因此不可能被绕过。
 *
 * ⚠ QEI 硬件计数器是 16 位,必须以「相邻两次采样计数变化 < 32767」的频率调用
 *   StepMotor_Tick()(内含采样),否则会丢圈。
 */
#include "step_motor.h"

#include <stdbool.h>
#include <stddef.h>

/* ===== 硬件描述 ===== */

static GPIO_Regs *const        motor_dir_port    = STEP_MOTOR_BEAM_DIR_PORT;
static const uint32_t          motor_dir_pin     = STEP_MOTOR_BEAM_DIR_PIN;
static GPIO_Regs *const        motor_en_port     = STEP_MOTOR_BEAM_EN_PORT;
static const uint32_t          motor_en_pin      = STEP_MOTOR_BEAM_EN_PIN;
static GPTIMER_Regs *const     motor_pwm_timer   = STEP_MOTOR_BEAM_PWM_TIMER;
static const DL_TIMER_CC_INDEX motor_pwm_channel = STEP_MOTOR_BEAM_PWM_CHANNEL;

/* 16 位定时器的装载值上限。纯实现细节,不对外。 */
#define STEP_MOTOR_MAX_ARR 65535U

/* 限位是否生效由宏一次性决定,编译器会把不生效的分支整个优化掉。 */
#define STEP_MOTOR_LIMIT_ON (STEP_MOTOR_ENC_LIMIT_ENABLED != 0U)

/* ===== 运行状态 ===== */

static int32_t  target_counts;          /* 目标位置,已过限幅。 */
static float    speed_limit_deg_s = STEP_MOTOR_SERVO_DEFAULT_SPEED_LIMIT_DEG_S;
static float    servo_kp_s_inv = STEP_MOTOR_SERVO_KP;
static float    servo_speed_deg_s;      /* 伺服本拍下发的速度(只读诊断)。 */
static bool     driver_enabled;         /* 驱动器 EN 当前状态。 */
/*
 * 伺服的到位状态,带回差。
 *
 * 进入靠 POSITION_TOLERANCE_COUNTS(紧),退出靠 SERVO_RESUME_COUNTS(松)。
 * 拆成两个阈值是为了让"停得准"不再以"漂一点就补偿"为代价:摆杆带重力负载,
 * 单阈值下把到位带收紧就会在末端形成补偿-下沉的极限环。
 *
 * 这也是 IsAtTarget() 的返回值——查询和伺服共用同一个状态,不会出现
 * "报到位了但还在出脉冲"。
 */
static bool     servo_at_target = true;
static uint32_t last_step_freq_hz;      /* 最近一次实际应用的步进频率(限幅后)。 */
static uint32_t step_tick_last_ms;
static uint32_t step_tick_interval_ms;
static uint32_t step_tick_max_interval_ms;
static bool     step_tick_have_last;

/* ===== QEI 编码器状态 ===== */

static uint16_t encoder_last_raw;   /* 上次采样到的硬件 16 位计数。 */
static int32_t  encoder_accum;      /* 累计计数,已消除 16 位环绕。 */

/* ===== 限位守护状态 ===== */

static STEP_MOTOR_GUARD_STATE guard_state = STEP_MOTOR_GUARD_DISABLED;
static uint32_t guard_recover_start_ms;   /* 本轮纠正的起始时刻,用于超时判定。 */

/* ===== 上电抬升状态 ===== */

/*
 * 抬升状态。只有 WAIT / LIFTING 两态算"忙",此时外部 MoveToCount 会被拒;
 * 其余三态控制权都在上层。不对外暴露——上层靠 MoveToCount 返回 BUSY 即可得知。
 */
typedef enum {
    STARTUP_DISABLED = 0,   /* 未启用,或已被 AbortStartup 放弃。 */
    STARTUP_WAIT,           /* 等 DELAY_MS 到点。 */
    STARTUP_LIFTING,        /* 正在抬升。 */
    STARTUP_DONE,           /* 已到位。 */
    STARTUP_FAULT           /* 超时未到位,已停机失能。 */
} STARTUP_STATE;

static STARTUP_STATE startup_state = STARTUP_DISABLED;
static int32_t  startup_target_counts;    /* 已夹进行程的抬升目标。 */
static uint32_t startup_phase_start_ms;   /* 当前阶段(等待/抬升)的起始时刻。 */
static bool     startup_clock_started;    /* 首次 Tick 才能取到有效时基。 */

/* ===== 内部工具 ===== */

static void StepMotor_HaltPulse(void);

static bool StepMotor_IsStartupBusy(void){
    return (startup_state == STARTUP_WAIT) || (startup_state == STARTUP_LIFTING);
}

/* 守护正在纠正越界(只影响状态显示,限幅在 MoveTo 里已经做完)。 */
static bool StepMotor_IsGuardBusy(void){
    return (guard_state == STEP_MOTOR_GUARD_RECOVER_NEG) ||
           (guard_state == STEP_MOTOR_GUARD_RECOVER_POS);
}

/* ===== 脉冲输出 ===== */

uint32_t StepMotor_SpeedToStepFreq(float speed_deg_per_s){
    float abs_speed = (speed_deg_per_s < 0.0f) ? -speed_deg_per_s : speed_deg_per_s;

    if (abs_speed <= 0.0f || STEP_MOTOR_STEP_ANGLE_DEG <= 0.0f || STEP_MOTOR_MICROSTEP <= 0.0f){
        return 0U;
    }

    return (uint32_t)((abs_speed / STEP_MOTOR_STEP_ANGLE_DEG) * STEP_MOTOR_MICROSTEP);
}

static void StepMotor_SetDirection(float speed_deg_per_s){
    bool set_high = (STEP_MOTOR_BEAM_POSITIVE_DIR_HIGH != 0U);

    if (speed_deg_per_s < 0.0f){
        set_high = !set_high;
    }

    if (set_high){
        DL_GPIO_setPins(motor_dir_port, motor_dir_pin);
    } else{
        DL_GPIO_clearPins(motor_dir_port, motor_dir_pin);
    }
}

static void StepMotor_WriteEnablePin(bool enable){
    bool set_high = (STEP_MOTOR_BEAM_ENABLE_HIGH != 0U) ? enable : !enable;

    if (set_high){
        DL_GPIO_setPins(motor_en_port, motor_en_pin);
    } else{
        DL_GPIO_clearPins(motor_en_port, motor_en_pin);
    }
}

/*
 * 停止脉冲：停计数器 + 用 ODIS 强制 CCP 输出为低。
 *
 * ⚠ 不能只把 CC 写 0 了事。EDGE_ALIGN_UP 的输出动作是
 *   ZACT_CCP_HIGH | CUACT_CCP_LOW（计数为 0 拉高、计数等于 CC 拉低），
 *   CC=0 会让这两个事件落在同一计数点上，每个定时器周期可能挤出一个窄毛刺，
 *   被驱动器当成有效 STEP 边沿。WHEELTEC 官方例程同样用 ODIS 强制低电平
 *   来规避（见其 Motor_ForceLow()）。
 */
static void StepMotor_DisablePulse(void){
    DL_TimerG_stopCounter(motor_pwm_timer);
    DL_TimerG_setCCPOutputDisabled(motor_pwm_timer,
                                   DL_TIMER_CCP_DIS_OUT_LOW,
                                   DL_TIMER_CCP_DIS_OUT_LOW);
    DL_TimerG_setCaptureCompareValue(motor_pwm_timer, 0U, motor_pwm_channel);
}

static void StepMotor_ApplySpeed(float speed_deg_per_s){
    uint32_t step_frequency = StepMotor_SpeedToStepFreq(speed_deg_per_s);

    /* 低于硬件可产生的最低频率就不出脉冲，改为保持——否则会被迫跑得比指令快。 */
    if (step_frequency < STEP_MOTOR_MIN_STEP_FREQ_HZ){
        StepMotor_DisablePulse();
        last_step_freq_hz = 0U;
        return;
    }

    if (step_frequency > STEP_MOTOR_MAX_STEP_FREQ_HZ){
        step_frequency = STEP_MOTOR_MAX_STEP_FREQ_HZ;
    }

    /* 四舍五入取周期，减少低速端的量化偏差。 */
    uint32_t period = (STEP_MOTOR_STEP_TIMER_CLK_HZ + (step_frequency / 2U)) / step_frequency;
    if (period > (STEP_MOTOR_MAX_ARR + 1U)){
        period = STEP_MOTOR_MAX_ARR + 1U;
    }
    if (period < 2U){
        period = 2U;
    }

    DL_TimerG_stopCounter(motor_pwm_timer);
    /*
     * DIR 必须在**停了脉冲之后**才翻。
     *
     * 反向时若上一拍的脉冲还在跑（伺服没经过到位带、直接换向就会这样），在计数器运行
     * 中改 DIR 等于让驱动器在一个 STEP 周期内看到方向跳变，那一步走哪边取决于跳变落在
     * 周期的哪个位置；驱动器还都要求 DIR 先于 STEP 边沿建立一段时间。停了再翻，同时
     * 借后面几条寄存器写作为建立时间。
     */
    StepMotor_SetDirection(speed_deg_per_s);
    DL_TimerG_setLoadValue(motor_pwm_timer, period - 1U);
    DL_TimerG_setCaptureCompareValue(motor_pwm_timer, period / 2U, motor_pwm_channel);
    /* 先配好再放开输出，避免起停瞬间挤出半个周期的毛刺。 */
    DL_TimerG_setCCPOutputDisabled(motor_pwm_timer,
                                   DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL,
                                   DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);
    DL_TimerG_startCounter(motor_pwm_timer);

    last_step_freq_hz = step_frequency;
}

/* ===== QEI 编码器 ===== */

static void StepMotor_UpdateEncoder(void){
    uint16_t raw = (uint16_t)DL_TimerG_getTimerCount(STEP_MOTOR_QEI_TIMER);
    /* 无符号相减后按有符号解释,自动处理 16 位环绕(要求单次变化 < 32768)。 */
    int16_t delta = (int16_t)(uint16_t)(raw - encoder_last_raw);

    encoder_accum += (int32_t)delta;
    encoder_last_raw = raw;
}

int32_t StepMotor_GetEncoderCount(void){
#if (STEP_MOTOR_ENCODER_INVERT != 0U)
    return -encoder_accum;
#else
    return encoder_accum;
#endif
}

float StepMotor_CountsToDeg(int32_t counts){
    if (STEP_MOTOR_ENCODER_COUNTS_PER_REV <= 0.0f){
        return 0.0f;
    }

    return ((float)counts * 360.0f) / STEP_MOTOR_ENCODER_COUNTS_PER_REV;
}

/* ===== 限幅 ===== */

/*
 * 全驱动唯一的位置限幅点。越界目标夹到边界而不是报错——上层多半是算出了一个
 * 够不着的目标,走到边界停住是它想要的行为,拒绝执行反而更难处理。
 */
static int32_t StepMotor_ClampCount(int32_t counts){
    if (!STEP_MOTOR_LIMIT_ON){
        return counts;
    }

    if (counts > STEP_MOTOR_ENC_SOFT_MAX_COUNTS){
        return STEP_MOTOR_ENC_SOFT_MAX_COUNTS;
    }

    if (counts < STEP_MOTOR_ENC_SOFT_MIN_COUNTS){
        return STEP_MOTOR_ENC_SOFT_MIN_COUNTS;
    }

    return counts;
}

/* ===== 初始化 ===== */

BSP_STATUS StepMotor_Init(void){
    servo_speed_deg_s = 0.0f;
    speed_limit_deg_s = STEP_MOTOR_SERVO_DEFAULT_SPEED_LIMIT_DEG_S;
    servo_kp_s_inv = STEP_MOTOR_SERVO_KP;
    last_step_freq_hz = 0U;
    step_tick_last_ms = 0U;
    step_tick_interval_ms = 0U;
    step_tick_max_interval_ms = 0U;
    step_tick_have_last = false;

    DL_GPIO_clearPins(motor_dir_port, motor_dir_pin);
    /*
     * 上电即失能:此刻摆杆的位置固件并不知道,通电就可能被误指令顶上机械死点。
     * 失能还让摆杆自重落到固定一端,给出一个可重复的位置参考——下面把编码器
     * 在这里清零,整套限位坐标系就是以它为原点建立的。
     */
    StepMotor_WriteEnablePin(false);
    driver_enabled = false;
    DL_TimerG_startCounter(motor_pwm_timer);
    StepMotor_DisablePulse();

    /* SysConfig 生成的 SYSCFG_DL_SMotor_QEI_init() 只做到 configQEI + enableClock,
     * 未调 startCounter;按 DL_Timer_configQEI 的文档要求在此补齐,否则不会计数。 */
    DL_TimerG_startCounter(STEP_MOTOR_QEI_TIMER);
    DL_TimerG_setTimerCount(STEP_MOTOR_QEI_TIMER, 0U);
    encoder_last_raw = 0U;
    encoder_accum    = 0;

    /* 目标 = 0 = 刚清零的当前位置:误差为零,伺服不会自己动起来。 */
    target_counts   = 0;
    servo_at_target = true;

    guard_state = STEP_MOTOR_LIMIT_ON ? STEP_MOTOR_GUARD_OK : STEP_MOTOR_GUARD_DISABLED;
    guard_recover_start_ms = 0U;

    /*
     * 抬升在这里只做"上膛":此刻 SysTick 多半还没放开(App_Init 最后才 EnableTick),
     * BSP_Time_GetMs() 取不到会走的时基,所以起算时刻推迟到第一次 Tick。
     */
#if (STEP_MOTOR_STARTUP_LIFT_ENABLED != 0U)
    startup_state = STARTUP_WAIT;
    /*
     * 目标要夹两道:
     *  1) 机械极限 HARD_MIN/MAX——那是物理事实,与限位开关无关。少了这道,
     *     限位没启用时一个填错的目标会把摆杆直接顶上死点;
     *  2) 软限位——限位启用时再向内收一层。
     */
    startup_target_counts = (int32_t)STEP_MOTOR_STARTUP_LIFT_TARGET_COUNTS;
    if (startup_target_counts > (int32_t)STEP_MOTOR_ENC_HARD_MAX_COUNTS){
        startup_target_counts = (int32_t)STEP_MOTOR_ENC_HARD_MAX_COUNTS;
    }
    if (startup_target_counts < (int32_t)STEP_MOTOR_ENC_HARD_MIN_COUNTS){
        startup_target_counts = (int32_t)STEP_MOTOR_ENC_HARD_MIN_COUNTS;
    }
    startup_target_counts = StepMotor_ClampCount(startup_target_counts);
#else
    startup_state = STARTUP_DISABLED;
    startup_target_counts = 0;
#endif
    startup_phase_start_ms = 0U;
    startup_clock_started  = false;

    return BSP_STATUS_OK;
}

/* ===== 驱动器使能 ===== */

BSP_STATUS StepMotor_SetEnabled(bool enable){
    /*
     * 失能→使能的瞬间把目标同步到当前实测位置。
     *
     * 断电期间摆杆会自重回落、也可能被人推过,重新通电时若还留着断电前的老目标,
     * 伺服下一拍就会算出一个大误差、以速度上限猛地把摆杆拽回去——这既吓人又容易
     * 撞机械。同步之后误差为零,通电只恢复保持力矩,要动得由上层重新下位置指令。
     */
    if (enable && !driver_enabled){
        target_counts = StepMotor_GetEncoderCount();
    }

    StepMotor_WriteEnablePin(enable);
    driver_enabled = enable;
    return BSP_STATUS_OK;
}

bool StepMotor_IsEnabled(void){
    return driver_enabled;
}

/* ===== 位置指令 ===== */

/* 停脉冲并把诊断量清零。不动目标,也不动使能。 */
static void StepMotor_HaltPulse(void){
    StepMotor_DisablePulse();
    servo_speed_deg_s = 0.0f;
    last_step_freq_hz = 0U;
}

/*
 * 目标登记的唯一实现。from_auto 区分调用来源:上电抬升期间目标由状态机独占,
 * 外部指令直接被拒(BUSY),否则上层一个 MoveTo 就把抬升目标冲掉了。
 */
static BSP_STATUS StepMotor_MoveToCountInternal(int32_t counts, bool from_auto){
    if (!from_auto && StepMotor_IsStartupBusy()){
        return BSP_STATUS_BUSY;
    }

    /* 下位置指令就是要动:顺手通电,调用方不必记得先使能。
     * 必须在赋目标之前——SetEnabled 会在复能时把目标同步成当前位置。 */
    (void)StepMotor_SetEnabled(true);

    target_counts = StepMotor_ClampCount(counts);
    /*
     * 新目标一律清掉到位状态,回差只在"停住不动"时挡漂移,不能挡指令——
     * 不清的话,一条小于 SERVO_RESUME_COUNTS 的点动指令会被当成漂移直接忽略。
     */
    servo_at_target = false;
    return BSP_STATUS_OK;
}

BSP_STATUS StepMotor_MoveToCount(int32_t counts){
    return StepMotor_MoveToCountInternal(counts, false);
}

BSP_STATUS StepMotor_Stop(void){
    /* 目标钉在当前实测位置:下一拍伺服算出的误差就是 0,不会又走起来。 */
    target_counts = StepMotor_GetEncoderCount();
    /* 就地停住按定义就是"到了",顺手置位,免得下一拍前 IsAtTarget() 还报未到位。 */
    servo_at_target = true;
    StepMotor_HaltPulse();
    return BSP_STATUS_OK;
}

BSP_STATUS StepMotor_SetSpeedLimit(float max_speed_deg_per_s){
    if (max_speed_deg_per_s <= 0.0f){
        return BSP_STATUS_INVALID_ARG;
    }

    if (max_speed_deg_per_s > STEP_MOTOR_MAX_SPEED_DEG_S){
        max_speed_deg_per_s = STEP_MOTOR_MAX_SPEED_DEG_S;
    }

    speed_limit_deg_s = max_speed_deg_per_s;
    return BSP_STATUS_OK;
}

BSP_STATUS StepMotor_SetServoGain(float kp_s_inv){
    if ((kp_s_inv <= 0.0f) || (kp_s_inv > STEP_MOTOR_SERVO_KP_MAX)){
        return BSP_STATUS_INVALID_ARG;
    }
    servo_kp_s_inv = kp_s_inv;
    return BSP_STATUS_OK;
}

float StepMotor_GetServoGain(void){
    return servo_kp_s_inv;
}

int32_t StepMotor_GetTargetCount(void){
    return target_counts;
}

int32_t StepMotor_GetPositionErrorCount(void){
    return target_counts - StepMotor_GetEncoderCount();
}

bool StepMotor_IsAtTarget(void){
    /* 直接报伺服的到位状态,不另算一遍——回差带下"算一遍"必然与伺服的判断不一致。 */
    return servo_at_target;
}

/* ===== 位置伺服 ===== */

static void StepMotor_ServoTick(void){
    /* 驱动器没通电时出脉冲没有意义。 */
    if (!driver_enabled){
        if (servo_speed_deg_s != 0.0f){
            StepMotor_HaltPulse();
        }
        return;
    }

    int32_t error   = StepMotor_GetPositionErrorCount();
    int32_t abs_err = (error < 0) ? -error : error;

    /*
     * 到位判定带回差:进门看紧阈值,出门看松阈值。
     *
     * 已到位时要漂过 SERVO_RESUME_COUNTS 才重新出脉冲——摆杆带重力负载,停脉冲后
     * 靠保持力矩顶着会下沉一点,单阈值下这一点下沉就会触发补偿,补完又沉,末端形成
     * 极限环。分开之后"停多准"由 POSITION_TOLERANCE_COUNTS 决定、"漂多少才动"由
     * SERVO_RESUME_COUNTS 决定,收紧前者不再以抖动为代价。
     */
    if (servo_at_target){
        if (abs_err <= STEP_MOTOR_SERVO_RESUME_COUNTS){
            /* 到位:停脉冲。保持通电,靠保持力矩把摆杆按在目标上。 */
            if (servo_speed_deg_s != 0.0f){
                StepMotor_HaltPulse();
            }
            return;
        }
        servo_at_target = false;
    } else if (abs_err <= STEP_MOTOR_POSITION_TOLERANCE_COUNTS){
        servo_at_target = true;
        StepMotor_HaltPulse();
        return;
    }

    /* 比例律:误差大时饱和到速度上限,接近目标自然减速。 */
    float speed = servo_kp_s_inv * StepMotor_CountsToDeg(error);

    if (speed > speed_limit_deg_s){
        speed = speed_limit_deg_s;
    } else if (speed < -speed_limit_deg_s){
        speed = -speed_limit_deg_s;
    }

    /* 末端提速到下限:否则会以极低频率蠕动,还可能落进 MIN_STEP_FREQ_HZ 死区。 */
    if (speed > 0.0f && speed < STEP_MOTOR_SERVO_MIN_SPEED_DEG_S){
        speed = STEP_MOTOR_SERVO_MIN_SPEED_DEG_S;
    } else if (speed < 0.0f && speed > -STEP_MOTOR_SERVO_MIN_SPEED_DEG_S){
        speed = -STEP_MOTOR_SERVO_MIN_SPEED_DEG_S;
    }

    StepMotor_ApplySpeed(speed);
    servo_speed_deg_s = speed;
}

/* ===== 上电抬升 ===== */

/* 抬升收场:停住,按 HOLD 决定是否留保持力矩。 */
static void StepMotor_StartupFinish(STARTUP_STATE final_state){
    (void)StepMotor_Stop();

    if (final_state == STARTUP_FAULT){
        /* 抬不上去多半是卡住或丢步,继续通电顶着没有意义,还会一直发热。 */
        (void)StepMotor_SetEnabled(false);
    } else {
#if (STEP_MOTOR_STARTUP_LIFT_HOLD == 0U)
        (void)StepMotor_SetEnabled(false);
#endif
    }

    startup_state = final_state;
}

/*
 * 上电抬升状态机。只管"设目标 + 判到位 + 超时",实际运动交给伺服——
 * 这正是位置式接口带来的简化。
 */
static void StepMotor_StartupTick(uint32_t now_ms){
    if (!StepMotor_IsStartupBusy()){
        return;
    }

    /* 时基在 Init 时还没走(EnableTick 在 App_Init 最后),起算推迟到这里的第一拍。 */
    if (!startup_clock_started){
        startup_phase_start_ms = now_ms;
        startup_clock_started  = true;
        return;
    }

    if (startup_state == STARTUP_WAIT){
        if ((now_ms - startup_phase_start_ms) < STEP_MOTOR_STARTUP_LIFT_DELAY_MS){
            return;
        }

        startup_state          = STARTUP_LIFTING;
        startup_phase_start_ms = now_ms;   /* 超时从真正开走起算。 */
        /* 抬升速度独立设定,免得上一次跑剩的速度上限影响起步。 */
        (void)StepMotor_SetSpeedLimit(STEP_MOTOR_STARTUP_LIFT_SPEED_DEG_S);
        /* from_auto:抬升期间外部 MoveTo 会被拒,这一发必须走内部通道。 */
        (void)StepMotor_MoveToCountInternal(startup_target_counts, true);
        return;
    }

    /* STARTUP_LIFTING */
    int32_t error = startup_target_counts - StepMotor_GetEncoderCount();
    if ((error <= STEP_MOTOR_STARTUP_LIFT_TOLERANCE_COUNTS) &&
        (error >= -STEP_MOTOR_STARTUP_LIFT_TOLERANCE_COUNTS)){
        StepMotor_StartupFinish(STARTUP_DONE);
        return;
    }

    /* 走不到 = 卡住 / 丢步 / 方向反了 / 目标填错,再顶下去只会更糟。 */
    if ((now_ms - startup_phase_start_ms) >= STEP_MOTOR_STARTUP_LIFT_TIMEOUT_MS){
        StepMotor_StartupFinish(STARTUP_FAULT);
    }
}

void StepMotor_AbortStartup(void){
    if (!StepMotor_IsStartupBusy()){
        return;
    }

    /* 就地停住,不动使能:抬到一半放弃时摆杆多半正需要保持力矩撑着。 */
    (void)StepMotor_Stop();
    startup_state = STARTUP_DISABLED;
}

/* ===== 限位守护 ===== */

/*
 * 编码器限位判定。抬升期间跳过——目标已事先夹进行程,让守护同时插手只会
 * 两个自动动作互相拉扯;抬升自己有超时兜底。
 */
static void StepMotor_LimitTick(uint32_t now_ms){
    if (!STEP_MOTOR_LIMIT_ON){
        guard_state = STEP_MOTOR_GUARD_DISABLED;
        return;
    }

    if (StepMotor_IsStartupBusy()){
        return;
    }

    /* FAULT 是终态:不查明原因就自动重试,只会让摆杆再往死点顶一次。 */
    if (guard_state == STEP_MOTOR_GUARD_FAULT){
        return;
    }

    int32_t counts = StepMotor_GetEncoderCount();
    bool in_limit  = (counts <= STEP_MOTOR_ENC_SOFT_MAX_COUNTS) &&
                     (counts >= STEP_MOTOR_ENC_SOFT_MIN_COUNTS);

    if (StepMotor_IsGuardBusy()){
        if (in_limit){
            /* 回到界内即收工。目标停在滞回点上,伺服会把它保持住。 */
            guard_state = STEP_MOTOR_GUARD_OK;
            return;
        }

        /* 走不回来 = 编码器断线 / 机械卡死 / 方向极性反了,继续走只会更糟。 */
        if ((now_ms - guard_recover_start_ms) >= STEP_MOTOR_GUARD_RECOVER_TIMEOUT_MS){
            (void)StepMotor_Stop();
            (void)StepMotor_SetEnabled(false);
            guard_state = STEP_MOTOR_GUARD_FAULT;
        }
        return;
    }

    if (in_limit){
        guard_state = STEP_MOTOR_GUARD_OK;
        return;
    }

    /*
     * 已越界。注意:目标进门就被限幅夹过,不可能指向界外——能走到这里说明是
     * **实测位置**越了界,即丢步、手推或机械打滑。
     */
    if (!driver_enabled){
        /*
         * 断电状态下没有力矩,纠正无从谈起——多半正是操作者主动失能在手推摆杆
         * 量机械行程,那是本驱动推荐的量法。只挂 STALLED 让上层看见,不擅自上电。
         */
        guard_state = STEP_MOTOR_GUARD_STALLED;
        return;
    }

    /*
     * 纠正 = 把目标设到界内的滞回点,剩下的交给伺服。
     * 用低速上限慢慢挪回来:越界纠正是异常处置,慢比快安全,也不容易冲过头。
     */
    bool from_max_side = (counts > STEP_MOTOR_ENC_SOFT_MAX_COUNTS);
    guard_state = from_max_side ? STEP_MOTOR_GUARD_RECOVER_NEG
                                : STEP_MOTOR_GUARD_RECOVER_POS;
    guard_recover_start_ms = now_ms;
    (void)StepMotor_SetSpeedLimit(STEP_MOTOR_GUARD_RECOVER_SPEED_DEG_S);
    (void)StepMotor_MoveToCountInternal(
        from_max_side
            ? (STEP_MOTOR_ENC_SOFT_MAX_COUNTS - STEP_MOTOR_GUARD_RECOVER_HYST_COUNTS)
            : (STEP_MOTOR_ENC_SOFT_MIN_COUNTS + STEP_MOTOR_GUARD_RECOVER_HYST_COUNTS),
        true);
}

STEP_MOTOR_GUARD_STATE StepMotor_GetGuardState(void){
    return guard_state;
}

/* ===== 周期入口 ===== */

void StepMotor_Tick(uint32_t now_ms){
    if (step_tick_have_last){
        step_tick_interval_ms = now_ms - step_tick_last_ms;
        if (step_tick_interval_ms > step_tick_max_interval_ms){
            step_tick_max_interval_ms = step_tick_interval_ms;
        }
    } else{
        step_tick_have_last = true;
    }
    step_tick_last_ms = now_ms;

    /* 采样必须最先且无条件做:后续判断都基于最新计数,也是 QEI 防环绕的硬要求。 */
    StepMotor_UpdateEncoder();

    /* 两个自动动作只负责改目标,统一由伺服驱动出脉冲。 */
    StepMotor_StartupTick(now_ms);
    StepMotor_LimitTick(now_ms);
    StepMotor_ServoTick();
}

/* ===== 诊断 ===== */

float StepMotor_GetSpeed(void){
    return servo_speed_deg_s;
}

uint32_t StepMotor_GetStepFrequencyHz(void){
    return last_step_freq_hz;
}

uint32_t StepMotor_GetPwmLoadValue(void){
    return DL_TimerG_getLoadValue(motor_pwm_timer);
}

uint32_t StepMotor_GetPwmCompareValue(void){
    return DL_TimerG_getCaptureCompareValue(motor_pwm_timer, motor_pwm_channel);
}

uint32_t StepMotor_GetPwmCounterValue(void){
    return DL_TimerG_getTimerCount(motor_pwm_timer);
}

uint32_t StepMotor_GetTickIntervalMs(void){
    return step_tick_interval_ms;
}

uint32_t StepMotor_GetMaxTickIntervalMs(void){
    return step_tick_max_interval_ms;
}
