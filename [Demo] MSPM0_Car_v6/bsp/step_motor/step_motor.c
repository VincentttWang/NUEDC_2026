/**
 * @file  step_motor.c
 * @brief BSP 步进电机开环控制实现。
 */
#include "step_motor.h"
#include "bsp_time.h"
#include <stdbool.h>

typedef struct {
    GPIO_Regs *dir_port;
    uint32_t dir_pin;
    GPIO_Regs *en_port;
    uint32_t en_pin;
    GPTIMER_Regs *pwm_timer;
    DL_TIMER_CC_INDEX pwm_channel;
    bool positive_dir_high;
    bool enable_high;
} STEP_MOTOR_HW_CONFIG;

typedef struct {
    float speed_deg_per_s;
    float estimated_position_deg;
    uint32_t last_update_ms;
    bool update_started;
} STEP_MOTOR_STATE;

static const STEP_MOTOR_HW_CONFIG s_step_motor_hw[STEP_MOTOR_CHANNEL_MAX] = {
    [STEP_MOTOR_CHANNEL_YAW] = {
        .dir_port = STEP_MOTOR_YAW_DIR_PORT,
        .dir_pin = STEP_MOTOR_YAW_DIR_PIN,
        .en_port = STEP_MOTOR_YAW_EN_PORT,
        .en_pin = STEP_MOTOR_YAW_EN_PIN,
        .pwm_timer = STEP_MOTOR_YAW_PWM_TIMER,
        .pwm_channel = STEP_MOTOR_YAW_PWM_CHANNEL,
        .positive_dir_high = (STEP_MOTOR_YAW_POSITIVE_DIR_HIGH != 0U),
        .enable_high = (STEP_MOTOR_YAW_ENABLE_HIGH != 0U),
    },
    [STEP_MOTOR_CHANNEL_PITCH] = {
        .dir_port = STEP_MOTOR_PITCH_DIR_PORT,
        .dir_pin = STEP_MOTOR_PITCH_DIR_PIN,
        .en_port = STEP_MOTOR_PITCH_EN_PORT,
        .en_pin = STEP_MOTOR_PITCH_EN_PIN,
        .pwm_timer = STEP_MOTOR_PITCH_PWM_TIMER,
        .pwm_channel = STEP_MOTOR_PITCH_PWM_CHANNEL,
        .positive_dir_high = (STEP_MOTOR_PITCH_POSITIVE_DIR_HIGH != 0U),
        .enable_high = (STEP_MOTOR_PITCH_ENABLE_HIGH != 0U),
    },
};

static STEP_MOTOR_STATE s_step_motor_state[STEP_MOTOR_CHANNEL_MAX];
static STEP_MOTOR_POSITION_LIMIT s_step_motor_pitch_limit = {
    .min_deg = STEP_MOTOR_PITCH_MIN_POSITION_DEG,
    .max_deg = STEP_MOTOR_PITCH_MAX_POSITION_DEG,
};

static void StepMotor_DisablePulse(const STEP_MOTOR_HW_CONFIG *hw);

static bool StepMotor_IsValidChannel(STEP_MOTOR_CHANNEL channel){
    return channel < STEP_MOTOR_CHANNEL_MAX;
}

static float StepMotor_Abs(float value){
    return (value < 0.0f) ? -value : value;
}

static float StepMotor_LimitSpeed(float speed_deg_per_s){
    if (speed_deg_per_s > STEP_MOTOR_MAX_SPEED_DEG_S){
        return STEP_MOTOR_MAX_SPEED_DEG_S;
    }

    if (speed_deg_per_s < -STEP_MOTOR_MAX_SPEED_DEG_S){
        return -STEP_MOTOR_MAX_SPEED_DEG_S;
    }

    return speed_deg_per_s;
}

static float StepMotor_LimitPitchSpeedByPosition(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s){
    if (channel != STEP_MOTOR_CHANNEL_PITCH){
        return speed_deg_per_s;
    }

    float position_deg = s_step_motor_state[STEP_MOTOR_CHANNEL_PITCH].estimated_position_deg;

    if (position_deg >= s_step_motor_pitch_limit.max_deg && speed_deg_per_s > 0.0f){
        return 0.0f;
    }

    if (position_deg <= s_step_motor_pitch_limit.min_deg && speed_deg_per_s < 0.0f){
        return 0.0f;
    }

    return speed_deg_per_s;
}

static uint32_t StepMotor_LimitPitchDurationMs(STEP_MOTOR_CHANNEL channel,
                                               float speed_deg_per_s,
                                               uint32_t duration_ms){
    if (channel != STEP_MOTOR_CHANNEL_PITCH){
        return duration_ms;
    }

    if (speed_deg_per_s == 0.0f){
        return 0U;
    }

    float position_deg = s_step_motor_state[STEP_MOTOR_CHANNEL_PITCH].estimated_position_deg;
    float remain_deg = (speed_deg_per_s > 0.0f) ?
        (s_step_motor_pitch_limit.max_deg - position_deg) :
        (position_deg - s_step_motor_pitch_limit.min_deg);

    if (remain_deg <= 0.0f){
        return 0U;
    }

    uint32_t allowed_ms = (uint32_t)((remain_deg / StepMotor_Abs(speed_deg_per_s)) * 1000.0f);
    return (allowed_ms < duration_ms) ? allowed_ms : duration_ms;
}

static void StepMotor_ClampPitchPosition(void){
    STEP_MOTOR_STATE *state = &s_step_motor_state[STEP_MOTOR_CHANNEL_PITCH];

    if (state->estimated_position_deg > s_step_motor_pitch_limit.max_deg){
        state->estimated_position_deg = s_step_motor_pitch_limit.max_deg;
        StepMotor_DisablePulse(&s_step_motor_hw[STEP_MOTOR_CHANNEL_PITCH]);
        state->speed_deg_per_s = 0.0f;
    } else if (state->estimated_position_deg < s_step_motor_pitch_limit.min_deg){
        state->estimated_position_deg = s_step_motor_pitch_limit.min_deg;
        StepMotor_DisablePulse(&s_step_motor_hw[STEP_MOTOR_CHANNEL_PITCH]);
        state->speed_deg_per_s = 0.0f;
    }
}

static uint32_t StepMotor_GetStepFrequency(float speed_deg_per_s){
    float abs_speed = StepMotor_Abs(speed_deg_per_s);

    if (abs_speed <= 0.0f || STEP_MOTOR_STEP_ANGLE_DEG <= 0.0f || STEP_MOTOR_MICROSTEP <= 0.0f){
        return 0U;
    }

    return (uint32_t)((abs_speed / STEP_MOTOR_STEP_ANGLE_DEG) * STEP_MOTOR_MICROSTEP);
}

static void StepMotor_SetDirection(const STEP_MOTOR_HW_CONFIG *hw, float speed_deg_per_s){
    bool set_high = hw->positive_dir_high;

    if (speed_deg_per_s < 0.0f){
        set_high = !set_high;
    }

    if (set_high){
        DL_GPIO_setPins(hw->dir_port, hw->dir_pin);
    } else{
        DL_GPIO_clearPins(hw->dir_port, hw->dir_pin);
    }
}

static void StepMotor_SetEnable(const STEP_MOTOR_HW_CONFIG *hw, bool enable){
    bool set_high = hw->enable_high ? enable : !enable;

    if (set_high){
        DL_GPIO_setPins(hw->en_port, hw->en_pin);
    } else{
        DL_GPIO_clearPins(hw->en_port, hw->en_pin);
    }
}

static void StepMotor_DisablePulse(const STEP_MOTOR_HW_CONFIG *hw){
    DL_TimerG_setCaptureCompareValue(hw->pwm_timer, 0U, hw->pwm_channel);
}

static BSP_STATUS StepMotor_ApplySpeed(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s){
    const STEP_MOTOR_HW_CONFIG *hw = &s_step_motor_hw[channel];
    uint32_t step_frequency = StepMotor_GetStepFrequency(speed_deg_per_s);

    if (step_frequency == 0U){
        StepMotor_DisablePulse(hw);
        return BSP_STATUS_OK;
    }

    StepMotor_SetDirection(hw, speed_deg_per_s);

    uint32_t denominator = step_frequency * STEP_MOTOR_TIMER_PRESCALER_FACTOR;
    if (denominator == 0U){
        return BSP_STATUS_INVALID_ARG;
    }

    uint32_t arr = (STEP_MOTOR_TIMER_CLOCK_HZ / denominator);
    if (arr > 0U){
        arr--;
    }

    if (arr > STEP_MOTOR_MAX_ARR){
        arr = STEP_MOTOR_MAX_ARR;
    }

    DL_TimerG_stopCounter(hw->pwm_timer);
    DL_TimerG_setLoadValue(hw->pwm_timer, arr);
    DL_TimerG_setCaptureCompareValue(hw->pwm_timer, arr / 2U, hw->pwm_channel);
    DL_TimerG_setTimerCount(hw->pwm_timer, 0U);
    DL_TimerG_startCounter(hw->pwm_timer);
    return BSP_STATUS_OK;
}

BSP_STATUS StepMotor_Init(void){
    for (uint8_t i = 0U; i < (uint8_t)STEP_MOTOR_CHANNEL_MAX; i++){
        const STEP_MOTOR_HW_CONFIG *hw = &s_step_motor_hw[i];

        s_step_motor_state[i].speed_deg_per_s = 0.0f;
        s_step_motor_state[i].estimated_position_deg = 0.0f;
        s_step_motor_state[i].last_update_ms = 0U;
        s_step_motor_state[i].update_started = false;

        DL_GPIO_clearPins(hw->dir_port, hw->dir_pin);
        StepMotor_SetEnable(hw, true);
        DL_TimerG_startCounter(hw->pwm_timer);
        StepMotor_DisablePulse(hw);
    }

    return BSP_STATUS_OK;
}

BSP_STATUS StepMotor_UpdateState(STEP_MOTOR_CHANNEL channel, uint32_t now_ms){
    if (!StepMotor_IsValidChannel(channel)){
        return BSP_STATUS_INVALID_ARG;
    }

    STEP_MOTOR_STATE *state = &s_step_motor_state[channel];

    if (!state->update_started){
        state->last_update_ms = now_ms;
        state->update_started = true;
        return BSP_STATUS_OK;
    }

    state->estimated_position_deg += state->speed_deg_per_s *
        (float)(now_ms - state->last_update_ms) * 1e-3f;
    state->last_update_ms = now_ms;

    if (channel == STEP_MOTOR_CHANNEL_PITCH){
        StepMotor_ClampPitchPosition();
    }

    return BSP_STATUS_OK;
}

BSP_STATUS StepMotor_UpdateAllState(uint32_t now_ms){
    BSP_STATUS status = BSP_STATUS_OK;

    for (uint8_t i = 0U; i < (uint8_t)STEP_MOTOR_CHANNEL_MAX; i++){
        BSP_STATUS channel_status = StepMotor_UpdateState((STEP_MOTOR_CHANNEL)i, now_ms);

        if (channel_status != BSP_STATUS_OK){
            status = channel_status;
        }
    }

    return status;
}

BSP_STATUS StepMotor_SetSpeed(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s){
    if (!StepMotor_IsValidChannel(channel)){
        return BSP_STATUS_INVALID_ARG;
    }

    (void)StepMotor_UpdateState(channel, BSP_Time_GetMs());

    float limited_speed_deg_per_s = StepMotor_LimitSpeed(speed_deg_per_s);
    limited_speed_deg_per_s = StepMotor_LimitPitchSpeedByPosition(channel, limited_speed_deg_per_s);

    BSP_STATUS status = StepMotor_ApplySpeed(channel, limited_speed_deg_per_s);
    if (status != BSP_STATUS_OK){
        return status;
    }

    s_step_motor_state[channel].speed_deg_per_s = limited_speed_deg_per_s;
    return BSP_STATUS_OK;
}

BSP_STATUS StepMotor_RunFor(STEP_MOTOR_CHANNEL channel,
                            float speed_deg_per_s,
                            uint32_t duration_ms){
    BSP_STATUS status = StepMotor_SetSpeed(channel, speed_deg_per_s);
    if (status != BSP_STATUS_OK){
        return status;
    }

    uint32_t limited_duration_ms = StepMotor_LimitPitchDurationMs(channel,
                                                                  StepMotor_GetSpeed(channel),
                                                                  duration_ms);
    BSP_DelayMs(limited_duration_ms);
    (void)StepMotor_UpdateState(channel, BSP_Time_GetMs());
    return StepMotor_Stop(channel);
}

BSP_STATUS StepMotor_Stop(STEP_MOTOR_CHANNEL channel){
    if (!StepMotor_IsValidChannel(channel)){
        return BSP_STATUS_INVALID_ARG;
    }

    (void)StepMotor_UpdateState(channel, BSP_Time_GetMs());
    StepMotor_DisablePulse(&s_step_motor_hw[channel]);
    s_step_motor_state[channel].speed_deg_per_s = 0.0f;
    return BSP_STATUS_OK;
}

BSP_STATUS StepMotor_StopAll(void){
    BSP_STATUS status = BSP_STATUS_OK;

    for (uint8_t i = 0U; i < (uint8_t)STEP_MOTOR_CHANNEL_MAX; i++){
        BSP_STATUS channel_status = StepMotor_Stop((STEP_MOTOR_CHANNEL)i);

        if (channel_status != BSP_STATUS_OK){
            status = channel_status;
        }
    }

    return status;
}

float StepMotor_GetSpeed(STEP_MOTOR_CHANNEL channel){
    if (!StepMotor_IsValidChannel(channel)){
        return 0.0f;
    }

    return s_step_motor_state[channel].speed_deg_per_s;
}

float StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL channel){
    if (!StepMotor_IsValidChannel(channel)){
        return 0.0f;
    }

    return s_step_motor_state[channel].estimated_position_deg;
}

void StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL channel){
    if (!StepMotor_IsValidChannel(channel)){
        return;
    }

    s_step_motor_state[channel].estimated_position_deg = 0.0f;
    s_step_motor_state[channel].last_update_ms = BSP_Time_GetMs();
    s_step_motor_state[channel].update_started = true;
}

BSP_STATUS StepMotor_SetPitchLimit(const STEP_MOTOR_POSITION_LIMIT *limit){
    if (limit == NULL){
        return BSP_STATUS_NULL;
    }

    if (limit->min_deg > limit->max_deg){
        return BSP_STATUS_INVALID_ARG;
    }

    s_step_motor_pitch_limit = *limit;
    (void)StepMotor_UpdateState(STEP_MOTOR_CHANNEL_PITCH, BSP_Time_GetMs());
    StepMotor_ClampPitchPosition();
    return BSP_STATUS_OK;
}

STEP_MOTOR_POSITION_LIMIT StepMotor_GetPitchLimit(void){
    return s_step_motor_pitch_limit;
}
