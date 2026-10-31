/**
 * @file  tb6612fng.c
 * @brief BSP TB6612FNG 双路直流电机驱动实现。
 */
#include "tb6612fng.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    GPIO_Regs *in1_port;
    uint32_t in1_pin;
    GPIO_Regs *in2_port;
    uint32_t in2_pin;
    GPTIMER_Regs *pwm_timer;
    DL_TIMER_CC_INDEX pwm_channel;
} TB6612FNG_HW_CONFIG;

typedef struct {
    float duty_percent;
    TB6612FNG_OUTPUT output;
} TB6612FNG_STATE;

static const TB6612FNG_HW_CONFIG tb6612fng_hw[TB6612FNG_CHANNEL_MAX] = {
    [TB6612FNG_CHANNEL_LEFT] = {
        .in1_port = TB6612FNG_LEFT_IN1_PORT,
        .in1_pin = TB6612FNG_LEFT_IN1_PIN,
        .in2_port = TB6612FNG_LEFT_IN2_PORT,
        .in2_pin = TB6612FNG_LEFT_IN2_PIN,
        .pwm_timer = TB6612FNG_LEFT_PWM_TIMER,
        .pwm_channel = TB6612FNG_LEFT_PWM_CHANNEL,
    },
    [TB6612FNG_CHANNEL_RIGHT] = {
        .in1_port = TB6612FNG_RIGHT_IN1_PORT,
        .in1_pin = TB6612FNG_RIGHT_IN1_PIN,
        .in2_port = TB6612FNG_RIGHT_IN2_PORT,
        .in2_pin = TB6612FNG_RIGHT_IN2_PIN,
        .pwm_timer = TB6612FNG_RIGHT_PWM_TIMER,
        .pwm_channel = TB6612FNG_RIGHT_PWM_CHANNEL,
    },
};

static TB6612FNG_STATE tb6612fng_state[TB6612FNG_CHANNEL_MAX];

static bool TB6612FNG_IsValidChannel(TB6612FNG_CHANNEL channel){
    return channel < TB6612FNG_CHANNEL_MAX;
}

static float TB6612FNG_ClampDuty(float duty_percent){
    if (duty_percent > 100.0f){
        return 100.0f;
    }

    if (duty_percent < -100.0f){
        return -100.0f;
    }

    return duty_percent;
}

static uint32_t TB6612FNG_DutyToCompare(float duty_percent){
    float abs_percent = duty_percent;

    if (abs_percent < 0.0f){
        abs_percent = -abs_percent;
    }

    return (uint32_t)((abs_percent * (float)TB6612FNG_PWM_PERIOD) / 100.0f);
}

static void TB6612FNG_SetOutputPins(const TB6612FNG_HW_CONFIG *hw, TB6612FNG_OUTPUT output){
    switch (output){
        /* IN1/IN2 极性: 与接线相反, 前进/后退对调, 使左右两轮实际转向修正 */
        case TB6612FNG_OUTPUT_FORWARD:
            DL_GPIO_clearPins(hw->in1_port, hw->in1_pin);
            DL_GPIO_setPins(hw->in2_port, hw->in2_pin);
            break;
        case TB6612FNG_OUTPUT_BACKWARD:
            DL_GPIO_setPins(hw->in1_port, hw->in1_pin);
            DL_GPIO_clearPins(hw->in2_port, hw->in2_pin);
            break;
        case TB6612FNG_OUTPUT_BRAKE:
            DL_GPIO_setPins(hw->in1_port, hw->in1_pin);
            DL_GPIO_setPins(hw->in2_port, hw->in2_pin);
            break;
        case TB6612FNG_OUTPUT_COAST:
        default:
            DL_GPIO_clearPins(hw->in1_port, hw->in1_pin);
            DL_GPIO_clearPins(hw->in2_port, hw->in2_pin);
            break;
    }
}

static BSP_STATUS TB6612FNG_Apply(TB6612FNG_CHANNEL channel,
                                  TB6612FNG_OUTPUT output,
                                  float duty_percent){
    if (!TB6612FNG_IsValidChannel(channel)){
        return BSP_STATUS_INVALID_ARG;
    }

    const TB6612FNG_HW_CONFIG *hw = &tb6612fng_hw[channel];
    float clamped_duty = TB6612FNG_ClampDuty(duty_percent);

    TB6612FNG_SetOutputPins(hw, output);
    DL_TimerG_setCaptureCompareValue(hw->pwm_timer,
                                     TB6612FNG_DutyToCompare(clamped_duty),
                                     hw->pwm_channel);

    tb6612fng_state[channel].duty_percent = clamped_duty;
    tb6612fng_state[channel].output = output;
    return BSP_STATUS_OK;
}

BSP_STATUS TB6612FNG_Init(void){
    for (uint8_t i = 0U; i < (uint8_t)TB6612FNG_CHANNEL_MAX; i++){
        const TB6612FNG_HW_CONFIG *hw = &tb6612fng_hw[i];

        DL_TimerG_startCounter(hw->pwm_timer);
        (void)TB6612FNG_Coast((TB6612FNG_CHANNEL)i);
    }

    return BSP_STATUS_OK;
}

BSP_STATUS TB6612FNG_SetDuty(TB6612FNG_CHANNEL channel, float duty_percent){
    if (!TB6612FNG_IsValidChannel(channel)){
        return BSP_STATUS_INVALID_ARG;
    }

    float clamped_duty = TB6612FNG_ClampDuty(duty_percent);
    TB6612FNG_OUTPUT output = TB6612FNG_OUTPUT_COAST;

    if (clamped_duty > 0.0f){
        output = TB6612FNG_OUTPUT_FORWARD;
    } else if (clamped_duty < 0.0f){
        output = TB6612FNG_OUTPUT_BACKWARD;
    }

    return TB6612FNG_Apply(channel, output, clamped_duty);
}

BSP_STATUS TB6612FNG_Brake(TB6612FNG_CHANNEL channel){
    return TB6612FNG_Apply(channel, TB6612FNG_OUTPUT_BRAKE, 0.0f);
}

BSP_STATUS TB6612FNG_Coast(TB6612FNG_CHANNEL channel){
    return TB6612FNG_Apply(channel, TB6612FNG_OUTPUT_COAST, 0.0f);
}

BSP_STATUS TB6612FNG_BrakeAll(void){
    BSP_STATUS status = BSP_STATUS_OK;

    for (uint8_t i = 0U; i < (uint8_t)TB6612FNG_CHANNEL_MAX; i++){
        BSP_STATUS channel_status = TB6612FNG_Brake((TB6612FNG_CHANNEL)i);

        if (channel_status != BSP_STATUS_OK){
            status = channel_status;
        }
    }

    return status;
}

BSP_STATUS TB6612FNG_CoastAll(void){
    BSP_STATUS status = BSP_STATUS_OK;

    for (uint8_t i = 0U; i < (uint8_t)TB6612FNG_CHANNEL_MAX; i++){
        BSP_STATUS channel_status = TB6612FNG_Coast((TB6612FNG_CHANNEL)i);

        if (channel_status != BSP_STATUS_OK){
            status = channel_status;
        }
    }

    return status;
}

float TB6612FNG_GetDuty(TB6612FNG_CHANNEL channel){
    if (!TB6612FNG_IsValidChannel(channel)){
        return 0.0f;
    }

    return tb6612fng_state[channel].duty_percent;
}

TB6612FNG_OUTPUT TB6612FNG_GetOutputStatus(TB6612FNG_CHANNEL channel){
    if (!TB6612FNG_IsValidChannel(channel)){
        return TB6612FNG_OUTPUT_COAST;
    }

    return tb6612fng_state[channel].output;
}
