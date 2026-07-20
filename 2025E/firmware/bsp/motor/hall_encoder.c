/**
 * @file  hall_encoder.c
 * @brief BSP 霍尔编码器实现。
 */
#include "hall_encoder.h"

typedef struct {
    volatile int32_t pending_count;
    int32_t sample_count;
    int32_t total_count;
    HALL_ENCODER_DIR dir;
    float speed_mps;
    float distance_m;
} HALL_ENCODER_STATE;

static HALL_ENCODER_STATE s_hall_encoder;

static float HallEncoder_CountToDistance(int32_t count){
    float revolutions = (float)count / (HALL_ENCODER_PPR * HALL_ENCODER_REDUCTION_RATIO);
    float circumference_m = HALL_ENCODER_PI * HALL_ENCODER_WHEEL_DIAMETER_M;

    return revolutions * circumference_m * HALL_ENCODER_DISTANCE_SCALE;
}

BSP_STATUS HallEncoder_Init(void){
    HallEncoder_Reset();
    NVIC_ClearPendingIRQ(HALL_ENCODER_GPIO_IRQN);
    NVIC_EnableIRQ(HALL_ENCODER_GPIO_IRQN);
    NVIC_ClearPendingIRQ(HALL_ENCODER_SAMPLE_TIMER_IRQN);
    NVIC_EnableIRQ(HALL_ENCODER_SAMPLE_TIMER_IRQN);
    return BSP_STATUS_OK;
}

void HallEncoder_HandleGpioIrq(uint32_t gpio_status){
    if ((gpio_status & HALL_ENCODER_A_PIN) == HALL_ENCODER_A_PIN){
        if (DL_GPIO_readPins(HALL_ENCODER_B_PORT, HALL_ENCODER_B_PIN) == 0U){
            s_hall_encoder.pending_count--;
        } else{
            s_hall_encoder.pending_count++;
        }
    }

    if ((gpio_status & HALL_ENCODER_B_PIN) == HALL_ENCODER_B_PIN){
        if (DL_GPIO_readPins(HALL_ENCODER_A_PORT, HALL_ENCODER_A_PIN) == 0U){
            s_hall_encoder.pending_count++;
        } else{
            s_hall_encoder.pending_count--;
        }
    }
}

void HallEncoder_UpdateSample(void){
    int32_t sample_count = s_hall_encoder.pending_count;

    s_hall_encoder.pending_count = 0;
    s_hall_encoder.sample_count = sample_count;
    s_hall_encoder.total_count += sample_count;
    s_hall_encoder.dir = (sample_count >= 0) ? HALL_ENCODER_DIR_FORWARD : HALL_ENCODER_DIR_REVERSE;
    s_hall_encoder.speed_mps = HallEncoder_CountToDistance(sample_count) / HALL_ENCODER_SAMPLE_PERIOD_S;
    s_hall_encoder.distance_m = HallEncoder_CountToDistance(s_hall_encoder.total_count);
}

int32_t HallEncoder_GetCount(void){
    return s_hall_encoder.sample_count;
}

HALL_ENCODER_DIR HallEncoder_GetDir(void){
    return s_hall_encoder.dir;
}

float HallEncoder_GetSpeed(void){
    return s_hall_encoder.speed_mps;
}

float HallEncoder_GetDistance(void){
    return s_hall_encoder.distance_m;
}

void HallEncoder_Reset(void){
    s_hall_encoder.pending_count = 0;
    s_hall_encoder.sample_count = 0;
    s_hall_encoder.total_count = 0;
    s_hall_encoder.dir = HALL_ENCODER_DIR_FORWARD;
    s_hall_encoder.speed_mps = 0.0f;
    s_hall_encoder.distance_m = 0.0f;
}

void HallEncoder_ResetDistance(void){
    s_hall_encoder.total_count = 0;
    s_hall_encoder.distance_m = 0.0f;
}

void GROUP1_IRQHandler(void){
    uint32_t gpio_status = DL_GPIO_getEnabledInterruptStatus(HALL_ENCODER_A_PORT,
        HALL_ENCODER_A_PIN | HALL_ENCODER_B_PIN);

    HallEncoder_HandleGpioIrq(gpio_status);
    DL_GPIO_clearInterruptStatus(HALL_ENCODER_A_PORT, HALL_ENCODER_A_PIN | HALL_ENCODER_B_PIN);
}

void TIMER_0_INST_IRQHandler(void){
    if (DL_TimerA_getPendingInterrupt(HALL_ENCODER_SAMPLE_TIMER) == DL_TIMER_IIDX_ZERO){
        HallEncoder_UpdateSample();
    }
}
