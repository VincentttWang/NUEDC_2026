/**
 * @file  hall_encoder.c
 * @brief BSP 霍尔编码器实现(左右双轮)。
 */
#include "hall_encoder.h"

#include <stdbool.h>

typedef struct {
    volatile int32_t pending_count;
    int32_t sample_count;
    int32_t total_count;
    HALL_ENCODER_DIR dir;
    float speed_mps;
    float distance_m;
} HALL_ENCODER_STATE;

static HALL_ENCODER_STATE hall_encoder[HALL_ENCODER_COUNT];

/* 每轮"整车前进为正"的符号(见 hall_encoder.h 说明)。 */
static const int32_t hall_encoder_dir_sign[HALL_ENCODER_COUNT] = {
    [HALL_ENCODER_LEFT]  = HALL_ENCODER_LEFT_DIR_SIGN,
    [HALL_ENCODER_RIGHT] = HALL_ENCODER_RIGHT_DIR_SIGN,
};

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

/* A 相上升沿时读 B 相电平定方向: B 高 → 正向(+1), B 低 → 反向(-1)。整车方向符号在结算时套用。 */
static void HallEncoder_Decode(HALL_ENCODER_ID id, bool b_high){
    if (b_high){
        hall_encoder[id].pending_count++;
    } else{
        hall_encoder[id].pending_count--;
    }
}

void HallEncoder_UpdateSample(void){
    for (uint8_t id = 0U; id < HALL_ENCODER_COUNT; id++){
        int32_t raw = hall_encoder[id].pending_count;
        hall_encoder[id].pending_count = 0;

        int32_t sample = raw * hall_encoder_dir_sign[id];
        hall_encoder[id].sample_count = sample;
        hall_encoder[id].total_count += sample;
        hall_encoder[id].dir = (sample >= 0) ? HALL_ENCODER_DIR_FORWARD : HALL_ENCODER_DIR_REVERSE;
        hall_encoder[id].speed_mps = HallEncoder_CountToDistance(sample) / HALL_ENCODER_SAMPLE_PERIOD_S;
        hall_encoder[id].distance_m = HallEncoder_CountToDistance(hall_encoder[id].total_count);
    }
}

int32_t HallEncoder_GetCount(HALL_ENCODER_ID id){
    return hall_encoder[id].sample_count;
}

HALL_ENCODER_DIR HallEncoder_GetDir(HALL_ENCODER_ID id){
    return hall_encoder[id].dir;
}

float HallEncoder_GetSpeed(HALL_ENCODER_ID id){
    return hall_encoder[id].speed_mps;
}

float HallEncoder_GetDistance(HALL_ENCODER_ID id){
    return hall_encoder[id].distance_m;
}

void HallEncoder_Reset(void){
    for (uint8_t id = 0U; id < HALL_ENCODER_COUNT; id++){
        hall_encoder[id].pending_count = 0;
        hall_encoder[id].sample_count = 0;
        hall_encoder[id].total_count = 0;
        hall_encoder[id].dir = HALL_ENCODER_DIR_FORWARD;
        hall_encoder[id].speed_mps = 0.0f;
        hall_encoder[id].distance_m = 0.0f;
    }
}

void HallEncoder_ResetDistance(void){
    for (uint8_t id = 0U; id < HALL_ENCODER_COUNT; id++){
        hall_encoder[id].total_count = 0;
        hall_encoder[id].distance_m = 0.0f;
    }
}

/*
 * 编码器 A 相 GPIO 边沿中断入口 (INT_GROUP1, GPIOA)。
 * 分别读取两轮 A 相的使能中断状态, 命中则读对应 B 相电平做正交解码并各自清中断
 * (见 HallEncoder_Decode)。
 */
void GROUP1_IRQHandler(void){
    uint32_t r_status = DL_GPIO_getEnabledInterruptStatus(HALL_ENCODER_R_A_PORT,
        HALL_ENCODER_R_A_PIN);
    if ((r_status & HALL_ENCODER_R_A_PIN) != 0U){
        bool b_high = (DL_GPIO_readPins(HALL_ENCODER_R_B_PORT, HALL_ENCODER_R_B_PIN) != 0U);
        HallEncoder_Decode(HALL_ENCODER_RIGHT, b_high);
        DL_GPIO_clearInterruptStatus(HALL_ENCODER_R_A_PORT, HALL_ENCODER_R_A_PIN);
    }

    uint32_t l_status = DL_GPIO_getEnabledInterruptStatus(HALL_ENCODER_L_A_PORT,
        HALL_ENCODER_L_A_PIN);
    if ((l_status & HALL_ENCODER_L_A_PIN) != 0U){
        bool b_high = (DL_GPIO_readPins(HALL_ENCODER_L_B_PORT, HALL_ENCODER_L_B_PIN) != 0U);
        HallEncoder_Decode(HALL_ENCODER_LEFT, b_high);
        DL_GPIO_clearInterruptStatus(HALL_ENCODER_L_A_PORT, HALL_ENCODER_L_A_PIN);
    }
}

/*
 * 固定周期采样定时器中断入口 (TIMER_0, 周期 = HALL_ENCODER_SAMPLE_PERIOD_S)。
 * 每次归零溢出时把本周期两轮累加的脉冲结算为速度/方向/里程 (见 UpdateSample)。
 */
void TIMER_0_INST_IRQHandler(void){
    if (DL_TimerA_getPendingInterrupt(HALL_ENCODER_SAMPLE_TIMER) == DL_TIMER_IIDX_ZERO){
        HallEncoder_UpdateSample();
    }
}
