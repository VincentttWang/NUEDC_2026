/**
 * @file  hall_encoder.h
 * @brief BSP 霍尔编码器接口。
 */
#ifndef HALL_ENCODER_H
#define HALL_ENCODER_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HALL_ENCODER_A_PORT
#define HALL_ENCODER_A_PORT Motor_IO_E1A_PORT
#endif

#ifndef HALL_ENCODER_A_PIN
#define HALL_ENCODER_A_PIN Motor_IO_E1A_PIN
#endif

#ifndef HALL_ENCODER_B_PORT
#define HALL_ENCODER_B_PORT Motor_IO_E1A_PORT
#endif

#ifndef HALL_ENCODER_B_PIN
#define HALL_ENCODER_B_PIN Motor_IO_E2A_PIN
#endif

#ifndef HALL_ENCODER_GPIO_IRQN
#define HALL_ENCODER_GPIO_IRQN GPIOA_INT_IRQn
#endif

#ifndef HALL_ENCODER_SAMPLE_TIMER
#define HALL_ENCODER_SAMPLE_TIMER TIMER_0_INST
#endif

#ifndef HALL_ENCODER_SAMPLE_TIMER_IRQN
#define HALL_ENCODER_SAMPLE_TIMER_IRQN TIMER_0_INST_INT_IRQN
#endif

#ifndef HALL_ENCODER_PI
#define HALL_ENCODER_PI 3.1415926f
#endif

#ifndef HALL_ENCODER_PPR
#define HALL_ENCODER_PPR 13.0f
#endif

#ifndef HALL_ENCODER_REDUCTION_RATIO
#define HALL_ENCODER_REDUCTION_RATIO 28.0f
#endif

#ifndef HALL_ENCODER_WHEEL_DIAMETER_M
#define HALL_ENCODER_WHEEL_DIAMETER_M 0.065f
#endif

#ifndef HALL_ENCODER_SAMPLE_PERIOD_S
#define HALL_ENCODER_SAMPLE_PERIOD_S 0.01f
#endif

#ifndef HALL_ENCODER_DISTANCE_SCALE
#define HALL_ENCODER_DISTANCE_SCALE 1.05f
#endif

/**
 * @brief 霍尔编码器方向。
 */
typedef enum {
    HALL_ENCODER_DIR_FORWARD = 0,
    HALL_ENCODER_DIR_REVERSE
} HALL_ENCODER_DIR;

/**
 * @brief 初始化霍尔编码器计数和采样状态。
 */
BSP_STATUS HallEncoder_Init(void);

/**
 * @brief 处理编码器 GPIO 中断。
 * @param gpio_status GPIO 中断状态寄存器值。
 */
void HallEncoder_HandleGpioIrq(uint32_t gpio_status);

/**
 * @brief 更新一次速度采样。
 * @note 通常由固定周期定时器中断调用。
 */
void HallEncoder_UpdateSample(void);

/**
 * @brief 获取累计编码器计数。
 */
int32_t HallEncoder_GetCount(void);

/**
 * @brief 获取最近判断的编码器方向。
 */
HALL_ENCODER_DIR HallEncoder_GetDir(void);

/**
 * @brief 获取估算线速度，单位 m/s。
 */
float HallEncoder_GetSpeed(void);

/**
 * @brief 获取累计距离估计，单位 m。
 */
float HallEncoder_GetDistance(void);

/**
 * @brief 清零计数、速度和距离状态。
 */
void HallEncoder_Reset(void);

/**
 * @brief 只清零累计距离，不清零编码器原始计数。
 */
void HallEncoder_ResetDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* HALL_ENCODER_H */
