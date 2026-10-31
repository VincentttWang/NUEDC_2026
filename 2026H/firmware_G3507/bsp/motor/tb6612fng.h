/**
 * @file  tb6612fng.h
 * @brief BSP TB6612FNG 双路直流电机驱动接口。
 */
#ifndef TB6612FNG_H
#define TB6612FNG_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 引脚/定时器映射到 SysConfig 生成的板级名。 */
#define TB6612FNG_LEFT_IN1_PORT     Motor_IO_BIN1_PORT
#define TB6612FNG_LEFT_IN1_PIN      Motor_IO_BIN1_PIN
#define TB6612FNG_LEFT_IN2_PORT     Motor_IO_BIN2_PORT
#define TB6612FNG_LEFT_IN2_PIN      Motor_IO_BIN2_PIN
#define TB6612FNG_LEFT_PWM_TIMER    Motor_Left_INST
#define TB6612FNG_LEFT_PWM_CHANNEL  DL_TIMER_CC_1_INDEX

#define TB6612FNG_RIGHT_IN1_PORT    Motor_IO_AIN1_PORT
#define TB6612FNG_RIGHT_IN1_PIN     Motor_IO_AIN1_PIN
#define TB6612FNG_RIGHT_IN2_PORT    Motor_IO_AIN2_PORT
#define TB6612FNG_RIGHT_IN2_PIN     Motor_IO_AIN2_PIN
#define TB6612FNG_RIGHT_PWM_TIMER   Motor_Right_INST
#define TB6612FNG_RIGHT_PWM_CHANNEL DL_TIMER_CC_0_INDEX

/* PWM 比较周期 (占空比 100% 对应的比较值)。 */
#define TB6612FNG_PWM_PERIOD        1000U

/**
 * @brief TB6612FNG 电机通道。
 */
typedef enum {
    TB6612FNG_CHANNEL_LEFT = 0,
    TB6612FNG_CHANNEL_RIGHT,
    TB6612FNG_CHANNEL_MAX
} TB6612FNG_CHANNEL;

/**
 * @brief TB6612FNG 输出状态。
 */
typedef enum {
    /** 断开输出，电机滑行。 */
    TB6612FNG_OUTPUT_COAST = 0,
    /** 正向输出。 */
    TB6612FNG_OUTPUT_FORWARD,
    /** 反向输出。 */
    TB6612FNG_OUTPUT_BACKWARD,
    /** 主动刹车。 */
    TB6612FNG_OUTPUT_BRAKE
} TB6612FNG_OUTPUT;

/**
 * @brief 初始化 TB6612FNG 双路电机输出。
 */
BSP_STATUS TB6612FNG_Init(void);

/**
 * @brief 设置指定通道占空比。
 * @param channel 电机通道。
 * @param duty_percent 占空比百分比，正负表示方向，范围会限制到 [-100, 100]。
 */
BSP_STATUS TB6612FNG_SetDuty(TB6612FNG_CHANNEL channel, float duty_percent);

/**
 * @brief 主动刹车指定通道。
 */
BSP_STATUS TB6612FNG_Brake(TB6612FNG_CHANNEL channel);

/**
 * @brief 关闭指定通道输出，使电机滑行。
 */
BSP_STATUS TB6612FNG_Coast(TB6612FNG_CHANNEL channel);

/**
 * @brief 主动刹车全部通道。
 */
BSP_STATUS TB6612FNG_BrakeAll(void);

/**
 * @brief 关闭全部通道输出。
 */
BSP_STATUS TB6612FNG_CoastAll(void);

/**
 * @brief 获取指定通道最近设置的占空比。
 */
float TB6612FNG_GetDuty(TB6612FNG_CHANNEL channel);

/**
 * @brief 获取指定通道最近输出状态。
 */
TB6612FNG_OUTPUT TB6612FNG_GetOutputStatus(TB6612FNG_CHANNEL channel);

#ifdef __cplusplus
}
#endif

#endif /* TB6612FNG_H */
