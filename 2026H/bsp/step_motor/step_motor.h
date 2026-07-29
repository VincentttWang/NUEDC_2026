/**
 * @file  step_motor.h
 * @brief BSP 步进电机开环控制接口。
 */
#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef STEP_MOTOR_YAW_DIR_PORT
#define STEP_MOTOR_YAW_DIR_PORT SMotor_IO_PORT
#endif

#ifndef STEP_MOTOR_YAW_DIR_PIN
#define STEP_MOTOR_YAW_DIR_PIN SMotor_IO_DIR2_PIN
#endif

#ifndef STEP_MOTOR_YAW_EN_PORT
#define STEP_MOTOR_YAW_EN_PORT SMotor_IO_PORT
#endif

#ifndef STEP_MOTOR_YAW_EN_PIN
#define STEP_MOTOR_YAW_EN_PIN SMotor_IO_EN2_PIN
#endif

#ifndef STEP_MOTOR_YAW_ENABLE_HIGH
#define STEP_MOTOR_YAW_ENABLE_HIGH 1U
#endif

#ifndef STEP_MOTOR_YAW_PWM_TIMER
#define STEP_MOTOR_YAW_PWM_TIMER SMotor_2_INST
#endif

#ifndef STEP_MOTOR_YAW_PWM_CHANNEL
#define STEP_MOTOR_YAW_PWM_CHANNEL DL_TIMER_CC_1_INDEX
#endif

#ifndef STEP_MOTOR_YAW_POSITIVE_DIR_HIGH
#define STEP_MOTOR_YAW_POSITIVE_DIR_HIGH 1U
#endif

#ifndef STEP_MOTOR_PITCH_DIR_PORT
#define STEP_MOTOR_PITCH_DIR_PORT SMotor_IO_PORT
#endif

#ifndef STEP_MOTOR_PITCH_DIR_PIN
#define STEP_MOTOR_PITCH_DIR_PIN SMotor_IO_DIR1_PIN
#endif

#ifndef STEP_MOTOR_PITCH_EN_PORT
#define STEP_MOTOR_PITCH_EN_PORT SMotor_IO_PORT
#endif

#ifndef STEP_MOTOR_PITCH_EN_PIN
#define STEP_MOTOR_PITCH_EN_PIN SMotor_IO_EN1_PIN
#endif

#ifndef STEP_MOTOR_PITCH_ENABLE_HIGH
#define STEP_MOTOR_PITCH_ENABLE_HIGH 1U
#endif

#ifndef STEP_MOTOR_PITCH_PWM_TIMER
#define STEP_MOTOR_PITCH_PWM_TIMER SMotor_1_INST
#endif

#ifndef STEP_MOTOR_PITCH_PWM_CHANNEL
#define STEP_MOTOR_PITCH_PWM_CHANNEL DL_TIMER_CC_0_INDEX
#endif

#ifndef STEP_MOTOR_PITCH_POSITIVE_DIR_HIGH
#define STEP_MOTOR_PITCH_POSITIVE_DIR_HIGH 0U
#endif

#ifndef STEP_MOTOR_STEP_ANGLE_DEG
#define STEP_MOTOR_STEP_ANGLE_DEG 1.8f
#endif

#ifndef STEP_MOTOR_MICROSTEP
#define STEP_MOTOR_MICROSTEP 32.0f
#endif

#ifndef STEP_MOTOR_TIMER_CLOCK_HZ
#define STEP_MOTOR_TIMER_CLOCK_HZ 32000000U
#endif

#ifndef STEP_MOTOR_TIMER_PRESCALER_FACTOR
#define STEP_MOTOR_TIMER_PRESCALER_FACTOR (32U * 8U * 2U)
#endif

#ifndef STEP_MOTOR_MAX_ARR
#define STEP_MOTOR_MAX_ARR 65535U
#endif

#ifndef STEP_MOTOR_MAX_SPEED_DEG_S
#define STEP_MOTOR_MAX_SPEED_DEG_S 240.0f
#endif

#ifndef STEP_MOTOR_PITCH_MIN_POSITION_DEG
#define STEP_MOTOR_PITCH_MIN_POSITION_DEG (-30.0f)
#endif

#ifndef STEP_MOTOR_PITCH_MAX_POSITION_DEG
#define STEP_MOTOR_PITCH_MAX_POSITION_DEG 30.0f
#endif

/**
 * @brief 步进电机通道。
 */
typedef enum {
    STEP_MOTOR_CHANNEL_YAW = 0,
    STEP_MOTOR_CHANNEL_PITCH,
    STEP_MOTOR_CHANNEL_MAX
} STEP_MOTOR_CHANNEL;

/**
 * @brief 步进电机开环估计位置限位。
 */
typedef struct {
    /** 最小估计位置，单位 deg。 */
    float min_deg;
    /** 最大估计位置，单位 deg。 */
    float max_deg;
} STEP_MOTOR_POSITION_LIMIT;

/**
 * @brief 初始化两路步进电机输出并使能驱动器。
 */
BSP_STATUS StepMotor_Init(void);

/**
 * @brief 设置指定通道开环速度。
 * @param channel 步进电机通道。
 * @param speed_deg_per_s 速度，单位 deg/s；正负表示方向，0 表示停止脉冲。
 * @note 速度会被限制到 [-STEP_MOTOR_MAX_SPEED_DEG_S, STEP_MOTOR_MAX_SPEED_DEG_S]。
 */
BSP_STATUS StepMotor_SetSpeed(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s);

/**
 * @brief 阻塞运行指定时间后停止。
 * @param channel 步进电机通道。
 * @param speed_deg_per_s 速度，单位 deg/s。
 * @param duration_ms 运行时间，单位 ms。
 */
BSP_STATUS StepMotor_RunFor(STEP_MOTOR_CHANNEL channel,
                            float speed_deg_per_s,
                            uint32_t duration_ms);

/**
 * @brief 停止指定通道脉冲输出。
 */
BSP_STATUS StepMotor_Stop(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 停止全部通道脉冲输出。
 */
BSP_STATUS StepMotor_StopAll(void);

/**
 * @brief 按当前速度和时间差更新开环估计位置。
 * @param now_ms 当前毫秒时间戳。
 */
BSP_STATUS StepMotor_UpdateState(STEP_MOTOR_CHANNEL channel, uint32_t now_ms);

/**
 * @brief 更新全部通道开环估计位置。
 */
BSP_STATUS StepMotor_UpdateAllState(uint32_t now_ms);

/**
 * @brief 获取指定通道最近设置速度，单位 deg/s。
 */
float StepMotor_GetSpeed(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 获取指定通道开环估计位置，单位 deg。
 * @note 该值不是编码器或传感器反馈位置。
 */
float StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 将指定通道开环估计位置清零。
 * @note 不执行物理归零，也不会停止电机。
 */
void StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 设置 pitch 通道开环估计位置限位。
 */
BSP_STATUS StepMotor_SetPitchLimit(const STEP_MOTOR_POSITION_LIMIT *limit);

/**
 * @brief 获取 pitch 通道开环估计位置限位。
 */
STEP_MOTOR_POSITION_LIMIT StepMotor_GetPitchLimit(void);

#ifdef __cplusplus
}
#endif

#endif /* STEP_MOTOR_H */
