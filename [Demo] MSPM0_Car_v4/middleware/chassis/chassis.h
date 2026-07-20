/**
 * @file  chassis.h
 * @brief Middleware 层底盘组合服务接口。
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include "bsp_common.h"
#include "hall_encoder.h"
#include "tb6612fng.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 底盘停止模式。
 */
typedef enum {
    /** 关闭 PWM，让电机自然滑行。 */
    CHASSIS_STOP_MODE_COAST = 0,
    /** IN1/IN2 同时制动，主动刹车。 */
    CHASSIS_STOP_MODE_BRAKE
} CHASSIS_STOP_MODE;

/**
 * @brief 左右轮占空比输出。
 */
typedef struct {
    /** 左轮占空比，范围通常为 [-100, 100]。 */
    float left_percent;
    /** 右轮占空比，范围通常为 [-100, 100]。 */
    float right_percent;
} CHASSIS_DUTY;

/**
 * @brief 底盘组合状态快照。
 */
typedef struct {
    /** 最近设置的左右轮占空比。 */
    CHASSIS_DUTY duty;
    /** 编码器估算速度，单位 m/s。 */
    float speed_mps;
    /** 编码器累计距离，单位 m。 */
    float distance_m;
    /** 编码器方向。 */
    HALL_ENCODER_DIR encoder_dir;
    /** 左电机驱动输出状态。 */
    TB6612FNG_OUTPUT left_output;
    /** 右电机驱动输出状态。 */
    TB6612FNG_OUTPUT right_output;
} CHASSIS_STATUS;

/**
 * @brief 初始化底盘相关 BSP 外设。
 */
BSP_STATUS Chassis_Init(void);

/**
 * @brief 设置左右轮占空比并立即输出到底盘电机。
 * @param left_percent 左轮占空比，正负表示方向。
 * @param right_percent 右轮占空比，正负表示方向。
 */
BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent);

/**
 * @brief 按指定模式停止底盘。
 */
BSP_STATUS Chassis_Stop(CHASSIS_STOP_MODE mode);

/**
 * @brief 主动刹停左右电机。
 */
BSP_STATUS Chassis_Brake(void);

/**
 * @brief 关闭左右电机输出，使其自然滑行。
 */
BSP_STATUS Chassis_Coast(void);

/**
 * @brief 更新底盘运行状态。
 */
BSP_STATUS Chassis_Update(void);

/**
 * @brief 转发 GPIO 编码器中断状态到底盘服务。
 */
void Chassis_HandleEncoderIrq(uint32_t gpio_status);

/**
 * @brief 转发编码器采样定时器中断到底盘服务。
 */
void Chassis_HandleEncoderTimerIrq(void);

/**
 * @brief 获取底盘状态快照。
 */
BSP_STATUS Chassis_GetStatus(CHASSIS_STATUS *out);

/**
 * @brief 获取最近设置的左右轮占空比。
 */
CHASSIS_DUTY Chassis_GetDuty(void);

/**
 * @brief 获取编码器估算速度，单位 m/s。
 */
float Chassis_GetSpeed(void);

/**
 * @brief 获取编码器累计距离，单位 m。
 */
float Chassis_GetDistance(void);

/**
 * @brief 获取最近编码器方向。
 */
HALL_ENCODER_DIR Chassis_GetEncoderDir(void);

/**
 * @brief 清零编码器累计距离。
 */
void Chassis_ResetDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
