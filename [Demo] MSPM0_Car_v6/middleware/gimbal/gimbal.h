/**
 * @file  gimbal.h
 * @brief Middleware 层云台组合服务接口。
 */
#ifndef GIMBAL_H
#define GIMBAL_H

#include "bsp_common.h"
#include "step_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GIMBAL_PITCH_MIN_DEG
#define GIMBAL_PITCH_MIN_DEG STEP_MOTOR_PITCH_MIN_POSITION_DEG
#endif

#ifndef GIMBAL_PITCH_MAX_DEG
#define GIMBAL_PITCH_MAX_DEG STEP_MOTOR_PITCH_MAX_POSITION_DEG
#endif

/**
 * @brief 云台轴枚举。
 */
typedef enum {
    GIMBAL_AXIS_YAW = 0,
    GIMBAL_AXIS_PITCH,
    GIMBAL_AXIS_MAX
} GIMBAL_AXIS;

/**
 * @brief 云台开环估计角度。
 */
typedef struct {
    /** yaw 轴开环估计角度，单位 deg。 */
    float yaw_deg;
    /** pitch 轴开环估计角度，单位 deg。 */
    float pitch_deg;
} GIMBAL_ANGLE;

/**
 * @brief 云台速度指令或状态。
 */
typedef struct {
    /** yaw 轴速度，单位 deg/s。 */
    float yaw_deg_s;
    /** pitch 轴速度，单位 deg/s。 */
    float pitch_deg_s;
} GIMBAL_SPEED;

/**
 * @brief 云台软件限位。
 */
typedef struct {
    /** pitch 轴最小角度，单位 deg。 */
    float pitch_min_deg;
    /** pitch 轴最大角度，单位 deg。 */
    float pitch_max_deg;
} GIMBAL_LIMIT;

/**
 * @brief 云台组合状态快照。
 */
typedef struct {
    GIMBAL_ANGLE angle;
    GIMBAL_SPEED speed;
    GIMBAL_LIMIT limit;
} GIMBAL_STATUS;

/**
 * @brief 初始化云台步进电机服务。
 */
BSP_STATUS Gimbal_Init(void);

/**
 * @brief 设置 yaw/pitch 速度并立即输出到步进电机。
 * @note 当前仅对 pitch 做软件角度限位，yaw 不做角度限位。
 */
BSP_STATUS Gimbal_SetSpeed(float yaw_deg_s, float pitch_deg_s);

/**
 * @brief 停止云台两轴输出。
 */
BSP_STATUS Gimbal_Stop(void);

/**
 * @brief 更新云台开环估计状态。
 */
BSP_STATUS Gimbal_Update(void);

/**
 * @brief 获取云台状态快照。
 */
BSP_STATUS Gimbal_GetStatus(GIMBAL_STATUS *out);

/**
 * @brief 获取云台开环估计角度。
 */
GIMBAL_ANGLE Gimbal_GetAngle(void);

/**
 * @brief 获取云台最近设置的速度。
 */
GIMBAL_SPEED Gimbal_GetSpeed(void);

/**
 * @brief 将 yaw/pitch 开环估计位置清零。
 */
void Gimbal_ResetPosition(void);

/**
 * @brief 将指定轴开环估计位置清零。
 */
void Gimbal_ResetAxisPosition(GIMBAL_AXIS axis);

/**
 * @brief 设置 pitch 软件限位。
 */
BSP_STATUS Gimbal_SetLimit(const GIMBAL_LIMIT *limit);

/**
 * @brief 获取当前 pitch 软件限位。
 */
GIMBAL_LIMIT Gimbal_GetLimit(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_H */
