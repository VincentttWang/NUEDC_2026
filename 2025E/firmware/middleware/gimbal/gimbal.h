/**
 * @file  gimbal.h
 * @brief Middleware 层云台组合服务接口。
 */
#ifndef GIMBAL_H
#define GIMBAL_H

#include "bsp_common.h"
#include "bsp/bldc/f32c_bldc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* pitch 软件限位默认值, 由无刷驱动的机械限位常量换算 (0.1° → °) */
#ifndef GIMBAL_PITCH_MIN_DEG
#define GIMBAL_PITCH_MIN_DEG ((float)BLDC_PITCH_MIN_X10 / 10.0f)
#endif

#ifndef GIMBAL_PITCH_MAX_DEG
#define GIMBAL_PITCH_MAX_DEG ((float)BLDC_PITCH_MAX_X10 / 10.0f)
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
 * @brief 初始化云台无刷电机服务 (双 F32C, yaw 速度模式、pitch 位置模式)。
 */
BSP_STATUS Gimbal_Init(void);

/**
 * @brief 开机发起 pitch 抬升到工作角 (非阻塞)。
 * @note  应在 Gimbal_Init() 之后调用一次。仅下发位置模式抬升指令后立即返回,
 *        不阻塞开机菜单; pitch 在后台抬升。进入 E2/E3 瞄准前须调用
 *        Gimbal_EnsurePitchReady() 确认到位并切回速度模式。
 */
void Gimbal_StartupElevatePitch(void);

/**
 * @brief 确保 pitch 已抬升到位并处于速度模式 (阻塞)。
 * @note  在 E2/E3 等需要 pitch 视觉闭环的任务开始前调用。若开机抬升已完成则
 *        立即返回; 否则阻塞轮询多圈角度反馈到位 (或超时), 再切回速度模式并
 *        对齐开环估计角。幂等, 且能在 pitch 状态被重置后自行重新归位。
 */
void Gimbal_EnsurePitchReady(void);

/**
 * @brief 设置 yaw/pitch 角速度 (deg/s): yaw 直接速度闭环，pitch 积分为位置设定点。
 * @note 当前仅对 pitch 做软件角度限位，yaw 不做角度限位。
 */
BSP_STATUS Gimbal_SetSpeed(float yaw_deg_s, float pitch_deg_s);

/**
 * @brief 设置 yaw/pitch 绝对目标角 (deg): 两轴均走位置模式 (前馈瞄准)。
 * @note 与 Gimbal_SetSpeed 互斥使用: 同一任务内应固定用其一, 需切换时先 Gimbal_Stop。
 *       yaw 走多圈位置模式且不设角度限位 (可连续多圈); pitch 受软件角度限位约束。
 *       目标角由 aim_solver 依据车位姿 + 已知靶心几何解算得到。
 */
BSP_STATUS Gimbal_SetAngle(float yaw_deg, float pitch_deg);

/**
 * @brief 停止云台两轴输出。
 */
BSP_STATUS Gimbal_Stop(void);

/**
 * @brief 目标丢失时: yaw 停转, pitch 停在当前位置并保持 (位置模式保持力矩)。
 * @note pitch 未就绪(仍在归位)时等价于 Gimbal_Stop()。
 */
BSP_STATUS Gimbal_HoldOnTargetLost(void);

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
