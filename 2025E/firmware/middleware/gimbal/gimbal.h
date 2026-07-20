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
 * @brief 设置 yaw/pitch 绝对目标角 (deg), 前馈瞄准接口。
 * @param yaw_deg 逻辑 yaw 目标角 (aim_solver 几何解, [-180,180), 内部解缠可多圈)。
 * @param pitch_deg pitch 目标角, 受软件角度限位约束 (位置模式)。
 * @param yaw_rate_ff_deg_s yaw 解析速率前馈 (deg/s, 运动学闭式解, 如
 *        AimFusion_MotionRate 的 yaw_rate); 直接叠加到速度内环, 消执行滞后。
 *        无运动信息时 (静态标定/测试) 传 0。
 * @note 与 Gimbal_SetSpeed 互斥使用: 同一任务内应固定用其一, 需切换时先 Gimbal_Stop。
 *       yaw 走速度前馈级联 (MCU 位置外环 + F32C 速度内环), 不设角度限位 (可连续多圈)。
 */
BSP_STATUS Gimbal_SetAngle(float yaw_deg, float pitch_deg, float yaw_rate_ff_deg_s);

/**
 * @brief 只设置 pitch 绝对目标角 (deg, 位置模式, 受软件限位); 不动 yaw。
 * @note 供采集/look-up 抬升用 (如 F1 进入让相机先看到标靶)。
 */
BSP_STATUS Gimbal_SetPitchDeg(float pitch_deg);

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

/** @brief 获取最近一次实际下发给 yaw F32C 的速度命令，单位 RPM。 */
int16_t Gimbal_GetYawCommandRpm(void);

/**
 * @brief 阻塞读取 yaw 编码器反馈的【逻辑角】(deg): 发起反馈请求并等待新帧
 *        (超时 ~100ms 则退回最后已知反馈)。
 * @note 控制器交接等场合必须用本接口, 不可用 Gimbal_GetAngle().yaw_deg:
 *       开环估计在 SetSpeed 路径按下发值积分(物理约定, 不含 GIMBAL_YAW_DIR),
 *       与 SetAngle/aim_solver 的逻辑约定差一个符号, 跨路径混用会得到反号角;
 *       编码器反馈经 GIMBAL_YAW_DIR 换算是唯一无歧义的逻辑角来源。
 */
float Gimbal_ReadYawFeedbackDeg(void);

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
