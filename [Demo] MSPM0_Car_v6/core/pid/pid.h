/**
 * @file  pid.h
 * @brief Core 层通用 PID 控制器，支持位置式和增量式两种更新模式。
 */
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID 控制器计算模式。
 */
typedef enum {
    /** 位置式 PID，输出由当前误差、积分项和微分项直接计算得到。 */
    PID_MODE_POSITION = 0,
    /** 增量式 PID，输出在上一拍基础上叠加本次增量。 */
    PID_MODE_INCREMENTAL
} PID_MODE;

/**
 * @brief PID 参数配置。
 */
typedef struct {
    /** 比例系数。 */
    float kp;
    /** 积分系数。 */
    float ki;
    /** 微分系数。 */
    float kd;
    /** 积分项绝对值限幅，<= 0 时不启用积分限幅。 */
    float integral_limit;
    /** 输出绝对值限幅，<= 0 时不启用输出限幅。 */
    float output_limit;
    /** PID 计算模式。 */
    PID_MODE mode;
} PID_CONFIG;

/**
 * @brief PID 运行状态。
 */
typedef struct {
    /** 最近一次目标值。 */
    float target;
    /** 最近一次反馈值。 */
    float feedback;
    /** 当前误差，等于 target - feedback。 */
    float error;
    /** 上一拍误差。 */
    float last_error;
    /** 上上拍误差，供增量式 PID 使用。 */
    float prev_error;
    /** 积分累计项。 */
    float integral;
    /** 微分项。 */
    float derivative;
    /** 本次增量式 PID 计算得到的输出增量。 */
    float increment;
    /** 当前输出值。 */
    float output;
} PID_STATE;

/**
 * @brief PID 控制器实例。
 */
typedef struct {
    /** 参数配置。 */
    PID_CONFIG config;
    /** 运行状态。 */
    PID_STATE state;
} PID_CONTROLLER;

/**
 * @brief 初始化 PID 控制器。
 * @param pid 控制器实例指针。
 * @param config 参数配置；传入 NULL 时使用全零配置。
 */
void PID_Init(PID_CONTROLLER *pid, const PID_CONFIG *config);

/**
 * @brief 清空 PID 运行状态，保留当前参数配置。
 * @param pid 控制器实例指针。
 */
void PID_Reset(PID_CONTROLLER *pid);

/**
 * @brief 更新 PID 参数配置并清空运行状态。
 * @param pid 控制器实例指针。
 * @param config 新参数配置；传入 NULL 时使用全零配置。
 */
void PID_SetConfig(PID_CONTROLLER *pid, const PID_CONFIG *config);

/**
 * @brief 执行一次 PID 更新。
 * @param pid 控制器实例指针。
 * @param target 目标值。
 * @param feedback 反馈值。
 * @param dt_s 控制周期，单位 s；<= 0 时微分按 0 处理。
 * @return 本次控制输出。
 */
float PID_Update(PID_CONTROLLER *pid, float target, float feedback, float dt_s);

/**
 * @brief 获取最近一次 PID 输出。
 * @param pid 控制器实例指针。
 * @return 当前输出；非法参数返回 0。
 */
float PID_GetOutput(const PID_CONTROLLER *pid);

/**
 * @brief 获取最近一次 PID 误差。
 * @param pid 控制器实例指针。
 * @return 当前误差；非法参数返回 0。
 */
float PID_GetError(const PID_CONTROLLER *pid);

/**
 * @brief 获取最近一次增量式 PID 输出增量。
 * @param pid 控制器实例指针。
 * @return 当前增量；非法参数返回 0。
 */
float PID_GetIncrement(const PID_CONTROLLER *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
