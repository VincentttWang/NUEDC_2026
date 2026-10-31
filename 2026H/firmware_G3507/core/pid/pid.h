/**
 * @file  pid.h
 * @brief Core 层通用 PID 控制器，支持位置式和增量式两种更新模式。
 *
 * 当前消费者为 middleware/chassis 的双轮速度环、middleware/line_follow 的退化巡线
 * 控制，以及 middleware/straight_drive 的陀螺直行控制；均采用位置式（PID_MODE_POSITION）。
 * 增量式（PID_MODE_INCREMENTAL）及其专用状态字段已实现并保留，但当前暂无调用者。
 */
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 限幅关闭哨兵值。
 *
 * 用于 PID_CONFIG.integral_limit / output_limit：取该值（或任意 <= 0 值）表示该项
 * 不启用限幅。判定沿用「<= 0 关闭」语义，本宏仅令配置点自解释，不改变逻辑。
 */
#define PID_LIMIT_DISABLED 0.0f

/**
 * @brief PID 控制器计算模式。
 */
typedef enum {
    /** 位置式 PID，输出由当前误差、积分项和微分项直接计算得到。 */
    PID_MODE_POSITION = 0,
    /** 增量式 PID，输出在上一拍基础上叠加本次增量。
     *  @note 预留：当前工程暂无调用者。 */
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
    /** 积分项绝对值限幅，取 PID_LIMIT_DISABLED（或任意 <= 0 值）时不启用积分限幅。 */
    float integral_limit;
    /** 输出绝对值限幅，取 PID_LIMIT_DISABLED（或任意 <= 0 值）时不启用输出限幅。 */
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
    /** 上上拍误差，供增量式 PID 使用。@note 预留：随增量式一并保留。 */
    float prev_error;
    /** 积分累计项（位置式使用）。 */
    float integral;
    /** 微分项。 */
    float derivative;
    /** 最近一次经输出限幅后的实际输出变化量。 */
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
 * @param config 参数配置；pid 或 config 为 NULL 时不做任何操作。
 */
void PID_Init(PID_CONTROLLER *pid, const PID_CONFIG *config);

/**
 * @brief 清空 PID 运行状态，保留当前参数配置。
 * @param pid 控制器实例指针。
 */
void PID_Reset(PID_CONTROLLER *pid);

/**
 * @brief 执行一次 PID 更新。
 * @param pid 控制器实例指针。
 * @param target 目标值。
 * @param feedback 反馈值。
 * @param dt_s 控制周期，单位 s；<= 0 时微分按 0 处理。
 * @return 本次控制输出。
 */
float PID_Update(PID_CONTROLLER *pid, float target, float feedback, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
