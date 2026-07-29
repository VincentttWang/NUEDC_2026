/**
 * @file  line_tracking.h
 * @brief Core 层巡线控制策略，将灰度传感器状态转换为底盘输出。
 */
#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#include "bsp_common.h"
#include "line_follow.h"
#include "pid/pid.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LINE_TRACKING_DEFAULT_BASE_DUTY 35.0f
#define LINE_TRACKING_DEFAULT_OUTPUT_LIMIT 100.0f
#define LINE_TRACKING_DEFAULT_CORRECTION_LIMIT 60.0f
#define LINE_TRACKING_DEFAULT_DIFFERENTIAL_LIMIT 10.0f
#define LINE_TRACKING_DEFAULT_POSITION_SCALE 10.0f
#define LINE_TRACKING_ACTIVE_SENSOR_MASK 0xFFU

/**
 * @brief 巡线控制配置。
 */
typedef struct {
    /** 基础占空比，左右轮在无偏差时使用该输出。 */
    float base_duty;
    /** 传感器横向位置到控制误差的缩放系数。 */
    float sensor_position_scale;
    /** 左右轮最终输出绝对值限幅。 */
    float output_limit;
    /** 左右轮占空比差值绝对值限幅；<= 0 时不启用。 */
    float differential_limit;
    /** 巡线偏差 PID 配置。 */
    PID_CONFIG pid_config;
} LINE_TRACKING_CONFIG;

/**
 * @brief 巡线控制计算结果。
 */
typedef struct {
    /** 缩放后的巡线误差，负值表示线偏左，正值表示线偏右。 */
    float error;
    /** PID 计算得到的转向修正量。 */
    float correction;
    /** 左轮占空比输出。 */
    float left_duty;
    /** 右轮占空比输出。 */
    float right_duty;
    /** 当前有效传感器数量。 */
    uint8_t active_count;
    /** 是否未检测到线。 */
    bool line_lost;
} LINE_TRACKING_OUTPUT;

/**
 * @brief 初始化巡线控制器。
 * @param config 配置指针；传入 NULL 时使用默认配置。
 */
void LineTracking_Init(const LINE_TRACKING_CONFIG *config);

/**
 * @brief 重置巡线 PID 状态和最近输出。
 */
void LineTracking_Reset(void);

/**
 * @brief 读取当前巡线状态并驱动底盘。
 * @param dt_s 控制周期，单位 s。
 * @return BSP_STATUS_OK 表示底盘输出已更新。
 */
BSP_STATUS LineTracking_Update(float dt_s);

/**
 * @brief 基于给定传感器快照计算巡线输出，不直接驱动底盘。
 * @param sensor 灰度传感器状态快照。
 * @param dt_s 控制周期，单位 s。
 * @param out 输出结果。
 * @return BSP_STATUS_OK 表示计算成功。
 */
BSP_STATUS LineTracking_Compute(const LINE_FOLLOW_SENSOR_STATE *sensor,
                                float dt_s,
                                LINE_TRACKING_OUTPUT *out);

/**
 * @brief 获取最近一次巡线输出。
 */
LINE_TRACKING_OUTPUT LineTracking_GetOutput(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_TRACKING_H */
