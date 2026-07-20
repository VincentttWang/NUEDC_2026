/**
 * @file  gimbal_tracking.h
 * @brief Core 层视觉云台跟踪控制，将 CanMV 目标点转换为云台速度输出。
 */
#ifndef GIMBAL_TRACKING_H
#define GIMBAL_TRACKING_H

#include "bsp_common.h"
#include "canmv_uart.h"
#include "common/core_types.h"
#include "pid/pid.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GIMBAL_TRACKING_DEFAULT_IMAGE_HEIGHT 240.0f
#define GIMBAL_TRACKING_DEFAULT_PAPER_WIDTH 315.0f
#define GIMBAL_TRACKING_DEFAULT_PAPER_HEIGHT 212.0f
#define GIMBAL_TRACKING_DEFAULT_CIRCLE_RADIUS 60.0f
#define GIMBAL_TRACKING_DEFAULT_LINK_TIMEOUT_MS 1000U
#define GIMBAL_TRACKING_DEFAULT_PID_KP 0.4f
#define GIMBAL_TRACKING_DEFAULT_PID_OUTPUT_LIMIT 240.0f

/**
 * @brief 视觉云台跟踪配置。
 */
typedef struct {
    /** yaw 轴 PID 配置。 */
    PID_CONFIG yaw_pid;
    /** pitch 轴 PID 配置。 */
    PID_CONFIG pitch_pid;
    /** 图像高度，单位 px；当前协议下作为图像尺寸配置保留。 */
    float image_height;
    /** 题目纸面宽度，单位 mm。 */
    float paper_width;
    /** 题目纸面高度，单位 mm。 */
    float paper_height;
    /** 默认圆轨迹半径，单位 mm。 */
    float circle_radius;
    /** yaw 输出方向系数，用于适配电机安装方向。 */
    float yaw_output_sign;
    /** pitch 输出方向系数，用于适配电机安装方向。 */
    float pitch_output_sign;
    /** K230 有效目标数据超时时间，单位 ms；超时后停止云台输出。 */
    uint32_t link_timeout_ms;
} GIMBAL_TRACKING_CONFIG;

/**
 * @brief 视觉云台跟踪运行状态。
 */
typedef struct {
    /** 当前目标点。 */
    CORE_POINT2F target;
    /** 当前激光点。 */
    CORE_POINT2F laser;
    /** 图像平面误差，等于 target - laser。 */
    CORE_POINT2F error;
    /** yaw 轴速度输出，单位 deg/s。 */
    float yaw_speed;
    /** pitch 轴速度输出，单位 deg/s。 */
    float pitch_speed;
    /** 激光点解析状态。 */
    CANMV_STATUS laser_status;
    /** 矩形目标解析状态。 */
    CANMV_STATUS rect_status;
    /** 目标点是否有效。 */
    bool target_valid;
    /** 激光点是否有效。 */
    bool laser_valid;
    /** K230 通信是否超时。 */
    bool link_timeout;
} GIMBAL_TRACKING_STATE;

/**
 * @brief 初始化视觉云台跟踪模块。
 * @param config 配置指针；传入 NULL 时使用默认配置。
 */
void GimbalTracking_Init(const GIMBAL_TRACKING_CONFIG *config);

/**
 * @brief 重置 PID 和最近跟踪状态。
 */
void GimbalTracking_Reset(void);

/**
 * @brief 从 CanMV 激光中心数据更新云台跟踪输出。
 * @param dt_s 控制周期，单位 s。
 */
BSP_STATUS GimbalTracking_UpdateLaserCenter(float dt_s);

/**
 * @brief 根据 CanMV 矩形角点和圆周角度计算圆上目标并跟踪。
 * @param edge_index 使用的矩形边/点索引，具体含义由任务流程约定。
 * @param angle_offset_deg 圆轨迹角度偏移，单位 deg。
 * @param dt_s 控制周期，单位 s。
 */
BSP_STATUS GimbalTracking_UpdateRectCircle(int32_t edge_index,
                                           float angle_offset_deg,
                                           float dt_s);

/**
 * @brief 根据 CanMV 矩形角点计算矩形中心并跟踪。
 * @param dt_s 控制周期，单位 s。
 */
BSP_STATUS GimbalTracking_UpdateRectCenter(float dt_s);

/**
 * @brief 检查当前是否已经收到有效矩形角点。
 */
bool GimbalTracking_IsRectValid(void);

/**
 * @brief 跟踪指定目标点和激光点。
 * @param target 目标点。
 * @param laser 激光点。
 * @param dt_s 控制周期，单位 s。
 */
BSP_STATUS GimbalTracking_TrackPoints(CORE_POINT2F target,
                                      CORE_POINT2F laser,
                                      float dt_s);

/**
 * @brief 停止云台跟踪输出。
 */
BSP_STATUS GimbalTracking_Stop(void);

/**
 * @brief 获取最近一次视觉云台跟踪状态。
 */
GIMBAL_TRACKING_STATE GimbalTracking_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_TRACKING_H */
