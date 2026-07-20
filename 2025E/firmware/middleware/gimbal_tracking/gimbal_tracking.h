/**
 * @file  gimbal_tracking.h
 * @brief Middleware 层视觉云台跟踪控制，将 CanMV 目标点转换为云台速度输出。
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
/*
 * 两轴机械负载不同，必须分别整定。输入为像素误差，输出为 deg/s；下列均为实机起调初值。
 * 两轴均为 F32C 位置模式 (角速度经 middleware 积分成位置设定点)：
 * yaw 惯量小、响应较快；pitch 需抑制重力负载下的过冲。
 */
#define GIMBAL_TRACKING_DEFAULT_YAW_PID_KP 6.10f
#define GIMBAL_TRACKING_DEFAULT_YAW_PID_KI 0.0f
#define GIMBAL_TRACKING_DEFAULT_YAW_PID_KD 1.000f
#define GIMBAL_TRACKING_DEFAULT_YAW_PID_INTEGRAL_LIMIT 1000.0f
#define GIMBAL_TRACKING_DEFAULT_YAW_PID_OUTPUT_LIMIT 120.0f

#define GIMBAL_TRACKING_DEFAULT_PITCH_PID_KP 6.10f
#define GIMBAL_TRACKING_DEFAULT_PITCH_PID_KI 0.00f
#define GIMBAL_TRACKING_DEFAULT_PITCH_PID_KD 1.000f
#define GIMBAL_TRACKING_DEFAULT_PITCH_PID_INTEGRAL_LIMIT 300.0f
#define GIMBAL_TRACKING_DEFAULT_PITCH_PID_OUTPUT_LIMIT 60.0f

/* 角度协议纯视觉闭环 (Aim track): 输入 deg、输出 deg/s。
 * P+I（无 D，D 会放大 K230 Kalman 后的动态噪声）：P 提供响应，I 把稳态误差(含机械零偏、
 * pitch 重力下垂、光轴/相机轴常量失配)持续调零。不设角度死区，让 I 能一直收敛到中心。
 * 积分限幅按“有效 I 贡献 = Ki*integral_limit”整定，防输出饱和时积分狂涨造成解饱和滞后/过冲。
 * yaw 直接驱动 F32C 速度模式；pitch 速度在 gimbal 层积分成受限位置目标，且机械减速比使
 * 等效增益更高，因此 pitch Kp/Ki 取 yaw 的约 1/3。以下均为实机起调初值。 */
#define GIMBAL_TRACKING_DEFAULT_ANGLE_YAW_PID_KP 6.0f
#define GIMBAL_TRACKING_DEFAULT_ANGLE_YAW_PID_KI 2.0f
#define GIMBAL_TRACKING_DEFAULT_ANGLE_YAW_PID_KD 0.0f
/* 原始积分和限幅(未乘 Ki); 有效 I 贡献上限 = Ki*该值。yaw: 2.0*20 = 40 deg/s。 */
#define GIMBAL_TRACKING_DEFAULT_ANGLE_YAW_PID_INTEGRAL_LIMIT 20.0f
#define GIMBAL_TRACKING_DEFAULT_ANGLE_PITCH_PID_KP 2.0f
#define GIMBAL_TRACKING_DEFAULT_ANGLE_PITCH_PID_KI 0.8f
#define GIMBAL_TRACKING_DEFAULT_ANGLE_PITCH_PID_KD 0.0f
/* pitch: 0.8*25 = 20 deg/s 有效 I 贡献上限。 */
#define GIMBAL_TRACKING_DEFAULT_ANGLE_PITCH_PID_INTEGRAL_LIMIT 25.0f
/* 已弃用: 角度模式不再使用死区(改由 I 项收敛到中心)。保留仅为兼容旧引用。 */
#define GIMBAL_TRACKING_DEFAULT_ANGLE_DEADBAND_DEG 0.75f
/* yaw 速度模式整数 RPM 的理论最小台阶约 6deg/s，但直接设 6 会在死区边缘形成极限环；
 * 当前保持 1deg/s 作为软件下限，实际量化由 F32C 链路决定。pitch 不使用速度地板。 */
#define GIMBAL_TRACKING_DEFAULT_MIN_MOVE_YAW_DEG_S 1.0f
#define GIMBAL_TRACKING_DEFAULT_MIN_MOVE_PITCH_DEG_S 1.0f  /* 仅旧像素控制保留 */
#define GIMBAL_TRACKING_DEFAULT_DEADBAND_PX 1.0f           /* 像素死区; 内部输出 0 停住 */

/**
 * @brief 视觉云台跟踪配置。
 */
typedef struct {
    /** yaw 轴 PID 配置。 */
    PID_CONFIG yaw_pid;
    /** pitch 轴 PID 配置。 */
    PID_CONFIG pitch_pid;
    /** 角度协议下 yaw 轴 PID 配置（输入单位 deg）。 */
    PID_CONFIG yaw_angle_pid;
    /** 角度协议下 pitch 轴 PID 配置（输入单位 deg）。 */
    PID_CONFIG pitch_angle_pid;
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
    /** true 表示最近一次跟踪使用 K230 角度误差协议。 */
    bool angle_mode;
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
 * @brief 仅消费 K230 yaw/pitch 角度误差帧执行跟踪，不回退到旧像素协议。
 * @param dt_s 控制周期，单位 s。
 */
BSP_STATUS GimbalTracking_UpdateAngle(float dt_s);

/**
 * @brief 根据 K230 发送的 yaw/pitch 角度误差执行一次跟踪。
 * @param yaw_error_deg 目标相对激光/参考点的水平角误差，单位 deg。
 * @param pitch_error_deg 目标相对激光/参考点的竖直角误差，单位 deg。
 */
BSP_STATUS GimbalTracking_TrackAngleErrors(float yaw_error_deg,
                                           float pitch_error_deg,
                                           float dt_s);

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

/**
 * @brief 运行时设置某个 PID 增益 (供上位机实时调参)。即时生效、不清积分。
 * @param key 参数名: "ykp"/"yki"/"ykd"/"pkp"/"pki"/"pkd"/"olim"(两轴输出限幅)。
 * @param value 新值。
 */
void GimbalTracking_SetGain(const char *key, float value);

/**
 * @brief 获取当前跟踪配置 (供上位机同步/回显 PID 参数)。
 */
GIMBAL_TRACKING_CONFIG GimbalTracking_GetConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_TRACKING_H */
