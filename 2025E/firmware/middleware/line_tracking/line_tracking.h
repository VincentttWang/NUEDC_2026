/**
 * @file  line_tracking.h
 * @brief Middleware 层巡线控制策略，将灰度传感器状态转换为底盘输出。
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

#define LINE_TRACKING_DEFAULT_BASE_DUTY 34.0f   /* 30→34: 直线略提速防超时(陀螺增稳保证高速稳) */
#define LINE_TRACKING_DEFAULT_OUTPUT_LIMIT 100.0f
#define LINE_TRACKING_DEFAULT_CORRECTION_LIMIT 60.0f
/* 10→16: 加大巡线差速权限, 让陀螺内环在出弯有足够刹车权限压住残余转速(减弯后直线过冲);
 * 正常直行陀螺输出小, 用不到这么大, 只在高 gz 瞬态(出弯)才吃满。 */
#define LINE_TRACKING_DEFAULT_DIFFERENTIAL_LIMIT 16.0f
#define LINE_TRACKING_DEFAULT_POSITION_SCALE 10.0f
#define LINE_TRACKING_ACTIVE_SENSOR_MASK 0xFFU

/*
 * 陀螺增稳串级默认参数。硬件"等占空比不直行"(电机/轮径不对称)会让纯位置反馈产生
 * "漂移-出死区-满舵-过冲"的蛇形极限环。串级: 外环灰度偏差 -> 期望航向角速度 ω_ref;
 * 内环陀螺闭环施加差速使实际 ω 跟上 ω_ref。居中时 ω_ref=0 -> 内环把 gz 压向 0 ->
 * 自动补偿不对称直行; 且 gz 反馈是航向的阻尼项 -> 直接压蛇形。以下为实机起调初值。
 */
#define LINE_TRACKING_DEFAULT_GYRO_STAB_ENABLED true
#define LINE_TRACKING_DEFAULT_GYRO_LINE_KP 3.0f    /* 外环: 偏差(scale后)->ω_ref, 10≈1格 -> 30°/s */
#define LINE_TRACKING_DEFAULT_GYRO_STAB_KP 0.20f   /* 内环: (ω_ref−gz)->差速, 25°/s误差≈满舵 */
#define LINE_TRACKING_DEFAULT_OMEGA_REF_LIMIT 60.0f
#define LINE_TRACKING_DEFAULT_GYRO_Z_SIGN (1.0f)  /* 使内环为负反馈; 若增稳后更抖/自旋则翻号 */

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
    /** 巡线偏差 PID 配置（陀螺增稳关闭时的纯位置控制路径使用）。 */
    PID_CONFIG pid_config;
    /** 陀螺增稳串级使能；true 时灰度出 ω_ref、陀螺内环闭环，false 退化为纯位置 PID。 */
    bool gyro_stab_enabled;
    /** 外环增益：灰度偏差（缩放后）→ 期望航向角速度 ω_ref。 */
    float gyro_line_kp;
    /** 内环增益：角速度误差（ω_ref − gz）→ 差速修正。 */
    float gyro_stab_kp;
    /** ω_ref 限幅，单位 deg/s。 */
    float omega_ref_limit;
    /** gz 符号系数：使陀螺内环成为负反馈（实机验证）。 */
    float gyro_z_sign;
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
 * @brief 获取默认巡线配置（供上层取默认后按需覆盖个别字段，如 base_duty）。
 */
LINE_TRACKING_CONFIG LineTracking_GetDefaultConfig(void);

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
 * @param omega_deg_s 车体航向角速度（已乘 gyro_z_sign 的 gz，deg/s）；陀螺增稳内环用。
 *                    增稳关闭或无 IMU 时传 0（退化为纯位置控制，无阻尼）。
 * @param out 输出结果。
 * @return BSP_STATUS_OK 表示计算成功。
 */
BSP_STATUS LineTracking_Compute(const LINE_FOLLOW_SENSOR_STATE *sensor,
                                float dt_s,
                                float omega_deg_s,
                                LINE_TRACKING_OUTPUT *out);

/**
 * @brief 获取最近一次巡线输出。
 */
LINE_TRACKING_OUTPUT LineTracking_GetOutput(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_TRACKING_H */
