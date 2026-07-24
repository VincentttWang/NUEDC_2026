/**
 * @file  motion.h
 * @brief Core 层底盘运动原语执行接口。
 */
#ifndef MOTION_H
#define MOTION_H

#include "bsp_common.h"
#include "chassis.h"
#include "line_tracking/line_tracking.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 底盘运动模式。
 */
typedef enum {
    /** 空闲，不主动输出新的底盘命令。 */
    MOTION_MODE_IDLE = 0,
    /** 主动刹车。 */
    MOTION_MODE_BRAKE,
    /** 关闭输出并滑行。 */
    MOTION_MODE_COAST,
    /** 左右轮同向前进。 */
    MOTION_MODE_STRAIGHT,
    /** 左右轮同向后退。 */
    MOTION_MODE_BACKWARD,
    /** 左轮后退、右轮前进，原地左转。 */
    MOTION_MODE_SPIN_LEFT,
    /** 左轮前进、右轮后退，原地右转。 */
    MOTION_MODE_SPIN_RIGHT,
    /** 调用巡线控制器输出左右轮占空比。 */
    MOTION_MODE_LINE_FOLLOW
} MOTION_MODE;

/**
 * @brief 单次运动命令。
 */
typedef struct {
    /** 运动模式。 */
    MOTION_MODE mode;
    /** 基础占空比，取值通常为 [0, 100]。 */
    float duty_percent;
} MOTION_COMMAND;

/**
 * @brief 最近一次运动输出状态。
 */
typedef struct {
    /** 最近一次执行的运动模式。 */
    MOTION_MODE mode;
    /** 最近一次命令参数。 */
    MOTION_COMMAND command;
    /** 最近一次左右轮占空比。 */
    CHASSIS_DUTY duty;
    /** 最近一次巡线输出；非巡线模式下保留上一次值。 */
    LINE_TRACKING_OUTPUT line_output;
    /** 最近一次底层调用返回值。 */
    BSP_STATUS last_status;
} MOTION_STATE;

/**
 * @brief 初始化运动原语执行器。
 */
void Motion_Init(void);

/**
 * @brief 按给定命令执行一次运动输出。
 * @param command 运动命令。
 * @param dt_s 控制周期，单位 s；仅巡线模式使用。
 * @return 底层输出状态。
 */
BSP_STATUS Motion_Apply(const MOTION_COMMAND *command, float dt_s);

/**
 * @brief 主动刹车并回到空闲状态。
 */
BSP_STATUS Motion_Stop(void);

/**
 * @brief 获取最近一次运动状态。
 */
MOTION_STATE Motion_GetState(void);

/**
 * @brief 构造主动刹车命令。
 */
MOTION_COMMAND Motion_CommandBrake(void);

/**
 * @brief 构造滑行命令。
 */
MOTION_COMMAND Motion_CommandCoast(void);

/**
 * @brief 构造直行命令。
 */
MOTION_COMMAND Motion_CommandStraight(float duty_percent);

/**
 * @brief 构造倒车命令。
 */
MOTION_COMMAND Motion_CommandBackward(float duty_percent);

/**
 * @brief 构造原地左转命令。
 */
MOTION_COMMAND Motion_CommandSpinLeft(float duty_percent);

/**
 * @brief 构造原地右转命令。
 */
MOTION_COMMAND Motion_CommandSpinRight(float duty_percent);

/**
 * @brief 构造巡线命令。
 */
MOTION_COMMAND Motion_CommandLineFollow(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_H */
