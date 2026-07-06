/**
 * @file  app_e_task.h
 * @brief App 层 2025 年电赛 E 题基本任务入口。
 */
#ifndef APP_E_TASK_H
#define APP_E_TASK_H

#include <stdint.h>

/**
 * @brief 执行指定圈数的巡线任务。
 * @param lap_count 目标圈数。
 */
void AppE_RunLineFollow(uint8_t lap_count);

/**
 * @brief 执行云台 Yaw 轴指定速度旋转测试，用于寻找最小启动阈值。
 * @param yaw_deg_s 目标速度 (deg/s)。
 */
void AppE_RunYawSpeedTest(float yaw_deg_s);

/**
 * @brief 执行 2 秒靶心瞄准任务。
 */
void AppE_RunAimCenter2s(void);

/**
 * @brief 执行 E3 Yaw+ 扫描和瞄准任务。
 */
void AppE_RunRectScanYawPositive(void);

/**
 * @brief 执行 E3 Yaw- 扫描和瞄准任务。
 */
void AppE_RunRectScanYawNegative(void);

#endif /* APP_E_TASK_H */
