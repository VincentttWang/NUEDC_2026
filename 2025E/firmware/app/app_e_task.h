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
 * @brief 发挥部分：巡线期间持续执行靶心瞄准。
 * @param lap_count 目标圈数。
 */
void AppE_RunLineAim(uint8_t lap_count);

/**
 * @brief 发挥部分：巡线一圈期间按里程相位在靶面画圆。
 */
void AppE_RunLineCircle(uint8_t lap_count);

/**
 * @brief 纯 K230 角度误差持续瞄准，不使用世界坐标、IMU、编码器或几何前馈。
 * @note 不设任务超时；视觉丢失时停 yaw/保持 pitch，重获目标后恢复。
 */
void AppE_RunContinuousAim(void);

/**
 * @brief 执行 2 秒靶心瞄准任务。
 */
void AppE_RunAimCenter2s(void);

/**
 * @brief 基本要求(3)：从规定起点用几何前馈瞄准靶心，视觉慢校正。
 */
void AppE_RunKnownPoseAim4s(void);

#endif /* APP_E_TASK_H */
