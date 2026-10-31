/**
 * @file  app_line_task.h
 * @brief 循迹测试任务描述符（供主菜单引用）。
 */
#ifndef APP_LINE_TASK_H
#define APP_LINE_TASK_H

#include "app_task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 车辆纵向安装尺寸；循迹任务的测量点现统一为前置灰度阵列。 */
#define APP_TRACK_REAR_TO_AXLE_M      0.070f
#define APP_TRACK_REAR_TO_SENSOR_M    0.195f
#define APP_TRACK_SENSOR_TO_AXLE_M \
    (APP_TRACK_REAR_TO_SENSOR_M - APP_TRACK_REAR_TO_AXLE_M)

/** 赛题要求 2：同一车辆装置高速整圈。 */
extern const APP_TASK_DESC APP_H2_LAP;
/** 赛题要求 4：载球 A→B 直线；进入任务后自动稳定球于中心，ENTER 确认后发车。 */
extern const APP_TASK_DESC APP_H4_LOADED_STRAIGHT;
/** 赛题要求 5：载球、目标 O 点整圈底盘循迹；进入任务后自动稳定球于中心，ENTER 确认后发车。 */
extern const APP_TASK_DESC APP_H5_LOADED_LAP_CENTER;
/** 赛题要求 6：载球、任意目标整圈；底盘循迹与 H5 完全一致。 */
/** 要求 6：第一下 ENTER 锁存当前球位置并启动稳定闭环；第二下 ENTER 发车。 */
extern const APP_TASK_DESC APP_H6_LOADED_LAP_TARGET;

#ifdef __cplusplus
}
#endif

#endif /* APP_LINE_TASK_H */
