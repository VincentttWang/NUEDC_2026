/**
 * @file  app_e_task.h
 * @brief App 层 2025 年电赛 E 题基本任务入口。
 */
#ifndef APP_E_TASK_H
#define APP_E_TASK_H

#include <stdint.h>

/**
 * @brief E3 扫描瞄准的 yaw 初始扫描方向。
 */
typedef enum {
    APP_E_SCAN_DIR_POSITIVE = 0, /**< 正向扫描 (yaw 速度取正)。 */
    APP_E_SCAN_DIR_NEGATIVE      /**< 负向扫描 (yaw 速度取负)。 */
} APP_E_SCAN_DIR;

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
 * @brief F1 慢速稳定版：与 AppE_RunLineAim 同逻辑，但全面降速且无总超时，追求稳定不脱靶/不冲出。
 * @param lap_count 目标圈数。
 */
void AppE_RunLineAimSlow(uint8_t lap_count);

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
 * @brief 基本要求(3)：靶标位置未知，按选定方向扫描 yaw 找到靶标后锁定追踪。
 * @param scan_dir 初始 yaw 扫描方向。
 * @note 发现靶标即切入角度闭环追踪；连续锁定后发射一次，之后激光常亮继续追踪，
 *       长按 K2 退出。不设总超时。
 */
void AppE_RunScanAim(APP_E_SCAN_DIR scan_dir);

#endif /* APP_E_TASK_H */
