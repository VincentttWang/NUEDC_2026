/**
 * @file  hc_sr04.h
 * @brief HC-SR04 超声波测距模块驱动 (GPIO 轮询方式)
 *
 * 引脚 (SysConfig 分配): PA27=Trig, PA26=Echo
 * 参考: HC-SR04 数据手册, TI MSPM0G3507 超声波驱动
 */
#ifndef HC_SR04_H
#define HC_SR04_H

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/* 由 SysConfig 生成, 参见 ti_msp_dl_config.h:
 *   SR04_Trig_PIN, SR04_Trig_IOMUX
 *   SR04_Echo_PIN, SR04_Echo_IOMUX
 *   SR04_PORT
 */

#define SR04_TRIG_PORT      SR04_PORT
#define SR04_TRIG_PIN       SR04_Trig_PIN
#define SR04_ECHO_PORT      SR04_PORT
#define SR04_ECHO_PIN       SR04_Echo_PIN

/* 超时: 最大测量距离 ~4m → 脉宽 ~23ms, 用 25ms (800K cycles @ 32MHz) */
#define SR04_TIMEOUT_CYCLES 800000U

/* 返回值: 距离(cm), 超时/无效返回 -1.0f */
float SR04_GetDistance(void);

/* 连续多次测量取平均值, count 建议 3~5 */
float SR04_GetDistanceAvg(uint8_t count);

#endif /* HC_SR04_H */
