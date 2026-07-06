#ifndef VISUAL_H
#define VISUAL_H

#include "AllHeader.h"
#include <stdbool.h>

/* --- 宏定义 --- */
#define FRAME_HEADER 0x12
#define FRAME_TRAILER 0x5B
#define Visual_Rx_Buff_Len 25

#define MAX_DIGITS_PER_FRAME 4

// --- 学习阶段配置 ---
#define INITIAL_SAMPLING_COUNT 30
#define INITIAL_STABILITY_THRESHOLD 20

// --- 导航阶段配置 ---
// 稳定性判断标准：在导航任务开始后，某个数字在对应位置出现的概率要大于等于这个值
#define NAV_STABILITY_PROBABILITY 0.6f // 60%


/* --- 对外公共函数 --- */

/**
 * @brief 视觉数据处理的主入口函数。
 * @param sampling_command 指向主函数中的“开始采样”标志位的指针。
 */
void visual_process_command(bool* sampling_command);

/**
 * @brief 重置导航阶段的采样状态。
 * @note  在主函数中每次发起“持续导航”任务前调用。
 */
void visual_reset_nav_status(void);

/**
 * @brief 完全重置视觉模块所有状态，包括已学习的“初始数字”。
 * @note  仅在整个程序开始时调用。
 */
void visual_full_reset(void);


#endif // VISUAL_H