/**
 * @file  bsp_time.h
 * @brief BSP 系统时间与阻塞延时服务。
 */
#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 BSP 毫秒计数器。
 */
void BSP_Time_Init(void);

/**
 * @brief 毫秒计数器自增一拍。
 * @note 通常由 SysTick_Handler 每 1 ms 调用一次。
 */
void BSP_Time_TickInc(void);

/**
 * @brief 获取系统启动后的毫秒计数。
 */
uint32_t BSP_Time_GetMs(void);

/**
 * @brief 阻塞延时，单位 us。
 */
void BSP_DelayUs(uint32_t us);

/**
 * @brief 阻塞延时，单位 ms。
 */
void BSP_DelayMs(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TIME_H */
