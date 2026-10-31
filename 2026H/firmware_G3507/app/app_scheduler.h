/**
 * @file  app_scheduler.h
 * @brief 协作式时间触发调度器（超循环 + 周期任务表）。
 *
 * 时基为 SysTick(1ms) 递增的 BSP_Time；主循环反复调用 Scheduler_Run() 分派到期任务。
 * 任务为非阻塞 run-to-completion 回调。SysTick 只做时基递增与按键扫描（app 自持中断）。
 */
#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 周期任务回调。
 */
typedef void (*APP_SCHED_CB)(void);

/**
 * @brief 复位任务表。
 */
void Scheduler_Init(void);

/**
 * @brief 注册一个周期任务。
 * @param cb 非阻塞回调，不可为 NULL。
 * @param period_ms 调用周期，单位 ms。
 * @return true 注册成功；false 表满或参数非法。
 */
bool Scheduler_AddTask(APP_SCHED_CB cb, uint32_t period_ms);

/**
 * @brief 使能 SysTick 内的时基递增与按键扫描。
 * @note 须在全部外设/框架初始化完成后调用，避免 init 期 SysTick 误触发未就绪模块。
 */
void Scheduler_EnableTick(void);

/**
 * @brief 分派所有到期任务，在主循环中反复调用。
 */
void Scheduler_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCHEDULER_H */
