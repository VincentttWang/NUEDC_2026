/**
 * @file  app_init.h
 * @brief 集中式上电初始化。
 */
#ifndef APP_INIT_H
#define APP_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 完成 SysConfig、BSP、中间件与框架初始化并注册调度任务。
 * @note 不开全局中断（由 main 在其后调用 __enable_irq）；初始化失败走 SystemFault 终态。
 */
void App_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_INIT_H */
