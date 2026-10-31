/**
 * @file  app_mode.h
 * @brief 顶层应用状态机与状态转移控制。
 *
 * 模式: INIT → MENU(任务选择) → RUN(执行选中任务) → 回 MENU;
 *       RUN/MENU 可跌入 FAULT(可恢复), 确认后回 MENU。致命故障走 SystemFault_Halt 终态。
 * 所有状态转移集中在本模块内部入口函数, 任务只通过 on_tick 返回值表达迁移意图。
 * 按键交互仅使用短按: MENU 用 UP/DOWN 选择、ENTER 确认; RUN 用 BACK 中止; FAULT 用 ENTER 复位。
 */
#ifndef APP_MODE_H
#define APP_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 顶层应用模式。
 */
typedef enum {
    APP_MODE_INIT = 0,  /**< 初始化未完成。 */
    APP_MODE_MENU,      /**< 任务选择界面。 */
    APP_MODE_RUN,       /**< 执行选中任务。 */
    APP_MODE_FAULT      /**< 可恢复故障。 */
} APP_MODE;

/**
 * @brief 初始化状态机：进入 MENU 并标记首帧待渲染。
 */
void App_Mode_Init(void);

/**
 * @brief 获取当前模式。
 */
APP_MODE App_Mode_Get(void);

/**
 * @brief 控制周期任务（调度器以固定控制周期调用）。
 *        仅在 RUN 态委派当前任务的 on_tick，并处理 BACK 中止与 DONE/FAULT 迁移。
 */
void App_ControlTick(void);

/**
 * @brief UI/输入周期任务（调度器以较慢周期调用）。
 *        MENU 态做导航+渲染，FAULT 态做渲染+复位；RUN 态由任务自渲染。
 */
void App_UiTick(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MODE_H */
