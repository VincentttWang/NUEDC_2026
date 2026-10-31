/**
 * @file  app_menu.h
 * @brief 嵌套菜单：菜单树类型 + 导航栈 + 渲染。
 *
 * 菜单是一棵 MENU_NODE 树，每个 MENU_ITEM 或指向子菜单、或指向可执行任务。
 * 导航仅用短按：UP/DOWN 移动选择、ENTER 进入(子菜单入栈/任务返回给调用方)、
 * BACK 返回上级(出栈)。菜单不直接进入 RUN，而是把"待执行任务"返回给 app_mode，
 * 由 app_mode 调 App_EnterRun——从而 app_menu 不反向依赖 app_mode。
 */
#ifndef APP_MENU_H
#define APP_MENU_H

#include "app_task.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 菜单项类型。
 */
typedef enum {
    MENU_ENTRY_SUBMENU = 0,   /**< 指向子菜单。 */
    MENU_ENTRY_TASK           /**< 指向可执行任务。 */
} MENU_ENTRY_KIND;

struct MENU_NODE;

/**
 * @brief 菜单项。
 */
typedef struct {
    const char *name;                 /**< 显示名。 */
    MENU_ENTRY_KIND kind;             /**< 子菜单 or 任务。 */
    union {
        const struct MENU_NODE *submenu;  /**< kind==SUBMENU。 */
        const APP_TASK_DESC    *task;     /**< kind==TASK。 */
    } u;
} MENU_ITEM;

/**
 * @brief 菜单节点（一页列表）。
 */
typedef struct MENU_NODE {
    const char *title;
    const MENU_ITEM *items;
    uint8_t item_count;
} MENU_NODE;

/** 根菜单（定义在 app_menu_def.c）。 */
extern const MENU_NODE APP_ROOT_MENU;

/**
 * @brief 复位导航到根菜单并标记待渲染。
 */
void Menu_Reset(void);

/**
 * @brief 强制下次 Menu_Tick 重绘（如从 RUN/FAULT 返回菜单后）。
 */
void Menu_MarkDirty(void);

/**
 * @brief 处理一次菜单按键与渲染。
 * @return 若本次选中了任务则返回其描述符（供 app_mode 进入 RUN），否则 NULL。
 */
const APP_TASK_DESC *Menu_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MENU_H */
