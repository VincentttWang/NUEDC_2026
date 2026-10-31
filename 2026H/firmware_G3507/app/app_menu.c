/**
 * @file  app_menu.c
 * @brief 嵌套菜单导航栈与渲染实现。
 */
#include "app_menu.h"

#include "ui.h"
#include "key.h"

#include <stdbool.h>
#include <stddef.h>

/* 导航栈深度上限（根 + 若干级子菜单）。 */
#define MENU_STACK_MAX 4U
/* 单个菜单最多容纳的项数，用于组装渲染指针数组(须 >= 最大子菜单项数)。 */
#define MENU_ITEMS_MAX 12U

static const MENU_NODE *node_stack[MENU_STACK_MAX];
static uint8_t          sel_stack[MENU_STACK_MAX];   /* 各级选中项，返回时恢复 */
static uint8_t          depth;
static bool             dirty;

static const MENU_NODE *CurrentNode(void){
    return node_stack[depth];
}

void Menu_Reset(void){
    depth = 0U;
    node_stack[0] = &APP_ROOT_MENU;
    sel_stack[0]  = 0U;
    dirty = true;
}

void Menu_MarkDirty(void){
    dirty = true;
}

static void Menu_Render(void){
    const MENU_NODE *node = CurrentNode();
    uint8_t count = node->item_count;
    if (count > MENU_ITEMS_MAX){
        count = MENU_ITEMS_MAX;
    }

    const char *names[MENU_ITEMS_MAX];
    for (uint8_t i = 0U; i < count; i++){
        names[i] = node->items[i].name;
    }

    uint8_t sel = sel_stack[depth];
    uint8_t first = 0U;
    if (sel >= UI_LIST_VISIBLE_COUNT){
        first = (uint8_t)(sel - (UI_LIST_VISIBLE_COUNT - 1U));
    }

    UI_LIST_PAGE page = {
        .title               = node->title,
        .items               = names,
        .item_count          = count,
        .selected_index      = sel,
        .first_visible_index = first,
    };
    Ui_RenderListPage(&page);
}

const APP_TASK_DESC *Menu_Tick(void){
    const MENU_NODE *node = CurrentNode();
    uint8_t count = node->item_count;
    if (count == 0U){
        return NULL;
    }

    if (Key_GetEvent(KEY_ID_DOWN) == KEY_EVENT_SHORT_PRESS){
        sel_stack[depth] = (uint8_t)((sel_stack[depth] + 1U) % count);
        dirty = true;
    }
    if (Key_GetEvent(KEY_ID_UP) == KEY_EVENT_SHORT_PRESS){
        sel_stack[depth] = (uint8_t)((sel_stack[depth] + count - 1U) % count);
        dirty = true;
    }
    if (Key_GetEvent(KEY_ID_BACK) == KEY_EVENT_SHORT_PRESS){
        if (depth > 0U){          /* 返回上级 */
            depth--;
            dirty = true;
        }
    }
    if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
        const MENU_ITEM *item = &node->items[sel_stack[depth]];
        if (item->kind == MENU_ENTRY_TASK){
            return item->u.task;      /* 交给 app_mode 进入 RUN */
        }
        if ((item->kind == MENU_ENTRY_SUBMENU) &&
            (item->u.submenu != NULL) &&
            (depth + 1U < MENU_STACK_MAX)){
            depth++;                  /* 进入子菜单 */
            node_stack[depth] = item->u.submenu;
            sel_stack[depth]  = 0U;
            dirty = true;
        }
    }

    if (dirty){
        Menu_Render();
        dirty = false;
    }
    return NULL;
}
