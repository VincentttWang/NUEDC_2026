/**
 * @file  app_mode.c
 * @brief 顶层应用状态机实现：菜单委派、任务执行委派、故障处理。
 */
#include "app_mode.h"
#include "app_task.h"
#include "app_menu.h"

#include "ui.h"
#include "key.h"
#include "chassis.h"
#include "system_fault.h"

#include <stdbool.h>
#include <stddef.h>

/* 控制周期，单位 s，须与调度器注册 App_ControlTick 的周期一致（APP_CONTROL_PERIOD_MS=10ms）。 */
#define APP_CONTROL_DT 0.01f

static APP_MODE app_mode;
static const APP_TASK_DESC *current_task;
static bool fault_dirty;

/* ---------------- 状态转移入口（唯一改 app_mode 的地方） ---------------- */

static void App_EnterRun(const APP_TASK_DESC *task){
    if (task == NULL){
        return;
    }
    current_task = task;
    Chassis_ResetDistance();
    if (current_task->on_enter != NULL){
        current_task->on_enter();
    }
    Key_ClearAllEvents();
    app_mode = APP_MODE_RUN;
}

static void App_ExitRun(APP_MODE next){
    if ((current_task != NULL) && (current_task->on_exit != NULL)){
        current_task->on_exit();
    }
    (void)Chassis_Brake();
    Key_ClearAllEvents();
    Menu_MarkDirty();
    app_mode = next;
}

static void App_RaiseFault(SYSTEM_FAULT_CODE code, const char *msg){
    (void)SystemFault_Set(code, msg);
    (void)Chassis_Brake();
    Key_ClearAllEvents();
    fault_dirty = true;
    app_mode = APP_MODE_FAULT;
}

/* ---------------- FAULT ---------------- */

static void Fault_Tick(void){
    if (fault_dirty){
        Ui_RenderStatusPage("FAULT", UI_STATUS_ERROR,
                            SystemFault_GetMessage(), "ENTER: reset");
        fault_dirty = false;
    }
    if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
        SystemFault_Clear();
        Key_ClearAllEvents();
        Menu_MarkDirty();
        app_mode = APP_MODE_MENU;
    }
}

/* ---------------- 对外接口 ---------------- */

void App_Mode_Init(void){
    Menu_Reset();
    current_task = NULL;
    fault_dirty  = false;
    app_mode     = APP_MODE_MENU;
}

APP_MODE App_Mode_Get(void){
    return app_mode;
}

void App_ControlTick(void){
    if (app_mode != APP_MODE_RUN){
        return;
    }

    /* BACK 短按中止优先。 */
    if (Key_GetEvent(KEY_ID_BACK) == KEY_EVENT_SHORT_PRESS){
        App_ExitRun(APP_MODE_MENU);
        return;
    }

    if ((current_task == NULL) || (current_task->on_tick == NULL)){
        App_ExitRun(APP_MODE_MENU);
        return;
    }

    APP_TASK_STATUS status = current_task->on_tick(APP_CONTROL_DT);
    if (status == APP_TASK_DONE){
        App_ExitRun(APP_MODE_MENU);
    } else if (status == APP_TASK_FAULT){
        App_RaiseFault(SYSTEM_FAULT_STATE_ERROR, current_task->name);
    } else {
        /* RUNNING：任务已设定本拍目标, 跑一步底盘速度环(仅闭环模式实际出力)。 */
        (void)Chassis_UpdateSpeedControl(APP_CONTROL_DT);
    }
}

void App_UiTick(void){
    switch (app_mode){
        case APP_MODE_MENU: {
            const APP_TASK_DESC *task = Menu_Tick();
            if (task != NULL){
                App_EnterRun(task);
            }
            break;
        }
        case APP_MODE_FAULT:
            Fault_Tick();
            break;
        case APP_MODE_RUN:
        case APP_MODE_INIT:
        default:
            /* RUN 由任务自渲染；INIT 不渲染。 */
            break;
    }
}
