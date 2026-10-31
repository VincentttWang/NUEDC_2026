/**
 * @file  app_scheduler.c
 * @brief 协作式时间触发调度器实现，含 app 自持的 SysTick_Handler。
 */
#include "app_scheduler.h"

#include "bsp_time.h"
#include "key.h"

#include <stddef.h>

#define SCHED_MAX_TASKS 8U

typedef struct {
    APP_SCHED_CB cb;
    uint32_t     period_ms;
    uint32_t     last_ms;
} SCHED_TASK;

static SCHED_TASK sched_tasks[SCHED_MAX_TASKS];
static uint8_t    sched_count;
static volatile bool tick_active;

void Scheduler_Init(void){
    sched_count = 0U;
    tick_active = false;
}

bool Scheduler_AddTask(APP_SCHED_CB cb, uint32_t period_ms){
    if ((cb == NULL) || (sched_count >= SCHED_MAX_TASKS)){
        return false;
    }
    sched_tasks[sched_count].cb        = cb;
    sched_tasks[sched_count].period_ms = period_ms;
    sched_tasks[sched_count].last_ms   = BSP_Time_GetMs();
    sched_count++;
    return true;
}

void Scheduler_EnableTick(void){
    tick_active = true;
}

void Scheduler_Run(void){
    uint32_t now = BSP_Time_GetMs();

    for (uint8_t i = 0U; i < sched_count; i++){
        /* uint32 差值天然处理溢出回绕。 */
        if ((now - sched_tasks[i].last_ms) >= sched_tasks[i].period_ms){
            sched_tasks[i].last_ms = now;   /* 不做漂移补偿，避免积压追赶。 */
            sched_tasks[i].cb();
        }
    }
}

/*
 * SysTick (1ms): 递增系统时基 + 扫描按键消抖。app 自持的调度型中断。
 * tick_active 门控确保 init 完成前(按键/时基未就绪)不做任何事。
 */
void SysTick_Handler(void){
    if (!tick_active){
        return;
    }
    BSP_Time_TickInc();
    Key_Scan();
}
