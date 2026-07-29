/**
 * @file  bsp_time.c
 * @brief BSP 系统时间与阻塞延时实现。
 */
#include "bsp_time.h"
#include <ti/driverlib/dl_common.h>

#ifndef BSP_TIME_CPUCLK_HZ
#define BSP_TIME_CPUCLK_HZ 32000000U
#endif

static volatile uint32_t g_bsp_time_ms;

void BSP_Time_Init(void){
    g_bsp_time_ms = 0U;
}

void BSP_Time_TickInc(void){
    g_bsp_time_ms++;
}

uint32_t BSP_Time_GetMs(void){
    return g_bsp_time_ms;
}

void BSP_DelayUs(uint32_t us){
    while (us-- > 0U){
        DL_Common_delayCycles(BSP_TIME_CPUCLK_HZ / 1000000U);
    }
}

void BSP_DelayMs(uint32_t ms){
    while (ms-- > 0U){
        BSP_DelayUs(1000U);
    }
}
