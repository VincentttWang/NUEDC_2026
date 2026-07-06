#include "app_device_check.h"
#include "app_launcher.h"
#include "bsp_time.h"
#include "canmv_uart.h"
#include "chassis.h"
#include "gimbal.h"
#include "key.h"
#include "line_follow.h"
#include "system_fault.h"
#include "ti_msp_dl_config.h"
#include "ui.h"

#include <stdint.h>

volatile uint32_t g_app_debug_uart_irq_count;
volatile uint32_t g_app_debug_uart_rx_irq_count;
volatile uint32_t g_app_debug_uart_drained_byte_count;
volatile uint32_t g_app_debug_uart_empty_rx_irq_count;
volatile uint32_t g_app_debug_uart_unhandled_irq_count;
volatile uint8_t g_app_debug_uart_last_iidx;

static void App_DrainDebugUartRxFifo(void){
    uint32_t drained = 0U;

    while (!DL_UART_Main_isRXFIFOEmpty(Debug_INST)){
        AppDeviceCheck_ProcessImuByte((uint8_t)DL_UART_Main_receiveData(Debug_INST));
        drained++;
    }

    g_app_debug_uart_drained_byte_count += drained;
    if (drained == 0U){
        g_app_debug_uart_empty_rx_irq_count++;
    }
}

static void App_InitSystems(void){
    SYSCFG_DL_init();
    BSP_Time_Init();
    __enable_irq();

    DL_UART_Main_setRXFIFOThreshold(Debug_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setRXInterruptTimeout(Debug_INST, 1U);
    DL_UART_Main_enableInterrupt(Debug_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    NVIC_SetPriority(Debug_INST_INT_IRQN, 1);
    NVIC_ClearPendingIRQ(Debug_INST_INT_IRQN);
    NVIC_EnableIRQ(Debug_INST_INT_IRQN);

    Ui_Init();
    Key_Init();

    (void)Chassis_Init();
    (void)Gimbal_Init();
    (void)LineFollow_Init();
    (void)CanMvUart_Init();
    SystemFault_Clear();

    DL_TimerA_startCounter(TIMER_0_INST);
}

int main(void){
    App_InitSystems();
    App_Launch();

    while (1){
    }
}

void UART0_IRQHandler(void){
    DL_UART_IIDX iidx = DL_UART_getPendingInterrupt(Debug_INST);

    g_app_debug_uart_irq_count++;
    g_app_debug_uart_last_iidx = (uint8_t)iidx;

    switch (iidx){
        case DL_UART_IIDX_RX:
        case DL_UART_IIDX_RX_TIMEOUT_ERROR:
            g_app_debug_uart_rx_irq_count++;
            App_DrainDebugUartRxFifo();
            break;
        default:
            g_app_debug_uart_unhandled_irq_count++;
            break;
    }
}

void UART2_IRQHandler(void){
    switch (DL_UART_getPendingInterrupt(CANMV_UART_INST)){
        case DL_UART_IIDX_RX:
        case DL_UART_IIDX_RX_TIMEOUT_ERROR:
            CanMvUart_ProcessRx();
            break;
        default:
            break;
    }
}

void SysTick_Handler(void){
    static uint8_t scan_divider = 0U;

    BSP_Time_TickInc();

    scan_divider++;
    if (scan_divider >= 10U){
        Key_Scan();
        scan_divider = 0U;
    }
}
