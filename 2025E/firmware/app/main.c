#include "app_debug_cmd.h"
#include "app_device_check.h"
#include "app_launcher.h"
#include "bsp_time.h"
#include "canmv_uart.h"
#include "chassis.h"
#include "bsp/debug_uart/debug_uart.h"
#include "gimbal.h"
#include "key.h"
#include "line_follow.h"
#include "system_fault.h"
#include "ti_msp_dl_config.h"
#include "ui.h"
#include "wit_sdk.h"

#include <stdint.h>

volatile uint32_t g_app_debug_uart_irq_count;
volatile uint32_t g_app_debug_uart_rx_irq_count;
volatile uint32_t g_app_debug_uart_drained_byte_count;
volatile uint32_t g_app_debug_uart_empty_rx_irq_count;
volatile uint32_t g_app_debug_uart_unhandled_irq_count;
volatile uint8_t g_app_debug_uart_last_iidx;

static void App_DrainDebugUartRxFifo(void){
    uint32_t drained = 0U;

    while (!DL_UART_Main_isRXFIFOEmpty(BlueTooth_INST)){
        (void)DL_UART_Main_receiveData(BlueTooth_INST);
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

    /*
     * IMU 初始化必须在 __enable_irq() 之前完成:
     *   - JY61P 挂在 I2C0, SysTick ISR 会调用 JY61P_I2C_Poll
     *   - 若在开中断后初始化, ISR 可能抢占初始化过程的 I2C 传输, 导致竞态
     *   - BSP_DelayMs 使用 busy-wait (DL_Common_delayCycles), 不依赖 SysTick
     */
    JY61P_I2C_Init();

    __enable_irq();

    DL_UART_Main_setRXFIFOThreshold(BlueTooth_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setRXInterruptTimeout(BlueTooth_INST, 1U);
    DL_UART_Main_enableInterrupt(BlueTooth_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    NVIC_SetPriority(BlueTooth_INST_INT_IRQN, 1);
    NVIC_ClearPendingIRQ(BlueTooth_INST_INT_IRQN);
    NVIC_EnableIRQ(BlueTooth_INST_INT_IRQN);

    /* debug 串口非阻塞输出 (TX 环形缓冲 + TX 中断, Debug_Ex=UART1)。
     * 必须使能其 NVIC 中断, 否则 TX 中断无法把环形缓冲排出 → 无任何输出。 */
    DebugUart_Init();
    /* 使能 Debug_Ex RX 中断以接收上位机命令 (实时 PID 调参)。 */
    DL_UART_Main_setRXFIFOThreshold(Debug_Ex_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setRXInterruptTimeout(Debug_Ex_INST, 1U);
    DL_UART_Main_enableInterrupt(Debug_Ex_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    NVIC_SetPriority(Debug_Ex_INST_INT_IRQN, 1);
    NVIC_ClearPendingIRQ(Debug_Ex_INST_INT_IRQN);
    NVIC_EnableIRQ(Debug_Ex_INST_INT_IRQN);

    Ui_Init();
    Key_Init();

    /* Gimbal_Init 内部完成无刷驱动初始化(UART3 唤醒)、两轴使能与速度模式配置 */
    (void)Chassis_Init();
    (void)Gimbal_Init();
    (void)LineFollow_Init();
    (void)CanMvUart_Init();

    /* 开机发起 pitch 抬升到工作角(非阻塞); 进入 E2/E3 瞄准前再确保到位并切速度模式 */
    Gimbal_StartupElevatePitch();

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
    DL_UART_IIDX iidx = DL_UART_getPendingInterrupt(BlueTooth_INST);

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

/* Debug_Ex(UART1) 非阻塞输出的中断消费端。用配套宏名, 实例变更自动跟随。 */
void Debug_Ex_INST_IRQHandler(void){
    switch (DL_UART_getPendingInterrupt(Debug_Ex_INST)){
        case DL_UART_IIDX_TX:
            DebugUart_TxIsr();      /* 把环形缓冲排入 TX FIFO */
            break;
        case DL_UART_IIDX_RX:
        case DL_UART_IIDX_RX_TIMEOUT_ERROR:
            /* 上位机命令: 逐字节喂入命令组行器 (线程侧 AppDebugCmd_Poll 解析)。 */
            while (!DL_UART_Main_isRXFIFOEmpty(Debug_Ex_INST)){
                AppDebugCmd_FeedByte(DL_UART_Main_receiveData(Debug_Ex_INST));
            }
            break;
        default:
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
    static uint8_t imu_poll_divider = 0U;

    BSP_Time_TickInc();

    scan_divider++;
    if (scan_divider >= 10U){
        Key_Scan();
        scan_divider = 0U;
    }

    imu_poll_divider++;
    if (imu_poll_divider >= JY61P_I2C_POLL_PERIOD_TICK){
        JY61P_I2C_Poll();
        imu_poll_divider = 0U;
    }
}
