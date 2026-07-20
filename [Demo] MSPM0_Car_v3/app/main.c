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
#include "wit_sdk.h"
#include "mpu6050.h"
#include "hc_sr04.h"
#include "f32c_bldc.h"

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
     *   - JY61P 和 MPU6050 共用 I2C0, SysTick ISR 会调用 JY61P_I2C_Poll
     *   - 若在开中断后初始化, ISR 可能抢占 MPU6050_Init 的 I2C 传输, 导致竞态
     *   - BSP_DelayMs 使用 busy-wait (DL_Common_delayCycles), 不依赖 SysTick
     */
    JY61P_I2C_Init();
    (void)MPU6050_Init();

    __enable_irq();

    DL_UART_Main_setRXFIFOThreshold(BlueTooth_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setRXInterruptTimeout(BlueTooth_INST, 1U);
    DL_UART_Main_enableInterrupt(BlueTooth_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    NVIC_SetPriority(BlueTooth_INST_INT_IRQN, 1);
    NVIC_ClearPendingIRQ(BlueTooth_INST_INT_IRQN);
    NVIC_EnableIRQ(BlueTooth_INST_INT_IRQN);

    Ui_Init();
    Key_Init();

    (void)Chassis_Init();
    (void)Gimbal_Init();
    (void)LineFollow_Init();
    (void)CanMvUart_Init();

    /* 启动无刷驱动: 俯仰电机上电后正转到约 120°, 并设 [0°,180°] 限位 */
    BLDC_Init();
    BLDC_StartupPitch();

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
