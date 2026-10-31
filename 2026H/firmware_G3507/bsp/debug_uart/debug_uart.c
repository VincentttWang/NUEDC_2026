/**
 * @file  debug_uart.c
 * @brief 非阻塞 debug 串口输出实现 (环形缓冲 + TX 中断)。
 */
#include "debug_uart.h"
#include "ti_msp_dl_config.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

/* debug 串口实例 (Debug_Ex/UART1, 见 board/sys_config)。 */
#define DEBUG_UART_INST Debug_Ex_INST

/* TX 环形缓冲长度, 必须是 2 的幂 (用掩码回绕)。1KB @115200 约 90ms 缓冲。 */
#define DEBUG_UART_TX_BUF_LEN 2048U
#define DEBUG_UART_TX_MASK (DEBUG_UART_TX_BUF_LEN - 1U)
#define DEBUG_UART_RX_BUF_LEN 256U
#define DEBUG_UART_RX_MASK (DEBUG_UART_RX_BUF_LEN - 1U)
#define DEBUG_UART_RX_ERROR_INTERRUPTS                 \
    (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |            \
     DL_UART_MAIN_INTERRUPT_BREAK_ERROR |              \
     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |            \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR)

/* 单条 Printf 格式化上限。H3 加入 PWM 寄存器诊断后约 560 字节；
 * 按 60 ms 输出仍低于 115200 baud 的持续吞吐能力。 */
#define DEBUG_UART_PRINTF_MAX 640U

static uint8_t tx_buf[DEBUG_UART_TX_BUF_LEN];
static volatile uint16_t tx_head;   /* 生产者(线程) 写入位置 */
static volatile uint16_t tx_tail;   /* 消费者(TX ISR) 读取位置 */
static volatile uint32_t tx_dropped;   /* 缓冲满丢弃的字节数 */
static uint8_t rx_buf[DEBUG_UART_RX_BUF_LEN];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_bytes;
static volatile uint32_t rx_dropped;
static volatile uint32_t rx_errors;

static void DebugUart_FlushRx(void){
    while (!DL_UART_Main_isRXFIFOEmpty(DEBUG_UART_INST)){
        (void)DL_UART_Main_receiveData(DEBUG_UART_INST);
    }
}

void DebugUart_Init(void){
    tx_head = 0U;
    tx_tail = 0U;
    tx_dropped = 0U;
    rx_head = 0U;
    rx_tail = 0U;
    rx_bytes = 0U;
    rx_dropped = 0U;
    rx_errors = 0U;
    /* TX FIFO 降到阈值即触发中断续传; 无数据时 TX 中断保持关闭 (见 DebugUart_Write)。 */
    DL_UART_Main_setTXFIFOThreshold(DEBUG_UART_INST, DL_UART_TX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_disableInterrupt(DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    DL_UART_Main_setRXFIFOThreshold(DEBUG_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DebugUart_FlushRx();
    DL_UART_Main_clearInterruptStatus(DEBUG_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX | DEBUG_UART_RX_ERROR_INTERRUPTS);
    /* 放开 UART 中断线; TX 中断本身按需在 Write/TxIsr 里动态开关。 */
    NVIC_ClearPendingIRQ(Debug_Ex_INST_INT_IRQN);
    NVIC_EnableIRQ(Debug_Ex_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(DEBUG_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX | DEBUG_UART_RX_ERROR_INTERRUPTS);
}

void DebugUart_TxIsr(void){
    /* 仅由 TX 中断调用: 把环形缓冲尽量塞进 TX FIFO; 缓冲空则关 TX 中断避免空触发。 */
    while ((tx_tail != tx_head) && !DL_UART_Main_isTXFIFOFull(DEBUG_UART_INST)){
        DL_UART_Main_transmitData(DEBUG_UART_INST, tx_buf[tx_tail]);
        tx_tail = (uint16_t)((tx_tail + 1U) & DEBUG_UART_TX_MASK);
    }
    if (tx_tail == tx_head){
        DL_UART_Main_disableInterrupt(DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    }
}

void DebugUart_Write(const uint8_t *data, uint16_t len){
    if (data == NULL){
        return;
    }

    for (uint16_t i = 0U; i < len; i++){
        uint16_t next = (uint16_t)((tx_head + 1U) & DEBUG_UART_TX_MASK);
        if (next == tx_tail){
            /* 缓冲满: 丢弃剩余, 绝不阻塞控制环。 */
            tx_dropped += (uint32_t)(len - i);
            break;
        }
        tx_buf[tx_head] = data[i];
        tx_head = next;
    }

    /* 使能 TX 中断以(重新)启动排空: TX FIFO 低于阈值会立即挂起该中断。幂等。 */
    DL_UART_Main_enableInterrupt(DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
}

void DebugUart_Puts(const char *s){
    uint16_t n = 0U;

    if (s == NULL){
        return;
    }
    while (s[n] != '\0'){
        n++;
    }
    DebugUart_Write((const uint8_t *)s, n);
}

void DebugUart_Printf(const char *fmt, ...){
    char buf[DEBUG_UART_PRINTF_MAX];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0){
        return;
    }
    if (n >= (int)sizeof(buf)){
        /* 溢出截断: vsnprintf 最多写 sizeof-1 个有效字符 + '\0'; 只发有效字符, 不发末尾 NUL。 */
        n = (int)sizeof(buf) - 1;
    }
    DebugUart_Write((const uint8_t *)buf, (uint16_t)n);
}

uint32_t DebugUart_GetDroppedBytes(void){
    return tx_dropped;
}

uint16_t DebugUart_Read(uint8_t *data, uint16_t max_len){
    uint16_t count = 0U;
    if (data == NULL){
        return 0U;
    }
    while ((count < max_len) && (rx_tail != rx_head)){
        data[count++] = rx_buf[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) & DEBUG_UART_RX_MASK);
    }
    return count;
}

uint32_t DebugUart_GetRxBytes(void){ return rx_bytes; }
uint32_t DebugUart_GetRxDroppedBytes(void){ return rx_dropped; }
uint32_t DebugUart_GetRxErrors(void){ return rx_errors; }

/*
 * Debug_Ex/UART1 ISR：TX 排空遥测环形缓冲；RX 仅保留给普通调试输入。
 * 树莓派视觉帧由 bsp/rpi_uart 独占 UART2 接收，绝不经过这里。
 */
void Debug_Ex_INST_IRQHandler(void){
    switch (DL_UART_Main_getPendingInterrupt(DEBUG_UART_INST)){
        case DL_UART_MAIN_IIDX_TX:
            DebugUart_TxIsr();
            break;
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(DEBUG_UART_INST)){
                uint8_t byte = DL_UART_Main_receiveData(DEBUG_UART_INST);
                uint16_t next = (uint16_t)((rx_head + 1U) & DEBUG_UART_RX_MASK);
                if (next == rx_tail){
                    rx_dropped++;
                } else{
                    rx_buf[rx_head] = byte;
                    rx_head = next;
                    rx_bytes++;
                }
            }
            break;
        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_BREAK_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
            DebugUart_FlushRx();
            rx_errors++;
            break;
        default:
            break;
    }
}
