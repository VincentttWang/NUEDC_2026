/**
 * @file  debug_uart.c
 * @brief 非阻塞 debug 串口输出实现 (环形缓冲 + TX 中断)。
 */
#include "debug_uart.h"
#include "ti_msp_dl_config.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

/* debug 串口实例 (UART0, 见 board/sys_config)。 */
#ifndef DEBUG_UART_INST
#define DEBUG_UART_INST Debug_Ex_INST
#endif

/* TX 环形缓冲长度, 必须是 2 的幂 (用掩码回绕)。1KB @115200 约 90ms 缓冲。 */
#ifndef DEBUG_UART_TX_BUF_LEN
#define DEBUG_UART_TX_BUF_LEN 1024U
#endif
#define DEBUG_UART_TX_MASK (DEBUG_UART_TX_BUF_LEN - 1U)

/* 单条 Printf 格式化上限。瞄准遥测行(A,...)加 pitch 环+转弯占空比后 ~150 字符,
 * 128 会截断, 提到 256 留裕量 (115200 下 150 字符 ~13ms/20ms, 1KB 环形缓冲不溢出)。 */
#ifndef DEBUG_UART_PRINTF_MAX
#define DEBUG_UART_PRINTF_MAX 256U
#endif

static uint8_t s_tx_buf[DEBUG_UART_TX_BUF_LEN];
static volatile uint16_t s_tx_head;   /* 生产者(线程) 写入位置 */
static volatile uint16_t s_tx_tail;   /* 消费者(TX ISR) 读取位置 */
static volatile uint32_t s_dropped;   /* 缓冲满丢弃的字节数 */

void DebugUart_Init(void){
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_dropped = 0U;
    /* TX FIFO 降到阈值即触发中断续传; 无数据时 TX 中断保持关闭 (见 DebugUart_Write)。 */
    DL_UART_Main_setTXFIFOThreshold(DEBUG_UART_INST, DL_UART_TX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_disableInterrupt(DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
}

void DebugUart_TxIsr(void){
    /* 仅由 TX 中断调用: 把环形缓冲尽量塞进 TX FIFO; 缓冲空则关 TX 中断避免空触发。 */
    while ((s_tx_tail != s_tx_head) && !DL_UART_Main_isTXFIFOFull(DEBUG_UART_INST)){
        DL_UART_Main_transmitData(DEBUG_UART_INST, s_tx_buf[s_tx_tail]);
        s_tx_tail = (uint16_t)((s_tx_tail + 1U) & DEBUG_UART_TX_MASK);
    }
    if (s_tx_tail == s_tx_head){
        DL_UART_Main_disableInterrupt(DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    }
}

void DebugUart_Write(const uint8_t *data, uint16_t len){
    if (data == NULL){
        return;
    }

    for (uint16_t i = 0U; i < len; i++){
        uint16_t next = (uint16_t)((s_tx_head + 1U) & DEBUG_UART_TX_MASK);
        if (next == s_tx_tail){
            /* 缓冲满: 丢弃剩余, 绝不阻塞控制环。 */
            s_dropped += (uint32_t)(len - i);
            break;
        }
        s_tx_buf[s_tx_head] = data[i];
        s_tx_head = next;
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
    if (n > (int)sizeof(buf)){
        n = (int)sizeof(buf);   /* 截断: 超长内容按缓冲上限发送 */
    }
    DebugUart_Write((const uint8_t *)buf, (uint16_t)n);
}

uint32_t DebugUart_GetDroppedBytes(void){
    return s_dropped;
}
