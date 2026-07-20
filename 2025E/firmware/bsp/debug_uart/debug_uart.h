/**
 * @file  debug_uart.h
 * @brief 非阻塞 debug 串口 (UART0/BlueTooth) 输出: 环形缓冲 + TX 中断排空。
 *
 * 控制环只把字节写入环形缓冲即返回 (缓冲满则丢弃并计数, 绝不忙等), TX 由 UART0
 * TX 中断慢慢排空, 对控制链路零阻塞。生产者(线程) 写 head, 消费者(TX ISR) 写 tail,
 * 单生产者单消费者, 无需加锁。
 */
#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 debug 串口输出 (设置 TX FIFO 阈值, 关闭 TX 中断待用)。
 * @note  须在 SYSCFG_DL_init() 之后调用; UART0 的 RX 中断由 main 另行配置。
 */
void DebugUart_Init(void);

/**
 * @brief 非阻塞写入若干字节到发送环形缓冲 (缓冲满丢弃剩余并计数)。
 */
void DebugUart_Write(const uint8_t *data, uint16_t len);

/**
 * @brief 非阻塞写入以 '\0' 结尾的字符串。
 */
void DebugUart_Puts(const char *s);

/**
 * @brief printf 风格格式化后非阻塞发送 (单条上限 DEBUG_UART_PRINTF_MAX 字节)。
 */
void DebugUart_Printf(const char *fmt, ...);

/**
 * @brief 在 debug 串口(Debug_Ex)中断处理的 TX 分支调用: 把环形缓冲排入 TX FIFO。
 */
void DebugUart_TxIsr(void);

/**
 * @brief 获取因缓冲满被丢弃的累计字节数 (诊断输出是否过载)。
 */
uint32_t DebugUart_GetDroppedBytes(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_UART_H */
