/**
 * @file  app_debug_cmd.h
 * @brief debug 串口(Debug_Ex)上行命令处理: 上位机实时调 PID 参数。
 *
 * 协议 (ASCII 行, PC->MCU):
 *   "s <key> <val>\n"  设置参数, key: ykp/yki/ykd/pkp/pki/pkd/olim
 *   "g\n" 或 "?\n"     查询当前参数
 * MCU 收到后应用到 GimbalTracking 并回显一行 "[CFG] ykp=.. ...".
 *
 * 分工: ISR 只组行 (AppDebugCmd_FeedByte), 线程侧解析/应用/回显 (AppDebugCmd_Poll),
 * 避免在 ISR 里调 DebugUart 破坏其单生产者约束。
 */
#ifndef APP_DEBUG_CMD_H
#define APP_DEBUG_CMD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 在 debug 串口 RX 中断里逐字节喂入 (组装命令行, 遇换行标记就绪)。
 */
void AppDebugCmd_FeedByte(uint8_t byte);

/**
 * @brief 线程侧轮询: 若有完整命令行则解析并应用, 回显当前配置。
 * @note 在会用到调参的任务循环里周期调用 (如持续瞄准循环)。
 */
void AppDebugCmd_Poll(void);

/**
 * @brief 经 debug 串口回显当前 PID 配置一行 (供上位机同步滑块)。
 */
void AppDebugCmd_EmitConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEBUG_CMD_H */
