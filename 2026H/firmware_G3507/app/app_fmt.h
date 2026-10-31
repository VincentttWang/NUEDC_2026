/**
 * @file  app_fmt.h
 * @brief 轻量数字格式化（不引入浮点 printf）。
 *
 * 面向 OLED 文本显示：把整数/定点浮点写成以 '\0' 结尾的字符串。
 * 调用方负责提供足够大的缓冲区（建议 >= 16 字节）。
 */
#ifndef APP_FMT_H
#define APP_FMT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 有符号 32 位整数转字符串，如 -123。
 */
void AppFmt_I32(char *buf, int32_t value);

/**
 * @brief 定点浮点转字符串，保留 decimals 位小数（四舍五入），如 3.14。
 * @param decimals 小数位数，0..4。
 */
void AppFmt_Fixed(char *buf, float value, uint8_t decimals);

#ifdef __cplusplus
}
#endif

#endif /* APP_FMT_H */
