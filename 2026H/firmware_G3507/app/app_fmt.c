/**
 * @file  app_fmt.c
 * @brief 轻量数字格式化实现。
 */
#include "app_fmt.h"

#include <stdbool.h>

/* 反转 buf[0..len-1]。 */
static void AppFmt_Reverse(char *buf, uint8_t len){
    uint8_t i = 0U;
    uint8_t j = (len > 0U) ? (uint8_t)(len - 1U) : 0U;
    while (i < j){
        char t = buf[i];
        buf[i] = buf[j];
        buf[j] = t;
        i++;
        j--;
    }
}

/* 无符号整数写入 buf，返回写入长度（不含 '\0'，不终止）。 */
static uint8_t AppFmt_U32Raw(char *buf, uint32_t value){
    uint8_t n = 0U;
    do {
        buf[n++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    AppFmt_Reverse(buf, n);
    return n;
}

void AppFmt_I32(char *buf, int32_t value){
    uint8_t n = 0U;
    uint32_t mag;

    if (value < 0){
        buf[n++] = '-';
        mag = (uint32_t)(-(value + 1)) + 1U;   /* 安全取绝对值，避免 INT32_MIN 溢出 */
    } else {
        mag = (uint32_t)value;
    }

    n = (uint8_t)(n + AppFmt_U32Raw(&buf[n], mag));
    buf[n] = '\0';
}

void AppFmt_Fixed(char *buf, float value, uint8_t decimals){
    static const uint32_t POW10[5] = { 1U, 10U, 100U, 1000U, 10000U };
    uint8_t n = 0U;
    uint8_t dec = (decimals > 4U) ? 4U : decimals;
    uint32_t scale = POW10[dec];

    bool negative = (value < 0.0f);
    float mag = negative ? -value : value;

    /* 四舍五入到定点整数。 */
    uint32_t scaled = (uint32_t)(mag * (float)scale + 0.5f);
    uint32_t ip = scaled / scale;
    uint32_t fp = scaled % scale;

    if (negative && (scaled != 0U)){
        buf[n++] = '-';
    }

    n = (uint8_t)(n + AppFmt_U32Raw(&buf[n], ip));

    if (dec > 0U){
        buf[n++] = '.';
        /* 小数部分需补前导零到 dec 位。 */
        for (uint8_t d = dec; d > 0U; d--){
            uint32_t digit = (fp / POW10[d - 1U]) % 10U;
            buf[n++] = (char)('0' + digit);
        }
    }

    buf[n] = '\0';
}
