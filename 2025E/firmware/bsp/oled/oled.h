/**
 * @file  oled.h
 * @brief SSD1306 OLED 显示接口。
 */

#ifndef OLED_H
#define OLED_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_WIDTH      128U
#define OLED_HEIGHT     64U
#define OLED_PAGE_COUNT 8U

/**
 * @brief 初始化 SSD1306 OLED。
 */
void OLED_Init(void);

/**
 * @brief 打开 OLED 显示。
 */
void OLED_DisplayOn(void);

/**
 * @brief 关闭 OLED 显示。
 */
void OLED_DisplayOff(void);

/**
 * @brief 兼容旧接口的打开显示函数。
 */
void OLED_Display_On(void);

/**
 * @brief 兼容旧接口的关闭显示函数。
 */
void OLED_Display_Off(void);

/**
 * @brief 设置正常/反色显示。
 * @param enable 非 0 表示反色显示，0 表示正常显示。
 */
void OLED_ColorTurn(uint8_t enable);

/**
 * @brief 清空整屏。
 */
void OLED_Clear(void);

/**
 * @brief 清空指定页。
 */
void OLED_ClearLine(uint8_t page);

/**
 * @brief 显示单个 ASCII 字符。
 */
void OLED_ShowChar(uint8_t x, uint8_t page, uint8_t chr, uint8_t sizey);

/**
 * @brief 显示无符号整数。
 */
void OLED_ShowNum(uint8_t x, uint8_t page, uint32_t num, uint8_t len, uint8_t sizey);

/**
 * @brief 显示字符串。
 */
void OLED_ShowString(uint8_t x, uint8_t page, const char *chr, uint8_t sizey);

/**
 * @brief 清空指定页后显示字符串。
 */
void OLED_ShowStringClearLine(uint8_t x, uint8_t page, const char *chr, uint8_t sizey);

/**
 * @brief 绘制位图。
 * @param x 起始列。
 * @param page 起始页。
 * @param sizex 位图宽度，单位 px。
 * @param sizey 位图高度，单位 px。
 * @param bmp 位图数据。
 */
void OLED_DrawBMP(uint8_t x, uint8_t page, uint8_t sizex, uint8_t sizey, const uint8_t *bmp);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */
