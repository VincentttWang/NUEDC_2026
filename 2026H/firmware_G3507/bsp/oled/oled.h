/**
 * @file  oled.h
 * @brief SSD1306 OLED 显示接口。
 */

#ifndef OLED_H
#define OLED_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
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
 * @brief 把帧缓冲整帧刷新到屏幕 (水平寻址一次事务, DMA 搬运)。
 * @note 绘制函数只写帧缓冲, 须由上层在一批绘制后调用本函数才真正显示。
 * @note 非阻塞: 拷完发送缓冲、起好事务就返回, 剩下约 23ms 由 DMA 后台完成。
 *       返回后可立刻继续绘制下一帧。若上一帧尚未发完, 本函数会先等它结束。
 */
void OLED_Flush(void);

/**
 * @brief 查询是否还有整帧 DMA 传输在途。
 * @return true 表示传输未结束, 此时调用 OLED_Flush() 会阻塞等待。
 */
bool OLED_IsFlushBusy(void);

/**
 * @brief 阻塞等待在途的整帧传输结束。
 * @return true 表示传输正常结束; false 表示以 NACK/仲裁丢失/超时收场
 *         (已复位控制器, 不会把 OLED 永久锁死)。
 */
bool OLED_WaitFlushDone(void);

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
