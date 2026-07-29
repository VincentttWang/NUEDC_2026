/**
 * @file  key.h
 * @brief BSP 按键驱动接口。
 */
#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef KEY1_PORT
#define KEY1_PORT Key_PORT
#endif

#ifndef KEY1_PIN
#define KEY1_PIN Key_PIN_1_PIN
#endif

#ifndef KEY1_ACTIVE_LOW
#define KEY1_ACTIVE_LOW 1U
#endif

#ifndef KEY_DEBOUNCE_MS
#define KEY_DEBOUNCE_MS 20U
#endif

#ifndef KEY_SHORT_PRESS_MIN_MS
#define KEY_SHORT_PRESS_MIN_MS 50U
#endif

#ifndef KEY_LONG_PRESS_MS
#define KEY_LONG_PRESS_MS 1000U
#endif

#ifndef KEY_DOUBLE_CLICK_MS
#define KEY_DOUBLE_CLICK_MS 300U
#endif

/**
 * @brief 按键编号。
 */
typedef enum {
    KEY_ID_1 = 0,
    KEY_ID_MAX
} KEY_ID;

/**
 * @brief 按键事件类型。
 */
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT_PRESS,
    KEY_EVENT_LONG_PRESS,
    KEY_EVENT_DOUBLE_CLICK
} KEY_EVENT;

/**
 * @brief 初始化按键状态机。
 */
void Key_Init(void);

/**
 * @brief 执行一次按键扫描和消抖。
 * @note 建议由固定周期定时器或 SysTick 调用。
 */
void Key_Scan(void);

/**
 * @brief 获取指定按键最近事件，不自动清除事件。
 */
KEY_EVENT Key_GetEvent(KEY_ID key_id);

/**
 * @brief 判断指定按键当前是否处于按下状态。
 */
bool Key_IsPressed(KEY_ID key_id);

/**
 * @brief 判断指定按键是否产生短按事件。
 */
bool Key_IsShortPress(KEY_ID key_id);

/**
 * @brief 判断指定按键是否刚完成一次短按释放确认。
 * @note 该接口在释放消抖完成后立即返回 true，不等待双击窗口超时。
 *       若被消费，本次点击后续不会再生成普通短按事件。
 */
bool Key_IsShortRelease(KEY_ID key_id);

/**
 * @brief 判断指定按键是否产生长按事件。
 */
bool Key_IsLongPress(KEY_ID key_id);

/**
 * @brief 判断指定按键是否产生双击事件。
 */
bool Key_IsDoubleClick(KEY_ID key_id);

/**
 * @brief 清除指定按键事件。
 */
void Key_ClearEvent(KEY_ID key_id);

/**
 * @brief 清除全部按键事件。
 */
void Key_ClearAllEvents(void);

#define Key_Read()         Key_IsPressed(KEY_ID_1)
#define Key_short_press()  Key_IsShortPress(KEY_ID_1)
#define Key_long_press()   Key_IsLongPress(KEY_ID_1)
#define Key_double_click() Key_IsDoubleClick(KEY_ID_1)

#ifdef __cplusplus
}
#endif

#endif /* KEY_H */
