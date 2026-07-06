/**
 * @file  ui.h
 * @brief Middleware 层轻量 OLED UI 渲染接口。
 */
#ifndef UI_H
#define UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UI_CONTENT_LINE_COUNT 6U
#define UI_LIST_VISIBLE_COUNT 6U
#define UI_FONT_SIZE 8U

/**
 * @brief 状态页等级。
 */
typedef enum {
    UI_STATUS_NORMAL = 0,
    UI_STATUS_OK,
    UI_STATUS_WARN,
    UI_STATUS_ERROR
} UI_STATUS_LEVEL;

/**
 * @brief 文本页描述。
 */
typedef struct {
    /** 页面标题。 */
    const char *title;
    /** 内容行文本，允许为 NULL。 */
    const char *line[UI_CONTENT_LINE_COUNT];
} UI_TEXT_PAGE;

/**
 * @brief 列表页描述。
 */
typedef struct {
    /** 页面标题。 */
    const char *title;
    /** 列表项字符串数组。 */
    const char *const *items;
    /** 列表项总数。 */
    uint8_t item_count;
    /** 当前选中项索引。 */
    uint8_t selected_index;
    /** 首个可见项索引。 */
    uint8_t first_visible_index;
} UI_LIST_PAGE;

/**
 * @brief 初始化 OLED 并清屏。
 */
void Ui_Init(void);

/**
 * @brief 清空整屏。
 */
void Ui_Clear(void);

/**
 * @brief 清空指定页行。
 */
void Ui_ClearLine(uint8_t page);

/**
 * @brief 在指定位置绘制文本，不主动清行。
 */
void Ui_DrawText(uint8_t x, uint8_t page, const char *text);

/**
 * @brief 清空指定行后绘制文本。
 */
void Ui_UpdateText(uint8_t x, uint8_t page, const char *text);

/**
 * @brief 绘制页面标题。
 */
void Ui_DrawTitle(const char *title);

/**
 * @brief 绘制内容区指定行。
 */
void Ui_DrawContentLine(uint8_t line_index, const char *text);

/**
 * @brief 清空并更新内容区指定行。
 */
void Ui_UpdateContentLine(uint8_t line_index, const char *text);

/**
 * @brief 渲染完整文本页。
 */
void Ui_RenderTextPage(const UI_TEXT_PAGE *page);

/**
 * @brief 便捷渲染标题和最多 6 行文本。
 */
void Ui_RenderLines(const char *title,
                    const char *line0,
                    const char *line1,
                    const char *line2,
                    const char *line3,
                    const char *line4,
                    const char *line5);

/**
 * @brief 渲染状态页。
 */
void Ui_RenderStatusPage(const char *title,
                         UI_STATUS_LEVEL level,
                         const char *message,
                         const char *hint);

/**
 * @brief 渲染列表页。
 */
void Ui_RenderListPage(const UI_LIST_PAGE *page);

#ifdef __cplusplus
}
#endif

#endif /* UI_H */
