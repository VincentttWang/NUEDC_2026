/**
 * @file  ui.c
 * @brief Middleware 层轻量 OLED UI 渲染实现。
 */
#include "ui.h"
#include "oled.h"
#include <stddef.h>

#define UI_TITLE_PAGE 0U
#define UI_CONTENT_X 0U
#define UI_LIST_MARK_X 0U
#define UI_LIST_TEXT_X 8U

static const uint8_t ui_content_page[UI_CONTENT_LINE_COUNT] = {
    2U, 3U, 4U, 5U, 6U, 7U,
};

static const char *Ui_NormalizeText(const char *text){
    return (text == NULL) ? "" : text;
}

static const char *Ui_StatusText(UI_STATUS_LEVEL level){
    switch (level){
        case UI_STATUS_OK:
            return "[OK]";
        case UI_STATUS_WARN:
            return "[WARN]";
        case UI_STATUS_ERROR:
            return "[ERR]";
        case UI_STATUS_NORMAL:
        default:
            return "[INFO]";
    }
}

static uint8_t Ui_ContentPageOf(uint8_t line_index){
    if (line_index >= UI_CONTENT_LINE_COUNT){
        return 0xFFU;
    }

    return ui_content_page[line_index];
}

void Ui_Init(void){
    OLED_Init();
}

void Ui_Flush(void){
    OLED_Flush();
}

bool Ui_IsFlushBusy(void){
    return OLED_IsFlushBusy();
}

bool Ui_WaitFlushDone(void){
    return OLED_WaitFlushDone();
}

void Ui_Clear(void){
    OLED_Clear();
}

void Ui_ClearLine(uint8_t page){
    OLED_ClearLine(page);
}

void Ui_DrawText(uint8_t x, uint8_t page, const char *text){
    OLED_ShowString(x, page, Ui_NormalizeText(text), UI_FONT_SIZE);
}

void Ui_UpdateText(uint8_t x, uint8_t page, const char *text){
    OLED_ShowStringClearLine(x, page, Ui_NormalizeText(text), UI_FONT_SIZE);
}

void Ui_DrawTitle(const char *title){
    Ui_DrawText(0U, UI_TITLE_PAGE, title);
}

void Ui_DrawContentLine(uint8_t line_index, const char *text){
    uint8_t page = Ui_ContentPageOf(line_index);

    if (page == 0xFFU){
        return;
    }

    Ui_DrawText(UI_CONTENT_X, page, text);
}

void Ui_UpdateContentLine(uint8_t line_index, const char *text){
    uint8_t page = Ui_ContentPageOf(line_index);

    if (page == 0xFFU){
        return;
    }

    Ui_UpdateText(UI_CONTENT_X, page, text);
    OLED_Flush();   /* 帧缓冲: 单行更新后刷屏 */
}

void Ui_RenderTextPage(const UI_TEXT_PAGE *page){
    Ui_Clear();

    if (page == NULL){
        return;
    }

    Ui_DrawTitle(page->title);

    for (uint8_t i = 0U; i < UI_CONTENT_LINE_COUNT; i++){
        Ui_DrawContentLine(i, page->line[i]);
    }

    OLED_Flush();   /* 帧缓冲: 整页绘制后一次刷屏 */
}

void Ui_RenderLines(const char *title,
                    const char *line0,
                    const char *line1,
                    const char *line2,
                    const char *line3,
                    const char *line4,
                    const char *line5){
    UI_TEXT_PAGE page = {
        .title = title,
        .line = {
            line0,
            line1,
            line2,
            line3,
            line4,
            line5,
        },
    };

    Ui_RenderTextPage(&page);
}

void Ui_RenderStatusPage(const char *title,
                         UI_STATUS_LEVEL level,
                         const char *message,
                         const char *hint){
    Ui_RenderLines(title,
                   Ui_StatusText(level),
                   message,
                   hint,
                   NULL,
                   NULL,
                   NULL);
}

void Ui_RenderListPage(const UI_LIST_PAGE *page){
    Ui_Clear();

    if (page == NULL){
        return;
    }

    Ui_DrawTitle(page->title);

    for (uint8_t row = 0U; row < UI_LIST_VISIBLE_COUNT; row++){
        uint8_t item_index = (uint8_t)(page->first_visible_index + row);

        if (row >= UI_CONTENT_LINE_COUNT || item_index >= page->item_count){
            continue;
        }

        uint8_t display_page = Ui_ContentPageOf(row);
        const char *mark = (item_index == page->selected_index) ? ">" : " ";
        const char *item_text = "";

        if (page->items != NULL && page->items[item_index] != NULL){
            item_text = page->items[item_index];
        }

        Ui_DrawText(UI_LIST_MARK_X, display_page, mark);
        Ui_DrawText(UI_LIST_TEXT_X, display_page, item_text);
    }

    OLED_Flush();   /* 帧缓冲: 整页绘制后一次刷屏 */
}
