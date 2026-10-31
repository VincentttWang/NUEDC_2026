# middleware/ui 接口说明

## 模块职责

`middleware/ui` 是面向当前 128x64 OLED 的轻量立即模式 UI 渲染层。它参考轻量 OLED UI 框架的页面、列表和状态页思想，但不引入对象树、动画、输入事件或菜单状态机。

该模块负责：

- OLED 初始化入口。
- 清屏和清行。
- 绘制基础文本。
- 绘制标题和正文行。
- 渲染文本页。
- 渲染状态页。
- 渲染列表页。

该模块不负责：

- 按键输入。
- 菜单节点和页面跳转。
- 任务流程。
- 动画和长文本滚动。
- 动态内存分配。
- 复杂窗口系统。

`app` 层仍负责决定显示哪个页面、如何响应按键、菜单如何跳转。`core` 原则上不直接调用 UI。

## 公开宏

```c
#define UI_CONTENT_LINE_COUNT 6U
#define UI_LIST_VISIBLE_COUNT 6U
#define UI_FONT_SIZE 8U
```

- `UI_CONTENT_LINE_COUNT`：文本页正文逻辑行数。
- `UI_LIST_VISIBLE_COUNT`：列表页最多显示的条目数。
- `UI_FONT_SIZE`：当前固定使用 8 点字体。

正文逻辑行映射到 OLED page：

```text
line 0 -> page 2
line 1 -> page 3
line 2 -> page 4
line 3 -> page 5
line 4 -> page 6
line 5 -> page 7
```

## 公开类型

### `UI_STATUS_LEVEL`

```c
typedef enum {
    UI_STATUS_NORMAL = 0,
    UI_STATUS_OK,
    UI_STATUS_WARN,
    UI_STATUS_ERROR
} UI_STATUS_LEVEL;
```

状态页等级。OLED 没有颜色，因此当前通过文本前缀表示：

- `UI_STATUS_NORMAL` -> `[INFO]`
- `UI_STATUS_OK` -> `[OK]`
- `UI_STATUS_WARN` -> `[WARN]`
- `UI_STATUS_ERROR` -> `[ERR]`

### `UI_TEXT_PAGE`

```c
typedef struct {
    const char *title;
    const char *line[UI_CONTENT_LINE_COUNT];
} UI_TEXT_PAGE;
```

文本页结构。`title` 显示在 page 0，`line[]` 按正文逻辑行显示。

### `UI_LIST_PAGE`

```c
typedef struct {
    const char *title;
    const char *const *items;
    uint8_t item_count;
    uint8_t selected_index;
    uint8_t first_visible_index;
} UI_LIST_PAGE;
```

列表页结构。

- `items`：字符串数组。
- `item_count`：条目数量。
- `selected_index`：当前选中项。
- `first_visible_index`：第一条可见项。

当前选中项用左侧 `>` 标记。

## 公开接口

### `void Ui_Init(void)`

初始化 UI 底层显示，内部调用 `OLED_Init()`。

### `void Ui_Flush(void)`

把帧缓冲整帧刷新到屏幕，透传 `OLED_Flush()`。**非阻塞**：整帧由 DMA 后台搬运（400kHz 下约 23ms），返回后可立刻继续绘制。上一帧未发完时本函数会先等它结束，因此两次刷屏间隔短于一帧传输时间时后一次仍会阻塞。

### `bool Ui_IsFlushBusy(void)`

查询是否还有整帧传输在途，透传 `OLED_IsFlushBusy()`。返回 `true` 时调用 `Ui_Flush()` 会阻塞等待。

### `bool Ui_WaitFlushDone(void)`

阻塞等待在途的整帧传输结束，透传 `OLED_WaitFlushDone()`。返回 `false` 表示该帧以总线异常或超时收场。

### `void Ui_Clear(void)`

清空屏幕。

### `void Ui_ClearLine(uint8_t page)`

清空指定 OLED page。

### `void Ui_DrawText(uint8_t x, uint8_t page, const char *text)`

在指定坐标绘制文本。`text == NULL` 时按空字符串处理。

### `void Ui_UpdateText(uint8_t x, uint8_t page, const char *text)`

先清空指定 page，再绘制文本。

### `void Ui_DrawTitle(const char *title)`

在 page 0 绘制标题。

### `void Ui_DrawContentLine(uint8_t line_index, const char *text)`

绘制正文逻辑行。非法 `line_index` 直接返回。

### `void Ui_UpdateContentLine(uint8_t line_index, const char *text)`

先清空正文逻辑行对应的 page，再绘制文本。

### `void Ui_RenderTextPage(const UI_TEXT_PAGE *page)`

清屏并渲染文本页。`page == NULL` 时只清屏。

### `void Ui_RenderLines(...)`

文本页便利接口：一次调用传标题与至多 6 行正文，内部只 Flush 一次。

### `void Ui_RenderStatusPage(const char *title, UI_STATUS_LEVEL level, const char *message, const char *hint)`

渲染状态页，适合后续 `fault` 或设备状态页使用。

### `void Ui_RenderListPage(const UI_LIST_PAGE *page)`

渲染列表页。该接口只负责显示，不处理按键、不修改选中项、不做页面跳转。

## 后续扩展

当前为最小实现。等其余工程迁移完成后，再评估是否增加：

- 弹窗。
- 长文本滚动。
- 光标动画。
- 进度条。
- 菜单对象模型。
