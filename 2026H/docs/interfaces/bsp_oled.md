# bsp/oled 接口说明

## 模块职责

`bsp/oled` 负责 SSD1306 OLED 的初始化、清屏、清行、字符、字符串、数字和位图显示。当前实现使用 SysConfig 配置的硬件 I2C 控制器，并依赖 `bsp/time` 提供初始化阶段所需的毫秒阻塞延时。

该模块不负责菜单布局、错误文案组织、页面刷新策略或业务状态显示。

## 硬件映射

I2C 外设实例、总线速率和 PA10/PA11 引脚复用由 SysConfig 生成文件提供。当前生成配置中存在：

```c
#define OLED_INST         I2C0
#define OLED_BUS_SPEED_HZ 400000
#define GPIO_OLED_SDA_PIN DL_GPIO_PIN_10
#define GPIO_OLED_SCL_PIN DL_GPIO_PIN_11
```

SSD1306 的 7 位 I2C 地址在驱动内部固定为 `0x3C`。旧软件 I2C 中使用的 `0x78` 是包含写方向位的 8 位地址，不再作为硬件 I2C 传输地址使用。

## 屏幕尺寸宏

```c
#define OLED_WIDTH      128U
#define OLED_HEIGHT     64U
#define OLED_PAGE_COUNT 8U
```

坐标约定：

- `x` 为列坐标，范围 `0..127`。
- `page` 为页坐标，范围 `0..7`。
- 8 点高字体占 1 页，16 点高字体占 2 页。

## 公开接口

### `void OLED_Init(void)`

初始化 SSD1306，并清屏后打开显示。

### `void OLED_DisplayOn(void)`

打开 OLED 显示。

### `void OLED_DisplayOff(void)`

关闭 OLED 显示。

### `void OLED_Display_On(void)`

兼容旧命名，内部调用 `OLED_DisplayOn()`。

### `void OLED_Display_Off(void)`

兼容旧命名，内部调用 `OLED_DisplayOff()`。

### `void OLED_ColorTurn(uint8_t enable)`

设置正常显示或反色显示。`enable == 0` 为正常显示，`enable == 1` 为反色显示。

### `void OLED_Clear(void)`

清空全屏。

### `void OLED_ClearLine(uint8_t page)`

清空指定页。`page >= OLED_PAGE_COUNT` 时直接返回。

### `void OLED_ShowChar(uint8_t x, uint8_t page, uint8_t chr, uint8_t sizey)`

显示一个 ASCII 字符。当前支持 `sizey == 8` 和 `sizey == 16`。不可显示字符会替换为 `?`。

### `void OLED_ShowNum(uint8_t x, uint8_t page, uint32_t num, uint8_t len, uint8_t sizey)`

按指定宽度显示无符号整数。

### `void OLED_ShowString(uint8_t x, uint8_t page, const char *chr, uint8_t sizey)`

显示以 `\0` 结尾的字符串。超过屏幕宽度的内容会截断。

### `void OLED_ShowStringClearLine(uint8_t x, uint8_t page, const char *chr, uint8_t sizey)`

先清空指定页，再显示字符串。

### `void OLED_DrawBMP(uint8_t x, uint8_t page, uint8_t sizex, uint8_t sizey, const uint8_t *bmp)`

按页格式绘制位图。`sizey` 按 8 点一页向上取整。

## 内部接口

以下能力已收敛到 `oled.c` 内部，不再由头文件公开：

- SSD1306 写命令和写数据
- 设置页地址和列地址
- 硬件 I2C 阻塞发送、总线空闲等待和分段页数据写入
- 数字显示使用的幂函数

上层应只调用公开显示接口。
