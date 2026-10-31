# bsp/oled 接口说明

## 模块职责

`bsp/oled` 负责 SSD1306 OLED 的初始化、清屏、清行、字符、字符串、数字和位图显示。当前实现使用 SysConfig 配置的硬件 I2C 控制器，整帧数据由 DMA 搬运，并依赖 `bsp/time` 提供初始化阶段所需的毫秒阻塞延时。

该模块不负责菜单布局、错误文案组织、页面刷新策略或业务状态显示。

## 显示模型：帧缓冲 + DMA 整帧刷新

所有绘制接口只写 RAM 帧缓冲，纯 CPU 操作，不碰 I2C 总线。真正上屏由 `OLED_Flush()` 触发：

1. 等上一帧传完（正常情况下已经完成，不阻塞）；
2. 用阻塞方式发两条 4 字节的水平寻址窗口命令（列 `0..127`、页 `0..7`）；
3. 把 1KB 帧缓冲整块拷进 DMA 发送缓冲，缓冲首字节是 `0x40` 数据控制字；
4. 起一次 1025 字节的 I2C 事务后**立即返回**，数据由 DMA 逐字节灌进 TX FIFO。

水平寻址下写满 128 列自动换页，1024 字节正好铺满全屏，因此整帧只需一次事务。

### 为什么是双缓冲

绘制缓冲和 DMA 发送缓冲是两块独立的 1KB RAM。`OLED_Flush()` 返回时 DMA 还在读发送缓冲，此时上层已经可以继续画下一帧；若两者共用一块内存，这些绘制会改到正在传输的字节上，屏幕会撕裂。代价是 1KB 额外 ZI 和每帧一次 1KB `memcpy`（约几十微秒，相对 23ms 的传输时间可以忽略）。

### DMA 与中断配置

DMA 通道、触发源、I2C 事件发布和完成中断**全部在 `G3507.syscfg` 中配置**，由 SysConfig 生成到 `ti_msp_dl_config.c/h`。`G3507.syscfg` 中 OLED 实例的相关配置行：

```js
I2C1.advControllerTXFIFOTRIG               = "BYTES_1";
I2C1.intController                         = ["ARBITRATION_LOST","NACK","STOP"];
I2C1.interruptPriority                     = "3";
I2C1.DMAEvent1                             = "CONTROLLER_TXFIFO_TRIGGER";
I2C1.enableDMAEvent1                       = true;
I2C1.DMA_CHANNEL_EVENT1.$name              = "DMA_CH_OLED_TX";
I2C1.DMA_CHANNEL_EVENT1.addressMode        = "b2f";
I2C1.DMA_CHANNEL_EVENT1.srcLength          = "BYTE";
I2C1.DMA_CHANNEL_EVENT1.dstLength          = "BYTE";
I2C1.DMA_CHANNEL_EVENT1.srcIncDec          = "INCREMENT";
I2C1.DMA_CHANNEL_EVENT1.peripheral.$assign = "DMA_CH0";
```

生成的相关宏：

```c
#define DMA_CH_OLED_TX_CHAN_ID    (0)
#define OLED_INST_DMA_TRIGGER     (DMA_I2C1_TX_TRIG)
```

| 项 | 取值 | 说明 |
|---|---|---|
| DMA 通道 | `DMA_CH_OLED_TX_CHAN_ID`（CH0，Full Channel） | 由 syscfg 分配；新增 DMA 通道时不会与之冲突 |
| 触发源 | `OLED_INST_DMA_TRIGGER` = `DMA_I2C1_TX_TRIG` | 随 `OLED_INST` 自动生成 |
| 传输模式 | `SINGLE` + `NORMAL`，字节宽度 | 源地址递增，目的地址固定 |
| TX FIFO 阈值 | `DL_I2C_TX_FIFO_LEVEL_BYTES_1` | FIFO 抽干前就补数据，避免控制器拉伸 SCL（默认 `EMPTY` 会每字节拉伸一次） |
| DMA 事件 | `DL_I2C_EVENT_ROUTE_1` ← `CONTROLLER_TXFIFO_TRIGGER` | SysConfig 自身的配对方式 |
| 完成中断 | `I2C1_IRQHandler`，优先级 3（最低） | `STOP` 收尾；`NACK`/仲裁丢失置错误标志并复位 |

`bsp/oled` 只做生成代码不覆盖的两件事：把通道目的地址指向 `OLED_INST->MASTER.MTXDATA`、`NVIC_EnableIRQ()`；每帧再填源地址与传输长度。

启动顺序是**先使能 DMA 通道、再拉起始位**：FIFO 空时 DMA 触发条件已成立，会先把 FIFO 填上，这样 SCL 一走就有数据可发。

### 异常与兜底

- `NACK`、仲裁丢失、总线错误、以及 `OLED_FLUSH_TIMEOUT` 圈自旋超时，都会停通道、复位控制器事务、清 TX FIFO，把状态放回空闲，一次故障不会把 OLED 永久锁死。
- `OLED_WaitFlushDone()` 除了等中断标志，还并行轮询「DMA `SIZE` 归零且控制器不 BUSY」。`main()` 的 `__enable_irq()` 在 `App_Init()` 之后，`OLED_Init()` 阶段中断可能还没放开，靠这条轮询兜底。
- 超时上限用自旋圈数而不是 `BSP_Time_GetMs()`：SysTick 要到 `Scheduler_EnableTick()` 才放开，初始化阶段毫秒计数还不走。
- `OLED_StartFrameDMA()` 失败（未初始化 / 总线不空闲）时退回阻塞整帧发送，不因此丢帧。

命令包（初始化序列、开关显示、反色、寻址窗口）仍走阻塞路径，每包 ≤ 8 字节。命令与整帧共用总线，因此阻塞写入口会先等在途的 DMA 帧结束。

## 硬件映射

I2C 外设实例、总线速率和 PA10/PA11 引脚复用由 SysConfig 生成文件提供。当前生成配置中存在：

```c
#define OLED_INST         I2C1
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

配置 DMA 通道与 I2C 完成中断，初始化 SSD1306，并清屏后打开显示。

### `void OLED_Flush(void)`

把帧缓冲整帧刷到屏幕。**非阻塞**：拷完发送缓冲、起好事务就返回，剩余传输（400kHz 下约 23ms）在后台完成，返回后可立刻继续绘制下一帧。

若上一帧尚未发完，本函数会先等它结束——发送缓冲要整块换掉，不能边发边改。因此两次 `OLED_Flush()` 间隔短于一帧传输时间时，后一次仍会阻塞。

绘制接口只写帧缓冲，不调用本函数就不会显示。

### `bool OLED_IsFlushBusy(void)`

查询是否还有整帧传输在途。返回 `true` 时调用 `OLED_Flush()` 会阻塞等待。

### `bool OLED_WaitFlushDone(void)`

阻塞等待在途的整帧传输结束。返回 `true` 表示正常结束；返回 `false` 表示该帧以 NACK、仲裁丢失或超时收场（已复位控制器）。

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

- SSD1306 写命令、寻址窗口设置
- 硬件 I2C 阻塞发送与总线空闲等待（命令包专用）
- DMA 通道配置、整帧传输的启动、完成判定与异常中止
- `I2C1_IRQHandler`（传输收尾）
- 数字显示使用的幂函数

上层应只调用公开显示接口。
