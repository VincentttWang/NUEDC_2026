# bsp/time 接口说明

## 模块职责

`bsp/time` 负责提供板级系统毫秒计数和阻塞延时能力。该模块位于 BSP 层，允许 `app`、`core`、`middleware` 和其他 BSP 外设驱动调用；它不依赖上层模块。

## 文件

- `bsp/time/bsp_time.h`：公开系统时间和延时接口。
- `bsp/time/bsp_time.c`：保存毫秒 tick，并基于 `DL_Common_delayCycles()` 实现阻塞延时。

## 公开接口

### `void BSP_Time_Init(void)`

初始化 BSP 时间模块，当前行为是清零毫秒计数。应在 `SYSCFG_DL_init()` 之后、其他依赖时间的模块初始化之前调用。

### `void BSP_Time_TickInc(void)`

递增 1 ms 系统计数。当前由 `SysTick_Handler()` 调用，要求 SysTick 周期保持 1 ms。

### `uint32_t BSP_Time_GetMs(void)`

返回当前毫秒计数。计数溢出按 `uint32_t` 自然回绕处理，调用方应使用减法比较时间间隔。

### `void BSP_DelayUs(uint32_t us)`

执行微秒级阻塞延时。延时基于 `BSP_TIME_CPUCLK_HZ` 和 `DL_Common_delayCycles()`，默认 CPU 时钟为 `32000000U`。

### `void BSP_DelayMs(uint32_t ms)`

执行毫秒级阻塞延时，内部循环调用 `BSP_DelayUs(1000U)`。

## 依赖关系

- 依赖 TI DriverLib 的 `DL_Common_delayCycles()`。
- 被 `bsp/key` 用作按键消抖时间源。
- 被 `bsp/oled` 用作软件 I2C 时序延时。
- 被上层控制和测试流程用于刷新周期、状态超时和短阻塞等待。

## 设计约束

- `bsp/time` 不保存业务状态，不感知按键、OLED、电机、视觉等具体设备。
- 调用方不得直接访问全局 tick 变量；统一通过 `BSP_Time_GetMs()` 读取。
- 如果后续系统时钟不是 32 MHz，应在编译配置中覆盖 `BSP_TIME_CPUCLK_HZ`，或在本模块内改为从时钟配置派生。
