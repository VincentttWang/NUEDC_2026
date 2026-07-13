# bsp/motor/hall_encoder 接口说明

## 模块职责

`bsp/motor/hall_encoder` 负责霍尔编码器 A/B 相脉冲计数、采样周期更新、方向判断、速度换算和轮端距离估计。

当前模块基于固定车体参数提供 `HallEncoder_GetSpeed()` 和 `HallEncoder_GetDistance()`。它不负责底盘运动控制，也不负责速度闭环。

## 硬件映射宏

```c
#define HALL_ENCODER_A_PORT Motor_IO_E1A_PORT
#define HALL_ENCODER_A_PIN  Motor_IO_E1A_PIN
#define HALL_ENCODER_B_PORT Motor_IO_E1A_PORT
#define HALL_ENCODER_B_PIN  Motor_IO_E2A_PIN
```

当前默认 A/B 相在同一个 GPIO 端口上。`GROUP1_IRQHandler()` 使用 `HALL_ENCODER_A_PORT` 获取并清除中断状态；如果后续 A/B 相拆到不同端口，需要同步调整中断入口策略。

## 中断和采样宏

```c
#define HALL_ENCODER_GPIO_IRQN         GPIOA_INT_IRQn
#define HALL_ENCODER_SAMPLE_TIMER      TIMER_0_INST
#define HALL_ENCODER_SAMPLE_TIMER_IRQN TIMER_0_INST_INT_IRQN
```

当前保留 `GROUP1_IRQHandler()` 和 `TIMER_0_INST_IRQHandler()` 外壳，内部转调：

- `HallEncoder_HandleGpioIrq()`
- `HallEncoder_UpdateSample()`

后续如果建立统一中断分发模块，可以迁移这两个外壳，保留内部接口。

## 物理参数宏

```c
#define HALL_ENCODER_PI               3.1415926f
#define HALL_ENCODER_PPR              13.0f
#define HALL_ENCODER_REDUCTION_RATIO  28.0f
#define HALL_ENCODER_WHEEL_DIAMETER_M 0.065f
#define HALL_ENCODER_SAMPLE_PERIOD_S  0.01f
#define HALL_ENCODER_DISTANCE_SCALE   1.05f
```

距离换算公式：

```text
distance_m = count / (PPR * reduction_ratio)
           * (PI * wheel_diameter_m)
           * distance_scale
```

速度由当前采样周期的距离增量除以 `HALL_ENCODER_SAMPLE_PERIOD_S` 得到，单位为 m/s。

## 公开类型

```c
typedef enum {
    HALL_ENCODER_DIR_FORWARD = 0,
    HALL_ENCODER_DIR_REVERSE
} HALL_ENCODER_DIR;
```

## 公开接口

### `BSP_STATUS HallEncoder_Init(void)`

清零内部状态，打开 GPIO 中断和采样定时器中断。

### `void HallEncoder_HandleGpioIrq(uint32_t gpio_status)`

处理 A/B 相 GPIO 中断状态并累计待采样脉冲数。

### `void HallEncoder_UpdateSample(void)`

将待采样脉冲数转为当前采样周期计数，更新方向、速度和累计距离。

### `int32_t HallEncoder_GetCount(void)`

返回最近一个采样周期的有符号脉冲数。

### `HALL_ENCODER_DIR HallEncoder_GetDir(void)`

返回最近一个采样周期方向。采样计数大于等于 0 时为 `HALL_ENCODER_DIR_FORWARD`。

### `float HallEncoder_GetSpeed(void)`

返回最近一个采样周期换算得到的有符号轮端线速度，单位 m/s。

### `float HallEncoder_GetDistance(void)`

返回自上次 `HallEncoder_Reset()` 或 `HallEncoder_ResetDistance()` 以来的有符号轮端距离估计，单位 m。

### `void HallEncoder_Reset(void)`

清空待采样计数、采样计数、累计计数、速度、距离和方向。

### `void HallEncoder_ResetDistance(void)`

只清空累计计数和距离，不影响当前待采样计数、采样计数和速度。

## 对接说明

旧 `Encoder_*` 接口已不再作为当前应用层入口使用。底盘组合服务通过 `HallEncoder_*` 获取速度和距离估计。
