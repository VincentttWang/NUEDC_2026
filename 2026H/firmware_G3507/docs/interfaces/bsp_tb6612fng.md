# bsp/motor/tb6612fng 接口说明

## 模块职责

`bsp/motor/tb6612fng` 负责 TB6612FNG 双路直流电机驱动芯片的 IN1、IN2 和 PWM 输出控制。

该模块只处理芯片级输出，不负责：

- 左右轮底盘语义之外的运动组合
- 速度、轮径、减速比换算
- 编码器反馈
- 闭环控制

这些能力后续应由 `middleware`、`service` 或 `core` 层继续承接。

## 硬件映射宏

驱动使用宏定义描述当前板级接线。默认映射如下：

- `TB6612FNG_LEFT_IN1_PORT/PIN` -> `Motor_IO_BIN1_PORT/PIN`
- `TB6612FNG_LEFT_IN2_PORT/PIN` -> `Motor_IO_BIN2_PORT/PIN`
- `TB6612FNG_LEFT_PWM_TIMER/CHANNEL` -> `Motor_Left_INST` / `DL_TIMER_CC_1_INDEX`（PA7，TB6612 PWMB）
- `TB6612FNG_RIGHT_IN1_PORT/PIN` -> `Motor_IO_AIN1_PORT/PIN`
- `TB6612FNG_RIGHT_IN2_PORT/PIN` -> `Motor_IO_AIN2_PORT/PIN`
- `TB6612FNG_RIGHT_PWM_TIMER/CHANNEL` -> `Motor_Right_INST` / `DL_TIMER_CC_0_INDEX`（PB10，TB6612 PWMA）

如果板级接线变化，应通过覆盖这些宏完成配置。

## PWM 周期

```c
#ifndef TB6612FNG_PWM_PERIOD
#define TB6612FNG_PWM_PERIOD 1000U
#endif
```

`TB6612FNG_SetDuty()` 对外接收 `-100.0f` 到 `100.0f` 的百分比，占空比换算由本模块内部完成。

## 公开类型

### `TB6612FNG_CHANNEL`

```c
typedef enum {
    TB6612FNG_CHANNEL_LEFT = 0,
    TB6612FNG_CHANNEL_RIGHT,
    TB6612FNG_CHANNEL_MAX
} TB6612FNG_CHANNEL;
```

### `TB6612FNG_OUTPUT`

```c
typedef enum {
    TB6612FNG_OUTPUT_COAST = 0,
    TB6612FNG_OUTPUT_FORWARD,
    TB6612FNG_OUTPUT_BACKWARD,
    TB6612FNG_OUTPUT_BRAKE
} TB6612FNG_OUTPUT;
```

`COAST` 表示 IN1/IN2 均为低电平，`BRAKE` 表示 IN1/IN2 均为高电平。

## 公开接口

### `BSP_STATUS TB6612FNG_Init(void)`

启动 PWM 定时器，并将所有通道置为滑行状态。

### `BSP_STATUS TB6612FNG_SetDuty(TB6612FNG_CHANNEL channel, float duty_percent)`

设置指定通道百分比占空比：

- `duty_percent > 0.0f`：正转
- `duty_percent < 0.0f`：反转
- `duty_percent == 0.0f`：滑行

输入会被限制到 `[-100.0f, 100.0f]`。非法通道返回 `BSP_STATUS_INVALID_ARG`。

### `BSP_STATUS TB6612FNG_Brake(TB6612FNG_CHANNEL channel)`

指定通道主动刹车，并将占空比记录为 `0.0f`。

### `BSP_STATUS TB6612FNG_Coast(TB6612FNG_CHANNEL channel)`

指定通道滑行，并将占空比记录为 `0.0f`。

### `BSP_STATUS TB6612FNG_BrakeAll(void)`

所有通道主动刹车。

### `BSP_STATUS TB6612FNG_CoastAll(void)`

所有通道滑行。

### `float TB6612FNG_GetDuty(TB6612FNG_CHANNEL channel)`

返回最近一次记录的带符号百分比占空比。非法通道返回 `0.0f`。

### `TB6612FNG_OUTPUT TB6612FNG_GetOutputStatus(TB6612FNG_CHANNEL channel)`

返回最近一次记录的输出状态。非法通道返回 `TB6612FNG_OUTPUT_COAST`。

## 对接说明

旧 `Motor_*` 类型和函数已不再作为当前应用层入口使用。底盘组合服务通过 `TB6612FNG_*` 设置左右轮输出。
