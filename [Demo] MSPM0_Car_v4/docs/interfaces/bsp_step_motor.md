# bsp/step_motor 接口说明

## 模块职责

`bsp/step_motor` 负责 yaw/pitch 两路步进电机的方向引脚、PWM 脉冲输出、速度设置、阻塞定时运行、开环位置估算和 pitch 基础位置限位。

该模块不负责云台组合控制、目标跟踪或闭环校正。这些能力应由上层 `middleware`、`service/core` 或 `app` 承接。

## 通道

```c
typedef enum {
    STEP_MOTOR_CHANNEL_YAW = 0,
    STEP_MOTOR_CHANNEL_PITCH,
    STEP_MOTOR_CHANNEL_MAX
} STEP_MOTOR_CHANNEL;
```

## 硬件映射宏

默认映射：

- `STEP_MOTOR_YAW_DIR_PORT/PIN` -> `SMotor_IO_PORT` / `SMotor_IO_DIR2_PIN`
- `STEP_MOTOR_YAW_EN_PORT/PIN` -> `SMotor_IO_PORT` / `SMotor_IO_EN2_PIN`
- `STEP_MOTOR_YAW_PWM_TIMER/CHANNEL` -> `SMotor_2_INST` / `DL_TIMER_CC_1_INDEX`
- `STEP_MOTOR_PITCH_DIR_PORT/PIN` -> `SMotor_IO_PORT` / `SMotor_IO_DIR1_PIN`
- `STEP_MOTOR_PITCH_EN_PORT/PIN` -> `SMotor_IO_PORT` / `SMotor_IO_EN1_PIN`
- `STEP_MOTOR_PITCH_PWM_TIMER/CHANNEL` -> `SMotor_1_INST` / `DL_TIMER_CC_0_INDEX`

方向宏：

```c
#define STEP_MOTOR_YAW_POSITIVE_DIR_HIGH   1U
#define STEP_MOTOR_PITCH_POSITIVE_DIR_HIGH 0U
```

当速度为正时，方向引脚是否置高由上述宏决定；速度为负时方向取反。

使能宏：

```c
#define STEP_MOTOR_YAW_ENABLE_HIGH   1U
#define STEP_MOTOR_PITCH_ENABLE_HIGH 1U
```

`StepMotor_Init()` 会按上述极性使能两路步进电机驱动器。当前硬件与旧版 `YP_SMotor_Init()` 行为一致，默认将 `EN1/EN2` 置高。若后续更换为低电平有效的驱动器，只需把对应 `ENABLE_HIGH` 宏改为 `0U`。

## 电机和定时器参数宏

```c
#define STEP_MOTOR_STEP_ANGLE_DEG        1.8f
#define STEP_MOTOR_MICROSTEP             32.0f
#define STEP_MOTOR_TIMER_CLOCK_HZ        32000000U
#define STEP_MOTOR_TIMER_PRESCALER_FACTOR (32U * 8U * 2U)
#define STEP_MOTOR_MAX_ARR               65535U
#define STEP_MOTOR_MAX_SPEED_DEG_S       240.0f
#define STEP_MOTOR_PITCH_MIN_POSITION_DEG (-30.0f)
#define STEP_MOTOR_PITCH_MAX_POSITION_DEG 30.0f
```

步进脉冲频率换算：

```text
step_frequency = abs(speed_deg_per_s) / step_angle_deg * microstep
```

`StepMotor_SetSpeed()` 会先将输入速度限制到 `[-STEP_MOTOR_MAX_SPEED_DEG_S, STEP_MOTOR_MAX_SPEED_DEG_S]`，默认最大速度为 `240 deg/s`。该限幅位于 BSP 入口，`Gimbal_SetSpeed()` 和 `StepMotor_RunFor()` 等上层调用都会统一生效。

pitch 通道额外使用开环估计位置做基础限位。当前位置达到 `STEP_MOTOR_PITCH_MAX_POSITION_DEG` 时，继续输入正向速度会被置零；当前位置达到 `STEP_MOTOR_PITCH_MIN_POSITION_DEG` 时，继续输入反向速度会被置零。该限位同样位于 BSP 入口，直接调用 `StepMotor_*` 和通过 `Gimbal_*` 调用都会统一生效。

## 公开接口

### `BSP_STATUS StepMotor_Init(void)`

初始化两个通道状态，使能步进电机驱动器，启动 PWM 定时器，并关闭脉冲输出。

### `BSP_STATUS StepMotor_SetSpeed(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s)`

设置指定通道开环速度，单位 deg/s。正负号决定方向，`0.0f` 表示停止输出脉冲。

调用前会先按当前时间更新一次开环估计位置，避免速度切换时丢失上一段运动。

输入速度超出 `STEP_MOTOR_MAX_SPEED_DEG_S` 时会按符号夹紧，`StepMotor_GetSpeed()` 返回夹紧后的实际设置速度。

对 pitch 通道，若当前开环估计位置已经处于限位边界，并继续输入越界方向速度，实际设置速度会变为 `0.0f`。

### `BSP_STATUS StepMotor_RunFor(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s, uint32_t duration_ms)`

阻塞式便捷接口：指定通道以 `speed_deg_per_s` 运行 `duration_ms` 后停止。

该函数内部会：

1. 调用 `StepMotor_SetSpeed()`
2. 调用 `BSP_DelayMs(duration_ms)`
3. 更新开环估计位置
4. 调用 `StepMotor_Stop()`

运行期间当前执行流会被阻塞。

对 pitch 通道，函数会根据当前开环估计位置、速度和目标运行时间计算允许运行时间，避免一次长时间阻塞点动直接越过软件限位。

### `BSP_STATUS StepMotor_Stop(STEP_MOTOR_CHANNEL channel)`

停止指定通道脉冲输出，不清零开环估计位置。

### `BSP_STATUS StepMotor_StopAll(void)`

停止所有通道脉冲输出。

### `BSP_STATUS StepMotor_UpdateState(STEP_MOTOR_CHANNEL channel, uint32_t now_ms)`

按当前速度和时间差更新指定通道开环估计位置。

### `BSP_STATUS StepMotor_UpdateAllState(uint32_t now_ms)`

更新所有通道开环估计位置。

### `float StepMotor_GetSpeed(STEP_MOTOR_CHANNEL channel)`

返回指定通道最近设置的速度，单位 deg/s。非法通道返回 `0.0f`。

### `float StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL channel)`

返回指定通道开环估计位置，单位 deg。该值不代表实际传感器反馈位置。

### `void StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL channel)`

将指定通道的开环估计位置清零，并把当前时刻作为新的估算起点。

该函数不会让电机实际回到零点，不会停止电机，也不会执行任何归零动作。它只是告诉软件：“从现在开始，把当前位置视为估计零点”。

### `BSP_STATUS StepMotor_SetPitchLimit(const STEP_MOTOR_POSITION_LIMIT *limit)`

设置 pitch 通道开环估计位置限位。`limit == NULL` 返回 `BSP_STATUS_NULL`；`min_deg > max_deg` 返回 `BSP_STATUS_INVALID_ARG`。

### `STEP_MOTOR_POSITION_LIMIT StepMotor_GetPitchLimit(void)`

读取当前 pitch 通道开环估计位置限位。

## 对接说明

旧 `SMotor_*` 类型和接口已不再作为当前应用层入口使用。云台组合服务通过 `StepMotor_*` 控制 yaw/pitch 双轴。
