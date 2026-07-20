# middleware/chassis 接口说明

## 模块职责

`middleware/chassis` 是底盘组合服务，负责把 `bsp/motor/tb6612fng` 和 `bsp/motor/hall_encoder` 组合成上层可调用的底盘接口。

该模块负责：

- 初始化底盘电机驱动和编码器。
- 设置左右轮输出占空比百分比。
- 底盘刹车和滑行。
- 更新编码器采样。
- 提供底盘速度、距离、方向和左右轮输出状态快照。

该模块不负责：

- 巡线传感器快照。
- `Digital[]`、`sInedge`、`edge`、`turning` 等巡线运行状态。
- PID、运动学和题目流程。
- 错误消息存储和错误显示。

## 公开类型

### `CHASSIS_STOP_MODE`

```c
typedef enum {
    CHASSIS_STOP_MODE_COAST = 0,
    CHASSIS_STOP_MODE_BRAKE
} CHASSIS_STOP_MODE;
```

用于指定停止方式：

- `CHASSIS_STOP_MODE_COAST`：滑行，调用 `TB6612FNG_CoastAll()`。
- `CHASSIS_STOP_MODE_BRAKE`：刹车，调用 `TB6612FNG_BrakeAll()`。

### `CHASSIS_DUTY`

```c
typedef struct {
    float left_percent;
    float right_percent;
} CHASSIS_DUTY;
```

保存左右轮占空比百分比。正值表示前进方向，负值表示后退方向。实际限幅由 BSP `TB6612FNG_SetDuty()` 完成。

### `CHASSIS_STATUS`

```c
typedef struct {
    CHASSIS_DUTY duty;
    float speed_mps;
    float distance_m;
    HALL_ENCODER_DIR encoder_dir;
    TB6612FNG_OUTPUT left_output;
    TB6612FNG_OUTPUT right_output;
} CHASSIS_STATUS;
```

底盘状态快照。

当前工程只有一个霍尔编码器通道，因此 `speed_mps` 和 `distance_m` 表示当前编码器通道估计值，不表示左右轮独立速度。

## 公开接口

### `BSP_STATUS Chassis_Init(void)`

初始化底盘组合服务。内部调用：

- `TB6612FNG_Init()`
- `HallEncoder_Init()`

### `BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent)`

一次性设置左右轮占空比百分比。

示例：

```c
Chassis_SetDuty(40.0f, 40.0f);
Chassis_SetDuty(-40.0f, -40.0f);
Chassis_SetDuty(-30.0f, 30.0f);
```

### `BSP_STATUS Chassis_Stop(CHASSIS_STOP_MODE mode)`

按指定方式停止底盘。成功后会清零当前软件记录的 duty。

### `BSP_STATUS Chassis_Brake(void)`

刹车停止，等价于 `Chassis_Stop(CHASSIS_STOP_MODE_BRAKE)`。

### `BSP_STATUS Chassis_Coast(void)`

滑行停止，等价于 `Chassis_Stop(CHASSIS_STOP_MODE_COAST)`。

### `BSP_STATUS Chassis_Update(void)`

更新底盘状态。当前内部调用 `HallEncoder_UpdateSample()`。

### `void Chassis_HandleEncoderIrq(uint32_t gpio_status)`

编码器 GPIO 中断分发入口，内部调用 `HallEncoder_HandleGpioIrq()`。

### `void Chassis_HandleEncoderTimerIrq(void)`

编码器采样定时器中断分发入口，内部调用 `HallEncoder_UpdateSample()`。

### `BSP_STATUS Chassis_GetStatus(CHASSIS_STATUS *out)`

读取底盘状态快照。`out == NULL` 时返回 `BSP_STATUS_NULL`。

### `CHASSIS_DUTY Chassis_GetDuty(void)`

返回当前软件记录的左右轮占空比。

### `float Chassis_GetSpeed(void)`

返回编码器估计速度，单位 m/s。

### `float Chassis_GetDistance(void)`

返回编码器估计距离，单位 m。

### `HALL_ENCODER_DIR Chassis_GetEncoderDir(void)`

返回编码器方向。

### `void Chassis_ResetDistance(void)`

复位编码器距离累计。

## 迁移说明

本轮重写不保留旧 `Motor_*`、`Encoder_*` 和 `UpdateSInedge()` 接口。

后续应将上层调用从：

- `Motor_SystemInit()`
- `Motor_SetLeft()`
- `Motor_SetRight()`
- `Motor_Brake()`
- `Encoder_GetSpeed()`
- `UpdateSInedge()`

迁移到 `Chassis_*` 或后续 `line_follow` 模块。
