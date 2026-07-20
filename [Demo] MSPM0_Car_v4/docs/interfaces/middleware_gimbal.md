# middleware/gimbal 接口说明

## 模块职责

`middleware/gimbal` 是云台组合服务，负责把 `bsp/step_motor` 中的 yaw/pitch 两路步进电机组合成上层可调用的云台接口。

该模块负责：

- 初始化 yaw/pitch 双轴步进电机。
- 设置 yaw/pitch 双轴速度。
- 停止双轴。
- 更新两轴开环估计位置。
- 读取两轴估计角度和当前速度。
- 配置和读取 pitch 软件限位。

该模块不负责：

- 视觉目标解析。
- 视觉 PID 控制。
- 坐标转换。
- 任务流程。
- OLED 显示。

## 限位约定

当前只保留 pitch 软件限位，yaw 不做软件限位。

默认限位：

```c
#define GIMBAL_PITCH_MIN_DEG STEP_MOTOR_PITCH_MIN_POSITION_DEG
#define GIMBAL_PITCH_MAX_DEG STEP_MOTOR_PITCH_MAX_POSITION_DEG
#define STEP_MOTOR_PITCH_MIN_POSITION_DEG (-30.0f)
#define STEP_MOTOR_PITCH_MAX_POSITION_DEG 30.0f
```

当 pitch 已经达到上限且继续输入正速度时，pitch 速度会被置为 `0.0f`；当 pitch 已经达到下限且继续输入负速度时，pitch 速度会被置为 `0.0f`。yaw 速度不受该限位影响。

该限位最终由 `bsp/step_motor` 执行，因此直接调用 `StepMotor_SetSpeed()` 或 `StepMotor_RunFor()` 的 pitch 测试路径也会受同一套限位约束。

## 公开类型

### `GIMBAL_AXIS`

```c
typedef enum {
    GIMBAL_AXIS_YAW = 0,
    GIMBAL_AXIS_PITCH,
    GIMBAL_AXIS_MAX
} GIMBAL_AXIS;
```

用于单轴位置复位。

### `GIMBAL_ANGLE`

```c
typedef struct {
    float yaw_deg;
    float pitch_deg;
} GIMBAL_ANGLE;
```

保存 yaw/pitch 估计角度，单位为度。

### `GIMBAL_SPEED`

```c
typedef struct {
    float yaw_deg_s;
    float pitch_deg_s;
} GIMBAL_SPEED;
```

保存 yaw/pitch 当前目标速度，单位为 `deg/s`。

### `GIMBAL_LIMIT`

```c
typedef struct {
    float pitch_min_deg;
    float pitch_max_deg;
} GIMBAL_LIMIT;
```

保存 pitch 软限位。yaw 暂不提供限位字段。

### `GIMBAL_STATUS`

```c
typedef struct {
    GIMBAL_ANGLE angle;
    GIMBAL_SPEED speed;
    GIMBAL_LIMIT limit;
} GIMBAL_STATUS;
```

云台状态快照。

## 公开接口

### `BSP_STATUS Gimbal_Init(void)`

初始化云台组合服务。内部调用 `StepMotor_Init()`，并恢复默认 pitch 限位。

### `BSP_STATUS Gimbal_SetSpeed(float yaw_deg_s, float pitch_deg_s)`

设置 yaw/pitch 双轴速度。内部会先更新当前位置，然后调用：

- `StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_YAW, yaw_deg_s)`
- `StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_PITCH, pitch_deg_s)`

pitch 位置限位在 `StepMotor_SetSpeed()` 内部统一执行。

### `BSP_STATUS Gimbal_Stop(void)`

停止 yaw/pitch 双轴，内部调用 `StepMotor_StopAll()`。

### `BSP_STATUS Gimbal_Update(void)`

更新两轴开环估计位置，内部调用 `StepMotor_UpdateAllState(BSP_Time_GetMs())`。

### `BSP_STATUS Gimbal_GetStatus(GIMBAL_STATUS *out)`

读取云台状态快照。`out == NULL` 时返回 `BSP_STATUS_NULL`。

### `GIMBAL_ANGLE Gimbal_GetAngle(void)`

读取 yaw/pitch 估计角度。

### `GIMBAL_SPEED Gimbal_GetSpeed(void)`

读取 yaw/pitch 当前目标速度。

### `void Gimbal_ResetPosition(void)`

复位 yaw/pitch 双轴估计位置。

### `void Gimbal_ResetAxisPosition(GIMBAL_AXIS axis)`

复位指定轴估计位置。非法轴直接返回。

### `BSP_STATUS Gimbal_SetLimit(const GIMBAL_LIMIT *limit)`

设置 pitch 软限位。`limit == NULL` 返回 `BSP_STATUS_NULL`；`pitch_min_deg > pitch_max_deg` 返回 `BSP_STATUS_INVALID_ARG`。该接口会同步调用 `StepMotor_SetPitchLimit()`，使底层直接调用路径也遵守同一限位。

### `GIMBAL_LIMIT Gimbal_GetLimit(void)`

读取当前 pitch 软限位。

## 迁移说明

本轮重写不保留旧 `YP_SMotor_*`、`GetYaw()` 和 `GetPitch()` 接口。

后续应将上层调用从：

- `YP_SMotor_Init()`
- `YP_SMotor_SetSpeed()`
- `YP_SMotor_UpdateState()`
- `GetYaw()`
- `GetPitch()`

迁移到 `Gimbal_*`。
