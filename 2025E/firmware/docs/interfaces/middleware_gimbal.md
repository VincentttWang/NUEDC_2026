# middleware/gimbal 接口说明

## 模块职责

`middleware/gimbal` 是云台组合服务，负责把 `bsp/bldc` 中的双 F32C 无刷电机（UART3，yaw=地址1、pitch=地址2）组合成上层可调用的云台接口。**两轴均工作在 F32C 位置闭环模式（`MODE_MULTI_POS`）**：对外仍是"速度指令（`deg/s`）"语义，模块内部把角速度按时间积分成**位置设定点**再下发多圈位置。位置模式带保持力矩、精确到位，`Gimbal_GetAngle()` 返回该设定点作为角度估计。

该模块负责：

- 初始化 yaw/pitch 双轴无刷电机（惰性使能，见下文）。
- 设置 yaw/pitch 双轴角速度（`deg/s`，内部积分成位置设定点）。
- 停止双轴（位置模式保持当前设定点）。
- 推进 yaw 位置设定点并刷新角度估计。
- 读取两轴估计角度和当前速度。
- 配置和读取 pitch 软件限位。

该模块不负责：

- 视觉目标解析。
- 视觉 PID 控制。
- 坐标转换。
- 任务流程。
- OLED 显示。

## 限位约定

pitch 施加软件限位 + 无刷硬件角度限位（归位后收紧到 `home ± range`）；yaw 近似不限位（使能时施加 `±GIMBAL_YAW_LIMIT_X10` 的多圈量程，以支持 E3 连续扫描）。

默认限位：

```c
/* 由无刷驱动机械限位常量换算 (0.1° → °)，默认 [0°, 180°] */
#define GIMBAL_PITCH_MIN_DEG ((float)BLDC_PITCH_MIN_X10 / 10.0f)
#define GIMBAL_PITCH_MAX_DEG ((float)BLDC_PITCH_MAX_X10 / 10.0f)
```

pitch 位置设定点在积分时被钳制到 `[pitch_min_deg, pitch_max_deg]`，同时无刷位置环有硬件角度限位兜底，二者共同保证"限位硬优先于追踪"。yaw 位置设定点不做此钳制。

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

初始化云台组合服务。内部调用 `BLDC_Init()`，复位设定点/估计角并把 pitch 限位设为 `home ± range`。**开机不使能任何一轴**：yaw 首次收到非零速度指令时惰性使能，pitch 由 `Gimbal_StartupElevatePitch()` / `Gimbal_EnsurePitchReady()` 管理。

### `void Gimbal_StartupElevatePitch(void)`

开机**非阻塞**发起 pitch 抬升到工作角（默认 `GIMBAL_PITCH_HOME_X10` = `BLDC_PITCH_INIT_X10`，即 150°）。应在 `Gimbal_Init()` 之后调用一次（`app/main.c` 已调用）：使能 pitch → 多圈位置模式 → 以当前物理位置为 0° 并施加 `[MIN,MAX]` 限位 → 下发目标角后**立即返回**，pitch 在后台抬升，不阻塞开机菜单。

### `void Gimbal_EnsurePitchReady(void)`

在需要 pitch 视觉闭环的任务（E2/E3 瞄准）开始前调用，**阻塞**确认 pitch 到位（全程保持位置模式）：

1. 若已就绪则立即返回；
2. 否则（必要时先补发抬升指令）轮询多圈角度反馈直到到位（容差 3°）或超时（2s）；
3. 到位后把无刷硬件角度限位收紧到 `home ± range`（保持位置模式），并把位置设定点/估计角对齐到工作角。

由于开机已非阻塞发起抬升，正常情况下进入 E2/E3 时 pitch 已物理到位，本函数只需快速确认即返回。pitch 抬升状态（`s_pitch_home_started` / `s_pitch_mode`）独立于 `Gimbal_Init()`，不因重复 `Gimbal_Init()` 而重置或重复归零（避免以非静止位为 0° 累加超程）。

### `BSP_STATUS Gimbal_SetSpeed(float yaw_deg_s, float pitch_deg_s)`

设置 yaw/pitch 双轴角速度（`deg/s`）。两轴均位置模式，内部把角速度按时间积分成**位置设定点**，去重后下发多圈位置（`BLDC_SetMultiAngle`）：

- yaw：先 `Gimbal_Update()` 应用上一速度的积分并下发，再记录本次角速度；
- pitch：在 `Gimbal_DrivePitchTrack()` 中积分设定点、钳制到限位后下发。

仅当下发的位置（0.1°）变化时才发送 UART 帧，以降低总线负载。

**上电/使能策略**：

- **yaw 惰性使能**：开机不使能 yaw；首次收到**非零** yaw 速度指令时才 `BLDC_Enable` + `MODE_MULTI_POS` + 运动速度/加速度，并以当前物理位置为 0° 基准（清多圈计数）。之后保持位置模式。
- **pitch 就绪门控**：pitch 仍在开机抬升归位中时，`Gimbal_SetSpeed` 对 pitch **不积分、不下发**，避免干扰归位；就绪后才按角速度积分驱动 pitch。

### `BSP_STATUS Gimbal_Stop(void)`

停止 yaw/pitch 双轴。位置模式自带保持力矩，清零角速度后维持当前设定点即可，不额外下发。

### `BSP_STATUS Gimbal_Update(void)`

按 yaw 当前角速度和经过时间（内部按 `last_update_ms` 计 dt）积分 yaw 位置设定点、去重下发多圈位置，并刷新 yaw 角度估计。供"设一次速度 + 循环调用连续转"的 E3 扫描使用；连续两次调用第二次 dt≈0，天然避免重复积分。pitch 不在此处理（由 `Gimbal_DrivePitchTrack` 在 `SetSpeed` 中驱动）。

### `BSP_STATUS Gimbal_GetStatus(GIMBAL_STATUS *out)`

读取云台状态快照。`out == NULL` 时返回 `BSP_STATUS_NULL`。

### `GIMBAL_ANGLE Gimbal_GetAngle(void)`

读取 yaw/pitch 估计角度。

### `GIMBAL_SPEED Gimbal_GetSpeed(void)`

读取 yaw/pitch 当前目标速度。

### `void Gimbal_ResetPosition(void)`

复位 yaw/pitch 双轴位置。yaw 以当前物理位置为新 0° 重新基准（清多圈计数），不会命令电机回跳。

### `void Gimbal_ResetAxisPosition(GIMBAL_AXIS axis)`

复位指定轴位置。非法轴直接返回。yaw 同 `Gimbal_ResetPosition`（重新基准，清多圈计数）。

### `BSP_STATUS Gimbal_SetLimit(const GIMBAL_LIMIT *limit)`

设置 pitch 软限位。`limit == NULL` 返回 `BSP_STATUS_NULL`；`pitch_min_deg > pitch_max_deg` 返回 `BSP_STATUS_INVALID_ARG`。限位在本模块内维护并对 pitch 位置设定点钳制。

### `GIMBAL_LIMIT Gimbal_GetLimit(void)`

读取当前 pitch 软限位。

## 迁移说明

2025E 工程已由步进云台迁移到双 F32C 无刷云台：`middleware/gimbal` 对外 API 完全不变，底层由 `bsp/step_motor` 改为 `bsp/bldc`（`f32c_bldc.c`）。上层 `middleware/gimbal_tracking` 与 `app_e_task` 无需改动。

后续可选优化：将 `Gimbal_GetAngle()` 从开环积分改为读取 `BLDC_MotorX.multi_angle` 反馈以获得闭环角度。
