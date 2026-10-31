# middleware/line_follow 接口说明

## 模块职责

`middleware/line_follow` 是完整巡线闭环控制器。它接收标准化八路观测，计算线中心、误差
和差速修正，并通过 `middleware/chassis` 输出左右轮占空比。当前菜单任务使用 Yahboom
`ReadDetectedMask()`；共享 I2C0 的采样调度由 app 负责，不塞入控制器。

该模块同时提供硬件无关的 `LineFollow_Compute()`，便于使用人工构造的观测做算法验证。

数据流：

```text
app/Yahboom 采样
        ↓  detected_mask: bit0=X1 ... bit7=X8，1=检测到黑线
middleware/line_follow
        ↓  left/right duty
middleware/chassis
```

该模块不负责直角弯、圈数、边线计数、丢线搜索或任务完成条件；这些属于 app 任务状态机。

## 输入语义

```c
typedef struct {
    uint8_t level[LINE_FOLLOW_SENSOR_COUNT];
} LINE_FOLLOW_INPUT;
```

- `level[i] == 0`：通道 `i` 检测到黑线。
- `level[i] != 0`：通道 `i` 未检测到黑线。
- 通道顺序与 BSP 的 `GRAYSCALE_SENSOR_CHANNEL_0..7` 一致。

这是保留的 GPIO 兼容输入语义。Yahboom 调用方应直接使用
`LineFollow_UpdateDetectedMask()` / `LineFollow_ObserveDetectedMask()`，避免自行处理协议原始位序
和低有效极性。

## 参数分组

```c
/* 传感器与输出。 */
#define LINE_FOLLOW_SENSOR_COUNT                       GRAYSCALE_SENSOR_CHANNEL_COUNT
#define LINE_FOLLOW_ACTIVE_SENSOR_MASK                 0xFFU
#define LINE_FOLLOW_SENSOR_SPAN_MM                     80.0f
#define LINE_FOLLOW_SENSOR_PITCH_MM                    (80.0f / 7U)
#define LINE_FOLLOW_DEFAULT_POSITION_SCALE             1.0f
#define LINE_FOLLOW_DEFAULT_BASE_DUTY                  34.0f
#define LINE_FOLLOW_DEFAULT_OUTPUT_LIMIT               100.0f
#define LINE_FOLLOW_DEFAULT_DIFFERENTIAL_LIMIT         16.0f

/* 误差预处理。 */
#define LINE_FOLLOW_ERROR_LPF_ALPHA                    0.5f
#define LINE_FOLLOW_ERROR_DEADBAND                     10.0f

/* 关闭陀螺增稳时使用的纯位置 PID。 */
#define LINE_FOLLOW_DEFAULT_PID_KP                     1.0f
#define LINE_FOLLOW_DEFAULT_PID_KI                     0.0f
#define LINE_FOLLOW_DEFAULT_PID_KD                     0.0f
#define LINE_FOLLOW_DEFAULT_PID_INTEGRAL_LIMIT         500.0f
#define LINE_FOLLOW_DEFAULT_PID_OUTPUT_LIMIT           60.0f
#define LINE_FOLLOW_DEFAULT_PID_MODE                   PID_MODE_POSITION

/* 默认启用的陀螺串级。 */
#define LINE_FOLLOW_DEFAULT_GYRO_STAB_ENABLED          true
#define LINE_FOLLOW_DEFAULT_GYRO_LINE_KP               3.0f
#define LINE_FOLLOW_DEFAULT_GYRO_STAB_KP               0.20f
#define LINE_FOLLOW_DEFAULT_OMEGA_REF_LIMIT            60.0f
#define LINE_FOLLOW_DEFAULT_OMEGA_LINE_LIMIT           60.0f
#define LINE_FOLLOW_DEFAULT_GYRO_Z_SIGN                (1.0f)
```

默认使用“灰度偏差 P 外环 + 角速度 P 内环”，只有关闭 `gyro_stab_enabled` 时才调用通用
位置式 PID。`DIFFERENTIAL_LIMIT=16` 表示左右轮占空比差值不超过 16，因此最终
`correction` 会限制在 `±8`。曲线调用者可传入 `omega_feedforward`；灰度贡献先由
`omega_line_limit` 单独限制，再与前馈相加并受 `omega_ref_limit` 限制，避免灰度误差瞬间
反转整条曲率参考，也避免陀螺内环抵消调用者的曲率前馈。

## 配置与输出

`LINE_FOLLOW_CONFIG` 保存基础占空比、毫米坐标标定缩放、输出/差值限幅、退化 PID 配置及陀螺串级
配置。H 题 app 基于默认配置覆盖 `gyro_line_kp=1.25`，并分别设置空载/载球的
`omega_line_limit=20/15 deg/s` 与 `omega_ref_limit=75/50 deg/s`；不改变其他任务的默认参数。

```c
typedef struct {
    uint8_t level_mask;
    uint8_t black_count;
    float error;
    float correction;
    float left_duty;
    float right_duty;
    bool line_lost;
} LINE_FOLLOW_OUTPUT;
```

- `level_mask`：本拍数字电平掩码；置 1 表示对应通道未检测到黑线，与 Device Check 一致。
- `black_count`：启用通道中检测到黑线（`level==0`）的数量。
- `error`：未经 EMA/死区处理的横向误差，单位 mm。X1/X8 感光中心相距 80 mm，八路等距
  分布，因此相邻通道间距约 11.43 mm，通道坐标为 `-40…+40 mm`。
- `correction`：陀螺串级或退化 PID 产生并经过差值限幅的修正量。
- `line_lost`：没有任何启用通道检测到线。

## 接口

### `LINE_FOLLOW_CONFIG LineFollow_GetDefaultConfig(void)`

返回默认配置副本，供上层覆盖个别参数后传给 `LineFollow_Init()`。

### `void LineFollow_Init(const LINE_FOLLOW_CONFIG *config)`

初始化控制器和运行状态；传 `NULL` 使用默认配置。

### `void LineFollow_Reset(void)`

复位 PID、EMA 滤波状态和最近输出，不改变配置。

### `BSP_STATUS LineFollow_UpdateDetectedMask(uint8_t detected_mask, float dt_s)`

当前 Yahboom 循迹任务使用的完整闭环入口：

1. 接收 app 已读取的 Yahboom 归一化黑线掩码。
2. 从 JY61P 缓存读取 `gz`（启用陀螺增稳时）。
3. 调用 `LineFollow_Compute()`。
4. 未丢线时调用 `Chassis_SetDuty()`。

丢线时返回 `BSP_STATUS_NOT_READY`，不自行刹车或搜索。JY61P 的初始化与
`JY61P_I2C_Poll()`、Yahboom 阻塞读取及共享 I2C0 分时仍由 app 任务负责。

### `BSP_STATUS LineFollow_EvaluateDetectedMask(...)`

与 `LineFollow_UpdateDetectedMask()` 使用相同的灰度、陀螺和控制器状态，但只返回本拍
`LINE_FOLLOW_OUTPUT`，不向底盘写占空比。需要保持纵向平均轮速、把循迹修正映射成反对称
左右轮速度差的任务使用该接口；当前 H 题一圈循迹任务即走此路径。

### `BSP_STATUS LineFollow_EvaluateDetectedMaskWithOmegaFeedforward(...)`

在上述只计算入口基础上增加标称角速度参数，单位 `deg/s`，符号须与 `gyro_z_sign` 校正后的
`gz` 一致。H 题 S2/S4 使用该接口，将曲线角速度和灰度误差共同组成陀螺内环参考。

`LineFollow_EvaluateOmegaFeedforwardOnly()` 不使用灰度横向位置，只按曲率角速度前馈与 JY61P
`gz` 计算内环输出。它仅供 app 在弯道灰度瞬时全白时做有时限的重捕降级，不应作为整段弯道
的长期控制方式。
`LINE_FOLLOW_OUTPUT` 的 `omega_ref_deg_s/omega_measured_deg_s` 可用于遥测核对参考与实测。

### `LineFollow_Observe()` / `LineFollow_ObserveDetectedMask()`

两个接口只计算 `level_mask`、`black_mask`、命中数量和质心误差，不读硬件、不持有 PID 状态。
前者接受 `0=黑线` 的电平数组，后者接受 `1=黑线` 的归一化掩码。

### `BSP_STATUS LineFollow_Compute(...)`

```c
BSP_STATUS LineFollow_Compute(const LINE_FOLLOW_INPUT *input,
                              float dt_s,
                              float omega_deg_s,
                              LINE_FOLLOW_OUTPUT *out);
```

纯计算入口，不读取灰度硬件、不驱动底盘。处理顺序为：

```text
level[]（0=黑线）→ 毫米坐标线中心均值 → EMA → 中心死区
           → 陀螺串级或位置 PID → 差值限幅 → 差速混控
```

`LineFollow_ComputeWithOmegaFeedforward()` 是对应的纯计算前馈版本；原接口保持兼容并以
`omega_feedforward=0` 调用它。

### `LINE_FOLLOW_OUTPUT LineFollow_GetOutput(void)`

返回最近一次输出和本拍观测摘要，供 app 显示和诊断。

## 已移除的旧状态包装

原 `middleware/line_follow` 仅缓存灰度快照，并保存无人使用的 `edge_count`；这层包装已合并。
以下接口不再存在：

- `LineFollow_UpdateSensor()`
- `LineFollow_GetSensor()` / `GetSensorMask()` / `GetActiveCount()`
- `LineFollow_GetEdgeCount()` / `IncrementEdge()`

如未来需要运行时切换多种传感器、坏道屏蔽或异步观测快照，再单独引入 `line_sensing`
观测层；当前不为单一 Yahboom 实机路径预建跨设备状态框架。
