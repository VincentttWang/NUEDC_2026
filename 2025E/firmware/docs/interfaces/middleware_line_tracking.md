# middleware/line_tracking 接口说明

## 模块职责

`middleware/line_tracking` 提供巡线控制算法。

该模块从 `middleware/line_follow` 获取八路灰度状态，使用 `core/pid` 计算转向修正，通过 `core/kinematics` 做差速混控，最后调用 `middleware/chassis` 输出左右轮占空比。

灰度传感器读取属于 BSP，巡线状态维护属于 middleware，具体题目流程仍属于 app。

## 数据结构

```c
typedef struct {
    float base_duty;
    float sensor_position_scale;
    float output_limit;
    float differential_limit;
    PID_CONFIG pid_config;
    bool gyro_stab_enabled;
    float gyro_line_kp;
    float gyro_stab_kp;
    float omega_ref_limit;
    float gyro_z_sign;
} LINE_TRACKING_CONFIG;
```

- `base_duty`：基础前进占空比百分比。
- `sensor_position_scale`：灰度传感器位置到误差的缩放系数。
- `output_limit`：左右轮最终占空比限幅。
- `differential_limit`：循迹时左右轮占空比差值限幅，`<= 0` 表示不启用。
- `pid_config`：巡线 PID 参数（陀螺增稳关闭时的纯位置控制路径使用）。
- `gyro_stab_enabled`：陀螺增稳串级使能。
- `gyro_line_kp`：外环增益，灰度偏差（缩放后）→ 期望航向角速度 `ω_ref`。
- `gyro_stab_kp`：内环增益，角速度误差（`ω_ref − gz`）→ 差速修正。
- `omega_ref_limit`：`ω_ref` 限幅（deg/s）。
- `gyro_z_sign`：`gz` 符号系数，使陀螺内环成为负反馈（实机验证）。

## 陀螺增稳串级

硬件"等占空比不直行"（电机/轮径不对称）会让纯位置反馈产生"漂移 → 出死区 → 满舵 →
过冲"的蛇形极限环。启用 `gyro_stab_enabled` 后采用串级：

- **外环**：灰度横向偏差 → 期望航向角速度 `ω_ref = gyro_line_kp · error`（`omega_ref_limit` 限幅）。
- **内环**：`correction = gyro_stab_kp · (ω_ref − gz)`，`gz` 由 `LineTracking_Update` 内部
  `WitGetData()` 读取并乘 `gyro_z_sign`。

居中时 `ω_ref = 0`，内环把实际 `gz` 压向 0，自动补偿硬件不对称直行；同时 `gz` 反馈是
航向的阻尼项，直接抑制蛇形。`gz` 反馈符号错会变成正反馈（自旋/更抖），需实机验证 `gyro_z_sign`。

IMU（JY61P/I2C0）在 `main.c` 开机全局初始化、SysTick ISR 持续轮询，`WitGetData` 随时可取，
因此 **所有走 `Motion_Apply(LINE_FOLLOW) → LineTracking_Update` 的巡线任务（E1/F1/F2）自动接入**，
无需各任务改动。增稳关闭或 IMU 读失败时 `omega` 传 0，退化为纯位置 PID 旧行为。

```c
typedef struct {
    float error;
    float correction;
    float left_duty;
    float right_duty;
    uint8_t active_count;
    bool line_lost;
} LINE_TRACKING_OUTPUT;
```

- `error`：当前巡线偏差。
- `correction`：PID 输出的转向修正。
- `left_duty` / `right_duty`：最终左右轮占空比。
- `active_count`：检测到线的灰度通道数量。
- `line_lost`：是否未检测到线。

## 默认参数

```c
#define LINE_TRACKING_DEFAULT_BASE_DUTY 35.0f
#define LINE_TRACKING_DEFAULT_OUTPUT_LIMIT 100.0f
#define LINE_TRACKING_DEFAULT_CORRECTION_LIMIT 60.0f
#define LINE_TRACKING_DEFAULT_DIFFERENTIAL_LIMIT 10.0f
#define LINE_TRACKING_DEFAULT_POSITION_SCALE 10.0f
#define LINE_TRACKING_ACTIVE_SENSOR_MASK 0xFFU
```

默认 PID 为位置式 PID：

```text
kp = 30.0
ki = 0.0
kd = 1.5
integral_limit = 500.0
output_limit = 60.0
```

`LINE_TRACKING_ACTIVE_SENSOR_MASK` 用于选择参与巡线控制的灰度通道。当前值 `0xFF` 启用 0 到 7 的全部 8 路灰度传感器。

## 接口

### `void LineTracking_Init(const LINE_TRACKING_CONFIG *config)`

初始化巡线控制器。传入 `NULL` 时使用默认配置。

### `void LineTracking_Reset(void)`

清空 PID 状态和最近一次输出。

### `BSP_STATUS LineTracking_Update(float dt_s)`

完整巡线闭环入口：

1. 调用 `LineFollow_Update()` 更新灰度状态。
2. 调用 `LineTracking_Compute()` 计算左右轮占空比，只统计 `LINE_TRACKING_ACTIVE_SENSOR_MASK` 启用的通道。
3. 若未丢线，则调用 `Chassis_SetDuty()` 输出到底盘。

循迹输出会先限制 PID 转向修正量，使左右轮占空比差值不超过 `differential_limit`，再执行最终输出限幅。当前默认差值上限为 `10%`。该限制只影响 `LineTracking_Update()` 路径，不影响 `middleware/motion` 的直行、倒车或原地转向动作。

如果当前未检测到线，函数返回 `BSP_STATUS_NOT_READY`，不主动输出底盘占空比。题目流程可根据自身策略决定刹车、继续搜索或切换状态。

### `BSP_STATUS LineTracking_Compute(const LINE_FOLLOW_SENSOR_STATE *sensor, float dt_s, float omega_deg_s, LINE_TRACKING_OUTPUT *out)`

纯计算接口，不读取传感器，也不输出到底盘。`omega_deg_s` 为已乘 `gyro_z_sign` 的车体航向
角速度（供陀螺增稳内环）；增稳关闭或无 IMU 时传 `0`。传入空指针时返回 `BSP_STATUS_NULL`。

### `LINE_TRACKING_OUTPUT LineTracking_GetOutput(void)`

返回最近一次巡线计算结果。

## 边界说明

- 空线、半线、十字、边线计数等流程判断不在本模块公开，后续应在 `middleware/line_follow` 或 app 任务流程中按实际需要设计。
- 直角弯识别和转弯动作编排不在本模块中实现；当前 E1 在 `app/app_e_task` 中通过任务状态机组合 `middleware/motion` 原语完成。
- 旧 `sInedge` 和 `UpdateSInedge()` 不迁入本模块。
- 旧 `Motion_Car_Control()` 的职责已经拆分为 `Kinematics_DifferentialMix()` 和 `Chassis_SetDuty()`。
