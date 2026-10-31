# middleware/chassis 接口说明

## 模块职责

`middleware/chassis` 是底盘组合服务，负责把 `bsp/motor/tb6612fng` 和 `bsp/motor/hall_encoder` 组合成上层可调用的底盘接口。

该模块负责：

- 初始化底盘电机驱动和编码器。
- 开环设置左右轮输出占空比百分比。
- **闭环：每轮速度环 PID 跟踪目标轮速**（见「速度闭环」节）。
- 底盘刹车和滑行。
- 提供底盘/每轮速度、距离和左右轮输出状态快照。

该模块不负责：

- 巡线传感器快照。
- `Digital[]`、`sInedge`、`edge`、`turning` 等巡线运行状态。
- 运动学模型和题目流程（速度环复用 `core/pid`，但不含差速/位姿逻辑）。
- 错误消息存储和错误显示。

## 速度闭环

底盘有两种控制模式（`CHASSIS_CONTROL_MODE`）：

- `CHASSIS_CONTROL_DUTY`（默认，开环）：`Chassis_SetDuty()` 直接下发占空比。
- `CHASSIS_CONTROL_SPEED`（闭环）：`Chassis_SetWheelSpeed()`/`Chassis_SetSpeed()` 设定目标轮速后，
  由 **每轮独立的「前馈 + PI 残差修正」** 跟踪：setpoint=目标轮速(m/s)、feedback=
  `HallEncoder_GetSpeed(id)`、output=占空比%。目标可为负（倒转）。

每轮出力 = **前馈** + **PI 修正**：

- 前馈由实测「占空比 → 稳态轮速」曲线反解，左右轮分别标定（`CHASSIS_FF_*`），承担主出力；
- PI（复用 `core/pid`，位置式）只补残差，修正量限幅 `CHASSIS_SPEED_OUTPUT_LIMIT`；
  反馈先过 `core/filter` 的一阶低通（`CHASSIS_SPEED_FEEDBACK_ALPHA`，fc≈6 Hz）再进 PI，
  压编码器量化跳变；前馈走目标值，不受该滞后影响。`HallEncoder_GetSpeed()` 与 `[SPD]`
  遥测的 `l`/`r` 仍是**原始**未滤波值，便于诊断；

- 目标绝对值低于 `CHASSIS_SPEED_ZERO_TARGET_MPS` 时视为**停止指令**：复位该轮 PID 并
  **主动刹车**，不让 PI 在起转死区内渐近爬行。

调用 `Chassis_SetWheelSpeed/SetSpeed` 进入闭环（从开环切入时复位 PID）；`Chassis_SetDuty/Stop/
Brake/Coast` 切回开环、清目标并复位 PID。实际出力由 `Chassis_UpdateSpeedControl(dt_s)` 周期
驱动——当前接在 `App_ControlTick`（RUN 态、10ms/100Hz）：任务每拍设目标速度，速度环随后跑一步。
退出 RUN 时 `App_ExitRun` 会 `Chassis_Brake` 自动停环。

参数（`chassis.h`，可用 Device Check「Speed PID」自检）：

```c
/* 前馈: 2026-07-29 Duty Sweep 抬轮空载标定, 10%~80% 区间拟合残差 < 2.4% */
#define CHASSIS_FF_LEFT_GAIN         81.85f   /* 左 duty% = 81.85*v + 1.03 */
#define CHASSIS_FF_LEFT_OFFSET        1.03f
#define CHASSIS_FF_RIGHT_GAIN        77.25f   /* 右 duty% = 77.25*v + 1.70 */
#define CHASSIS_FF_RIGHT_OFFSET       1.70f

#define CHASSIS_SPEED_KP             60.0f
#define CHASSIS_SPEED_KI             60.0f
#define CHASSIS_SPEED_KD             0.0f
#define CHASSIS_SPEED_OUTPUT_LIMIT   40.0f    /* PI 修正量限幅 */
#define CHASSIS_SPEED_INTEGRAL_LIMIT (CHASSIS_SPEED_OUTPUT_LIMIT / CHASSIS_SPEED_KI)
#define CHASSIS_SPEED_FEEDBACK_ALPHA  0.31f   /* fc≈6Hz @10ms; 取 1.0 即直通 */
#define CHASSIS_SPEED_ZERO_TARGET_MPS 0.01f
```

> ⚠ 前馈曲线是**抬轮空载**标定；落地后负载更大、同占空比对应更低速度，上地面后应重扫
> Duty Sweep 重新拟合这四个常数。
>
> ⚠ `CHASSIS_SPEED_INTEGRAL_LIMIT` **必须**从 `OUTPUT_LIMIT/KI` 派生，不要写成独立常数：
> `PID_Limit` 限的是积分累加量本身，而输出中的积分项是 `ki*integral`，两者配比错开就等于
> 抗积分饱和失效。旧配置 `ki=17 / integral_limit=96` 相差 16 倍，实测目标清零后 55 s 才退绕完。

`CHASSIS_SPEED_INTEGRAL_LIMIT` 限制的是 PID 内部积分累计值本身，不是已乘 `Ki` 的积分项；
最终占空比仍由 `CHASSIS_SPEED_OUTPUT_LIMIT` 限制。

**这组参数的抬轮空载实测表现**（2026-07-29，16043 拍）：目标归零后 189 ms 完全停住且此后
165 s 无蠕动；积分全程停在 −0.087~+0.045，远未触及 0.667 的限值；占空比无一拍饱和
（峰值 84.7% @1.0 m/s）；稳态占空比 σ 0.26~0.55%；0.2/0.4/0.6/1.0 m/s 四个已收敛长段的
跟踪误差为 +0.003~−0.000 m/s，已低于编码器 0.0295 m/s 的量化步长，该数字本身不再可分辨。
短阶梯（0.5~0.7 s）上可见 +0.012 左右的误差，那是 PI 尚未收敛的暂态，不是稳态偏差。

> 实测轮距为 `0.206 m`。通用 `Chassis_SetSpeed()` 目前仍只下发同速直行目标；循迹应用按
> `v_L = v + v·L/(2r)`、`v_R = v - v·L/(2r)` 计算差速，并通过 `Chassis_SetWheelSpeed()`
> 分别下发。若后续开放通用车体 `v/ω` 接口，可直接复用 `core/kinematics` 的
> `Kinematics_BodyToWheel()`。

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

工程现为**左右双轮**霍尔编码器。`Chassis_GetSpeed()`/`Chassis_GetDistance()` 返回**车体量**（左右轮均值），每轮独立值用 `Chassis_GetWheelSpeed(id)`/`Chassis_GetWheelDistance(id)` 获取。

## 公开接口

### `BSP_STATUS Chassis_Init(void)`

初始化底盘组合服务。内部调用：

- `TB6612FNG_Init()`
- `HallEncoder_Init()`

### `BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent)`

一次性设置左右轮占空比百分比（开环，切回 `CHASSIS_CONTROL_DUTY`）。

示例：

```c
Chassis_SetDuty(40.0f, 40.0f);
Chassis_SetDuty(-40.0f, -40.0f);
Chassis_SetDuty(-30.0f, 30.0f);
```

### `void Chassis_SetWheelSpeed(float left_mps, float right_mps)`

设置左右轮目标线速度（m/s，负为倒转）并进入速度闭环。仅设定目标，实际出力由 `Chassis_UpdateSpeedControl()` 周期驱动；从开环切入时复位 PID。

### `void Chassis_SetSpeed(float body_mps)`

设置车体直行目标线速度（两轮同速），进入速度闭环。该接口不接收角速度；差速转向由上层按
实测轮距 `0.206 m` 换算后调用 `Chassis_SetWheelSpeed()`。

### `BSP_STATUS Chassis_UpdateSpeedControl(float dt_s)`

速度闭环单步更新：闭环模式下跑两轮 PID 并出力；开环模式为空操作（恒返回 `BSP_STATUS_OK`）。须由控制周期任务周期调用（当前接在 `App_ControlTick`，10ms）。

### `CHASSIS_CONTROL_MODE Chassis_GetControlMode(void)`

返回当前控制模式（开环/闭环）。

### `float Chassis_GetWheelSpeedTarget(HALL_ENCODER_ID wheel)`

返回指定轮当前目标线速度（m/s），仅闭环模式有意义。

### `float Chassis_GetWheelSpeedIntegral(HALL_ENCODER_ID wheel)`

返回指定轮速度环 PID 的积分累加值，仅闭环模式有意义。诊断/整定用：输出中的积分项为
`CHASSIS_SPEED_KI * 本值`，故本值达到 `CHASSIS_SPEED_OUTPUT_LIMIT / CHASSIS_SPEED_KI`
时积分项已单独占满输出，继续累积即为过度累积（退绕时会拖长恢复时间）。切回开环不清零
（下次进闭环时才 `PID_Reset`），开环期间读到的是上次残留值。Device Check「Speed PID」
的 `[SPD]` 遥测以 `il`/`ir` 字段输出本值。

### `BSP_STATUS Chassis_Stop(CHASSIS_STOP_MODE mode)`

按指定方式停止底盘。成功后会清零当前软件记录的 duty。

### `BSP_STATUS Chassis_Brake(void)`

刹车停止，等价于 `Chassis_Stop(CHASSIS_STOP_MODE_BRAKE)`。

### `BSP_STATUS Chassis_Coast(void)`

滑行停止，等价于 `Chassis_Stop(CHASSIS_STOP_MODE_COAST)`。

### `CHASSIS_DUTY Chassis_GetDuty(void)`

返回当前软件记录的左右轮占空比。

### `float Chassis_GetSpeed(void)`

返回车体估计线速度（左右轮均值），单位 m/s。内部用 `core/kinematics` 的
`Kinematics_WheelToBody` 取 `linear` 分量；该接口只公开与轮距无关的线速度，若后续需要同时
输出角速度，应向运动学函数传入实测轮距 `0.206 m` 并扩展返回接口。

### `float Chassis_GetWheelSpeed(HALL_ENCODER_ID wheel)`

返回指定轮估计线速度，单位 m/s。

### `float Chassis_GetDistance(void)`

返回车体估计距离（左右轮均值），单位 m。

### `float Chassis_GetWheelDistance(HALL_ENCODER_ID wheel)`

返回指定轮估计距离，单位 m。

> 编码器采样与 GPIO/定时器中断由 `bsp/motor/hall_encoder.c` 内的 `GROUP1_IRQHandler()`（服务 GPIOA 上的两轮 A 相）/ `TIMER_0_INST_IRQHandler()` 直接处理，middleware 不再转发。

### `void Chassis_ResetDistance(void)`

复位两轮编码器距离累计。

## 迁移说明

早期的 `Motor_*`、`Encoder_*` 和 `UpdateSInedge()` 接口在底盘层重写时已废弃，上层调用
**均已迁移完毕**，现统一走 `Chassis_*`（底盘组合服务）与 `HallEncoder_*`（BSP 层测速）。
仓库内已无这些旧接口的定义与调用者。
