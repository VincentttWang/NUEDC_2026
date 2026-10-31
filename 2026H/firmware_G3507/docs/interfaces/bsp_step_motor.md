# bsp/step_motor 接口说明

摆杆执行器的步进电机**位置式**控制。STEP 脉冲由定时器 PWM 产生，DIR/EN 为普通 GPIO，
位置反馈来自 QEI 硬件正交计数。

## 硬件映射

| 信号 | SysConfig 符号 | 引脚 | 外设 |
|---|---|---|---|
| STEP | `SMotor_INST` + `DL_TIMER_CC_0_INDEX` | PA29 | TIMG6_CCP0 |
| DIR | `SMotor_IO_DIR1_PIN` | PB14 | GPIO |
| EN | `SMotor_IO_EN1_PIN` | PB11 | GPIO |
| 编码器 A/B | `SMotor_QEI_INST` | PA26 / PA27 | TIMG8（QEI 2-input） |

## 控制模型：只有位置，没有速度

驱动**不提供速度式接口**。唯一的运动指令是设定目标位置：

```
MoveToCount(target) ──► 限幅 ──► target_counts
                                                        │
        Tick 每 10ms：speed = clamp(KP × 误差角, ±speed_limit)
                                                        │
                                                   出脉冲 / 到位停
```

这样设计的原因是**限幅**：速度指令绕得过位置限幅——给一个方向持续转就出界了，
只能靠事后守护拽回来；位置指令进门就被夹住，越界的目标根本发不出去。限幅因此
只需在 `MoveToCount` 一个地方做一次，不可能被绕开。

转速由 `StepMotor_SetSpeedLimit()` 约束——那是**上限**不是指令，误差大时跑到上限，
接近目标自动减速。

> 自检页里"持续转"的效果用「把目标设到很远处」实现（`SM_FAR_COUNTS`），
> 途中即匀速；区别只在于这条路径同样受限幅约束。

## 时钟约束（改配置前必读）

驱动把速度换算成定时器重载值，公式假定定时器时钟为 `STEP_MOTOR_STEP_TIMER_CLK_HZ`
= 500000 Hz，即 SysConfig 中 `SMotor` 须配 `clockDivider = 8`、`clockPrescale = 8`
（32MHz / 64），生成的 `SMotor_INST_CLK_FREQ` 应为 `500000`。

> 二者不一致时不会有编译错误，只会让转速整体偏离固定倍数。改动任一侧后务必核对
> `ti_msp_dl_config.h` 中的 `SMotor_INST_CLK_FREQ`。

## 参数

标定手册见 [`../step-motor-calibration.md`](../step-motor-calibration.md)。

| 宏 | 默认 | 说明 |
|---|---|---|
| `STEP_MOTOR_STEP_ANGLE_DEG` | 1.8 | 电机固有步距角 |
| `STEP_MOTOR_MICROSTEP` | 32.0 | 细分数 = **步进精细度**，**须与驱动器拨码一致** |
| `STEP_MOTOR_ENCODER_COUNTS_PER_REV` | 2000 | QEI 每转计数（2 倍频，已实测） |
| `STEP_MOTOR_MAX_SPEED_DEG_S` | 960 | 速度硬上限 |
| `STEP_MOTOR_SERVO_KP` | 3.0 | 伺服比例增益，1/s |
| `STEP_MOTOR_SERVO_KP_MAX` | 12.0 | 运行期增益上限；H3 可临时提高，退出恢复默认值 |
| `STEP_MOTOR_POSITION_TOLERANCE_COUNTS` | 0 | 到位容差，**已压到 0**（只接受精确相等） |
| `STEP_MOTOR_SERVO_RESUME_COUNTS` | 6 | 回差带：已到位后漂过它才重新出脉冲，须 > 容差 |
| `STEP_MOTOR_SERVO_MIN_SPEED_DEG_S` | 5.0 | 末端最低出力速度，防蠕动 |
| `STEP_MOTOR_SERVO_DEFAULT_SPEED_LIMIT_DEG_S` | 90 | 速度上限的上电默认值 |
| `STEP_MOTOR_ENC_SOFT_MIN/MAX_COUNTS` | 由 HARD ∓ MARGIN 推出 | **唯一生效的行程边界** |
| `STEP_MOTOR_ENC_LIMIT_ENABLED` | 1 | 限位总开关，全程有效；**标定前保持 0** |
| `STEP_MOTOR_STARTUP_LIFT_*` | 见头文件 | 上电自动抬升一组 |
| `STEP_MOTOR_GUARD_RECOVER_*` | 见头文件 | 越界纠正一组 |
| `STEP_MOTOR_TICK_PERIOD_MS` | 10 | Tick 调用周期，改大须同步加大 MARGIN |
| `STEP_MOTOR_BEAM_ENABLE_HIGH` / `POSITIVE_DIR_HIGH` / `ENCODER_INVERT` | 1 / 1 / 1 | 三项极性，均已上板实测 |

## 接口

共 **16 个**，单通道无参——只有摆杆一路步进电机，接口不带通道参数。

```c
/* 生命周期 */
BSP_STATUS StepMotor_Init(void);       /* 上电即失能，编码器清零 = 坐标系原点 */
void       StepMotor_Tick(uint32_t now_ms);   /* 唯一周期入口，10ms */

/* 运动指令 —— 唯一入口，一律过限幅 */
BSP_STATUS StepMotor_MoveToCount(int32_t target_counts);
BSP_STATUS StepMotor_Stop(void);                               /* 就地停住 */
BSP_STATUS StepMotor_SetSpeedLimit(float max_speed_deg_per_s); /* 上限，不是指令 */

/* 状态查询 */
int32_t  StepMotor_GetEncoderCount(void);
int32_t  StepMotor_GetTargetCount(void);
int32_t  StepMotor_GetPositionErrorCount(void);   /* 丢步的直接指标 */
bool     StepMotor_IsAtTarget(void);
float    StepMotor_CountsToDeg(int32_t counts);
uint32_t StepMotor_SpeedToStepFreq(float speed_deg_per_s);

/* 使能 */
BSP_STATUS StepMotor_SetEnabled(bool enable);
bool       StepMotor_IsEnabled(void);

/* 异常可见性 */
STEP_MOTOR_GUARD_STATE StepMotor_GetGuardState(void);
void                   StepMotor_AbortStartup(void);

/* 诊断 */
float    StepMotor_GetSpeed(void);            /* 伺服实际下发速度，只读 */
uint32_t StepMotor_GetStepFrequencyHz(void);
uint32_t StepMotor_GetPwmLoadValue(void);      /* TIMG6 LOAD */
uint32_t StepMotor_GetPwmCompareValue(void);   /* TIMG6 CC0 */
uint32_t StepMotor_GetPwmCounterValue(void);   /* TIMG6 CTR */
```

### 刻意不提供的能力

| 缺失的接口 | 理由 |
|---|---|
| 任何速度式指令 | 速度指令绕得过位置限幅，位置指令绕不过 |
| `ResetEncoder` / `SetEncoderCount` | 清零/平移坐标系会让限位边界失去意义。零点只在 `Init` 建立一次 |
| 限位的运行期开关 | 限位由 `ENC_LIMIT_ENABLED` 宏一次性决定，全程有效。要越过软限位量行程就 `SetEnabled(false)` 手推 |
| `ClearGuardFault` | `GUARD_FAULT` 意味着极性反了/编码器断线/机械卡死，重新上电才是正确处置 |
| `ServoTick` / `UpdateEncoder` 单独暴露 | 都由 `Tick()` 统一驱动，顺序不能由调用方决定 |
| `IsStartupDone` | 上层靠 `MoveToCount()` 返回 `BSP_STATUS_BUSY` 得知抬升未结束 |

## 使用约束

- **`StepMotor_Tick()` 必须以 10ms 周期持续调用**（`app_init.c` 已注册进调度器），
  且与应用状态无关。它一身四职：采样编码器、跑上电抬升、判越界并纠正、驱动位置伺服。
  **不调它电机根本不会动**——位置指令只登记目标，脉冲由这里的伺服下发。
- 上电后驱动器**失能**、编码器清零。`MoveToCount()` 会自动通电，无需先手动使能。
- **失能→使能的瞬间目标会被同步到当前实测位置**，防止断电期间摆杆回落后一通电就被
  猛拽回旧目标。因此复能之后要动必须重新下位置指令。
- 上电抬升进行中，外部 `MoveToCount()` 返回 `BSP_STATUS_BUSY` 并被忽略；
  据此可知抬升未结束，或调 `StepMotor_AbortStartup()` 主动放弃。
- **限位全程有效，没有运行期开关。** 要越过软限位量机械行程，走
  `StepMotor_SetEnabled(false)` 手推摆杆——失能后伺服不出力、守护也无从纠正，
  编码器却照常计数。这比"临时关限位"安全，也不会留下忘了开回来的隐患。
- 编码器计数的**零点是上电位置**，所有限位常数都相对它。上电时摆杆必须停在同一个
  可重复的机械参考位（推荐断电自重靠住的那一端），否则限位护错地方。详见标定手册 #11。
- **丢步看 `GetPositionErrorCount()`**：不丢步时它稳定在跟踪误差附近
  （约 `速度 ÷ SERVO_KP` 换算的计数），丢步时电机没走到、编码器落后，它会超出该值
  且不收敛。原先的开环 `est` / `slip` 已删除——闭环下 `err` 是更直接的同一件事。
- **闭环的最小可指令位移是 1 个编码器计数 = 0.18°，由反馈分辨率决定，不由细分数决定。**
  伺服误差是**整数计数**，比一个计数小的位移表达不出来。一个微步是
  `STEP_ANGLE_DEG / MICROSTEP` = 0.05625°（×32），只有 0.3125 个编码器计数——**开环意义
  上的最小步在闭环下是发不出去的**，提高细分只让这 1 个计数走得更平顺。
  点动步长的公式是 `2 × 容差 + 1`（`Device Check -> Step Motor` 的 `JOG` 最细一档），
  容差取 0 时退化为 1 计数；`2×` 是因为容差非 0 时伺服停下的残余误差与行进方向同号、
  换向会把它抵掉，不留这个 2 倍则换向第一下必然死键。
  要走进 1 个计数以内，只能开环按微步数发脉冲——权衡见标定手册「还想更细：开环微步点动」，
  结论是不值得做（多一种绕过位置限幅唯一入口的接口，且位移开环、丢步测不出来）。
- **步进精细度与定位精细度是两件独立的事。** 前者 = 一个微步走多远，由**驱动器拨码**定
  （固件只能跟随 `MICROSTEP`），管的是运动平顺度；后者 = 停下时离目标多远，由到位容差定，
  管的是落点准不准。提高细分不会让定位更准（微步本就比容差细），细节见标定手册
  「精细度：两件独立的事」。
- **到位判定带回差**：进门看 `POSITION_TOLERANCE_COUNTS`（0）、出门看
  `SERVO_RESUME_COUNTS`（6）。这样「停多准」和「漂多少才重新动」是两个独立参数，
  收紧前者不会引出末端抖动——`RESUME` 挡的是摆杆自重下沉，与定位精度无关，所以容差
  收到 0 时它刻意不跟着收。**这个解耦正是容差敢取 0 的前提**：单阈值下容差 0 会让摆杆
  一沉就补偿、永不停歇。`IsAtTarget()` 直接返回该状态，不另算一遍；
  **新目标一下发即清掉到位状态**，所以回差不会吃掉小位移指令。
- ⚠ **容差 0 下 `IsAtTarget()` 要求误差恰好为 0。** 电机若因静摩擦/卡死停在差一个计数处，
  依赖它做时序的调用方（自检页的 `SWEEP` 翻头、`TURN` 报 `DONE`）会一直等不到。
  那本身是故障态，但表现从「报错」变成「卡住」——排查时先怀疑容差，抬回 1 即可。
- 软限位防的是控制器和操作者的失误，**不能替代物理限位开关**。
