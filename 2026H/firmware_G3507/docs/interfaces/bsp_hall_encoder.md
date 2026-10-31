# bsp/motor/hall_encoder 接口说明

## 模块职责

`bsp/motor/hall_encoder` 负责**左右双轮**霍尔编码器的 A/B 相脉冲计数、采样周期更新、方向判断、速度换算和轮端距离估计。

每轮采用「**A 相上升沿中断计数 + 读 B 相电平定方向**」的软件正交解码。模块基于固定车体参数提供每轮 `HallEncoder_GetSpeed(id)` 与 `HallEncoder_GetDistance(id)`，不负责底盘运动控制，也不负责速度闭环。

## 硬件映射宏

```c
/* 右轮: A=ENC_R_A(PA28, 中断), B=ENC_R_B(PA2, 读向) */
#define HALL_ENCODER_R_A_PORT Motor_IO_ENC_R_A_PORT
#define HALL_ENCODER_R_A_PIN  Motor_IO_ENC_R_A_PIN
#define HALL_ENCODER_R_B_PORT Motor_IO_ENC_R_B_PORT
#define HALL_ENCODER_R_B_PIN  Motor_IO_ENC_R_B_PIN
/* 左轮: A=ENC_L_A(PA22, 中断), B=ENC_L_B(PA25, 读向) */
#define HALL_ENCODER_L_A_PORT Motor_IO_ENC_L_A_PORT
#define HALL_ENCODER_L_A_PIN  Motor_IO_ENC_L_A_PIN
#define HALL_ENCODER_L_B_PORT Motor_IO_ENC_L_B_PORT
#define HALL_ENCODER_L_B_PIN  Motor_IO_ENC_L_B_PIN
```

引脚在 SysConfig 中按轮命名为 `ENC_R_A/ENC_R_B/ENC_L_A/ENC_L_B`（原 `E1A/E1B/E2A/E2B` 语义不清，已改名）。**两轮 A 相均在 GPIOA**，中断聚合到 `INT_GROUP1`（`GPIOA_INT_IRQn`）。`GROUP1_IRQHandler()` 会分别读取两轮 A 相的使能中断状态、命中则读对应 B 相电平解码、并各自清除。

> 历史坑：旧单轮实现把右轮 B 相错接到 `PB22`（实为一个导航按键脚），导致 B 相恒为高、方向恒判「前进」——仅前进时距离/方向看似正确，倒车会判错。现已修正为右轮 B = `PA2`。

## 中断和采样宏

```c
#define HALL_ENCODER_GPIO_IRQN         GPIOA_INT_IRQn
#define HALL_ENCODER_SAMPLE_TIMER      TIMER_0_INST
#define HALL_ENCODER_SAMPLE_TIMER_IRQN TIMER_0_INST_INT_IRQN
```

`GROUP1_IRQHandler()`（GPIO 边沿）与 `TIMER_0_INST_IRQHandler()`（定周期采样）为中断外壳，后者转调 `HallEncoder_UpdateSample()` 结算两轮。

## 物理参数宏

```c
#define HALL_ENCODER_PI               3.1415926f
#define HALL_ENCODER_PPR              13.0f
#define HALL_ENCODER_REDUCTION_RATIO  28.0f
#define HALL_ENCODER_WHEEL_DIAMETER_M 0.065f
#define HALL_ENCODER_SAMPLE_PERIOD_S  0.02f
#define HALL_ENCODER_DISTANCE_SCALE   1.05f
```

两轮同型号，共用换算参数。距离换算公式：

```text
distance_m = count / (PPR * reduction_ratio)
           * (PI * wheel_diameter_m)
           * distance_scale
```

速度由当前采样周期的距离增量除以 `HALL_ENCODER_SAMPLE_PERIOD_S` 得到，单位 m/s。

> ⚠ `HALL_ENCODER_SAMPLE_PERIOD_S` **必须与 SysConfig 里采样定时器 `TIMER_0`(TIMA1)的实际周期一致**。
> 二者不符时：距离不含时间仍准，但**速度会按比例算错**（曾出现定时器 100ms、常数 10ms → 速度偏大 10 倍，
> 且速度仅 10Hz 刷新拖慢速度环）。当前为 **20ms**（`LOAD=799`）。
>
> ⚠ 测速窗**刻意不跟随控制拍**。控制环已过采样到 10ms/100Hz，但测速窗每缩短一半，
> 速度量化台阶就翻一倍：
>
> | 测速窗 | 量化台阶 | @0.26 m/s 巡航 |
> |---|---|---|
> | 20 ms（当前）| 0.0295 m/s | 11.3% |
> | 10 ms | 0.0590 m/s | 22.7% |
>
> 因此编码器测速与控制拍解耦：控制环 100Hz 读，测速值 50Hz 更新，每个值被连读两拍。
> 底盘速度环是纯 PI（`CHASSIS_SPEED_KD = 0`），阶梯输入不会激发微分尖峰。
> 要真正提升速度分辨率应做 **A 相双边沿（2×）或 AB 四倍频（4×）**，而不是缩短窗口。

## 方向符号宏

```c
#define HALL_ENCODER_LEFT_DIR_SIGN  (+1)
#define HALL_ENCODER_RIGHT_DIR_SIGN (+1)
```

左右轮镜像安装，同样的电气解码在整车前进时符号可能相反。用本符号把两轮统一到「整车前进为正」。**上板验证**：整车前进时两轮 `speed` 应同为正；若某轮相反，翻转对应符号即可。

## 公开类型

```c
typedef enum {
    HALL_ENCODER_LEFT = 0,
    HALL_ENCODER_RIGHT,
    HALL_ENCODER_COUNT
} HALL_ENCODER_ID;

typedef enum {
    HALL_ENCODER_DIR_FORWARD = 0,
    HALL_ENCODER_DIR_REVERSE
} HALL_ENCODER_DIR;
```

## 公开接口

所有按轮查询接口以 `HALL_ENCODER_ID` 选择左右轮。

### `BSP_STATUS HallEncoder_Init(void)`

清零两轮内部状态，打开 GPIO 中断和采样定时器中断。

### `void HallEncoder_UpdateSample(void)`

将两轮待采样脉冲数转为当前采样周期计数，更新各自方向、速度和累计距离（套用方向符号）。

### `int32_t HallEncoder_GetCount(HALL_ENCODER_ID id)`

返回指定轮最近一个采样周期的有符号脉冲数。

### `HALL_ENCODER_DIR HallEncoder_GetDir(HALL_ENCODER_ID id)`

返回指定轮最近一个采样周期方向。采样计数大于等于 0 时为 `HALL_ENCODER_DIR_FORWARD`。

### `float HallEncoder_GetSpeed(HALL_ENCODER_ID id)`

返回指定轮最近一个采样周期换算得到的有符号轮端线速度，单位 m/s。

### `float HallEncoder_GetDistance(HALL_ENCODER_ID id)`

返回指定轮自上次复位以来的有符号轮端距离估计，单位 m。

### `void HallEncoder_Reset(void)`

清空两轮全部状态（待采样计数、采样计数、累计计数、速度、距离、方向）。

### `void HallEncoder_ResetDistance(void)`

只清空两轮累计计数和距离，不影响当前待采样/采样计数和速度。

## 对接说明

底盘组合服务 `middleware/chassis` 通过 `HallEncoder_*` 获取每轮速度/距离，并用 `core/kinematics` 的 `Kinematics_WheelToBody` 聚合车体线速度（见 `docs/interfaces/middleware_chassis.md`）。
