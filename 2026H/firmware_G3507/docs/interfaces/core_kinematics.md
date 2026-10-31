# core/kinematics 接口说明

## 模块职责

`core/kinematics` 提供限幅、角度归一化、差速混控以及两轮差速运动学模型等纯算法计算能力，属于纯 `core` 模块。

该模块不读取传感器、不控制电机、不调用 BSP 或 middleware。调用方负责提供输入数据，并根据计算结果调用对应执行接口。

> 使用现状：`middleware/line_follow` 使用 `Kinematics_Clamp` 与
> `Kinematics_DifferentialMix`；`middleware/chassis` 使用 `Kinematics_WheelToBody`
> 将左右轮速度聚合为车体线速度。
> 本文其余标注为「预留」的类型/宏/函数已实现并保留，
> 但当前工程暂无调用者，供后续路径/姿态/动作策略模块复用。

## 基础宏

```c
#define KINEMATICS_PI 3.1415926f
#define KINEMATICS_DEG_TO_RAD(deg) ((deg) * (KINEMATICS_PI / 180.0f))
#define KINEMATICS_RAD_TO_DEG(rad) ((rad) * (180.0f / KINEMATICS_PI))

#define KINEMATICS_TURN_RADIUS_STRAIGHT 1.0e6f
```

角度/弧度换算宏供下方「两轮差速运动学模型」内部换算使用（公开 API 角度统一用 deg）。
`KINEMATICS_TURN_RADIUS_STRAIGHT` 是 `Kinematics_TurnRadius` 在 `ω≈0`（直线）时返回的转弯半径哨兵（m）。

## 数据结构

```c
typedef enum {
    KINEMATICS_DIR_LEFT = 0,
    KINEMATICS_DIR_RIGHT,
    KINEMATICS_DIR_FORWARD,
    KINEMATICS_DIR_BACKWARD,
    KINEMATICS_DIR_UNSTABLE
} KINEMATICS_DIR;
```

通用方向枚举。**预留，当前工程暂无调用者**，供后续路径和动作策略模块表达方向。

```c
typedef struct {
    float yaw;
    float pitch;
    float roll;
} KINEMATICS_ATTITUDE;
```

姿态角，单位为度。**预留，当前工程暂无调用者。**

```c
typedef struct {
    float x_m;
    float y_m;
    float heading_deg;
} KINEMATICS_POSE;
```

二维位姿，`Kinematics_IntegratePose` 的位姿类型。`x_m` 和 `y_m` 单位为米，`heading_deg` 单位为度。**预留，当前工程暂无调用者。**

```c
typedef struct {
    float linear_mps;
    float angular_deg_s;
} KINEMATICS_VELOCITY;
```

平面运动速度，`Kinematics_WheelToBody`/`Kinematics_IntegratePose` 的车体速度类型。线速度单位为米每秒，角速度单位为度每秒（逆时针为正）。当前由 `middleware/chassis` 使用其 `linear_mps` 字段。

```c
typedef struct {
    float left;
    float right;
} KINEMATICS_DIFFERENTIAL_OUTPUT;
```

差速混控输出。`left` 和 `right` 的单位由调用方输入决定：如果输入为占空比百分比，输出也为占空比百分比；如果输入为物理速度，输出也为对应物理量。

```c
typedef struct {
    float left_mps;
    float right_mps;
} KINEMATICS_WHEEL_SPEED;
```

两轮差速运动学模型的左右轮线速度输出，单位为米每秒。与占空比语义的 `KINEMATICS_DIFFERENTIAL_OUTPUT` 区分。**预留，当前工程暂无调用者。**

## 接口

### `float Kinematics_Clamp(float value, float min_value, float max_value)`

将 `value` 限制在 `[min_value, max_value]` 范围内。如果最小值大于最大值，函数内部会自动交换二者。

### `float Kinematics_NormalizeAngleDeg(float angle_deg)`（预留）

将角度归一化到 `[-180, 180)` 范围。**预留，当前工程暂无调用者**（仅被 `Kinematics_AngleDiffDeg` 内部使用）。

### `float Kinematics_AngleDiffDeg(float target_deg, float current_deg)`（预留）

返回从 `current_deg` 到 `target_deg` 的最短角度差，结果范围为 `[-180, 180)`。**预留，当前工程暂无调用者。**

### `KINEMATICS_DIFFERENTIAL_OUTPUT Kinematics_DifferentialMix(float forward, float turn, float output_limit)`

根据前进量和转向量计算差速左右输出：

```text
left  = forward + turn
right = forward - turn
```

当 `output_limit > 0.0f` 且任一侧输出超过限制时，函数按比例缩放左右输出，保持差速比例不变。该函数只计算输出，不调用 `Chassis_SetDuty()`。

## 两轮差速运动学模型

建立 **轮速 ↔ 车体 `(v, ω)` ↔ 位姿** 的速度层几何关系，全部为纯函数，车体参数（轮距 `L`、轮半径 `r`）一律传参、不在本模块固化。当前 `middleware/chassis` 已用
`Kinematics_WheelToBody(..., track_width_m=0)` 计算双轮平均线速度；角速度、逆运动学和位姿积分
仍为预留能力。当前实测轮距 `L=0.206 m` 已由循迹应用直接用于四段状态机的弯道差速前馈；
通用车体 `v/ω` 接口接入后应将该值作为参数传给本模块。

### 约定

| 量 | 符号 | 单位 |
|---|---|---|
| 左右轮着地点线速度 | `v_l` / `v_r` | m/s |
| 车体前进线速度 | `v` | m/s |
| 车体航向角速度（yaw，逆时针为正） | `ω` | deg/s |
| 轮自转角速度 | — | rad/s（直接配 `v = r·ω`） |
| 轮距 / 轮半径 | `L` / `r` | m |
| 位姿 | `(x, y, θ)` | m, m, deg |

> 车体角速度用 **deg/s**（与陀螺 `gz`、`KINEMATICS_VELOCITY.angular_deg_s` 一致，其符号与 `gz` 对齐由调用方保证）；轮自转角速度用 **rad/s**（无换算因子直接配 `v = r·ω`，便于对接编码器）。

### 公式

```text
正运动学：v = (v_r + v_l) / 2          ω = (v_r − v_l) / L
逆运动学：v_l = v − ω·L/2              v_r = v + ω·L/2
轮速换算：ω_wheel = v / r              v = r · ω_wheel
转弯半径：R = v / ω    （ω→0 直线 R→∞；v_l = −v_r 原地自转 R=0）
位姿积分（中点欧拉）：
    θ_mid = θ + ω·dt/2
    x += v·dt·cos(θ_mid)   y += v·dt·sin(θ_mid)   θ = normalize(θ + ω·dt)
```

### 接口

```c
KINEMATICS_VELOCITY    Kinematics_WheelToBody(float left_mps, float right_mps, float track_width_m);
KINEMATICS_WHEEL_SPEED Kinematics_BodyToWheel(float linear_mps, float angular_deg_s, float track_width_m);
float Kinematics_WheelLinearToAngular(float linear_mps, float wheel_radius_m);
float Kinematics_WheelAngularToLinear(float angular_rad_s, float wheel_radius_m);
float Kinematics_TurnRadius(float linear_mps, float angular_deg_s);
KINEMATICS_POSE Kinematics_IntegratePose(KINEMATICS_POSE pose, KINEMATICS_VELOCITY vel, float dt_s);
```

- `Kinematics_WheelToBody`：正运动学，左右轮线速度 → 车体 `(v, ω)`；`track_width_m <= 0` 时角速度按 0 处理。
- `Kinematics_BodyToWheel`：逆运动学，车体 `(v, ω)` → 左右轮线速度。
- `Kinematics_WheelLinearToAngular` / `Kinematics_WheelAngularToLinear`：轮线速度与轮自转角速度互换；半径 `<= 0` 时前者返回 0。
- `Kinematics_TurnRadius`：瞬时转弯半径；`|ω|` 近 0 时返回 `KINEMATICS_TURN_RADIUS_STRAIGHT`。
- `Kinematics_IntegratePose`：按 `(v, ω)` 积分一步位姿（中点欧拉，直线自然退化）；`dt_s <= 0` 原样返回，航向归一化到 `[-180, 180)`。
