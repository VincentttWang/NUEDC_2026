# core/motion 接口说明

## 模块职责

`core/motion` 提供底盘运动原语执行器。它只负责把上层给出的运动命令转换为底盘输出，不负责判断任务是否完成。

该模块负责：

- 将直行、倒车、刹车、滑行、原地左转、原地右转和循线命令转换为底盘输出。
- 保存最近一次运动命令、左右轮占空比、循线输出和底层返回状态。
- 为 app 或任务状态机提供统一的运动命令构造函数。

该模块不负责：

- 不判断运行时间、运行距离或超时。
- 不判断是否捕获中心线。
- 不判断边线计数、圈数或赛题阶段。
- 不操作 OLED、K230 或云台。
- 不直接访问 BSP 电机驱动。

任务状态机应在 app 或后续专门模块中组合这些运动原语，例如“直角转弯 = 刹车 + 原地转动 + 外部条件判断中心线重新捕获”。

## 数据结构

```c
typedef enum {
    MOTION_MODE_IDLE = 0,
    MOTION_MODE_BRAKE,
    MOTION_MODE_COAST,
    MOTION_MODE_STRAIGHT,
    MOTION_MODE_BACKWARD,
    MOTION_MODE_SPIN_LEFT,
    MOTION_MODE_SPIN_RIGHT,
    MOTION_MODE_LINE_FOLLOW
} MOTION_MODE;
```

- `MOTION_MODE_IDLE`：空闲，不主动输出新的底盘命令。
- `MOTION_MODE_BRAKE`：主动刹车。
- `MOTION_MODE_COAST`：关闭输出并滑行。
- `MOTION_MODE_STRAIGHT`：左右轮同向前进。
- `MOTION_MODE_BACKWARD`：左右轮同向后退。
- `MOTION_MODE_SPIN_LEFT`：左轮后退、右轮前进，原地左转。
- `MOTION_MODE_SPIN_RIGHT`：左轮前进、右轮后退，原地右转。
- `MOTION_MODE_LINE_FOLLOW`：调用 `LineTracking_Update()` 执行一次循线输出。

```c
typedef struct {
    MOTION_MODE mode;
    float duty_percent;
} MOTION_COMMAND;
```

- `mode`：运动模式。
- `duty_percent`：基础占空比。构造函数会取绝对值，方向由 `mode` 决定。

```c
typedef struct {
    MOTION_MODE mode;
    MOTION_COMMAND command;
    CHASSIS_DUTY duty;
    LINE_TRACKING_OUTPUT line_output;
    BSP_STATUS last_status;
} MOTION_STATE;
```

- `mode`：最近一次执行的运动模式。
- `command`：最近一次命令参数。
- `duty`：最近一次底盘左右轮占空比。
- `line_output`：最近一次循线输出，非循线模式下保留上一次值。
- `last_status`：最近一次底层调用返回状态。

## 公开接口

### `void Motion_Init(void)`

初始化运动原语执行器，清空最近状态。

### `BSP_STATUS Motion_Apply(const MOTION_COMMAND *command, float dt_s)`

执行一次运动命令。

各模式当前行为：

- `IDLE`：不输出新的底盘命令。
- `BRAKE`：调用 `Chassis_Brake()`。
- `COAST`：调用 `Chassis_Coast()`。
- `STRAIGHT`：调用 `Chassis_SetDuty(+duty, +duty)`。
- `BACKWARD`：调用 `Chassis_SetDuty(-duty, -duty)`。
- `SPIN_LEFT`：调用 `Chassis_SetDuty(-duty, +duty)`。
- `SPIN_RIGHT`：调用 `Chassis_SetDuty(+duty, -duty)`。
- `LINE_FOLLOW`：调用 `LineTracking_Update(dt_s)`。

传入空指针时返回 `BSP_STATUS_NULL`。

### `BSP_STATUS Motion_Stop(void)`

主动刹车并将最近模式记录为 `IDLE`。

### `MOTION_STATE Motion_GetState(void)`

返回最近一次运动状态快照。

## 命令构造函数

```c
MOTION_COMMAND Motion_CommandBrake(void);
MOTION_COMMAND Motion_CommandCoast(void);
MOTION_COMMAND Motion_CommandStraight(float duty_percent);
MOTION_COMMAND Motion_CommandBackward(float duty_percent);
MOTION_COMMAND Motion_CommandSpinLeft(float duty_percent);
MOTION_COMMAND Motion_CommandSpinRight(float duty_percent);
MOTION_COMMAND Motion_CommandLineFollow(void);
```

这些函数只构造命令，不执行动作。调用方仍需要周期调用 `Motion_Apply()`。
