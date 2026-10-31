# core/pid 接口说明

## 模块职责

`core/pid` 提供通用 PID 控制器算法，属于纯 `core` 模块。

该模块只负责根据目标值、反馈值和时间间隔计算控制输出，不直接访问 BSP、middleware 或 app，也不绑定任何具体外设。

## 控制模式

```c
typedef enum {
    PID_MODE_POSITION = 0,
    PID_MODE_INCREMENTAL
} PID_MODE;
```

- `PID_MODE_POSITION`：位置式 PID，输出值直接表示当前控制量。
- `PID_MODE_INCREMENTAL`：增量式 PID，输出值在上一次输出基础上累加本次增量。**预留**：
  当前消费者 `middleware/line_follow` 与 `middleware/chassis` 都使用位置式；
  增量式路径已实现并保留但暂无调用者。

位置式 PID 适合角度、视觉误差、巡线偏差等场景。增量式 PID 适合速度闭环、占空比微调等希望输出连续变化的场景。

## 数据结构

```c
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    PID_MODE mode;
} PID_CONFIG;
```

- `kp`：比例系数。
- `ki`：积分系数。
- `kd`：微分系数。
- `integral_limit`：位置式 PID 的积分限幅。取 `PID_LIMIT_DISABLED`（或任意 `<= 0` 值）时不启用限幅。
- `output_limit`：输出限幅。取 `PID_LIMIT_DISABLED`（或任意 `<= 0` 值）时不启用限幅。
- `mode`：控制器模式。

```c
typedef struct {
    float target;
    float feedback;
    float error;
    float last_error;
    float prev_error;
    float integral;
    float derivative;
    float increment;
    float output;
} PID_STATE;
```

- `target`：最近一次目标值。
- `feedback`：最近一次反馈值。
- `error`：当前误差。
- `last_error`：上一次误差。
- `prev_error`：上上次误差，主要用于增量式 PID。
- `integral`：位置式 PID 的积分项累计值。
- `derivative`：最近一次微分项。
- `increment`：最近一次经过输出限幅后的实际输出变化量。
- `output`：最近一次控制输出。

```c
typedef struct {
    PID_CONFIG config;
    PID_STATE state;
} PID_CONTROLLER;
```

`PID_CONTROLLER` 保存一个 PID 实例的参数和运行状态。调用方应为每个控制环路单独维护一个实例。

## 接口

```c
void PID_Init(PID_CONTROLLER *pid, const PID_CONFIG *config);
```

初始化 PID 控制器，写入配置并清空运行状态。

```c
void PID_Reset(PID_CONTROLLER *pid);
```

清空运行状态，但保留当前配置。

```c
float PID_Update(PID_CONTROLLER *pid, float target, float feedback, float dt_s);
```

更新目标值、反馈值和时间间隔，并返回当前输出。

- `dt_s` 单位为秒。
- 当 `dt_s <= 0.0f` 时，不更新状态，直接返回上一次输出。
- 位置式 PID 使用积分限幅和输出限幅。
- 增量式 PID 根据误差变化计算本次增量，再将增量累加到输出，并执行输出限幅。

> 说明：`PID_Update()` 返回当前输出；运行结果同时保存在 `PID_CONTROLLER.state`
> （`output`/`error`/`increment` 等字段），调用方可直接读取该结构，模块暂不提供独立 getter。

## 使用约束

- `PID_CONTROLLER` 不应作为全局共享控制器跨多个控制对象复用。
- 调用方负责提供稳定的 `dt_s`。
- PID 参数整定不属于本模块职责。
