# core/filter 接口说明

## 模块职责

`core/filter` 提供通用信号调理算法，属于纯 `core` 模块。

该模块只做数值运算，不访问 BSP、middleware 或 app，也不绑定任何具体外设。所有接口均为**无状态自由函数**，滤波状态由调用方持有（与 `core/kinematics` 的 `Kinematics_Clamp`/`Kinematics_DifferentialMix` 同风格）。

当前消费者为 `middleware/line_follow`：用一阶低通平滑数字灰度的量化跳变、用中心死区对小偏心置零，抑制转向尖峰与蛇形极限环。

## 接口

```c
float Filter_LowpassEma(float state, float sample, float alpha);
```

一阶低通滤波（指数移动平均）单步更新：`new = state + alpha * (sample - state)`。

- `state`：上一拍滤波值，由调用方持有。
- `sample`：本拍原始输入。
- `alpha`：平滑系数，有效范围 `(0, 1]`；越小越平滑、滞后越大，取 `1` 则直通 `sample`。
- 返回本拍滤波值，供下一拍作为 `state` 传入。

```c
float Filter_Deadband(float value, float threshold);
```

中心死区：`|value| < threshold` 时返回 `0`，否则原样返回 `value`。

- `threshold`：死区半宽；`<= 0` 时不处理，原样返回。

## 使用约束

- 无状态设计，调用方需自行保存 `Filter_LowpassEma` 的滤波状态并在复位时清零。
- `alpha` 与 `threshold` 的整定值属于调用方语义，不在本模块固化。
