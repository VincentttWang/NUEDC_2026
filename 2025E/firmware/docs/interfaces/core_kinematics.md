# core/kinematics 接口说明

## 模块职责

`core/kinematics` 提供运动学和二维几何计算能力，属于纯算法模块。

该模块不读取传感器、不控制电机、不调用 BSP 或 middleware。调用方负责提供输入数据，并根据计算结果调用对应执行接口。

## 基础宏

```c
#define KINEMATICS_PI 3.1415926f
#define KINEMATICS_DEG_TO_RAD(deg) ((deg) * (KINEMATICS_PI / 180.0f))
#define KINEMATICS_RAD_TO_DEG(rad) ((rad) * (180.0f / KINEMATICS_PI))
```

为兼容当前尚未重写的 `app` 代码，模块暂时保留：

```c
#define DEG_TO_RAD(deg) KINEMATICS_DEG_TO_RAD(deg)
#define RAD_TO_DEG(rad) KINEMATICS_RAD_TO_DEG(rad)
```

后续对应模块完成重写后，可评估是否移除兼容宏。

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

通用方向枚举。当前模块自身不强制使用该枚举，预留给后续路径和动作策略模块表达方向。

```c
typedef struct {
    float yaw;
    float pitch;
    float roll;
} KINEMATICS_ATTITUDE;
```

姿态角，单位为度。为兼容当前 `rotation` 接口，暂时提供：

```c
typedef KINEMATICS_ATTITUDE RotationAngles;
```

```c
typedef struct {
    float x_m;
    float y_m;
    float heading_deg;
} KINEMATICS_POSE;
```

二维位姿。`x_m` 和 `y_m` 单位为米，`heading_deg` 单位为度。

```c
typedef struct {
    float linear_mps;
    float angular_deg_s;
} KINEMATICS_VELOCITY;
```

平面运动速度。线速度单位为米每秒，角速度单位为度每秒。

```c
typedef struct {
    float left;
    float right;
} KINEMATICS_DIFFERENTIAL_OUTPUT;
```

差速混控输出。`left` 和 `right` 的单位由调用方输入决定：如果输入为占空比百分比，输出也为占空比百分比；如果输入为物理速度，输出也为对应物理量。

## 接口

### `float Kinematics_Clamp(float value, float min_value, float max_value)`

将 `value` 限制在 `[min_value, max_value]` 范围内。如果最小值大于最大值，函数内部会自动交换二者。

### `float Kinematics_NormalizeAngleDeg(float angle_deg)`

将角度归一化到 `[-180, 180)` 范围。

### `float Kinematics_AngleDiffDeg(float target_deg, float current_deg)`

返回从 `current_deg` 到 `target_deg` 的最短角度差，结果范围为 `[-180, 180)`。

### `float Kinematics_Distance2D(float x0_m, float y0_m, float x1_m, float y1_m)`

计算二维平面两点之间的距离，单位与输入坐标一致。

### `KINEMATICS_DIFFERENTIAL_OUTPUT Kinematics_DifferentialMix(float forward, float turn, float output_limit)`

根据前进量和转向量计算差速左右输出：

```text
left  = forward + turn
right = forward - turn
```

当 `output_limit > 0.0f` 且任一侧输出超过限制时，函数按比例缩放左右输出，保持差速比例不变。该函数只计算输出，不调用 `Chassis_SetDuty()`。

### `void Kinematics_PoseInit(KINEMATICS_POSE *pose)`

将二维位姿清零。

### `void Kinematics_PoseUpdate(KINEMATICS_POSE *pose, float linear_mps, float heading_deg, float dt_s)`

根据线速度、航向角和时间间隔积分更新二维位姿。

- `linear_mps`：线速度，单位米每秒。
- `heading_deg`：当前航向角，单位度。
- `dt_s`：时间间隔，单位秒。
- 当 `pose == NULL` 或 `dt_s <= 0.0f` 时不更新。

## 迁移说明

旧接口 `PID_Move()`、`runCircle()`、`Straight()` 和 `track()` 带有任务流程或控制动作语义，不再由 `kinematics` 提供。后续应分别迁移到 `middleware/line_tracking`、具体题目流程或更高层控制策略中。
