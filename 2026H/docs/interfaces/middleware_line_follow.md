# middleware/line_follow 接口说明

## 模块职责

`middleware/line_follow` 负责巡线相关运行状态。它组合 `bsp/grayscale_sensor` 的 8 路灰度传感器读取能力，并维护边线计数和转弯状态。

该模块负责：

- 读取并缓存 8 路灰度传感器状态。
- 提供传感器数组、bit mask 和单通道读取接口。
- 维护边线计数 `edge_count`。
- 维护转弯状态 `turning`。

该模块不负责：

- 距离积分。
- 里程估计。
- PID 控制。
- 底盘电机控制。
- 题目流程。
- OLED 显示。

如果后续业务需要当前阶段距离，应在实际使用点基于 `Chassis_GetDistance()`、`Chassis_ResetDistance()` 或任务上下文重新设计，不在本模块预留距离积分接口。

## 公开类型

### `LINE_FOLLOW_SENSOR_COUNT`

```c
#define LINE_FOLLOW_SENSOR_COUNT 8U
```

灰度传感器通道数量。

### `LINE_FOLLOW_SENSOR_STATE`

```c
typedef struct {
    uint8_t value[LINE_FOLLOW_SENSOR_COUNT];
    uint8_t mask;
} LINE_FOLLOW_SENSOR_STATE;
```

- `value[i]`：第 `i` 路灰度传感器值，保留旧 `Digital[i]` 的逻辑语义。
- `mask`：8 路状态的 bit 表示，bit0 对应通道 0，bit7 对应通道 7。当前约定 `value[i] != 0` 时对应 bit 置 1。

### `LINE_FOLLOW_STATE`

```c
typedef struct {
    LINE_FOLLOW_SENSOR_STATE sensor;
    int32_t edge_count;
    bool turning;
} LINE_FOLLOW_STATE;
```

巡线状态快照。

## 公开接口

### `BSP_STATUS LineFollow_Init(void)`

清空状态并读取一次灰度传感器初始值。

### `void LineFollow_Reset(void)`

清空传感器缓存、边线计数和转弯状态。

### `BSP_STATUS LineFollow_Update(void)`

更新巡线状态。当前等价于 `LineFollow_UpdateSensor()`，保留该接口用于后续增加滤波或状态判定。

### `BSP_STATUS LineFollow_UpdateSensor(void)`

调用 `GrayscaleSensor_Read()` 更新传感器数组，并重新计算 `mask`。

该接口不会屏蔽、估计或消抖任意通道。Device Check 的 Line Sensor 页面显示同一份最新状态。

### `BSP_STATUS LineFollow_GetState(LINE_FOLLOW_STATE *out)`

复制完整巡线状态。`out == NULL` 时返回 `BSP_STATUS_NULL`。

### `BSP_STATUS LineFollow_GetSensor(LINE_FOLLOW_SENSOR_STATE *out)`

复制传感器状态。`out == NULL` 时返回 `BSP_STATUS_NULL`。

### `uint8_t LineFollow_GetSensorMask(void)`

返回当前传感器 bit mask。

### `uint8_t LineFollow_GetSensorValue(uint8_t index)`

返回指定通道传感器值。`index >= LINE_FOLLOW_SENSOR_COUNT` 时返回 `0`。

### `uint8_t LineFollow_GetActiveCount(void)`

返回当前检测到黑线的通道数量。当前灰度传感器语义为 `0` 表示检测到线。

### `bool LineFollow_IsActiveCountInRange(uint8_t min_count, uint8_t max_count)`

判断当前检测到线的通道数量是否位于指定范围。

### `bool LineFollow_IsEmpty(void)`

判断当前是否没有任何通道检测到线。

### `bool LineFollow_IsHalfDetected(void)`

判断当前是否满足半线/边线粗略判定。当前实现为检测到线的通道数量在 `3..6` 之间。

### `bool LineFollow_IsCrossDetected(void)`

判断当前是否满足十字线粗略判定。当前实现为检测到线的通道数量在 `7..8` 之间。

### `bool LineFollow_IsCenterActive(void)`

判断中间通道是否检测到线。当前实现使用第 `3`、`4` 两路通道中的任意一路。

### `int32_t LineFollow_GetEdgeCount(void)`

返回当前边线计数。

### `void LineFollow_SetEdgeCount(int32_t edge_count)`

设置边线计数。

### `void LineFollow_IncrementEdge(void)`

边线计数加一。

### `void LineFollow_ResetEdge(void)`

清零边线计数。

### `bool LineFollow_IsTurning(void)`

返回当前是否处于转弯状态。

### `void LineFollow_SetTurning(bool turning)`

设置转弯状态。

## 迁移说明

旧变量和接口的迁移关系：

```text
Digital[i]      -> LineFollow_GetSensorValue(i)
Digital[]       -> LineFollow_GetSensor()
edge            -> LineFollow_GetEdgeCount()
edge++          -> LineFollow_IncrementEdge()
turning         -> LineFollow_IsTurning()
turning = true  -> LineFollow_SetTurning(true)
```

旧 `sInedge` 和 `UpdateSInedge()` 不迁入本模块。后续在具体使用场景中重新设计阶段距离逻辑。
