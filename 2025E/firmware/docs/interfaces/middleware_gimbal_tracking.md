# middleware/gimbal_tracking 接口说明

## 模块职责

`middleware/gimbal_tracking` 提供基于 CanMV 视觉目标的云台跟踪控制。

该模块负责：

- 从 `bsp/canmv` 获取激光点和矩形角点数据。
- 使用 `core/geometry` 计算纸面圆点到图像矩形的映射。
- 使用 `core/pid` 计算 yaw/pitch 跟踪速度。
- 调用 `middleware/gimbal` 输出云台速度。

该模块不直接调用 BSP 步进电机驱动，不访问旧 `Laser_Loc`、`Rect_Loc` 全局变量，不处理 LED 调试和巡线阶段距离积分。

## 数据结构

```c
typedef struct {
    PID_CONFIG yaw_pid;
    PID_CONFIG pitch_pid;
    PID_CONFIG yaw_angle_pid;
    PID_CONFIG pitch_angle_pid;
    float image_height;
    float paper_width;
    float paper_height;
    float circle_radius;
    float yaw_output_sign;
    float pitch_output_sign;
    uint32_t link_timeout_ms;
} GIMBAL_TRACKING_CONFIG;
```

- `yaw_pid` / `pitch_pid`：坐标帧（像素误差）使用的独立 PID 参数。当前实机起调值为：yaw `Kp/Ki/Kd=1.20/0.12/0.006`、积分/输出限幅 `1000/120 deg/s`；pitch `0.20/0.02/0.003`、限幅 `300/60 deg/s`。
- `yaw_angle_pid` / `pitch_angle_pid`：角度帧（deg 误差）使用的独立 PID 参数。其初值按当前约 `5 px/deg` 的视场近似从像素 PID 换算，须在实机上单独整定。
- `image_height`：图像高度，当前作为图像尺寸配置保留；K230 端已经在发送前完成摄像头倒装的中心对称修正，MCU 侧直接使用收到的图像坐标，不再额外翻转 y 轴。
- `paper_width` / `paper_height`：纸面尺寸。
- `circle_radius`：纸面圆路径半径。
- `yaw_output_sign` / `pitch_output_sign`：输出方向系数，用于适配安装方向。
- `link_timeout_ms`：K230 有效目标数据超时时间，默认 `1000ms`；在该时间内没有收到可用于跟踪的有效目标数据时，视觉跟踪会立即停止云台输出。

```c
typedef struct {
    CORE_POINT2F target;
    CORE_POINT2F laser;
    CORE_POINT2F error;
    float yaw_speed;
    float pitch_speed;
    CANMV_STATUS laser_status;
    CANMV_STATUS rect_status;
    bool target_valid;
    bool laser_valid;
    bool link_timeout;
} GIMBAL_TRACKING_STATE;
```

保存最近一次跟踪状态，供调试显示和上层状态判断使用。

## 接口

### `void GimbalTracking_Init(const GIMBAL_TRACKING_CONFIG *config)`

初始化云台跟踪控制器。传入 `NULL` 时使用默认配置。

初始化会记录当前系统时间，作为后续有效目标数据超时判断的起点。

### `void GimbalTracking_Reset(void)`

清空 PID 状态和最近一次跟踪状态。

### `BSP_STATUS GimbalTracking_UpdateLaserCenter(float dt_s)`

从 `CANMV_TARGET_LASER` 读取目标中心和激光点，并执行一次云台跟踪控制。

若 UART2 收到新的 `A5 5A` 角度帧，本接口自动切换为角度模式，使用 `yaw_angle_pid` / `pitch_angle_pid`；帧间保持最近速度命令，达到 `link_timeout_ms` 后停止。一次初始化周期内不混用旧坐标帧和角度帧。

当前 K230 侧 `rect_recognition.py` 发送的激光段为：修正后的目标中心 `target_x/target_y`，以及作为当前实际点位的图像中心 `laser_x/laser_y`。因此该接口直接跟踪目标中心到图像中心，不再在 MCU 侧重复处理摄像头倒装。

若 K230 在 `link_timeout_ms` 内没有新的有效目标数据到达，该接口会停止云台输出并返回未就绪状态。短暂丢失目标时不会立刻停止，云台会保持上一速度继续调整，直到超时保护触发。

该接口对应旧流程中的：

```text
SetLaserPosition()
SetTargetCenter()
PID_SMotor_Cont()
```

### `BSP_STATUS GimbalTracking_UpdateRectCircle(int32_t edge_index, float angle_offset_deg, float dt_s)`

从 `CANMV_TARGET_RECT` 读取矩形角点，根据 `edge_index` 和 `angle_offset_deg` 计算纸面圆点，并映射到图像矩形后执行跟踪。

该接口不读取旧 `edge` 和 `sInedge`，调用方负责传入阶段参数。

### `BSP_STATUS GimbalTracking_UpdateRectCenter(float dt_s)`

从 `CANMV_TARGET_RECT` 读取矩形角点，计算矩形中心，并让云台跟踪该中心点。

### `bool GimbalTracking_IsRectValid(void)`

检查当前是否已经收到有效矩形角点。该接口只用于任务流程判断，不输出云台速度，也不触发视觉跟踪超时停机；扫描阶段可以安全地反复调用该接口等待矩形出现。

### `BSP_STATUS GimbalTracking_TrackPoints(CORE_POINT2F target, CORE_POINT2F laser, float dt_s)`

直接根据目标点和激光点执行一次 PID 控制并调用 `Gimbal_SetSpeed()`。

### `BSP_STATUS GimbalTracking_Stop(void)`

调用 `Gimbal_Stop()` 停止云台运动。

### `GIMBAL_TRACKING_STATE GimbalTracking_GetState(void)`

返回最近一次云台跟踪状态。

## 边界说明

- 旧 `Compute_excur()` 不迁入本模块。该逻辑混合巡线转弯状态、编码器速度和 LED GPIO 调试，后续应按实际任务重新设计。
- 旧 `getDistance()` 不迁入本模块。阶段距离逻辑后续基于 `Chassis_GetDistance()` 或任务上下文重新设计。
- 本模块不直接操作 GPIO，不直接调用 `StepMotor_*` 或 `YP_SMotor_*`。
