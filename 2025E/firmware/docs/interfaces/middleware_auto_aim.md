# middleware/auto_aim 接口说明

## 职责

`auto_aim` 将硬件观测与纯计算模块组合为统一的绝对角控制链：

```text
编码器 + gyro-z ─► 单轮车心位移修正 ─┐
IMU yaw ──────────────────────────────┼─► localization
角点事件 ────────────────────────────┘         │
                                               ▼
K230 角度误差 ─► AimVisionBias ─► aim_solver ─► Gimbal_SetAngle
```

## 公开接口

- `AutoAim_DefaultConfig()`：返回规定起点、赛道尺寸、视觉方向和标定参数默认值。
- `AutoAim_Init()`：装载配置并初始化解算器。
- `AutoAim_Start()`：采样当前 IMU yaw，建立规定世界航向与传感器航向之间的偏置。
- `AutoAim_Update(dt)`：更新里程、位姿、视觉 bias 和云台绝对角。
- `AutoAim_AnchorCorner()`：在角点同时重锚世界坐标和理论累计里程。
- `AutoAim_Stop()`：停止自动瞄准并停止云台输出。
- `AutoAim_GetState()`：读取位姿、前馈角、bias、命令角和观测有效性。

## 模式

- `AUTO_AIM_MODE_CENTER`：持续指向靶心。
- `AUTO_AIM_MODE_CIRCLE`：用圈内里程进度驱动靶面圆周相位。

视觉没有新有效帧时 bias 保持不变，几何前馈仍持续运行。
