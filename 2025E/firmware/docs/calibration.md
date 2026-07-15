# 实机标定入口

MCU 侧比赛参数集中在：

```c
APP_E_CALIBRATION_CONFIG AppE_GetCalibrationConfig(void)
```

文件：`app/app_e_calibration.c`。

标定时只修改该函数中的值，不要在 `app_e_task.c` 状态机内散改常量。函数依次包含：

1. 规定起点、车头航向和赛道实测边长。
2. 单编码轮相对车体中心的横向偏移。
3. 靶心世界坐标和云台安装坐标。
4. 云台 yaw/pitch 机械零位。
5. 视觉 bias 增益、限幅和方向。
6. F3 圆半径和起始相位。
7. E2/E3 锁定阈值、确认帧数和超时打点时间。

编码器 PPR、减速比、轮径和距离比例仍属于 BSP 硬件参数，位于 `bsp/motor/hall_encoder.h`。K230 的 FOV、靶框尺寸、相机/激光偏移和理想激光角位于 `k230/aim_track_angle.py`，不能由 MCU 配置函数直接设置。

每轮标定后执行 Keil 全量重建，先验证纯前馈，再开启视觉 bias。

## 主界面实测入口

开机主菜单选择 `Calibration`，用 K1/K4 上下选择、K3 进入，测试中长按 K2 返回。入口提供以下互相隔离的测试函数：

| 菜单项 | 函数 | 屏幕观测量 | 对应参数 |
|---|---|---|---|
| `Encoder distance` | `AppE_CalibrationTest_RunEncoder()` | 距离、速度、原始计数 | `HALL_ENCODER_DISTANCE_SCALE`，必要时复核 PPR、减速比和轮径 |
| `IMU heading` | `AppE_CalibrationTest_RunImu()` | yaw、相对零点转角、z 轴角速度 | 起点 `heading_deg`、IMU 安装方向 |
| `Geo feedforward` | `AppE_CalibrationTest_RunFeedforward()` | yaw/pitch 指令、视觉误差；视觉 bias 强制关闭 | 靶心坐标、云台安装坐标、yaw/pitch 零位 |
| `Vision bias` | `AppE_CalibrationTest_RunVisionBias()` | 指令、视觉误差、累计 bias | `vision_*_sign`、`vision_yaw/vision_pitch`（逐轴 gain0/gain_min/limit）、`vision_*_startup_limit_deg` |
| `F3 circle phase` | `AppE_CalibrationTest_RunCircle()` | 手动相位和对应云台指令 | `circle_radius_m`、`circle_phase0_deg` |

所有瞄准测试进入时激光默认关闭，只有短按 K3 才切换激光；返回时会强制关闭激光并停止云台。编码器测试中短按 K3 清零；IMU 测试中短按 K3 将当前 yaw 设为相对零点；F3 测试中 K1/K4 以 10° 步长改变相位。

### 建议实测顺序

1. 在地面标出精确距离，进入 `Encoder distance` 清零并推动车辆，按 `新比例 = 旧比例 × 实际距离 / 屏显距离` 修正 `HALL_ENCODER_DISTANCE_SCALE`。
2. 固定底盘后进入 `IMU heading` 设零，机械旋转已知角度，确认相对转角的大小和符号。
3. 将车放在配置的 `start_pose`，进入 `Geo feedforward`，先校机械零位，再校安装坐标和靶心坐标。此时屏幕上的 `Bias` 应保持为 0。
4. 进入 `Vision bias`。若误差持续增大，先翻转对应轴 `vision_*_sign`；方向正确后再逐轴调衰减增益（`gain0` 管首帧收敛快慢，`gain_min` 管稳态跟踪速度；pitch 因减速比等效增益更高，从更小值起调）。
5. 进入 `F3 circle phase` 手动扫一圈，确认 0° 位于靶心右侧、90° 位于上方，再设置半径和起始相位。

实测函数只提供安全、可重复的观测过程，不会把参数写入 Flash。确认实测结果后，将结果回填到 `AppE_GetCalibrationConfig()` 或编码器 BSP 宏并重新编译。
