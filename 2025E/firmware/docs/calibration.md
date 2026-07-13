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
