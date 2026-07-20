# app/app_e_task 接口说明

## 模块职责

`app/app_e_task` 只负责任务编排、比赛计时、激光得分策略、异常退出和 UI。定位、几何解算、视觉 bias 与云台位置指令由 `middleware/auto_aim` 完成。

## 任务入口

### `AppE_RunLineFollow(lap_count)`

执行 E1。使用灰度循迹、直角弯状态机和角点计数完成 1～5 圈，瞄准与激光保持关闭。

### `AppE_RunAimCenter2s()`

执行 E2 任意位姿瞄准。该任务保留 `middleware/gimbal_tracking` 纯视觉速度闭环；误差连续 8 个新视觉帧进入 yaw/pitch ±0.8° 后触发一次激光脉冲，1.8 秒仍未锁定时强制打点兜底，2 秒结束。

### `AppE_RunKnownPoseAim4s()`

执行 E3 规定位置瞄准。使用规定起点位姿建立几何前馈，K230 角度误差只更新慢速 bias。连续锁定后打点；临近 4 秒仍未锁定时强制打点。

### `AppE_RunLineAim(lap_count)`

执行 F1/F2。前馈就绪后先开启激光，再启动循迹。每次确认角点后同时重置世界坐标和理论累计里程。视觉丢失只冻结 bias，不中断前馈或激光。

### `AppE_RunLineCircle(lap_count)`

执行 F3。底盘链与 F1/F2 相同，瞄准目标改为 `AimSolver_SolveCircle()`；一圈内里程进度映射为圆周相位。

### `AppE_RunContinuousAim()`

纯视觉持续瞄准调试入口，不参与发挥任务。

## 默认赛道配置

- 世界系原点：AB 中点。
- 默认起点：A `(-0.5m, 0m)`。
- 默认车头：沿 A→C，航向 `-90°`。
- 默认左转角点序列：`C → D → B → A`。

上述参数必须按实际比赛起点和机械安装复核，不应散落修改任务逻辑。

## 安全与降级

- 发挥任务连续 10 个周期读取不到 IMU 时刹车并关闭激光。
- 丢线超过约 1 秒时刹车并关闭激光。
- 视觉 `NOT_FOUND/LOST` 时保持几何前馈，不更新 bias。
- 用户长按返回时先停止底盘和激光，再显示结果页。
