# 接口说明

## app

- `main.c`：完成 SysConfig、BSP、middleware 初始化和中断分发，然后进入 `App_Launch()`。
- `app_launcher.h`：提供 E 题前三项任务和设备检查的顶层菜单入口，详细说明见 `docs/interfaces/app_launcher.md`。
- `app_e_task.h`：提供 E 题基本要求 1、2、3 的任务流程入口，详细说明见 `docs/interfaces/app_e_task.md`。
- `app_device_check.h`：提供设备检查页面和 IMU 调试 UART 字节处理入口，详细说明见 `docs/interfaces/app_device_check.md`。

## core

- `pid/pid.h`：位置式和增量式 PID 控制器，详细说明见 `docs/interfaces/core_pid.md`。
- `common/core_types.h`：core 层基础二维点和二维姿态类型，详细说明见 `docs/interfaces/core_common.md`。
- `kinematics/kinematics.h`：角度、位姿、差速混控和二维几何计算，详细说明见 `docs/interfaces/core_kinematics.md`。
- `rotation/rotation.h`：欧拉角、旋转矩阵和三维向量旋转计算，详细说明见 `docs/interfaces/core_rotation.md`。
- `geometry/geometry.h`：二维矩形插值、纸面到矩形映射和圆点计算，详细说明见 `docs/interfaces/core_geometry.md`。
- `line_tracking/line_tracking.h`：巡线偏差计算、PID 修正和底盘输出接口，详细说明见 `docs/interfaces/core_line_tracking.md`。
- `gimbal_tracking/gimbal_tracking.h`：基于 CanMV 目标和 PID 的云台视觉跟踪控制，详细说明见 `docs/interfaces/core_gimbal_tracking.md`。

## middleware

- `chassis.h`：底盘组合服务，详细说明见 `docs/interfaces/middleware_chassis.md`。
- `gimbal.h`：云台组合服务，详细说明见 `docs/interfaces/middleware_gimbal.md`。
- `line_follow.h`：巡线运行状态服务，详细说明见 `docs/interfaces/middleware_line_follow.md`。
- `ui.h`：轻量 OLED UI 渲染层，详细说明见 `docs/interfaces/middleware_ui.md`。
- `system_fault.h`：系统故障处理服务，详细说明见 `docs/interfaces/middleware_system_fault.md`。

## bsp

- `common/bsp_common.h`：BSP 公共状态，详细说明见 `docs/interfaces/bsp_common.md`。
- `time/bsp_time.h`：BSP 系统时间和阻塞延时服务，详细说明见 `docs/interfaces/bsp_time.md`。
- `key.h`：按键读取、消抖和事件生成，详细说明见 `docs/interfaces/bsp_key.md`。
- `oled.h`：SSD1306 OLED 软件 I2C 显示接口，详细说明见 `docs/interfaces/bsp_oled.md`。
- `step_motor.h`：步进电机开环速度、阻塞定时运行和估计位置接口，详细说明见 `docs/interfaces/bsp_step_motor.md`。
- `canmv_uart.h`：CanMV UART 收发、帧解析和目标状态接口，详细说明见 `docs/interfaces/bsp_canmv_uart.md`。
- `wit_sdk.h`：WitMotion IMU 厂家驱动、JY61P 数据解析和结构体读取接口，详细说明见 `docs/interfaces/bsp_wit_sdk.md`。
- `tb6612fng.h`：TB6612FNG 双路直流电机驱动芯片接口，详细说明见 `docs/interfaces/bsp_tb6612fng.md`。
- `hall_encoder.h`：霍尔编码器计数、速度和距离估计接口，详细说明见 `docs/interfaces/bsp_hall_encoder.md`。
- `grayscale_sensor.h`：8 路光敏灰度传感器数字量接口，详细说明见 `docs/interfaces/bsp_grayscale_sensor.md`。
