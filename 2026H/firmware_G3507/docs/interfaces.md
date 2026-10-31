# 接口说明

> 二维云台/瞄准子系统（gimbal / gimbal_tracking / auto_aim / aim_solver / aim_fusion /
> localization / bldc / step_motor）已整体移除。`app` 已重建为菜单驱动的协作式调度框架；
> 下表仅列出当前仍存在并参与构建的模块。

## app

菜单驱动的协作式调度框架，详见 `docs/app-design.md`。

- `app_init.h`：`App_Init()` 集中式上电初始化并注册调度任务。
- `app_scheduler.h`：`Scheduler_Init/AddTask/Run/EnableTick` 时间触发调度器；自持 `SysTick_Handler`。
- `app_mode.h`：状态机 `APP_MODE`（INIT/MENU/RUN/FAULT）与 `App_Mode_Init/Get`、`App_ControlTick/UiTick`。
- `app_task.h`：任务生命周期契约——`APP_TASK_STATUS` 与 `APP_TASK_DESC`（`on_enter/on_tick/on_exit`），不含具体任务。
- `app_menu.h`：菜单树类型 `MENU_NODE`/`MENU_ITEM` 与 `Menu_Tick` 导航；`app_menu_def.c` 定义菜单树实例 `APP_ROOT_MENU`。
- `app_checks.h`：外设自检任务描述符 `APP_CHK_*`（JY61P / Gray I2C / TB6612 / Encoder /
  Speed PID / Duty Sweep）。
- `app_line_task.h`：赛题要求 2/4/5/6 的循迹任务入口，复用 Yahboom 循迹与 JY61P 共享
  I2C0 分时调度，整圈任务含 S1～S4 里程状态机和弯道差速前馈。
- `app_ball_task.h`：要求 3 的滚球任务入口；当前完成小车静止、树莓派反馈守 0 cm，完整
  ±5 cm 往返尚未接入。实机参数集中在 `app_ball_config.h`。

## core

- `pid/pid.h`：位置式和增量式 PID 控制器，详细说明见 `docs/interfaces/core_pid.md`。
- `filter/filter.h`：一阶低通(EMA)与中心死区等通用信号调理算法，详细说明见 `docs/interfaces/core_filter.md`。
- `common/core_types.h`：core 层基础二维点类型，详细说明见 `docs/interfaces/core_common.md`。
- `kinematics/kinematics.h`：限幅、角度归一化、差速混控与两轮差速运动学模型（正/逆运动学、转弯半径、位姿积分），详细说明见 `docs/interfaces/core_kinematics.md`。

## middleware

- `ball_balance.h`：纯计算的一维滚球在线终端约束控制；每拍从实测位置、速度和水管角
  重规划到 `(target,0,0)` 的五次轨迹，以倾角动力学前馈配合轻量位置/速度反馈，
  并提供受监督零偏学习、限斜率与截停诊断。正倾角定义为使球向正方向加速。
- `chassis.h`：底盘组合服务（开环占空比 + 每轮速度闭环 PID），详细说明见 `docs/interfaces/middleware_chassis.md`。
- `line_follow.h`：标准化八路黑线观测、巡线偏差计算、PID/陀螺修正和底盘输出，详细说明见
  `docs/interfaces/middleware_line_follow.md`。
- `ui.h`：轻量 OLED UI 渲染层，详细说明见 `docs/interfaces/middleware_ui.md`。
- `system_fault.h`：系统故障处理服务，详细说明见 `docs/interfaces/middleware_system_fault.md`。

## bsp

- `common/bsp_common.h`：BSP 公共状态，详细说明见 `docs/interfaces/bsp_common.md`。
- `time/bsp_time.h`：BSP 系统时间和阻塞延时服务，详细说明见 `docs/interfaces/bsp_time.md`。
- `key.h`：按键读取、消抖和事件生成，详细说明见 `docs/interfaces/bsp_key.md`。
- `oled.h`：SSD1306 OLED 帧缓冲显示接口（I2C1），详细说明见 `docs/interfaces/bsp_oled.md`。
- `wit_sdk.h`：JY61P（WIT 协议）I2C0 中断驱动、数据解析和结构体读取接口，详细说明见 `docs/interfaces/bsp_wit_sdk.md`。
- `tb6612fng.h`：TB6612FNG 双路直流电机驱动芯片接口，详细说明见 `docs/interfaces/bsp_tb6612fng.md`。
- `hall_encoder.h`：左右双轮霍尔编码器计数、速度和距离估计接口（按 `HALL_ENCODER_ID` 选轮），详细说明见 `docs/interfaces/bsp_hall_encoder.md`。
- `yahboom_track.h`：Yahboom 8 路循线模块 I2C 驱动（I2C0，地址 `0x12`），详细说明见 `docs/interfaces/bsp_yahboom_track.md`。
- `step_motor.h`：摆杆步进电机开环控制（STEP=PA29/TIMG6，DIR=PB14，EN=PB11），含速度限幅与开环位置限位，详细说明见 `docs/interfaces/bsp_step_motor.md`。
- `debug_uart.h`：调试串口（Debug_Ex/UART1，115200）非阻塞收发，供文本遥测和调试使用；
  树莓派视觉数据不走此接口。
- `rpi_uart.h`：树莓派滚球视觉专线（Rpi_UART/UART2，PA24 RX，115200）11 字节协议解析、
  诊断统计、失效判定、旧版倾角动力学前推接口，以及不注入倾角加速度的可信观测接口；
  H3 使用后者，与 Debug_Ex 完全独立。

### 灰度接口的位序与极性

驱动的两个读接口保留了不同语义，调用方不得只根据“灰度掩码”名称假设位序和极性：

| 接口 | 位序 | 置 `1` 的含义 |
|---|---|---|
| `YahboomTrack_ReadRaw()` | `bit7=X1` … `bit0=X8` | 白底，设备端 LED 灭 |
| `YahboomTrack_ReadDetectedMask()` | `bit0=X1` … `bit7=X8` | 检测到黑线 |

循迹任务只调用 `YahboomTrack_ReadDetectedMask()`，把“黑线置 1”的归一化掩码传入
`LineFollow_UpdateDetectedMask()`；middleware 侧不感知设备协议。
