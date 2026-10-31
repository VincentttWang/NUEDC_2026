# app 层设计（裸机菜单调度框架）

## 目标形态

上电初始化后进入**任务选择菜单**；短按选择一个任务执行；任务完成或用户中止后退回菜单。
裸机（nortos）下用**协作式时间触发超循环**实现：ISR 只做时基/采样，主循环按周期分派
run-to-completion 任务。

## 顶层状态机

```
                ┌───────────── 致命故障 ──────────► SystemFault_Halt (终态)
                │
 INIT ──ok──► MENU ──短按ENTER──► RUN ──DONE/短按BACK──► MENU
                ▲                 │
                │                 └──on_tick 返回 FAULT──► FAULT ──短按ENTER──► MENU
                └──────────────────────────────────────────┘
```

- **INIT**：`App_Init()` 期间；完成后置 MENU。
- **MENU**：委派 `app_menu` 的嵌套菜单导航（见下）；选中任务时返回给 app_mode 进入 RUN。
- **RUN**：调度器每 10ms（100Hz）调 `App_ControlTick` 委派 `current_task->on_tick`；短按 BACK 中止。
- **FAULT**：可恢复故障；显示错误页，短按 ENTER 复位回 MENU。

## 嵌套菜单（app_menu）

菜单是一棵 `MENU_NODE` 树，每个 `MENU_ITEM` 或指向子菜单、或指向任务；导航用一个
`node_stack`/`sel_stack` 深度栈。`app_menu` 不反向依赖 `app_mode`——选中任务时由
`Menu_Tick()` 返回该任务描述符，`app_mode` 再调 `App_EnterRun`。当前菜单树（`app_menu_def.c`）：

```
Main Menu
├── H2 Lap              (task)   要求 2：同一车辆装置高速整圈
├── H3 Challenge        (task)   要求 3：O→+5 cm→−5 cm，5 s 内稳定
├── H4 Loaded A-B       (task)   要求 4：载球 A→B，记录通过 B 的时间
├── H5 Loaded Lap O     (task)   要求 5：目标 O 点载球整圈循迹
├── H6 Loaded Any       (task)   要求 6：任意目标载球整圈，底盘管线与 H5 一致
└── Device Check        (submenu)
    ├── Gyro JY61P      (task)   JY61P 陀螺/姿态/温度 + 诊断计数
    ├── Gray I2C        (task)   Yahboom I2C 版(0x12), 8 路数字量 + 成功/失败与 I2C 诊断计数
    ├── TB6612          (task)   短按单次低速脉冲(20%/300ms) + 编码器响应, 抬轮提示
    ├── Step Motor      (task)   摆杆步进七模式(JOG/LEVEL/RUN/TURN/HAND/SWEEP/SPAN);
    │                            LEVEL 短按 ENTER 捕获本次上电的 H3 水平偏置，长按进入 RUN
    ├── Encoder         (task)   双轮 count/speed/dir
    ├── Speed PID       (task)   双轮速度闭环, 按键给目标 + 目标/实测对比, 整定用, 抬轮提示
    └── Duty Sweep      (task)   开环占空比阶梯, 查各占空比下编码器读速(死区/噪声诊断), 抬轮提示
```

H2/H5/H6 的整圈任务内部继续按轮轴平均里程运行 `S1(A→B) → S2(B→C) → S3(C→D) →
S4(D→A)` 单向状态机；测量点统一为灰度阵列，起跑时阵列对准 A 并清零编码器，四段标称
边界为 `1.500/3.071/4.571/6.142 m`。`[TRK] seg` 输出当前段，`vs` 输出半差速指令。直线段无曲率前馈，
右半圆段叠加差速轮速前馈（S2/S4 比例分别为 `1.10/1.00`），灰度与陀螺闭环在四段内持续工作。
H2 在电机启动前锁存 `yaw_start`：S1/S3 分别以 `yaw_start`/`yaw_start+180°` 运行航向角
外环与 `gz` 角速度内环；S2/S4 恢复原有曲率角速度前馈闭环，不叠加航向角修正。弯道期间
仍按编码器圆弧进度生成动态参考角供遥测和下一直道使用。H2 起步阶段暂时使用
`min(0.45·v_cmd, 0.07 m/s)` 差速权限，实测平均轮速连续 5 拍达到 `0.09 m/s` 或满 `1.0 s`
后，经 `300 ms` 回到原 H2 循迹权限；减速停车阶段不使用该放宽权限。
H5/H6 直接引用唯一的 `LT_PROFILE_LOADED_LAP`，使用相同的分段闭环和载球权限：航向角速度限幅 `±12 deg/s`，起步差速为
`min(0.45·v_cmd, 0.04 m/s)`，轮速连续 5 拍达到 `0.06 m/s` 或满 `1.2 s` 后经 `300 ms`
回到正常权限。H2/H5/H6 使用相同车辆与机械装置，H5/H6 的低速 S2 曲率前馈使用几何理论
比例 `1.00`，不沿用 H2 在 `0.45 m/s` 下用于补偿侧滑的 `1.10`。H5/H6 识别 A 横线时只锁存
整圈时间并进入 `RUNOUT`；S4 前馈平滑退出，控制切到
`yaw_start` 对应的新 S1 直道，并按限 jerk 速度曲线停稳。横线晚于编码器到达时仍可把
`ODOM` 计时校正为 `LINE`；缓停超过 `0.25 m/2.5 s` 才强制制动。
H5/H6 的减速预警距离由公共参数 `LT_LOADED_LAP_DECEL_WARNING_DISTANCE_M` 配置，当前为
A 点前 `0.25 m`；横线识别窗口仍为 `0.40 m`，调节减速距离不会改变横线检测范围。
H5/H6 进入 `ODOM` 的编码器阈值为
`LT_LAP_STOP_DISTANCE_M + LT_LOADED_LAP_ODOM_ARRIVAL_OFFSET_M`；偏置默认为 `0.000 m`，正值
延后、负值提前。H2 仍使用独立的 `LT_LAP_ODOM_FALLBACK_DISTANCE_M`，不受该偏置影响。
H2 在 A 点前 `0.4 m` 开始平滑减速，首帧检测到横线即制动；漏检横线则在标称圈长后
`0.2 m` 强制停车。`[TRK] run/fs` 分别标识试跑编号和 `LINE/ODOM/END` 停车来源。
H4 使用独立的 `ACCEL→CRUISE→DECEL→PASS_B→STOP` 距离状态机，灰度只做观测；
运动前先锁存启动航向，航向角外环生成 `omega_ref`，陀螺角速度内环将残差映射为反对称
轮速差，左右轮平均目标仍由纵向 S 曲线决定。起步阶段放宽差速权限，平均实测轮速连续
5 拍超过 `0.06 m/s` 或达到 `1.2 s` 后，经 `300 ms` 平滑收紧；进入减速阶段后再经
`300 ms` 平滑放宽到同一防打滑权限，并随纵向速度一起归零。IMU 数据超过 `60 ms`
未更新时差速平滑归零、退化为双轮等速，不误刹车；首次有效航向锁存前保持制动。
`s=1.500 m` 锁存 B 点时间，随后在约 `1.600 m` 低速停车。
H2/H5/H6 的灰度全白或灰度数据超时不再触发中途退出：控制自动令灰度角速度残差归零，
继续使用编码器分段、赛道几何/直道航向参考与陀螺角速度内环，且不改变原速度曲线；
`[TRK] gm=1` 表示当前拍正使用该模型/陀螺路径。后续任一有效灰度样本会自动恢复受限灰度
残差；若直到终点仍未恢复，则由既有编码器里程兜底完成停车或通过 A 后的 RUNOUT。

导航按键（仍仅短按）：`UP/DOWN` 移动、`ENTER` 进入子菜单/任务、`BACK` 返回上级。

所有状态转移集中在 `app_mode.c` 的入口函数（`App_EnterRun/App_ExitRun/App_RaiseFault`）；
进 RUN 必 `Chassis_ResetDistance`+`on_enter`，出 RUN 必 `Chassis_Brake`。任务只通过 `on_tick`
返回值表达迁移意图，不直接改模式/调度。

## 两级故障模型

- **致命/不可恢复**：`SystemFault_Handler → Halt`（刹车+显示+`__disable_irq`+死循环），
  用于 init 失败等。绕过状态机。
- **可恢复**：app 的 FAULT 态，`App_RaiseFault` 记录+刹车+进 FAULT，调度器继续跑，短按复位。

## 调度器

`app_scheduler`：任务表 `{cb, period_ms, last_ms}`，`Scheduler_Run()` 按 `BSP_Time_GetMs()`
分派到期任务。注册：`App_ControlTick`(10ms/100Hz)、`App_UiTick`(50ms)。

`SysTick_Handler`(1ms，app 自持)：`BSP_Time_TickInc()` + `Key_Scan()`；`tick_active` 门控
避免初始化完成前误触发。

## 中断归属

| ISR | 归属 | 职责 |
|---|---|---|
| `SysTick_Handler` (1ms) | app/app_scheduler | 时基递增 + 按键扫描 |
| `GROUP1_IRQHandler` | bsp/hall_encoder | 编码器边沿 |
| `TIMER_0_INST_IRQHandler` (20ms) | bsp/hall_encoder | 编码器测速（不跟随控制拍）|
| `I2C0_IRQHandler` | bsp/imu | JY61P I2C 状态机 |

## 按键交互（仅短按）

| 模式 | UP | DOWN | ENTER | BACK |
|---|---|---|---|---|
| MENU | 上移选择 | 下移选择 | 进入任务 | — |
| RUN | 任务定义 | 任务定义 | 任务定义 | 中止回菜单 |
| FAULT | — | — | 复位回菜单 | — |

`KEY_ID`：`KEY_ID_UP / KEY_ID_DOWN / KEY_ID_ENTER / KEY_ID_BACK`。用 `Key_GetEvent()==KEY_EVENT_SHORT_PRESS` 消费。

---

## 任务开发规范

### 三步接入
1. 在任务源文件（如 `app_checks.c`）写私有状态 + 三个钩子 `TaskXxx_Enter/Tick/Exit`，
   导出一个 `const APP_TASK_DESC`。
2. 在 `app_menu_def.c` 的菜单树挂一项 `{ .name="名字", .kind=MENU_ENTRY_TASK, .u.task=&描述符 }`（`on_exit` 可 NULL）。
3. 编译。菜单出现该项，其它文件不动。

### 钩子契约

| 钩子 | 必须 | 禁止 |
|---|---|---|
| `on_enter` | 复位本任务全部 `static` 私有状态（相位/计数/滤波/`PID_Reset`/基准） | 驱动执行器、写模式 |
| `on_tick(dt)` | 走取值→计算→执行→监督，返回 RUNNING/DONE/FAULT | 阻塞/延时、直接读慢速外设、写模式/调度 |
| `on_exit` | 仅任务专属收尾（可 NULL） | 把安全停机只放这里（框架已 Brake） |

返回值：`RUNNING` 继续；`DONE` 框架刹停回菜单；`FAULT` 进 FAULT 态（致命才用 `SystemFault_Handler`）。

### 硬性规范
1. **非阻塞**：多步等待拆成相位，用 `BSP_Time_GetMs()` 差值推进，不忙等。
2. **dt 权威**：只用传入 `dt`（=控制周期 0.02f），传给 `PID_Update` 等。
3. **输入取快照/getter**：经 `Chassis_Get*` / IMU getter / 采样快照，不在任务里做阻塞读。
4. **执行器单写者**：仅 `on_tick` 内、每拍至多一次 `Chassis_SetDuty/Brake`。
5. **不碰框架内部**：只靠返回值迁移，不写 `app_mode`、不调调度/迁移函数。
6. **进出对称**：`on_enter` Reset 的每项保证可重复启动（任务从菜单可反复进入）。
7. **每相位带超时**兜底 → 返回 FAULT。
8. **风格**：`static` 私有状态不加 `s_` 前缀；钩子命名 `TaskXxx_*`；魔法数提常量。
9. **UI**：RUN 运行页由任务 `on_tick` 低频自渲染；MENU/FAULT 页归框架。

### 参考实现
Device Check 共 8 项（均在 `app_checks.c`），演示了 enter 复位、tick 非阻塞采样、
按变化节流刷屏、仅靠 BACK 短按退出；灰度检查页还示范了 on_enter/on_exit
挂起/恢复 JY61P，以分时共用 I2C0。

### 公共支持
- `app_fmt`：`AppFmt_I32/AppFmt_Fixed` 定点数字→字符串，供自检显示，不引浮点 printf。

### 后续扩展点
- **传感器采样进 ISR**：灰度/编码器等便宜同步量可在 SysTick/专用定时器采进 volatile 快照，
  IMU 用「定时器 kick + I2C ISR 完成」；控制任务只消费快照（降低控制环输入抖动）。
- **命令层**：调试 UART 的 RX/TX 中断 + 环形缓冲 + 命令解析，按需在 app 层新增
  `UARTx_IRQHandler`。
