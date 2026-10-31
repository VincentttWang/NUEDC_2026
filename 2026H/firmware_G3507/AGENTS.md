# AGENTS.md — MSPM0G3507 车载固件 开发工作流

## 项目概述

- **平台**: MSPM0G3507 (LQFP-64) 车载固件
- **IDE**: Keil MDK 5 + MDK 6/Keil Studio (ARMCLANG V6.22) / CCS (TICLANG)
- **SDK**: `third_party/mspm0-sdk` submodule，固定 mspm0_sdk_2_10_00_04
- **当前状态**: 原 2025E 二维云台/瞄准子系统已移除；`app` 已重建为菜单驱动的裸机协作式
  调度框架，现有四个循迹入口、九种直行实验、两种转向实验和 12 项设备检查。当前重点为
  底盘闭环整定与上板验证。

> **注意**: 外部工具路径（Keil、SysConfig）需向用户确认实际安装位置；SDK 使用仓库内
> submodule，不应恢复成本机绝对路径。

> **摆杆步进**: 驱动是**位置式单通道**的（16 个接口，无通道参数）——
> `StepMotor_MoveToCount()` 是唯一的运动入口，没有速度接口（速度指令绕得过位置限幅，
> 位置指令绕不过），转速由 `StepMotor_SetSpeedLimit()` 约束。上电即失能并把编码器清零
> （当前位置 = 坐标系原点），延时后自动抬升到 `STARTUP_LIFT_TARGET_COUNTS` 并保持。
> 行程边界写死在 `step_motor.h` 宏里，**限位全程有效、无运行期开关**——要越过软限位量
> 机械行程就失能手推（`Device Check -> Step Motor` 的 `HAND`/`SPAN` 模式）。
> 该页共六个模式、进页落在 `JOG`（点动，UP/DOWN 单击走一步、长按调步长，
> 四档 1/5/10/20 计数 = 0.18/0.9/1.8/3.6°）。
> **步进精细度与定位精细度是两件独立的事**：前者 = 一个微步走多远
> （`STEP_ANGLE_DEG / MICROSTEP`，当前 ×32 → 0.05625° = 0.3125 计数），**只由驱动器拨码
> 决定、固件只能跟随**，管运动平顺度；后者 = 停下时离目标多远（到位容差 **0**，
> 即只接受精确相等），管落点准不准。
> **闭环的最小可指令位移是 1 个编码器计数 = 0.18°，由反馈分辨率决定，不由细分数决定**
> ——误差是整数计数，比一个计数小的位移表达不出来；`JOG` 最细一档就是这 1 个计数
> （公式 `2×容差+1`，容差 0 时退化为 1；`2×` 是因为容差非 0 时残余误差与行进方向同号、
> 换向会抵掉一份，否则换向第一下必然死键）。要走进 1 个计数以内只能开环按微步发脉冲，
> 权衡见标定手册「还想更细：开环微步点动」——结论是不值得做。
> ⚠ 改细分必须**先拨拨码再改宏**，两边不一致不报错，只让转速整体差固定倍数。
> 到位判定**带回差**：进门看容差（0）、出门看 `SERVO_RESUME_COUNTS`（6），使"停得准"与
> "漂多少才重新动"解耦（后者挡机械下沉，容差收紧时刻意不跟着收——**这个解耦正是容差敢
> 取 0 的前提**）；`IsAtTarget()` 直接返回该状态，因此容差 0 下它要求误差恰好为 0，
> `SWEEP` 不翻头 / `TURN` 不报 DONE 时先怀疑这里。完整分析见标定手册「精细度：两件独立的事」。
> `StepMotor_Tick()` 由调度器每 10ms 跑位置伺服 + 抬升状态机 + 越界纠正，
> **不调它电机不会动**。详见
> [`docs/step-motor-calibration.md`](docs/step-motor-calibration.md) #11 与
> [`docs/interfaces/bsp_step_motor.md`](docs/interfaces/bsp_step_motor.md)。

## 硬件外设映射

| 子系统 | 器件 | 接口 | 驱动 |
|--------|------|------|------|
| 底盘轮 | 直流电机 ×2 + 霍尔编码器 | PWM/GPIO | `bsp/motor` (TB6612) |
| 姿态 | JY61P (WIT 协议) | I2C0（与 Yahboom 循线共总线） | `bsp/imu` (wit_sdk, 中断驱动) |
| 循迹 | Yahboom 8 路循线 | I2C0（地址 `0x12`，与 JY61P 共总线） | `bsp/yahboom_track` |
| 摆杆执行器 | 步进电机 + QEI | STEP PA29(TIMG6) / DIR PB14 / EN PB11 / QEI TIMG8 | `bsp/step_motor` |
| 显示/输入 | OLED / 按键 | I2C1/GPIO | `bsp/oled`, `bsp/key` |
| 调试遥测 | Debug_Ex | UART1 115200（PA8 TX / PA9 RX） | `bsp/debug_uart` |
| 视觉链路 | 树莓派（球位置检测） | Rpi_UART 115200（PB15 TX / PA24 RX） | 驱动待实现 |

## 目录结构

```
MSPM0_Car/
├── app/                  # 初始化、调度、状态机、菜单与测试任务
├── bsp/                  # 板级驱动 (motor, step_motor, imu, yahboom_track, uart, oled, key, time)
├── board/
│   ├── startup/          # 启动文件, 链接脚本
│   └── sys_config/       # SysConfig 源文件及生成代码
├── core/                 # 纯计算算法 (pid, filter, kinematics, common)
├── middleware/           # 中间件 (chassis, line_follow, ui, fault)
├── k230/                 # K230 板端程序 (2025E 历史资料, 2026H 视觉改用树莓派)
├── project/{keil,ccs}/     # Keil（MDK 5 + MDK 6/CMSIS Solution）与 CCS 工程
├── third_party/mspm0-sdk/ # TI 官方 SDK submodule（2.10.00.04）
├── docs/                 # 架构/接口/构建文档
└── tools/                # checks/jlink/k230/visualizers 分类开发工具
```

## 分层与依赖边界

```text
app ─► middleware ─► {core (纯计算), bsp}
```

- `bsp` 只含标准头、厂商/板级头和 `bsp` 内部头。
- `middleware` 可含 `middleware` / `core` / `bsp` 头。
- `core` 只含标准头和 `core` 内部头（不读硬件）。
- `app` 可含任意下层公开头。
- 中断按“谁拥有该外设，谁定义 ISR”划分，`middleware` 不做中断转发；跨子系统分发/调度类中断放 `app`。

## 问题排查思路

1. **查官方例程**: `<sdk>/examples/nortos/LP_MSPM0G3507/`。
2. **读底层实现**: 深入 DriverLib 头 (如 `dl_uart.h`, `dl_i2c.h`) 的源码注释。
3. **搜 TI E2E 论坛**: 芯片型号 + 特性词。
4. **对比验证**: 找已知可工作参考逐行对比。
5. **先简后优**: 先验证连通性再切目标方案。
6. **加诊断手段**: 运行时计数器 + OLED/串口观察。

## 开发操作流程

### 修改引脚/外设配置
1. 编辑 `board/sys_config/G3507.syscfg`
2. 运行 SysConfig CLI 重生成 `ti_msp_dl_config.c/h`
3. 检查生成代码（勿手改生成文件）

> I2C0 实例在 syscfg 中名为 `Gray_JY61P_I2C`，由 JY61P（`0x50`）与 Yahboom 循线（`0x12`）共用；
> 两者须由上层分时，见 `app/app_line_task.c` 的 `LineSensor_Tick()`。

### 编译
```
<UV4.exe> -b "NUEDC2025_MSPM0G3507.uvprojx" -o "build_log.txt"
```
> 运行前 `cd` 到 `project/keil/`，UV4.exe 路径需向用户确认。

MDK 6 使用 `project/keil/NUEDC2025_MSPM0G3507.csolution.yml`，通过 Keil Studio Pack
或 `cbuild` 构建。新增/删除编译输入后运行 `python tools/checks/check_keil_project_sync.py`。

### Keil 工程维护
- `.uvprojx` 是 XML，可手编增删 `<File>` 条目；漏加会在链接阶段报 `L6218E`。
- **`.uvprojx` 必须保存为不带 BOM 的 UTF-8**。带 BOM 时 uVision 无法解析，`UV4.exe` 直接以
  exit code 15（error reading import XML file）退出且不产生日志，报错完全指不到 BOM 上。
  PowerShell 的 `Set-Content`/`Out-File` 在部分场景默认写 BOM，改这个文件时须留意。
- 已删文件引用须同步维护 MDK 5、MDK 6 与 CCS 三套工程。
- MDK 5 与 MDK 6 的源码/库输入必须通过 `tools/checks/check_keil_project_sync.py` 核对一致。
- MDK 5 与 MDK 6 的优化级别必须同步：`.uvprojx` 的 `<Optim>1</Optim>` 对应 `csolution.yml` 的
  `optimize: none`，两者都是 `-O0`。本工程含步进脉冲时序与控制周期预算，两侧不一致会导致
  一边上板标定过的时序在另一边不成立。
- `csolution.yml` 用了 `target-set` 节点，**要求 CMSIS-Toolbox ≥ 2.12.0**。MDK 5 自带的
  `<KEIL>\ARM\cmsis-toolbox` 是 2.6.0，会报 `schema check failed, verify syntax` 而非版本错误；
  命令行构建须使用 `vcpkg-configuration.json` 激活的 toolbox。
- CCS projectspec 已改用仓库内 SDK 并修正已知元数据，但重新构建验证前不要宣称 CCS 可用。

### 提交
- 格式: `type: 中文描述`（`feat` | `fix` | `docs` | `refactor`）
