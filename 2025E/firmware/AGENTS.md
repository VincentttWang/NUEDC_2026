# AGENTS.md — NUEDC 2025E 简易自行瞄准装置 开发工作流

## 项目概述

- **赛题**: 2025 电赛 E 题「简易自行瞄准装置」(自动寻迹小车 + 二维激光瞄准云台)
- **MCU**: MSPM0G3507 (LQFP-64)
- **工程路径**: `2025E/firmware/`
- **IDE**: Keil MDK (ARMCLANG V6.22) / CCS (TICLANG)
- **SDK**: mspm0_sdk_2_10_00_04

> **注意**: 外部工具路径 (Keil、SysConfig、SDK) 需向用户确认实际安装位置。

## 硬件外设映射

| 子系统 | 器件 | 接口 | 驱动 |
|--------|------|------|------|
| 底盘轮 | 直流电机 ×2 + 霍尔编码器 | PWM/GPIO | `bsp/motor` (TB6612) |
| 二维云台 | F32C 无刷 ×2 (yaw=addr1, pitch=addr2) | UART3 | `bsp/bldc` |
| 姿态 | JY61P (WIT 协议) | I2C0 | `bsp/imu` (wit_sdk) |
| 视觉 | K230 (CanMV) | UART2 | `bsp/canmv` |
| 循迹 | 8 路灰度 | GPIO | `bsp/grayscale_sensor` |
| 显示/输入 | OLED / 按键 | I2C/GPIO | `bsp/oled`, `bsp/key` |

## 目录结构

```
2025E/firmware/
├── app/                  # 应用层 (main, 启动器, E 题任务, 设备检查)
├── bsp/                  # 板级驱动 (bldc, imu, motor, oled, key, canmv, grayscale, time)
├── board/
│   ├── startup/          # 启动文件, 链接脚本
│   └── sys_config/       # SysConfig 源文件及生成代码
├── core/                 # 控制算法 (PID, 运动学, 循线, 云台视觉)
├── middleware/           # 中间件 (底盘, 云台, UI, 故障)
├── project/{keil,ccs}/   # IDE 工程
├── docs/                 # 模块接口文档
└── tools/                # 调试/烧录脚本
```

## 云台说明 (重点)

云台两轴均为 F32C 无刷 (UART3 速度闭环)。`middleware/gimbal` 对外保持 `deg/s` 速度 + 开环角度估计接口，底层用 `bsp/bldc`：

- `Gimbal_SetSpeed(deg/s)` 把角速度换算成 RPM 下发 (`GIMBAL_GEAR_RATIO` 需实机标定)。
- `Gimbal_GetAngle()` 为开环积分角 (供 E3 扫描判角)；后续可改读 `BLDC_MotorX.multi_angle` 反馈作闭环。
- 上层 `core/gimbal_tracking` 与 `app_e_task` 不感知底层电机类型。
- **yaw 惰性使能**: 开机不使能 yaw, 首次非零 yaw 速度指令时才使能 (避免速度环锁 0 时 yaw 空转/抖动)。
- **pitch 非阻塞抬升**: `main` 在 `Gimbal_Init()` 后调用 `Gimbal_StartupElevatePitch()` 发起位置模式抬升(默认 150°)后立即返回, 不阻塞菜单; 进入 E2/E3 前由 `Gimbal_EnsurePitchReady()` 阻塞确认到位并切回速度模式 (E1 循迹不用激光, 不受影响)。E2/E3 瞄准期间 pitch 由 `Gimbal_SetSpeed` 速度驱动。

## 问题排查思路

1. **查官方例程**: `<sdk>/examples/nortos/LP_MSPM0G3507/`。
2. **读底层实现**: 深入 DriverLib 头 (如 `dl_uart.h`, `dl_i2c.h`) 的源码注释。
3. **搜 TI E2E 论坛**: 芯片型号 + 特性词。
4. **对比验证**: 找已知可工作参考逐行对比。
5. **先简后优**: 先验证连通性再切目标方案。
6. **加诊断手段**: 运行时计数器 + OLED/串口观察 (设备检查页已内建)。

## 开发操作流程

### 修改引脚/外设配置
1. 编辑 `board/sys_config/G3507.syscfg`
2. 运行 SysConfig CLI 重生成 `ti_msp_dl_config.c/h`
3. 检查生成代码

> I2C0 实例在 syscfg 中名为 `MPU6050_JY61P_Tracking`，现由 JY61P 使用 (历史命名，未重命名以免牵连重生成)。SR04 的 GPIO 定义为历史残留，未使用。

### 编译
```
<UV4.exe> -b "NUEDC2025_MSPM0G3507.uvprojx" -o "build_log.txt"
```
> 运行前 `cd` 到 `project/keil/`，UV4.exe 路径需向用户确认。

### Keil 工程维护
- `.uvprojx` 是 XML，可手编增删 `<File>` 条目；漏加会在链接阶段报 `L6218E`。
- 已删文件引用须同步移除。

### 提交
- 格式: `type: 中文描述`（`feat` | `fix` | `docs` | `refactor`）
