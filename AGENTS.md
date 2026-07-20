# AGENTS.md — 天猛星 v3 开发工作流

## 项目概述

- **MCU**: MSPM0G3507 (LQFP-64)
- **工程路径**: `[Demo] MSPM0_Car_v3/`
- **IDE**: Keil MDK (ARMCLANG V6.22) / CCS (TICLANG)
- **SDK**: mspm0_sdk_2_10_00_04

> **注意**: 外部工具路径 (Keil、SysConfig、SDK) 需向用户确认实际安装位置。

## 目录结构

```
[Demo] MSPM0_Car_v3/
├── app/                  # 应用层 (main, 启动器, 设备检查)
├── bsp/                  # 板级驱动 (IMU, OLED, 电机, 传感器)
├── board/
│   ├── startup/          # 启动文件, 链接脚本
│   └── sys_config/       # SysConfig 源文件及生成代码
├── core/                 # 控制算法 (PID, 运动学, 循线)
├── middleware/            # 中间件 (底盘, 云台, UI, 故障)
├── project/
│   ├── keil/             # Keil 工程文件
│   └── ccs/              # CCS 工程文件
├── hardware/             # 硬件设计文件
├── docs/                 # 模块接口文档
└── tools/                # 调试/烧录脚本
```

## 问题排查思路

遇到驱动或外设问题时，按以下优先级推进：

### 1. 查找官方例程

在 SDK 的 `examples/` 目录中搜索相关外设的官方示例（如 `examples/nortos/LP_MSPM0G3507/`）。官方例程是经过验证的最佳实践，应优先参考其实现模式而非自行造轮子。

### 2. 阅读底层实现

不要只看 API 文档，要深入阅读 DriverLib 头文件（如 `dl_i2c.h`）中的函数源码和注释。关键行为（如 STOP 自动生成、状态位时序）往往只在源码注释中说明。

### 3. 搜索官方论坛

TI E2E 论坛（`e2e.ti.com`）是解决外设疑难问题的核心资源。搜索时使用芯片型号 + 关键特性词（如 "MSPM0G3507 repeated start"），很多外设的边界行为在论坛中有官方工程师的明确解答。

### 4. 对比验证

当某个实现不工作时，找一个已知可工作的参考（官方例程或已验证的实现），逐行对比差异。重点关注：
- 函数调用参数的差异
- 状态检查的时机和顺序
- 传输之间的延时
- 错误处理的逻辑

### 5. 先简后优

遇到复杂问题时，先用最简单的方式验证硬件连通性（如用软件 I2C 验证传感器响应），确认硬件正常后再切换到目标方案（如硬件 I2C）。分步验证比一步到位更高效。

### 6. 添加诊断手段

在驱动中加入运行时计数器（成功/失败/超时等），通过已有的输出通道（OLED、串口）实时观察，辅助定位问题所在阶段。

## 开发操作流程

### 修改引脚或外设配置

1. 编辑 `board/sys_config/G3507.syscfg`
2. 运行 SysConfig CLI 重生成代码
3. 检查生成的 `ti_msp_dl_config.c/h`

SysConfig CLI 命令格式:
```
<sysconfig_cli> --product <sdk_path>/.metadata/product.json \
  --device "MSPM0G3507" --package "LQFP-64(PM)" \
  --script "G3507.syscfg" --output "."
```
> `sdk_path` 和 `sysconfig_cli` 的具体路径需向用户确认。

### 编译

```
<UV4.exe> -b "NUEDC2025_MSPM0G3507.uvprojx" -o "build_log.txt"
```
> UV4.exe 路径需向用户确认，运行前需 `cd` 到 `project/keil/`。

### Keil 工程维护

- `.uvprojx` 是 XML 格式，可手动编辑添加/删除源文件引用
- 新文件需在 `<Group>` 中添加 `<File>` 条目，否则链接阶段报 `L6218E`
- 已删除的文件引用须同步移除

### 提交

- 格式: `type: 中文描述`（无方括号，如 `fix: 修复I2C读取异常`）
- type: `feat` | `fix` | `docs` | `refactor`

## 参考资料

| 文档 | 路径 |
|------|------|
| 引脚分配 | `Hardware/天猛星拓展板_v3/MSPM0G3507_引脚分配.md` |
| 模块资料 | `Hardware/模块参考资料.md` |
| 模块接口 | `docs/interfaces/` |
| 项目架构 | `docs/architecture.md` |
| SDK 示例 | `<sdk_path>/examples/nortos/LP_MSPM0G3507/` |
| TI E2E 论坛 | `https://e2e.ti.com/` |
