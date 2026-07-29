# 构建说明

## 工具链

本工程维护两套构建链：

- `CCS + ticlang`
- `Keil MDK 5 + ArmClang`

两套工程共享 `app`、`core`、`middleware`、`bsp`、`board/sys_config` 中的源码和配置，但各自维护启动文件、链接配置和输出目录。

## 共享源码

主要源码目录如下：

- `app/`
- `core/`
- `middleware/`
- `bsp/`
- `board/sys_config/`

构建时必须同时包含这些 include path。按照当前重写流程，新增源码先完成分层框架和接口收敛；整体框架确认后，再统一维护 `project/ccs/NUEDC2025_MSPM0G3507_ticlang.projectspec` 和 `project/keil/NUEDC2025_MSPM0G3507.uvprojx`。

当前两套工程文件已同步到分层源码树。

## CCS 构建

入口文件：

- `project/ccs/NUEDC2025_MSPM0G3507_ticlang.projectspec`

关键约定：

- 使用 `projectspec` 导入，不在仓库根目录维护 `.project`、`.cproject`、`.ccsproject`
- 推荐使用仓库外 CCS workspace
- 若必须在仓库内生成导入工程，应放入专用本地产物目录并保持忽略
- SysConfig 输入和生成文件位于 `board/sys_config/`
- CCS 构建输出目录视为可再生本地产物

本地已验证的直接交叉编译命令使用：

- 编译器：`C:\ti\ti_cgt_arm_llvm_4.0.2.LTS\bin\tiarmclang.exe`
- SDK：`C:\ti\mspm0_sdk_2_10_00_04`
- 产物：`build/ccs/NUEDC2025_MSPM0G3507.out`

## Keil 构建

入口文件：

- `project/keil/NUEDC2025_MSPM0G3507.uvprojx`

关键约定：

- Keil 输出目录为 `project/keil/Objects/`
- Keil 使用 SDK 中的 uVision 启动文件
- Scatter 文件位于 `board/startup/mspm0g3507.sct`
- `uvoptx`、`uvguix.*` 属于本地会话文件，不作为稳定源码依赖

本地已验证的 Keil 命令：

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -r 'NUEDC2025_MSPM0G3507.uvprojx' -o 'keil_build.log'
```

运行目录为 `project/keil/`，构建结果为 `Objects/NUEDC2025_MSPM0G3507.axf` 和对应 hex，最近一次 rebuild 为 `0 Error(s), 0 Warning(s)`。

## SysConfig 和生成文件

以下文件由工具或配置生成，业务逻辑不要写入其中：

- `board/sys_config/empty.syscfg`
- `board/sys_config/ti_msp_dl_config.c`
- `board/sys_config/ti_msp_dl_config.h`

如重生成后产生差异，优先判断是否来自工具版本或配置变更。

## 运行验证清单

编译通过后仍需上板验证：

- 上电后进入启动页
- E 题前三项任务菜单可进入
- E1 可选择 1 到 5 圈
- `Device check` 五个测试页可正常切换和刷新
- `UART0_IRQHandler` 可处理 IMU 调试串口数据
- `UART2_IRQHandler` 可更新视觉定位数据
- `SysTick_Handler` 可按周期扫描按键

## 本地产物边界

以下内容默认视为本地产物或 IDE 会话文件，不作为稳定源码维护：

- `Debug/`
- `project/keil/Objects/`
- `project/keil/*.map`
- `project/keil/*.uvoptx`
- `project/keil/*.uvguix.*`
- `.settings/`
- `.theia/`
- `.clangd/`
