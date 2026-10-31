# MSPM0G3507 车载固件

基于 `TI MSPM0G3507` 的分层车载固件工程。

> **当前状态**：面向 2026 电赛 H 题（车载平衡滚球）。原 2025E 二维云台/瞄准子系统，
> 以及直行测试、转向测试、航向辅助循迹与 K230 视觉循迹等实验任务已按赛题需求整体移除。
> `app` 为菜单驱动的裸机协作式调度框架，主菜单按赛题要求 2～6 拆为五个任务入口，另有
> `Device Check` 子菜单；要求 3 已接入正式 O→+5 cm→−5 cm、5 s 内稳定业务链路；
> 要求 5/6 仍只运行底盘部分。
>
> **摆杆步进电机**：细分数、编码器每转计数与方向、EN/DIR 极性均已上板标定完成；最大步进
> 频率、摆杆减速比与软限位待机械装配后测定，流程见
> [`docs/step-motor-calibration.md`](docs/step-motor-calibration.md)。
> 底盘速度闭环增益仍为待整车标定的初始值。

## 硬件构成

| 子系统 | 器件 | 接口 | 驱动 |
|--------|------|------|------|
| 主控 | MSPM0G3507 (LQFP-64) | — | — |
| 底盘轮 | 直流电机 ×2 (TB6612FNG) + 霍尔编码器 | PWM / GPIO | `bsp/motor` |
| 姿态 | JY61P 六轴 (WIT 协议) | I2C0（与 Yahboom 循线共总线） | `bsp/imu` (wit_sdk) |
| 循迹 | Yahboom 8 路循线模块 | I2C0（地址 `0x12`，与 JY61P 共总线） | `bsp/yahboom_track` |
| 摆杆执行器 | 步进电机 + 驱动器 | STEP PA29（TIMG6）/ DIR PB14 / EN PB11 | `bsp/step_motor` |
| 人机 | OLED + 独立按键 | I2C1 / GPIO | `bsp/oled`, `bsp/key` |
| 调试遥测 | Debug_Ex | UART1 115200（PA8 TX / PA9 RX） | `bsp/debug_uart` |
| 视觉链路 | 树莓派（球位置检测） | Rpi_UART/UART2 115200（Pi TX → PA24 RX） | `bsp/rpi_uart` |

## 分层架构

```text
app ─► middleware ─► bsp
          │
          └────────► core（纯计算）
```

- `app`：初始化、协作式调度器、状态机、菜单树与具体测试任务。
- `core`：PID、滤波、运动学与纯角速度积分等纯计算能力。
- `middleware`：组合 core 与 BSP 的系统能力（`ball_balance` / `ball_scurve` / `chassis` / `line_follow` / `ui` / `fault`）。
- `bsp`：直接面向板级外设的驱动（见上表 + `time` / `common`）。
- `board`：SysConfig 源文件与生成代码、启动/链接资源。
- `third_party/mspm0-sdk`：TI 官方 MSPM0 SDK 2.10.00.04 submodule，供各工程使用相对路径。

依赖单向：`core` 不读硬件、不调用 middleware；`app` 只编排、不重复实现算法。

## 快速开始

### 1. 获取源码与 SDK

首次克隆建议同时初始化子模块：

```powershell
git clone --recurse-submodules <仓库地址> MSPM0_Car
cd MSPM0_Car
```

若已经克隆过仓库，执行：

```powershell
git submodule update --init --depth 1 third_party/mspm0-sdk
```

SDK 固定为 `mspm0_sdk_2_10_00_04`，位于 `third_party/mspm0-sdk`。Keil 与 CCS 工程均通过
仓库相对路径引用它，不需要另外配置 SDK 绝对路径。

### 2. 选择开发环境

推荐工具组合：

- Keil MDK 5：uVision + Arm Compiler 6.22。
- Keil MDK 6：VS Code + Keil Studio Pack + CMSIS Toolbox + Arm Compiler 6.22。
- CCS：保留 projectspec，但当前版本尚未完成重新导入后的编译回归。
- Python 3：仅运行遥测可视化工具时需要。

## 编译

### Keil MDK 6 / VS Code

0. 建议在工作区 `settings.json` 里配 `cmsis-csolution.exclude`：

   ```json
   "cmsis-csolution.exclude": "{**/third_party/**,**/out/**,**/tmp/**,**/Objects/**}"
   ```

   CMSIS 扩展按 `**/*.csolution.yml` 与 `**/*.uvprojx` 搜索工程，而 `third_party/mspm0-sdk`
   里有 **1309 个** TI 示例 `.uvprojx`。加上这条后候选从 1311 个降到 2 个，扩展的工程发现
   明显更快、更不容易认错活动 Solution。

   > 若 `${command:cmsis-csolution.getCbuildRunFile}` 返回空串（表现为 `.vscode/tasks.json`
   > 里的 pyocd 任务报 `argument --cbuild-run: expected one argument`），说明扩展当时没有
   > 活动 Solution，按第 2 步手动激活即可。`.vscode/tasks.json` 也可以直接写死
   > `out/<solution>+<target>.cbuild-run.yml` 绕开这个变量。
1. 用 VS Code 打开仓库根目录。
2. 打开 `project/keil/NUEDC2025_MSPM0G3507.csolution.yml`，将其设为活动 Solution。
3. 等待 Keil Studio Pack 完成 CMSIS Pack 和工具环境解析。
4. 在 CMSIS 侧栏选择当前 context，执行 **Build Solution** 或 **Rebuild Solution**。

`project/keil/vcpkg-configuration.json` 可让 Arm Tools Environment Manager 获取匹配版本的
AC6、CMSIS Toolbox、CMake 与 Ninja；需要时可右键该文件并执行 **Activate Environment**。
若希望复用本机 Keil 已安装的 AC6，在 VS Code 设置中搜索
`CMSIS Solution: Environment Variables`，向工作区 `settings.json` 加入：

```json
"cmsis-csolution.environmentVariables": {
  "AC6_TOOLCHAIN_6_22_0": "<KEIL>\\ARM\\ARMCLANG\\bin"
}
```

> 上面的 `\\` 在 JSON 里就是**一个**反斜杠，照抄时不要再加一层写成 `\\\\`——那样解析出来是
> `D:\\Keil_v5\\ARM\\ARMCLANG\\bin`，扩展找不到编译器。

本机 Keil 安装在 D 盘时，`<KEIL>` 例如 `D:\Keil_v5`。修改后执行
`Developer: Reload Window`，再重新构建。成功日志应包含：

```text
Using AC6 V6.22.0 compiler
Build summary: 1 succeeded, 0 failed
```

命令行等价构建方式：

```powershell
$env:AC6_TOOLCHAIN_6_22_0 = "<KEIL>\ARM\ARMCLANG\bin"
cbuild project/keil/NUEDC2025_MSPM0G3507.csolution.yml --rebuild --toolchain AC6
```

> **CMSIS-Toolbox 必须 ≥ 2.12.0**：`csolution.yml` 使用了 `target-set` 节点。命令行构建前请先对
> `project/keil/vcpkg-configuration.json` 执行 **Activate Environment**，或显式指定 vcpkg 装好的
> toolbox 路径。**不要**直接使用 MDK 5 自带的 `<KEIL>\ARM\cmsis-toolbox`——它是 2.6.0，会报
> `csolution.yml:9:7 - error csolution: schema check failed, verify syntax`，该信息不会提示真实
> 原因是版本过低。

输出位于 `project/keil/out/`。TI 启动文件使用 legacy armasm 语法，当前会出现一条 `A1950W`
弃用警告，不影响链接产物生成。

MDK 5 与 MDK 6 的优化级别保持一致，均为 **-O0**（`.uvprojx` 的 `<Optim>1</Optim>` 对应
`csolution.yml` 的 `optimize: none`）。本工程含步进脉冲时序与控制周期预算，两侧优化级别若不一致，
在一边上板标定过的时序换到另一边不成立，修改时须同步。

### Keil MDK 5 / uVision

打开：

```text
project/keil/NUEDC2025_MSPM0G3507.uvprojx
```

选择 Arm Compiler 6.22 后执行 Build/Rebuild。也可以在 PowerShell 中运行：

```powershell
Push-Location project/keil
& "<KEIL>\UV4\UV4.exe" -r "NUEDC2025_MSPM0G3507.uvprojx" -o "keil_build.log"
Pop-Location
```

输出位于 `project/keil/Objects/`。当前工程已使用 D 盘 Keil 5.41 / AC6 6.22 验证为
`0 Error(s), 0 Warning(s)`。

更完整的 CCS、SysConfig、命令行构建和 Flash 对齐说明见
[`docs/build-guide.md`](docs/build-guide.md)。

## 烧录与首次运行

连接调试器并确认目标器件为 `MSPM0G3507`：

- MDK 5：在 uVision 中执行 Download，或使用工程已配置的下载按钮。
- MDK 6：在 CMSIS 视图中选择正确调试适配器后执行 Load/Run。

首次上板请先架空驱动轮并准备随时断电：

- `Speed PID`、`Duty Sweep` 与 `TB6612` 会驱动车轮，必须先架空。
- `Step Motor` 会驱动摆杆，且当前软限位取 ±100000° 等效于不限位，摆杆装机后须自行
  注意行程，随时可按 `ENTER` 暂停或 `BACK` 退出。

上电初始化成功后，OLED 显示 `Main Menu`。四个按键均使用短按：

| 按键 | 菜单中 | 任务中 |
|---|---|---|
| `UP` / `DOWN` | 移动选择 | 调整支持该操作的速度或占空比指令 |
| `ENTER` | 进入子菜单或启动任务 | 按任务定义执行，直行/速度测试中通常将指令归零 |
| `BACK` | 返回上级 | 中止任务、主动刹车并返回菜单 |

主菜单功能：

| 入口 | 用途 |
|---|---|
| `H2 Empty Lap` | 要求 2：空载高速整圈 |
| `H3 Challenge` | 要求 3 正式入口：先守住 O，任务内 ENTER 后执行 +5 cm→−5 cm；+5 cm 剖面结束即折返（无 VERIFY），最终 −5 cm 严格停稳并持续保持 |
| `H4 Loaded A-B` | 要求 4：载球 A→B 直线循迹 |
| `H5 Loaded Lap O` | 要求 5：目标 O 点载球整圈，当前只运行底盘 |
| `H6 Loaded Any` | 要求 6：进入后保持制动，任务内按 ENTER 锁存当前视觉球位置为固定目标，同时启动与 H5 完全相同的载球整圈循迹 |
| `Device Check` | JY61P、Yahboom 灰度、TB6612、摆杆步进标定、编码器、速度 PID、占空比扫描、Rpi UART |

`H3 Static PID` 与 `H3 Hold 0cm` 是控制器调试入口，已移入 `Device Check`，避免比赛时误选。

循迹页面按 `X1 → X8` 显示 Yahboom 归一化掩码：`1` 表示该路检测到黑线。
H2/H5/H6 整圈任务按平均轮轴里程切换 `S1～S4`，弯道段使用差速前馈；Debug_Ex 的
`[TRK] seg=S1…S4` 可在上位机中查看实时分段，`run` 区分同一采集文件中的多次试跑，
`fs=LINE/ODOM/END` 标记终点横线、里程兜底或 H4 末端线结束三种停车来源；`gm=1` 表示
当前拍未使用灰度残差、正由赛道几何/航向参考与陀螺维持转向。H2/H5/H6 遇到灰度全白或
灰度数据过期时不再中途退出，也不改变速度曲线；后续重新检测到黑线会自动恢复灰度残差，
若一直未恢复则按编码器分段并由终点里程兜底完成任务。

## 串口遥测与可视化

调试遥测使用 `Debug_Ex/UART1`，参数为 `115200 8N1`。先安装 Python 依赖：

```powershell
python -m pip install pyserial matplotlib
```

列出串口并启动直行测试可视化：

```powershell
python tools/visualizers/straight_test_viz.py --list
python tools/visualizers/straight_test_viz.py --port COM7
```

界面同时显示两轮占空比、速度、距离以及融合 yaw、积分 yaw 和角速度。需要保存采样时可运行：

```powershell
python tools/visualizers/straight_test_viz.py --port COM7 --csv straight.csv --log straight_raw.txt
```

速度 PID 与占空比扫描使用：

```powershell
python tools/visualizers/speed_pid_viz.py --port COM7 --csv spd.csv --log raw.txt
```

H3 静止守球通过 `[BALL]` 以 20 Hz 回传位置、速度、低通加速度、非线性反馈分量、动态
限斜率、摆杆角、控制量和视觉链路状态：

```powershell
python tools/visualizers/ball_balance_viz.py --port COM7 --csv ball.csv --log ball_raw.txt
```

`raw.txt`、`spd.csv`、`straight_raw.txt` 和 `straight.csv` 已默认忽略。蓝牙测试使用
UART0，参数为 `9600 8N1`；它与 UART1 调试遥测是两条不同通道。

## 修改工程

- 修改引脚或外设时，只编辑 `board/sys_config/G3507.syscfg`，随后用 SysConfig CLI 重生成
  `ti_msp_dl_config.c/h`；不要直接修改生成文件。
- 新增或删除源码时，同时维护 MDK 5 `.uvprojx`、MDK 6 `.cproject.yml` 和 CCS projectspec。
- 修改 Keil 工程输入后运行：

```powershell
python tools/checks/check_keil_project_sync.py
python tools/checks/check_docs.py
```

- JY61P 与 Yahboom 循线模块共用 I2C0；循迹任务只在 JY61P 异步事务
  空闲时短暂读取 Yahboom，其他阻塞式检查任务通过 `JY61P_I2C_SetSuspended()` 独占总线。
- 不可同时使用无线调试器的虚拟串口和 Ex Uart。

## 文档索引

- 架构与依赖：[`docs/architecture.md`](docs/architecture.md)
- app 菜单和调度：[`docs/app-design.md`](docs/app-design.md)
- 构建与 SysConfig：[`docs/build-guide.md`](docs/build-guide.md)
- 目录职责：[`docs/project-structure.md`](docs/project-structure.md)
- 模块接口：[`docs/interfaces/`](docs/interfaces/)
  - Yahboom I2C 循线：[`bsp_yahboom_track.md`](docs/interfaces/bsp_yahboom_track.md)
  - JY61P：[`bsp_wit_sdk.md`](docs/interfaces/bsp_wit_sdk.md)
  - 树莓派赛道调参：[`rpi_track_tuning.md`](docs/interfaces/rpi_track_tuning.md)
- H 题控制方案（循迹 + 摆杆滚球的控制律与参数）：[`docs/control-plan.md`](docs/control-plan.md)
- 静止稳球上板与机械标定：[`docs/ball-balance-bringup.md`](docs/ball-balance-bringup.md)
- S 曲线滚球控制、参数与仿真：[`docs/ball-scurve.md`](docs/ball-scurve.md)
- 摆杆步进电机上板标定流程：[`docs/step-motor-calibration.md`](docs/step-motor-calibration.md)
- 当前待办和上板风险：[`docs/todo.md`](docs/todo.md)
- K230 开发与远程部署流程（**2025E 历史资料**，2026H 视觉已改用树莓派）：
  [`docs/k230-development.md`](docs/k230-development.md) /
  [`docs/k230-remote-development.md`](docs/k230-remote-development.md)
- 电脑端工具、K230 探针、J-Link 脚本和遥测可视化索引：[`tools/README.md`](tools/README.md)
