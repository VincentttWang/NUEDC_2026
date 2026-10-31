# 构建说明

## 工具链

本工程维护三套工程入口：

- `CCS + ticlang`
- `Keil MDK 5 + ArmClang`（uVision `.uvprojx`）
- `Keil MDK 6 + ArmClang`（Keil Studio/CMSIS Solution）

三套工程共享 `app`、`core`、`middleware`、`bsp`、`board` 中的源码和配置。MDK 5/6 还共享
`board/startup/startup_mspm0g350x_uvision.s` 与 `mspm0g3507.sct`，只分别维护工程描述和输出目录。

## 获取仓库内 SDK

TI MSPM0 SDK 来自官方 GitHub submodule，固定版本为 `mspm0_sdk_2_10_00_04`：

```powershell
git submodule update --init --depth 1 third_party/mspm0-sdk
```

若未初始化 submodule，三套工程都会因缺少 DriverLib/CMSIS 头或预编译库而失败。工程文件禁止
重新写入 `C:\ti\...` 等本机 SDK 绝对路径。

## 共享源码

主要源码目录如下：

- `app/`
- `core/`
- `middleware/`
- `bsp/`
- `board/sys_config/`

构建时必须同时包含这些 include path。新增或删除源码后须维护 CCS、MDK 5、MDK 6 三套入口，
并运行：

```powershell
python tools/checks/check_keil_project_sync.py
```

MDK 5/6 已同步且完成构建验证。CCS projectspec 已修正 `G3507.syscfg`、debug UART include 与
仓库内 SDK 路径，但尚未重新导入 CCS/ticlang 构建，因此仍不能宣称 CCS 已验证可用。

## CCS 构建

入口文件：

- `project/ccs/NUEDC2025_MSPM0G3507_ticlang.projectspec`

关键约定：

- 使用 `projectspec` 导入，不在仓库根目录维护 `.project`、`.cproject`、`.ccsproject`
- 推荐使用仓库外 CCS workspace
- 若必须在仓库内生成导入工程，应放入专用本地产物目录并保持忽略
- SysConfig 输入和生成文件位于 `board/sys_config/`
- CCS 构建输出目录视为可再生本地产物

历史上已验证的直接交叉编译环境使用：

- 编译器：TI ARM Clang（`ti_cgt_arm_llvm` 4.0.2 LTS）的 `bin/tiarmclang.exe`
- SDK：MSPM0 SDK 2.10.00.04
- 产物：`build/ccs/NUEDC2025_MSPM0G3507.out`

该历史结果早于上述 projectspec 漂移；当前版本应先修正元数据，再重新执行 CCS/ticlang 构建。

> 工具链与 SDK 的安装路径因机器而异，本文不写死绝对路径；下文命令中的 `<...>` 均为占位符，
> 请替换为本机实际安装位置。

## Keil MDK 5 构建

入口文件：

- `project/keil/NUEDC2025_MSPM0G3507.uvprojx`

关键约定：

- Keil 输出目录为 `project/keil/Objects/`
- Keil 使用 SDK 中的 uVision 启动文件
- Scatter 文件位于 `board/startup/mspm0g3507.sct`
- `uvoptx`、`uvguix.*` 属于本地会话文件，不作为稳定源码依赖

构建命令（`<KEIL>` = Keil MDK 安装目录）：

```powershell
& '<KEIL>/UV4/UV4.exe' -r 'NUEDC2025_MSPM0G3507.uvprojx' -o 'keil_build.log'
```

运行目录为 `project/keil/`，构建结果为 `Objects/NUEDC2025_MSPM0G3507.axf` 和对应 hex。
2026-07-26 使用 D 盘 Keil 5.41 / AC6 6.22 相对 SDK 路径构建结果为 `0 Error(s), 0 Warning(s)`。

## Keil MDK 6 / Keil Studio 构建

入口文件：

- `project/keil/NUEDC2025_MSPM0G3507.csolution.yml`
- `project/keil/NUEDC2025_MSPM0G3507.cproject.yml`

VS Code 已安装 Keil Studio Pack 时可直接打开 `.csolution.yml`，选择 Debug 或 Release context
后构建。`vcpkg-configuration.json` 固定 AC6 6.22 与 CMSIS Toolbox/CMake/Ninja 工具环境。
若复用 D 盘 Keil 自带编译器，在 VS Code 的 `CMSIS Solution: Environment Variables`
工作区设置中注册：

```json
"cmsis-csolution.environmentVariables": {
  "AC6_TOOLCHAIN_6_22_0": "D:\\Keil_v5\\ARM\\ARMCLANG\\bin"
}
```

命令行构建：

```powershell
cbuild project/keil/NUEDC2025_MSPM0G3507.csolution.yml --rebuild --toolchain AC6
```

输出位于 `project/keil/out/`。2026-07-26 已使用 CMSIS Toolbox 2.14 调用 D 盘 Keil AC6 6.22
验证自动转换所得 context，结果为 `1 succeeded, 0 failed`。启动文件沿用 MDK 5 的 TI legacy
armasm 语法，因此 MDK 6 有一条 `A1950W` 弃用警告；这是兼容性提示，不影响生成 axf。

命令行烧录（调试器已连接）：

```bash
"<KEIL>/UV4/UV4.exe" -f "NUEDC2025_MSPM0G3507.uvprojx" -o "flash_log.txt"
```

成功日志应含 `Programming Done. Verify OK.`。

## 烧录已知问题：镜像须 8 字节对齐

**现象**：`Erase Done.` 之后立即 `Programming Failed! Error: Flash Download failed - "Cortex-M0+"`
（擦除成功、写入失败）。与镜像大小/内容相关，与调试时钟无关（降频无效）。

**根因**：MSPM0 Flash 按 **64 位（8 字节）字 + ECC** 编程，整镜像长度必须是 8 的倍数，
否则最后不足一整字的部分无法写入。分散加载文件的 `ALIGNALL 8` 只对齐各段**起始**、不保证
整镜像**尾长**对齐；TI 官方 `.sct` 亦如此，本质是 Keil 的 MSPM0 烧录算法(FLM)未补齐末字。
社区有同现象反馈（见文末 TI E2E 帖）。

**当前修复**：`app/main.c` 定义 8 字节对齐的尾填充 `g_flash_tail_pad`（`__ARMCC_VERSION`
限定仅 Keil），经 `board/startup/mspm0g3507.sct` 的 `*(.flash_tail_pad, +Last)` 放在
`ER_IROM1` 末尾，强制该段 8 对齐。

**⚠ 局限与复发条件**：该填充位于 `ER_IROM1`（代码+只读段）末尾。当前 `RW-data=0`，故它就是
整镜像尾部、有效。但 Flash 镜像 = `ER_IROM1(含填充) + RW 初值数据`，RW 初值数据排在填充**之后**
且无对齐保护。因此：

- 只改代码/函数/`const`/零值或未初始化全局（`RW-data` 保持 0）→ **永远安全，不会复发**。
- 新增**带非零初值的全局/静态变量**（`RW-data > 0`）→ 风险回归（概率性，约 1/8 的改动会触发）。

**每次编译自查**：看 Keil `Program Size` 行——`RW-data=0` 即稳；若 `RW-data` 非 0，检查
`.map` 的 `Total ROM Size` 是否为 8 的倍数，不是则会烧录失败。

**长期根治选项（暂未采用，记录备选）**：
1. 升级 Keil 的 MSPM0 DFP（器件包）——若新版 FLM 已补齐末字则问题消失。
2. 改用 J-Link 的 SEGGER 内部 flash 下载器（自动补齐末字）。
3. post-build 步骤把最终 `.bin/.hex` 补齐到 8/16 字节（工具链无关，对任意 RW 大小都成立）。

## SysConfig 和生成文件

输入文件是 `board/sys_config/G3507.syscfg`（唯一真值源）；以下文件**由工具生成，禁止手改**
（手改会在下次再生成时丢失，且易与 `.syscfg` 不一致）：

- `board/sys_config/ti_msp_dl_config.c`
- `board/sys_config/ti_msp_dl_config.h`

### 修改外设配置的正确流程

1. 改 `board/sys_config/G3507.syscfg`（纯配置行，**不要在其中写注释**——再生成会丢失）。
2. 用 SysConfig CLI 重新生成（`<SYSCONFIG>` = SysConfig 安装目录，`<OUT>` = 输出目录，
   `<REPO>` = 本仓库根目录）：

```powershell
& "<SYSCONFIG>/sysconfig_cli.bat" `
    --product "<REPO>/third_party/mspm0-sdk/.metadata/product.json" `
    --compiler ticlang --output "<OUT>" `
    "<REPO>/board/sys_config/G3507.syscfg"
```

3. 建议先生成到临时目录，`git diff` 核对只有预期变化后再拷回 `board/sys_config/`。

> 坑：外设引脚的 `internalResistor`（如给 UART RX 加上拉）必须**同时**设 `xxxPinConfig.enableConfig = true`，
> 否则该属性**静默无效**（`enableConfig` 默认 false，且 CLI 不报警告）。生成结果应从
> `DL_GPIO_initPeripheralInputFunction` 变为 `...InputFunctionFeatures` 并带 `DL_GPIO_RESISTOR_*` 才算生效。

如重生成后产生差异，优先判断是否来自工具版本或配置变更。

## 运行验证清单

编译通过后仍需上板验证（当前 app 框架，详见 `docs/app-design.md`）：

- 上电后 OLED 显示 `Main Menu`，含赛题 H2～H6 五个任务入口与 `Device Check` 子菜单
- 短按 UP/DOWN 移动选择、ENTER 进入、BACK 返回上级
- `Device Check` 子菜单内 8 个自检（Gyro JY61P / Gray I2C / TB6612 / Step Motor /
  Encoder / Speed PID / Duty Sweep / Rpi UART）可进入并刷新数据
- `Gray I2C` 进入后显示 8 路数字量二进制、`act` 触发数与 `ok/er`、`W/R/s` 诊断计数；
  断开传感器应显示 `READ FAIL` 且 `er` 递增
- TB6612 自检短按发单次脉冲、左右轮编码器计数（`encL`/`encR`）随之变化
- `Step Motor` 须**摆杆脱开或行程内**运行；页面含 JOG/LEVEL/RUN/TURN/HAND/SWEEP/SPAN
  七个模式，**进页为 JOG**（UP/DOWN 单击点动一步，长按调步长 1/5/10/20 计数）。
  JOG 长按 ENTER 进入 LEVEL；LEVEL 短按 ENTER 捕获本次上电的 H3 动力学水平点并显示
  `LEVEL SAVED`，长按 ENTER 跳过/离开并进入 RUN。
  RUN 模式下按 UP 时 `cnt` 应增大、`err`(位置误差) 收敛不发散（`err` 就是丢步的直接指标，
  原先的开环 `est`/`slip` 已删除）；HAND 模式自动断电，手转 N 圈后 `cpr` 栏即
  `STEP_MOTOR_ENCODER_COUNTS_PER_REV` 的实测值。完整流程见
  `docs/step-motor-calibration.md`
- `Encoder` 自检整车前进时两轮 `spd` 应同为正（方向符号见 `bsp/motor/hall_encoder.h`）
- `Gyro JY61P` 自检页的 `Ay m/s2` 是车体纵向（JY61P Y 轴）原始加速度；静止时应接近 0，
  当前安装下向前加速应为负、停车减速应为正，控制侧会乘以 `-1` 归一化为前向正值。
- `H3 Hold 0cm` 中连接 `ball_tune.html`，确认 `imuok=1`；前后推动小车时 `araw/aflt` 应随
  纵向加速度变化。先保持 `imu_gain=0` 验证原闭环，再从小增益开始确认 `aff` 补偿方向。
- `ball_tune.html` 的“导出配置 JSON”应下载包含当前全部固件回显参数的独立 `.json` 文件。
- `Speed PID` / `Duty Sweep` 须**抬起车轮**运行；可配合
  `tools/visualizers/speed_pid_viz.py` 看曲线
- `SysTick_Handler` 可按周期扫描按键（菜单响应正常即证明）

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

## 参考

- 烧录 8 字节对齐问题的社区反馈：TI E2E “MSPM0G3507: Keil: *Specific* code can be compiled
  but cannot be downloaded”
  <https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/1407593>
