# app/app_launcher 接口说明

## 模块职责

`app/app_launcher` 是应用层顶层入口。它负责显示启动菜单，并根据按键事件进入 2025 年电赛 E 题基本要求 1、2、3、拐角刹车测试或设备检查页面。

该模块负责：

- 渲染顶层菜单。
- 用短按切换菜单项。
- 用长按进入当前菜单项。
- 为 E 题基本要求 1 提供子菜单，在子菜单内选择 1 到 5 圈。

该模块不负责：

- 巡线控制算法。
- 云台视觉跟踪算法。
- 具体外设驱动。
- 题目结果判分。

## 公开接口

### `void App_Launch(void)`

进入应用顶层菜单。该函数通常只由 `main()` 调用，并在内部长期运行。

当前菜单项：

- `E1 Line`
- `Corner test`
- `E2 Aim`
- `E3 Yaw+`
- `E3 Yaw-`
- `Device check`

进入 `E1 Line` 后显示圈数子菜单：

- `1 lap`
- `2 laps`
- `3 laps`
- `4 laps`
- `5 laps`

E1 子菜单中短按或双击切换圈数，长按确认并调用 `AppE_RunLineFollow(1..5)`；任务返回后回到顶层菜单。

`Corner test` 直接调用 `AppE_RunCornerBrakeTest()`，用于测试循线识别到拐角后刹车，再以左轮负值、右轮正值的原地差速左转，并由中心 3/4 重新检测到轨道时刹车停止的触发时机和动作参数。

`E3 Yaw+` 和 `E3 Yaw-` 分别调用 `AppE_RunRectScanYawPositive()` 与 `AppE_RunRectScanYawNegative()`。二者只改变任务 3 的扫描方向：识别到矩形后先停止约 `200ms`，再进入与 `E2 Aim` 相同的靶心瞄准控制链。
