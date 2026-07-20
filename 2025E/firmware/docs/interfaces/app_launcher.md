# app/app_launcher 接口说明

`App_Launch()` 提供长期运行的顶层菜单，负责菜单选择和任务入口分发，不包含控制算法。

当前菜单：

- `E1 Line`：进入 1～5 圈子菜单。
- `Aim track`：纯视觉持续跟踪调试。
- `E2 Aim`：任意位姿 2 秒视觉瞄准。
- `E3 Geo aim`：规定位置几何前馈瞄准。
- `F1 Line+aim`：循迹一圈并连续瞄准靶心。
- `F2 Line+aim`：循迹两圈并连续瞄准靶心。
- `F3 Line+circle`：循迹一圈并在靶面画圆。
- `Calibration`：进入编码器、IMU、几何前馈、视觉偏置和 F3 圆相位实测菜单。
- `Device check`：外设检查与调试页面。

任务返回后重新显示原菜单项。
