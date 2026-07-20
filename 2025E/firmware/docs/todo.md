# Todo

- 后续评估移除 `core/kinematics` 中的 `DEG_TO_RAD`、`RAD_TO_DEG` 和 `RotationAngles` 兼容定义。
- 实机标定 `AUTO_AIM_CONFIG.encoder_lateral_offset_m` 的左右侧符号和轮距等效偏移。
- 上板校准 E1 循迹圈数计数策略，重点检查 `LineFollow_IsHalfDetected()` 判定范围、最小距离门限和不同车速下的重复计数问题。
- 后续根据真实系统时钟复核 `BSP_TIME_CPUCLK_HZ`，必要时改为从时钟配置自动派生。
- 复核规定起点是否为 A 点、初始车头是否沿 A→C，以及角点序列 `C→D→B→A`。
- 标定靶心世界坐标、云台安装偏移、yaw/pitch 零位和 F3 起始相位。
- 在实景 ROI 内评估 Otsu 或自适应 Canny 阈值，保留固定阈值兜底。
- 后续如增加第二个实体按键，在 `KEY_ID`、硬件映射宏和 `bsp/key` 内部配置表中同步扩展。
- 后续构建具体 BSP 外设时，继续按 SDK 风格统一公开类型和枚举成员命名。
- 如需要更彻底的 API 统一，后续可把公开 C 函数也改为 snake_case。
- 上板验证启动页、任务菜单、设备自检、UART0 IMU、UART2 视觉和 SysTick 按键扫描。
