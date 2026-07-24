# Todo

- 后续评估移除 `core/kinematics` 中的 `DEG_TO_RAD`、`RAD_TO_DEG` 和 `RotationAngles` 兼容定义。
- 后续重新设计旧 `Compute_excur()` 对应的转弯期间云台补偿策略，避免在 core 中直接依赖 LED GPIO、旧 `edge/sInedge` 和编码器全局接口。
- 上板校准 E1 循迹圈数计数策略，重点检查 `LineFollow_IsHalfDetected()` 判定范围、最小距离门限和不同车速下的重复计数问题。
- 后续根据真实系统时钟复核 `BSP_TIME_CPUCLK_HZ`，必要时改为从时钟配置自动派生。
- 后续在具体使用点重新设计旧 `sInedge` 和 `UpdateSInedge()` 对应的阶段距离逻辑，优先评估 `Chassis_GetDistance()` / `Chassis_ResetDistance()`。
- 后续如增加第二个实体按键，在 `KEY_ID`、硬件映射宏和 `bsp/key` 内部配置表中同步扩展。
- 后续构建具体 BSP 外设时，继续按 SDK 风格统一公开类型和枚举成员命名。
- 如需要更彻底的 API 统一，后续可把公开 C 函数也改为 snake_case。
- 上板验证启动页、任务菜单、设备自检、UART0 IMU、UART2 视觉和 SysTick 按键扫描。
