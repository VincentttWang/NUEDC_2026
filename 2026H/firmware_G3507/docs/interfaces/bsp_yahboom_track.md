# bsp/yahboom_track 接口说明

## 模块职责

`bsp/yahboom_track` 驱动 Yahboom 八路循线模块的 I2C 数字量接口。它复用 SysConfig 已初始化的
I2C0（实例 `Gray_JY61P_I2C_INST`，`PA0=SDA / PA1=SCL`），模块 7 位地址为 `0x12`，与
JY61P（`0x50`）不冲突，**无需修改 SysConfig**。

官方协议定义：

| 寄存器 | 方向 | 含义 |
|---|---|---|
| `0x30` | 读 | 八路数字状态，`bit7=X1` … `bit0=X8`；`0`=黑线/灯亮，`1`=白底/灯灭 |
| `0x01` | 写 | `1`=进入校准，`0`=退出校准 |

## 接口

```c
BSP_STATUS YahboomTrack_Init(void);
BSP_STATUS YahboomTrack_ReadRaw(uint8_t *raw);
BSP_STATUS YahboomTrack_ReadDetectedMask(uint8_t *mask);
BSP_STATUS YahboomTrack_SetCalibration(bool enabled);
void YahboomTrack_GetDiag(uint32_t *read_fail, uint32_t *write_fail,
                          int32_t *last_status);
```

- `ReadRaw` 完整保留设备协议位序与低有效极性。
- `ReadDetectedMask` 输出便于应用消费的检测掩码：`bit0=X1` … `bit7=X8`，置 `1` 表示检测到黑线。
  该语义与 `LineFollow_UpdateDetectedMask()` 的约定一致，app 层无需再做位序或极性转换。
- `SetCalibration(true)` 只让设备进入校准；仍须按官方流程，用板载按键依次记录黑线和白底。
- `Init` 最多用约 100 ms 尝试确认 I2C 应答，不会阻塞等待官方建议的 20 秒探头预热。
- `GetDiag` 的形参顺序是 **read, write**（与页面显示的 `W`/`R` 顺序相反，接线时注意别传反）。

## 使用约束

- 驱动使用阻塞 I2C，只能在线程上下文调用，禁止在 ISR 中调用。
- 它与 JY61P 共用 I2C0。调用前必须由 app 层执行 `JY61P_I2C_SetSuspended(true)`，结束占用后
  再恢复，不能与 JY61P 异步状态机并发。周期循迹还必须先用 `JY61P_I2C_IsIdle()` 确认进行中的
  IMU 事务已经自然结束，不能强制截断。
- 模块官方建议每次上电后等待至少 20 秒，使探头稳定；更换环境、安装高度或地图材质后应重新校准。

## 当前集成状态

- 已接入 `Main Menu -> Device Check -> Gray I2C`，页面显示 X1→X8 的检测位图、有效路数、
  累计成功/失败次数与 I2C 诊断计数。
- H2/H4/H5/H6 使用 `ReadDetectedMask()`；app 负责共享总线的时间片，middleware 只接收标准化
  掩码，不依赖本阻塞驱动。
- Device Check 页面只读状态，不会主动进入校准模式；校准需显式调用接口并配合板载按键。

## 协议依据

- [Yahboom 八路循线模块官方协议](https://www.yahboom.net/public/upload/upload-html/1731564712/8-channel%20line%20tracking%20module%20usage.html)
- [Yahboom MSPM0 小车八路循线官方示例](https://www.yahboom.net/public/upload/upload-html/1744716503/03.8-channel%20tracking.html)
- [Yahboom 官方示例仓库](https://github.com/YahboomTechnology/8-Channel-tracking-module)

官方协议页的 X1 单路示例中二进制与十六进制文本存在排版不一致；本驱动的位序以官方
MSPM0 示例代码为准：`X1=bit7`、`X8=bit0`，黑线/LED 亮为 `0`。
