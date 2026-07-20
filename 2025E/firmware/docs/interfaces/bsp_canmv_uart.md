# bsp/canmv/canmv_uart 接口说明

## 模块职责

`bsp/canmv/canmv_uart` 负责 CanMV 视觉模块的 UART 收发和帧解析。该模块只处理字节流、帧边界、目标数据解析和 CanMV 状态，不负责视觉业务决策、目标选择或坐标转换。

## 帧格式

当前协议使用固定帧头和帧尾：

```c
#define CANMV_FRAME_START 0x12U
#define CANMV_FRAME_END   0x5BU
```

除原坐标帧外，UART2 同时接受 K230 角度误差帧：

```text
A5 5A yaw_i16_be pitch_i16_be checksum
```

总长 9 字节：`A5 5A status seq yaw_i16_be pitch_i16_be checksum`。`yaw/pitch` 为有符号大端整数，单位 `0.01 deg`；`status` 为 `0=VALID`、`1=NOT_FOUND`、`2=LOST`，`seq` 为滚动帧序号，校验和为前 8 个字节的低 8 位。角度帧的数据通过 `CANMV_TARGET_ANGLE` 读取。原 `0x12...0x5B` 坐标帧保持不变。

每个相机周期都发送角度状态帧。未识别到目标时显式发送 `NOT_FOUND/LOST`，MCU 因而不会继续使用上一帧的陈旧角度误差。

已知数据段：

- 激光点数据：从 `CANMV_LASER_BEGIN` 开始，共 `CANMV_LASER_BYTE_COUNT` 字节，解析为 4 个 `uint16_t`
- 矩形点数据：从 `CANMV_RECT_BEGIN` 开始，共 `CANMV_RECT_BYTE_COUNT` 字节，解析为 8 个 `uint16_t`

每个 `uint16_t` 按高字节在前、低字节在后的方式组合。

为了兼容 K230 端脚本或库函数误用低字节在前发送的情况，解析器会对坐标做轻量合理性检查：若按高字节在前解析得到明显异常的五位数，而交换字节后落在 `CANMV_COORDINATE_SANITY_LIMIT` 范围内，则采用交换后的值。当前默认合理范围上限为 `1000`，覆盖 320x240 图像坐标和当前矩形角点数据。

解析器还要求固定帧长度必须精确等于 `CANMV_MIN_FRAME_LEN`。若随机串口字节中偶然出现 `0x12` 和 `0x5B`，但帧长不匹配或坐标合理性检查失败，整帧会被标记为丢帧，不会更新目标缓存。

当前 `k230/rect_07.py` 会发送兼容旧 CanMV 协议的固定二进制帧：

```text
0x12, reserved,
target_x_hi, target_x_lo,
target_y_hi, target_y_lo,
laser_x_hi,  laser_x_lo,
laser_y_hi,  laser_y_lo,
bottom_left_x/y,
bottom_right_x/y,
top_right_x/y,
top_left_x/y,
0x5B
```

其中 `target_x/target_y` 为 K230 识别到的靶心坐标。当前 K230 脚本尚未识别激光点，`laser_x/laser_y` 暂时填入图像中心，使上层可执行“相机中心对准靶心”的控制链路。若后续增加激光光斑识别，应将该字段改为实际激光点坐标。

`k230/uart1_comm_test.py` 是独立的通信测试脚本，使用 K230 UART1 从 GPIO3/TX1、GPIO4/RX1 以 115200 8N1 周期发送同格式测试帧。测试帧中的目标坐标会随计数递增而变化，便于在 Device Check 的 K230 页面确认链路实时工作。

## 硬件映射宏

```c
#define CANMV_UART_INST UART2
#define CANMV_UART_IRQN UART2_INT_IRQn
#define CANMV_RX_BUFFER_LEN 100U
#define CANMV_MIN_FRAME_LEN (CANMV_RECT_BEGIN + CANMV_RECT_BYTE_COUNT + 1U)
```

如果后续 UART 实例或缓冲区大小变化，应通过覆盖这些宏调整。

当前 MSPM0 工程的 SysConfig 将 K230 通信映射到 `UART2`，K230 端测试脚本使用的是 K230 芯片自己的 `UART1`。两者不要求同名，只要物理连线满足 K230 UART1 TX 接 MSPM0 K230 RX、K230 UART1 RX 接 MSPM0 K230 TX，并且双方共地即可。

`CanMvUart_Init()` 会显式关闭 SysConfig 可能开启的 TX 中断，只保留 RX 和 RX timeout 中断，并把 RX FIFO 阈值设为 1 字节，保证通信路径只通过接收中断进入解析器。

## 公开类型

### `CANMV_TARGET`

```c
typedef enum {
    CANMV_TARGET_LASER = 0,
    CANMV_TARGET_RECT,
    CANMV_TARGET_ANGLE,
    CANMV_TARGET_MAX
} CANMV_TARGET;
```

### `CANMV_STATUS`

```c
typedef enum {
    CANMV_STATUS_INIT = -1,
    CANMV_STATUS_OK = 0,
    CANMV_STATUS_NOT_FOUND = 1,
    CANMV_STATUS_LOST = 2,
    CANMV_STATUS_FRAME_DROP = 3,
} CANMV_STATUS;
```

状态说明：

- `CANMV_STATUS_INIT`：尚未收到有效帧。
- `CANMV_STATUS_OK`：目标数据有效。
- `CANMV_STATUS_NOT_FOUND`：从未识别到目标。
- `CANMV_STATUS_LOST`：曾经识别到目标，但当前帧目标为空。
- `CANMV_STATUS_FRAME_DROP`：帧格式异常、接收溢出或非帧数据进入解析器。

### `CANMV_TARGET_DATA`

```c
typedef struct {
    uint16_t value[CANMV_TARGET_VALUE_CAPACITY];
    uint8_t count;
    CANMV_STATUS status;
} CANMV_TARGET_DATA;
```

`value` 保存目标数据，`count` 表示当前目标实际有效的 `uint16_t` 数量，`status` 表示该目标状态。

## 公开接口

### `BSP_STATUS CanMvUart_Init(void)`

清空接收状态和目标数据，初始化目标状态为 `CANMV_STATUS_INIT`，配置 UART2 接收中断，并使能 CanMV UART IRQ。

### `void CanMvUart_ProcessByte(uint8_t byte)`

处理一个输入字节。该接口只做协议解析，不直接读取 UART 硬件，便于后续统一中断分发或测试代码喂入数据。

### `void CanMvUart_ProcessRx(void)`

排空 `CANMV_UART_INST` 当前 RX FIFO，并逐字节调用 `CanMvUart_ProcessByte()`。

## 诊断变量

```c
extern volatile uint32_t g_canmv_uart_rx_byte_count;
extern volatile uint32_t g_canmv_uart_valid_frame_count;
extern volatile uint32_t g_canmv_uart_drop_count;
extern volatile uint8_t g_canmv_uart_last_byte;
```

这些变量用于上板调试 UART2 通信：

- `rx_byte_count`：UART2 已收到的字节数。
- `valid_frame_count`：已解析到帧头帧尾完整且长度满足要求的帧数。
- `drop_count`：接收失步、短帧或缓冲区溢出导致的丢帧次数。
- `last_byte`：最近收到的原始字节。

### `BSP_STATUS CanMvUart_SendByte(uint8_t byte)`

等待 UART 空闲后发送一个字节。

### `BSP_STATUS CanMvUart_SendString(const char *str)`

发送以 `\0` 结尾的字符串。`str == NULL` 时返回 `BSP_STATUS_NULL`。

### `CANMV_STATUS CanMvUart_GetStatus(CANMV_TARGET target)`

返回指定目标状态。非法目标返回 `CANMV_STATUS_INIT`。

### `uint8_t CanMvUart_GetData(CANMV_TARGET target, uint16_t *out, uint8_t max_count)`

将指定目标数据复制到调用方缓冲区，返回实际复制数量。非法目标、空指针或 `max_count == 0` 时返回 `0`。

### `const CANMV_TARGET_DATA *CanMvUart_GetTargetData(CANMV_TARGET target)`

返回指定目标内部数据的只读指针。非法目标返回 `NULL`。

## 对接说明

旧 `Laser_*`、`Rect_*`、`CanMV_Process()` 和全局视觉状态接口已不再作为当前应用层入口使用。新的视觉数据读取统一通过 `CanMvUart_*` 接口完成。
