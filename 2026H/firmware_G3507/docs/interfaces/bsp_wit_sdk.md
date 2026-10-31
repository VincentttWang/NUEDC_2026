# bsp/imu/wit_sdk 接口说明

## 模块职责

`bsp/imu/wit_sdk` 是 WitMotion IMU 厂家驱动与当前工程 JY61P 数据采集入口。该模块保留厂家
SDK 的命名、串口协议解析、寄存器定义和已有全局变量，同时追加当前实际使用的 I2C0 异步驱动，
不按工程自维护代码的命名规则强制重写。

控制器应优先读取原子发布的 `JY61P_I2C_SAMPLE`，不应继续直接访问
`GyroscopeChannelData[]`。

## 保留的厂家接口

以下接口和全局变量暂时保留：

- `WitInit()`
- `WitSerialDataIn()`
- `WitWriteReg()`
- `WitReadReg()`
- `WitStartAccCali()` / `WitStopAccCali()`
- `WitStartMagCali()` / `WitStopMagCali()`
- `JY61P_Init()`
- `GYROSCOPE_DATA_Decoder()`
- `IT_JY61P()`
- `sReg[]`
- `GyroscopeChannelData[]`

上述串口解析接口为厂家兼容路径，当前 app 不调用；UART0 已分配给蓝牙。

## 当前 I2C0 接口

当前 JY61P 使用 `Gray_JY61P_I2C_INST`（I2C0，PA0/PA1，400kHz），7 位地址
`0x50`。采集采用非阻塞状态机：线程上下文调用 `JY61P_I2C_Poll()` 发起
“读 acc → 读 angle → 读 gyro”链，`I2C0_IRQHandler()` 分阶段完成 TX/RX 和数据发布。

- `JY61P_I2C_Init()`：清零诊断和状态机，打开 I2C0 NVIC。
- `JY61P_I2C_Poll()`：空闲时发起一轮加速度、姿态角与角速度读取；忙或总线未空闲时立即返回。
- `JY61P_I2C_SetSuspended(bool)`：挂起/恢复 JY61P 事务和 I2C0 中断，供 Yahboom 循线
  等同总线阻塞驱动分时。
- `JY61P_I2C_IsIdle()`：仅当 JY61P 状态机和 I2C0 控制器都空闲时返回 `true`；app 可据此
  安全地给 Yahboom 阻塞读取分配一个短时间片。
- `JY61P_I2C_GetPollCount()` / `GetErrorCount()` / `GetNackCount()` /
  `GetTimeoutCount()`：读取诊断计数。
- `JY61P_I2C_GetSampleCount()`：读取已完整发布的 acc + angle + gyro 样本数。
- `JY61P_I2C_IsDataFresh(max_age_ms)`：仅当至少发布过一个完整样本，且最近样本年龄不超过
  指定阈值时返回 `true`。上层控制器应使用此接口判断数据有效性，不应根据 poll/error
  计数反推事务是否成功。
- `JY61P_I2C_GetSnapshot()`：用序列锁原子复制同一轮 acc + angle + gyro 以及其样本计数和发布时间戳。

当前由 `app/app_line_task.c` 的循迹任务、独立 `H3 Hold 0cm` 和 JY61P 自检任务在各自控制拍
（10ms/100Hz）调用 `JY61P_I2C_Poll()`；SysTick ISR 不轮询 JY61P。

## 结构化读取类型

### `WIT_VECTOR3F`

```c
typedef struct {
    float x;
    float y;
    float z;
} WIT_VECTOR3F;
```

用于加速度和角速度三轴数据。

### `WIT_ATTITUDE`

```c
typedef struct {
    float roll;
    float pitch;
    float yaw;
} WIT_ATTITUDE;
```

用于 roll、pitch、yaw 三轴姿态角，单位为度。

### `WIT_IMU_DATA`

```c
typedef struct {
    WIT_VECTOR3F acc_g;
    WIT_VECTOR3F gyro_deg_s;
    WIT_ATTITUDE attitude_deg;
    float temperature_c;
} WIT_IMU_DATA;
```

用于一次性读取当前缓存中的 IMU 数据。

## 结构化读取接口

### `int32_t WitGetAcc(WIT_VECTOR3F *out)`

读取加速度缓存。`out == NULL` 时返回 `WIT_HAL_INVAL`，成功返回 `WIT_HAL_OK`。

当前数据来源为 `GyroscopeChannelData[0..2]`。I2C 路径读取 `AX/AY/AZ`（`0x34..0x36`）
并按 ±16 g 量程换算；旧串口解析路径继续兼容同一单位。

### `int32_t WitGetGyro(WIT_VECTOR3F *out)`

读取角速度缓存，单位为 `deg/s`。`out == NULL` 时返回 `WIT_HAL_INVAL`，成功返回 `WIT_HAL_OK`。

当前数据来源为 `GyroscopeChannelData[3..5]`。

### `int32_t WitGetAttitude(WIT_ATTITUDE *out)`

读取姿态角缓存，单位为度。`out == NULL` 时返回 `WIT_HAL_INVAL`，成功返回 `WIT_HAL_OK`。

当前数据来源为 `GyroscopeChannelData[6..8]`。

### `int32_t WitGetData(WIT_IMU_DATA *out)`

一次性读取加速度、角速度、姿态角和温度缓存。`out == NULL` 时返回 `WIT_HAL_INVAL`，成功返回 `WIT_HAL_OK`。

当前数据来源为 `GyroscopeChannelData[0..9]`。现用 I2C 路径更新加速度 `[0..2]`、角速度
`[3..5]` 和姿态角 `[6..8]`；温度 `[9]` 不在当前 I2C 采集链内更新。

## 数据来源说明

当前 I2C0 状态机依次读取加速度寄存器 `0x34`、角度寄存器 `0x3D` 和角速度寄存器 `0x37`，
各 6 字节，并发布到 `GyroscopeChannelData[0..8]`。结构化 getter 只是复制这份缓存，不主动访问总线。

厂家串口兼容路径仍可通过 `GYROSCOPE_DATA_Decoder()` 解析 `0x51/0x52/0x53` 三类 11 字节
子帧，旧 `IT_JY61P()` 也仍能处理 33 字节组合帧；但当前固件没有 UART ISR 或 app 调用者接入
这条路径。
