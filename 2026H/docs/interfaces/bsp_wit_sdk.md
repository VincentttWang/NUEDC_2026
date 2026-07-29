# bsp/imu/wit_sdk 接口说明

## 模块职责

`bsp/imu/wit_sdk` 是 WitMotion IMU 厂家驱动与当前工程 JY61P 数据解析入口。该模块保留厂家 SDK 的命名、协议解析、寄存器定义和已有全局变量，不按工程自维护代码的命名规则强制重写。

本轮只追加少量读取接口，目的是让后续上层逐步从直接访问 `GyroscopeChannelData[]` 迁移到明确的数据结构接口。

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

## 新增公开类型

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

## 新增读取接口

### `int32_t WitGetAcc(WIT_VECTOR3F *out)`

读取加速度缓存。`out == NULL` 时返回 `WIT_HAL_INVAL`，成功返回 `WIT_HAL_OK`。

当前数据来源为 `GyroscopeChannelData[0..2]`。

### `int32_t WitGetGyro(WIT_VECTOR3F *out)`

读取角速度缓存，单位为 `deg/s`。`out == NULL` 时返回 `WIT_HAL_INVAL`，成功返回 `WIT_HAL_OK`。

当前数据来源为 `GyroscopeChannelData[3..5]`。

### `int32_t WitGetAttitude(WIT_ATTITUDE *out)`

读取姿态角缓存，单位为度。`out == NULL` 时返回 `WIT_HAL_INVAL`，成功返回 `WIT_HAL_OK`。

当前数据来源为 `GyroscopeChannelData[6..8]`。

### `int32_t WitGetData(WIT_IMU_DATA *out)`

一次性读取加速度、角速度、姿态角和温度缓存。`out == NULL` 时返回 `WIT_HAL_INVAL`，成功返回 `WIT_HAL_OK`。

当前数据来源为 `GyroscopeChannelData[0..9]`。

## 数据来源说明

当前应用自检路径通过 `GYROSCOPE_DATA_Decoder()` 解析 JY61P 标准串口子帧，并写入 `GyroscopeChannelData[]`。每个子帧固定 11 字节，以 `0x55` 开头，第二字节表示数据类型：

- `0x51`：加速度和温度。
- `0x52`：角速度，单位换算为 `deg/s`。
- `0x53`：姿态角，单位换算为度。

`GYROSCOPE_DATA_Decoder()` 当前解析单个 11 字节子帧；旧 `IT_JY61P()` 仍可处理 33 字节三帧连续缓存。应用层 UART0 中断当前按 11 字节子帧校验和解析，因此启用或关闭某类 JY61P 回传数据后，不再要求加速度、角速度、姿态角必须以固定 33 字节组合出现。

如果后续统一改为厂家 `WitSerialDataIn()` 和 `sReg[]` 路径，再评估是否将 getter 的数据来源切换到 `sReg[]`。
