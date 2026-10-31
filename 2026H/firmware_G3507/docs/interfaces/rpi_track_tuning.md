# 树莓派—MSPM0 赛道调参接口

## 1. 适用范围

本接口用于在任务开始前调整 H2、H5、H6 的分段阈值、终点阈值和弯道前馈。H4 不读取这些参数，行为保持不变。

当前 H2/H5/H6 的灰度转向增益固定使用固件值 `LT_GYRO_LINE_KP`；灰度正常时提供受限转向残差，全白时退回模型/陀螺继续运行。该增益不属于现场调参项。

参数只保存在 MSPM0 RAM 中：上电或复位后恢复固件默认值，不写 Flash。H2/H5/H6 进入任务时复制一次完整快照，本次运行始终使用该快照。

## 2. 物理接口

- 外设：`Rpi_UART` / UART2
- MSPM0 引脚：PB15 TX、PA24 RX
- 格式：115200 baud、8 data bits、no parity、1 stop bit
- 多字节整数：小端序
- 帧长：固定 11 字节

该链路同时承载既有滚球视觉帧。视觉帧类型 `0x01` 保持不变；调参请求使用 `0x81`，响应使用 `0x82`。

## 3. 帧格式

### 3.1 调参请求（树莓派 → MSPM0）

| 字节 | 字段 | 说明 |
|---:|---|---|
| 0 | SYNC0 | `0xA5` |
| 1 | SYNC1 | `0x5A` |
| 2 | TYPE | `0x81` |
| 3 | SEQ | 请求序号，响应原样返回 |
| 4 | OP | 操作码 |
| 5–6 | PARAM_ID | `uint16`，小端 |
| 7–8 | VALUE | `uint16`，小端；仅 SET 使用 |
| 9 | RESERVED | 必须发送 `0` |
| 10 | CRC8 | 覆盖字节 2～9 |

### 3.2 调参响应（MSPM0 → 树莓派）

| 字节 | 字段 | 说明 |
|---:|---|---|
| 0 | SYNC0 | `0xA5` |
| 1 | SYNC1 | `0x5A` |
| 2 | TYPE | `0x82` |
| 3 | SEQ | 对应请求序号 |
| 4 | STATUS | 状态码 |
| 5–6 | PARAM_ID | 参数 ID，小端 |
| 7–8 | VALUE | 当前有效值，小端 |
| 9 | PRIORITY | 参数优先级 1～3；结束帧为 0 |
| 10 | CRC8 | 覆盖字节 2～9 |

CRC 为 CRC-8：多项式 `0x07`、初值 `0xFF`、不反转、无最终异或。

## 4. 操作和状态

| OP | 名称 | 行为 |
|---:|---|---|
| `0x01` | GET | 读取一个参数；VALUE 忽略 |
| `0x02` | SET | 设置一个参数并回显实际值 |
| `0x03` | GET_ALL | 依表顺序返回 13 帧，随后发送结束帧 |
| `0x04` | RESET_DEFAULTS | 全部恢复编译期默认值 |

GET_ALL 结束帧为 `PARAM_ID=0xFFFF`、`VALUE=13`、`PRIORITY=0`。RESET_DEFAULTS 成功响应也使用 `PARAM_ID=0xFFFF`、`VALUE=13`。

| STATUS | 名称 | 含义 |
|---:|---|---|
| `0x00` | OK | 成功 |
| `0x01` | UNKNOWN_ID | 参数 ID 不存在 |
| `0x02` | BUSY | 当前不在主菜单，拒绝 SET/RESET |
| `0x03` | BAD_OP | 操作码不存在 |

GET 和 GET_ALL 可随时使用；SET 和 RESET_DEFAULTS 仅在 `APP_MODE_MENU` 时生效。

## 5. 参数表

所有距离单位均为 mm，前馈使用千分比定点数。固件只校验 ID，不限制值域或参数间关系；推荐范围、危险提示和二次确认由树莓派 UI 实现。

| 优先级 | ID | 参数 | 默认值 | 建议范围 | 实际用途 |
|---:|---:|---|---:|---:|---|
| 1 | `0x0101` | `s1_end_mm` | 1500 | 1300～1700 | S1→S2 切换里程 |
| 1 | `0x0102` | `s2_end_mm` | 3071 | 2750～3400 | S2→S3 切换里程 |
| 1 | `0x0103` | `s3_end_mm` | 4641 | 4200～5000 | S3→S4 切换里程 |
| 1 | `0x0104` | `s4_heading_end_mm` | 6212 | 5600～6800 | S4 航向参考完成 360° 的里程 |
| 1 | `0x0105` | `s3_gyro_recover_mm` | 300 | 0～600 | S2 结束后陀螺仪直行回正距离 |
| 2 | `0x0110` | `lap_stop_mm` | 6242 | 现场确定 | A 点减速/横线参考绝对里程 |
| 2 | `0x0111` | `finish_arm_margin_mm` | 400 | 现场确定 | 横线武装起点为 `lap_stop_mm - 本值` |
| 2 | `0x0112` | `loaded_decel_warning_mm` | 250 | 现场确定 | H5/H6 减速起点为 `lap_stop_mm - 本值` |
| 2 | `0x0113` | `loaded_odom_arrival_mm` | 6342 | 现场确定 | H5/H6 编码器到达 A 的绝对里程 |
| 2 | `0x0114` | `h2_odom_fallback_mm` | 6442 | 现场确定 | H2 横线漏检兜底停车绝对里程 |
| 3 | `0x0120` | `h2_s2_ff_x1000` | 1100 | 850～1300 | H2 第一弯前馈倍率 |
| 3 | `0x0121` | `loaded_s2_ff_x1000` | 1000 | 850～1300 | H5/H6 第一弯前馈倍率 |
| 3 | `0x0122` | `s4_ff_x1000` | 1000 | 850～1300 | H2/H5/H6 第二弯前馈倍率 |

## 6. 树莓派端设计要求

1. 配置页只允许在车辆主菜单/闲置态使用。进入任务按钮前先执行 GET_ALL，展示 MSPM0 的实际回读值。
2. 同一时刻只保留一个未完成请求。整帧一次写入串口，收到相同 SEQ 的完整响应后再发下一请求；GET_ALL 必须读到 `0xFFFF` 结束帧。
3. SET 成功后以响应中的 VALUE 更新 UI，不以本地发送值假定成功。收到 BUSY 时不重试写入，应提示先退出任务。
4. UI 对优先级 1 和 3 使用上表建议范围。允许工程人员越界时，应显示醒目警告并二次确认，而不是静默截断。
5. UI 至少检查 `s1_end_mm < s2_end_mm < s3_end_mm < s4_heading_end_mm`。还应提示 `s2_end_mm + s3_gyro_recover_mm < s3_end_mm`，否则回正窗口会覆盖到下一弯；两个 margin 不得大于 `lap_stop_mm`。
6. 推荐提示关系为 `s4_heading_end_mm <= lap_stop_mm`、`lap_stop_mm <= loaded_odom_arrival_mm`、`lap_stop_mm <= h2_odom_fallback_mm`。后两项分别服务不同任务，不要求彼此绑定。
7. 树莓派若要跨掉电保存赛场配置，应自行保存带版本号的 profile；每次 MSPM0 复位后重新 SET 全部参数并 GET_ALL 核对。不要假定 MCU 会持久化。
8. 配置模式下建议暂停视觉帧发送并由一个串口线程独占 UART。协议解析器虽然能区分 `0x01` 与 `0x81/0x82`，但单一请求/响应流更容易做超时、重试和日志追踪。

仓库提供命令行参考实现 [`tools/rpi/track_tune_cli.py`](../../tools/rpi/track_tune_cli.py)。
运行前需在树莓派安装 `pyserial`：`python3 -m pip install pyserial`。

## 7. 示例

读取全部参数：

```bash
python3 tools/rpi/track_tune_cli.py /dev/serial0 get-all
```

把 H2 第一弯前馈改为 1.05：

```bash
python3 tools/rpi/track_tune_cli.py /dev/serial0 set 0x0120 1050
```

恢复固件默认值：

```bash
python3 tools/rpi/track_tune_cli.py /dev/serial0 reset
```
