# BSP Common 接口

## 职责

`bsp/common/bsp_common.h` 只定义跨 BSP 外设复用、且不属于任何具体外设协议的 BSP 状态类型。

该文件不包含业务状态，不包含 CanMV、OLED、按键、电机等具体外设状态，也不提供通用 inline 工具。

## 公开类型

### `BSP_STATUS`

BSP 驱动函数的通用返回状态。

```c
typedef enum {
    BSP_STATUS_OK = 0,
    BSP_STATUS_ERROR = -1,
    BSP_STATUS_NULL = -2,
    BSP_STATUS_INVALID_ARG = -3,
    BSP_STATUS_TIMEOUT = -4,
    BSP_STATUS_BUSY = -5,
    BSP_STATUS_NOT_READY = -6,
} BSP_STATUS;
```

使用约定：

- `BSP_STATUS_OK` 表示操作成功。
- `BSP_STATUS_NULL` 表示传入空指针或必要实例为空。
- `BSP_STATUS_INVALID_ARG` 表示参数值非法。
- `BSP_STATUS_TIMEOUT` 表示等待硬件状态超时。
- `BSP_STATUS_BUSY` 表示外设忙。
- `BSP_STATUS_NOT_READY` 表示驱动尚未初始化或外设未就绪。
- `BSP_STATUS_ERROR` 用于无法归类的通用错误。

## 不放入 common 的内容

- 各外设专属的状态码/枚举：属于对应外设驱动的头文件。
- 时间和延时函数：已由 `bsp/time` 单独提供。
- duty 限幅、频率换算等工具：放在对应外设驱动的 `.c` 文件中。
- 二维点和二维姿态类型：属于 `core/common/core_types.h`。
- 业务运行时状态：放在明确拥有该状态的上层模块或具体 BSP 驱动中。
