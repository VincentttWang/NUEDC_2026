# middleware/fault 接口说明

## 模块职责

`middleware/fault` 负责系统故障处理。它保存最近一次故障信息，并在故障停机时尽量让系统进入安全状态。

该模块负责：

- 保存结构化故障码和故障消息。
- 故障时刹车底盘。
- 故障时通过 `middleware/ui` 显示错误页。
- 故障时进入停机循环。

该模块不负责：

- 判断业务是否发生故障。
- 菜单、任务流程或状态机逻辑。
- 日志持久化。
- 故障恢复策略。

## 公开类型

### `SYSTEM_FAULT_MESSAGE_LEN`

```c
#define SYSTEM_FAULT_MESSAGE_LEN 64U
```

故障消息缓存长度，包含字符串结尾 `\0`。

### `SYSTEM_FAULT_CODE`

```c
typedef enum {
    SYSTEM_FAULT_NONE = 0,
    SYSTEM_FAULT_UNKNOWN,
    SYSTEM_FAULT_INVALID_ARG,
    SYSTEM_FAULT_ALLOC_FAILED,
    SYSTEM_FAULT_STATE_ERROR,
    SYSTEM_FAULT_HARDWARE
} SYSTEM_FAULT_CODE;
```

故障码说明：

- `SYSTEM_FAULT_NONE`：无故障。
- `SYSTEM_FAULT_UNKNOWN`：未分类故障。
- `SYSTEM_FAULT_INVALID_ARG`：参数非法。
- `SYSTEM_FAULT_ALLOC_FAILED`：静态池或节点分配失败。
- `SYSTEM_FAULT_STATE_ERROR`：状态机或流程状态异常。
- `SYSTEM_FAULT_HARDWARE`：硬件或外设故障。

### `SYSTEM_FAULT_INFO`

```c
typedef struct {
    SYSTEM_FAULT_CODE code;
    char message[SYSTEM_FAULT_MESSAGE_LEN];
} SYSTEM_FAULT_INFO;
```

保存最近一次故障信息。

## 公开接口

### `BSP_STATUS SystemFault_Set(SYSTEM_FAULT_CODE code, const char *message)`

记录故障信息但不停机。`message == NULL` 时记录默认消息 `"Fault"`。

### `BSP_STATUS SystemFault_Get(SYSTEM_FAULT_INFO *out)`

读取最近一次故障信息。`out == NULL` 时返回 `BSP_STATUS_NULL`。

### `SYSTEM_FAULT_CODE SystemFault_GetCode(void)`

返回最近一次故障码。

### `const char *SystemFault_GetMessage(void)`

返回最近一次故障消息的只读指针。

### `void SystemFault_Clear(void)`

清除最近一次故障信息。

### `void SystemFault_Handler(SYSTEM_FAULT_CODE code, const char *message)`

记录故障信息并进入停机处理。内部调用：

- `SystemFault_Set()`
- `SystemFault_Halt()`

### `void SystemFault_Halt(void)`

使用当前故障信息进入停机处理。内部执行：

1. `Chassis_Brake()`
2. `Ui_RenderStatusPage("System Fault", UI_STATUS_ERROR, message, code_text)`
3. `while (1)`

故障处理阶段会忽略底盘刹车接口的返回值，因为此时只能尽力进入安全状态。

## 迁移说明

本轮重写不保留旧接口：

- `error_message`
- `error_handler()`

后续上层应迁移为直接调用：

```c
SystemFault_Handler(SYSTEM_FAULT_ALLOC_FAILED, "No free tree nodes");
```
