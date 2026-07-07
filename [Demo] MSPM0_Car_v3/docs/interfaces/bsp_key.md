# bsp/key 接口说明

## 模块职责

`bsp/key` 负责板级按键读取、消抖和事件生成。当前按天猛星 v3 拓展板实现 4 个低电平有效按键：

- `KEY_ID_UP`：K1 Up，PA14
- `KEY_ID_CALIB`：K2 校准，PB22
- `KEY_ID_ENTER`：K3 Enter，PB24
- `KEY_ID_DOWN`：K4 Down，PB25

该模块属于 BSP 层，只依赖：

- `board/sys_config/ti_msp_dl_config.h` 中的 GPIO 配置宏
- `bsp/time` 提供的毫秒时间源
- TI DriverLib 的 GPIO 读取接口

## 硬件映射宏

默认映射到 SysConfig 生成的 `Key_PIN_1` 到 `Key_PIN_4` 引脚：

```c
#ifndef KEY1_PORT
#define KEY1_PORT Key_PIN_1_PORT
#endif

#ifndef KEY1_PIN
#define KEY1_PIN Key_PIN_1_PIN
#endif

#ifndef KEY1_ACTIVE_LOW
#define KEY1_ACTIVE_LOW 1U
#endif

#define KEY2_PORT       Key_PIN_2_PORT
#define KEY2_PIN        Key_PIN_2_PIN
#define KEY2_ACTIVE_LOW 1U

#define KEY3_PORT       Key_PIN_3_PORT
#define KEY3_PIN        Key_PIN_3_PIN
#define KEY3_ACTIVE_LOW 1U

#define KEY4_PORT       Key_PIN_4_PORT
#define KEY4_PIN        Key_PIN_4_PIN
#define KEY4_ACTIVE_LOW 1U
```

如果后续改板，可以在编译配置中覆盖这些宏，或调整 `key.h` 中的默认映射。

## 时间参数宏

```c
#define KEY_DEBOUNCE_MS          20U
#define KEY_SHORT_PRESS_MIN_MS   50U
#define KEY_LONG_PRESS_MS        1000U
#define KEY_DOUBLE_CLICK_MS      300U
```

这些宏允许在编译期覆盖。当前不提供运行时配置接口，避免 BSP 驱动保存额外业务配置。

## 公开类型

### `KEY_ID`

```c
typedef enum {
    KEY_ID_UP = 0,
    KEY_ID_CALIB,
    KEY_ID_ENTER,
    KEY_ID_DOWN,
    KEY_ID_MAX,
    KEY_ID_1 = KEY_ID_ENTER
} KEY_ID;
```

### `KEY_EVENT`

```c
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT_PRESS,
    KEY_EVENT_LONG_PRESS,
    KEY_EVENT_DOUBLE_CLICK
} KEY_EVENT;
```

事件类型仅描述稳定事件，不导出按下、释放、消抖等内部状态。

## 公开接口

### `void Key_Init(void)`

初始化按键内部状态。GPIO 初始化仍由 SysConfig 生成代码完成。

### `void Key_Scan(void)`

扫描并推进按键状态机。当前由 `SysTick_Handler()` 每 10 ms 调用。

### `KEY_EVENT Key_GetEvent(KEY_ID key_id)`

读取并消费指定按键的待处理事件。无事件或 `key_id` 非法时返回 `KEY_EVENT_NONE`。

### `bool Key_IsPressed(KEY_ID key_id)`

返回指定按键的稳定按下状态。

### `bool Key_IsShortPress(KEY_ID key_id)`

仅当待处理事件为短按时返回 `true`，并消费该事件；如果当前是长按或双击事件，不会误清除。

普通短按需要等待 `KEY_DOUBLE_CLICK_MS` 双击窗口超时后才会生成，因此它适合用于需要同时区分单击和双击的菜单逻辑。

### `bool Key_IsShortRelease(KEY_ID key_id)`

短按释放消抖完成后立即返回 `true`，不等待双击窗口超时。该接口被消费后，本次点击后续不会再生成普通短按事件。

该接口主要用于需要低延迟响应的测试动作，例如 `Device check` 中的 yaw/pitch 步进电机点动测试。使用时需要注意：它无法在触发前等待判断用户是否还会继续第二次点击，因此不适合替代普通菜单中的单击确认逻辑。

### `bool Key_IsLongPress(KEY_ID key_id)`

仅当待处理事件为长按时返回 `true`，并消费该事件。

### `bool Key_IsDoubleClick(KEY_ID key_id)`

仅当待处理事件为双击时返回 `true`，并消费该事件。

### `void Key_ClearEvent(KEY_ID key_id)`

清除指定按键的待处理事件。

### `void Key_ClearAllEvents(void)`

清除所有按键的待处理事件。

## 兼容宏

为了降低本轮重写对上层应用的影响，保留单按键便捷宏：

```c
#define Key_Read()         Key_IsPressed(KEY_ID_1)
#define Key_short_press()  Key_IsShortPress(KEY_ID_1)
#define Key_long_press()   Key_IsLongPress(KEY_ID_1)
#define Key_double_click() Key_IsDoubleClick(KEY_ID_1)
```

其中 `KEY_ID_1` 兼容映射到 `KEY_ID_ENTER`。后续如果上层流程统一改用语义化 `KEY_ID_*`，可以再考虑移除这些兼容宏。
