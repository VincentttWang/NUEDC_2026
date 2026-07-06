/**
 * @file  key.c
 * @brief BSP 按键驱动实现。
 */
#include "key.h"
#include "bsp_time.h"

typedef enum {
    KEY_LEVEL_RELEASED = 0,
    KEY_LEVEL_PRESSED
} KEY_LEVEL;

typedef enum {
    KEY_STATE_RELEASED = 0,
    KEY_STATE_PRESS_DEBOUNCE,
    KEY_STATE_PRESSED,
    KEY_STATE_RELEASE_DEBOUNCE,
    KEY_STATE_WAIT_DOUBLE,
    KEY_STATE_DOUBLE_DEBOUNCE
} KEY_STATE;

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    bool active_low;
} KEY_HW_CONFIG;

typedef struct {
    KEY_STATE state;
    KEY_LEVEL stable_level;
    KEY_LEVEL candidate_level;
    uint32_t debounce_start_ms;
    uint32_t press_start_ms;
    uint32_t release_time_ms;
    KEY_EVENT pending_event;
    bool long_reported;
    bool suppress_release_event;
    bool short_release_pending;
    bool short_release_consumed;
} KEY_STATE_CONTEXT;

static const KEY_HW_CONFIG s_key_hw[KEY_ID_MAX] = {
    [KEY_ID_1] = {
        .port = KEY1_PORT,
        .pin = KEY1_PIN,
        .active_low = (KEY1_ACTIVE_LOW != 0U),
    },
};

static KEY_STATE_CONTEXT s_key_state[KEY_ID_MAX];

static bool Key_IsValidId(KEY_ID key_id){
    return key_id < KEY_ID_MAX;
}

static KEY_LEVEL Key_ReadLevel(KEY_ID key_id){
    const KEY_HW_CONFIG *hw = &s_key_hw[key_id];
    bool high_level = (DL_GPIO_readPins(hw->port, hw->pin) != 0U);
    bool pressed = hw->active_low ? !high_level : high_level;

    return pressed ? KEY_LEVEL_PRESSED : KEY_LEVEL_RELEASED;
}

static void Key_PushEvent(KEY_ID key_id, KEY_EVENT event){
    if (event != KEY_EVENT_NONE){
        s_key_state[key_id].pending_event = event;
    }
}

void Key_Init(void){
    for (uint8_t i = 0U; i < (uint8_t)KEY_ID_MAX; i++){
        KEY_ID key_id = (KEY_ID)i;
        KEY_LEVEL level = Key_ReadLevel(key_id);

        s_key_state[i].state = (level == KEY_LEVEL_PRESSED) ? KEY_STATE_PRESSED : KEY_STATE_RELEASED;
        s_key_state[i].stable_level = level;
        s_key_state[i].candidate_level = level;
        s_key_state[i].debounce_start_ms = BSP_Time_GetMs();
        s_key_state[i].press_start_ms = BSP_Time_GetMs();
        s_key_state[i].release_time_ms = BSP_Time_GetMs();
        s_key_state[i].pending_event = KEY_EVENT_NONE;
        s_key_state[i].long_reported = false;
        s_key_state[i].suppress_release_event = false;
        s_key_state[i].short_release_pending = false;
        s_key_state[i].short_release_consumed = false;
    }
}

static void Key_UpdateReleased(KEY_ID key_id, KEY_LEVEL level, uint32_t now_ms){
    KEY_STATE_CONTEXT *state = &s_key_state[key_id];

    if (level == KEY_LEVEL_PRESSED){
        state->candidate_level = level;
        state->debounce_start_ms = now_ms;
        state->state = KEY_STATE_PRESS_DEBOUNCE;
    }
}

static void Key_UpdatePressDebounce(KEY_ID key_id, KEY_LEVEL level, uint32_t now_ms){
    KEY_STATE_CONTEXT *state = &s_key_state[key_id];

    if (level != state->candidate_level){
        state->state = KEY_STATE_RELEASED;
        return;
    }

    if (now_ms - state->debounce_start_ms >= KEY_DEBOUNCE_MS){
        state->stable_level = KEY_LEVEL_PRESSED;
        state->press_start_ms = now_ms;
        state->long_reported = false;
        state->suppress_release_event = false;
        state->state = KEY_STATE_PRESSED;
    }
}

static void Key_UpdatePressed(KEY_ID key_id, KEY_LEVEL level, uint32_t now_ms){
    KEY_STATE_CONTEXT *state = &s_key_state[key_id];

    if (level == KEY_LEVEL_RELEASED){
        state->candidate_level = level;
        state->debounce_start_ms = now_ms;
        state->state = KEY_STATE_RELEASE_DEBOUNCE;
        return;
    }

    if (!state->long_reported && (now_ms - state->press_start_ms >= KEY_LONG_PRESS_MS)){
        state->long_reported = true;
        Key_PushEvent(key_id, KEY_EVENT_LONG_PRESS);
    }
}

static void Key_UpdateReleaseDebounce(KEY_ID key_id, KEY_LEVEL level, uint32_t now_ms){
    KEY_STATE_CONTEXT *state = &s_key_state[key_id];

    if (level != state->candidate_level){
        state->state = KEY_STATE_PRESSED;
        return;
    }

    if (now_ms - state->debounce_start_ms < KEY_DEBOUNCE_MS){
        return;
    }

    state->stable_level = KEY_LEVEL_RELEASED;
    state->release_time_ms = now_ms;

    /* 长按已在按下期间上报，释放时不再生成短按事件。 */
    if (state->long_reported || state->suppress_release_event){
        state->suppress_release_event = false;
        state->state = KEY_STATE_RELEASED;
        return;
    }

    if (now_ms - state->press_start_ms >= KEY_SHORT_PRESS_MIN_MS){
        state->short_release_pending = true;
        state->short_release_consumed = false;
        state->state = KEY_STATE_WAIT_DOUBLE;
    } else{
        state->state = KEY_STATE_RELEASED;
    }
}

static void Key_UpdateWaitDouble(KEY_ID key_id, KEY_LEVEL level, uint32_t now_ms){
    KEY_STATE_CONTEXT *state = &s_key_state[key_id];

    /* 双击窗口超时后，才确认前一次释放对应的是短按。 */
    if (now_ms - state->release_time_ms >= KEY_DOUBLE_CLICK_MS){
        if (!state->short_release_consumed){
            Key_PushEvent(key_id, KEY_EVENT_SHORT_PRESS);
        }
        state->short_release_pending = false;
        state->short_release_consumed = false;
        state->state = KEY_STATE_RELEASED;
        return;
    }

    if (level == KEY_LEVEL_PRESSED){
        state->candidate_level = level;
        state->debounce_start_ms = now_ms;
        state->state = KEY_STATE_DOUBLE_DEBOUNCE;
    }
}

static void Key_UpdateDoubleDebounce(KEY_ID key_id, KEY_LEVEL level, uint32_t now_ms){
    KEY_STATE_CONTEXT *state = &s_key_state[key_id];

    if (level != state->candidate_level){
        state->state = KEY_STATE_WAIT_DOUBLE;
        return;
    }

    if (now_ms - state->debounce_start_ms >= KEY_DEBOUNCE_MS){
        Key_PushEvent(key_id, KEY_EVENT_DOUBLE_CLICK);
        state->short_release_pending = false;
        state->short_release_consumed = false;
        state->stable_level = KEY_LEVEL_PRESSED;
        state->press_start_ms = now_ms;
        state->long_reported = false;
        /* 双击第二次释放不应再生成短按。 */
        state->suppress_release_event = true;
        state->state = KEY_STATE_PRESSED;
    }
}

static void Key_Update(KEY_ID key_id){
    KEY_STATE_CONTEXT *state = &s_key_state[key_id];
    KEY_LEVEL level = Key_ReadLevel(key_id);
    uint32_t now_ms = BSP_Time_GetMs();

    switch (state->state){
        case KEY_STATE_RELEASED:
            Key_UpdateReleased(key_id, level, now_ms);
            break;
        case KEY_STATE_PRESS_DEBOUNCE:
            Key_UpdatePressDebounce(key_id, level, now_ms);
            break;
        case KEY_STATE_PRESSED:
            Key_UpdatePressed(key_id, level, now_ms);
            break;
        case KEY_STATE_RELEASE_DEBOUNCE:
            Key_UpdateReleaseDebounce(key_id, level, now_ms);
            break;
        case KEY_STATE_WAIT_DOUBLE:
            Key_UpdateWaitDouble(key_id, level, now_ms);
            break;
        case KEY_STATE_DOUBLE_DEBOUNCE:
            Key_UpdateDoubleDebounce(key_id, level, now_ms);
            break;
        default:
            state->state = KEY_STATE_RELEASED;
            state->stable_level = KEY_LEVEL_RELEASED;
            break;
    }
}

void Key_Scan(void){
    for (uint8_t i = 0U; i < (uint8_t)KEY_ID_MAX; i++){
        Key_Update((KEY_ID)i);
    }
}

KEY_EVENT Key_GetEvent(KEY_ID key_id){
    if (!Key_IsValidId(key_id)){
        return KEY_EVENT_NONE;
    }

    KEY_EVENT event = s_key_state[key_id].pending_event;
    s_key_state[key_id].pending_event = KEY_EVENT_NONE;
    return event;
}

bool Key_IsPressed(KEY_ID key_id){
    if (!Key_IsValidId(key_id)){
        return false;
    }

    return s_key_state[key_id].stable_level == KEY_LEVEL_PRESSED;
}

static bool Key_ConsumeIf(KEY_ID key_id, KEY_EVENT expected_event){
    if (!Key_IsValidId(key_id)){
        return false;
    }

    if (s_key_state[key_id].pending_event != expected_event){
        return false;
    }

    s_key_state[key_id].pending_event = KEY_EVENT_NONE;
    return true;
}

bool Key_IsShortPress(KEY_ID key_id){
    return Key_ConsumeIf(key_id, KEY_EVENT_SHORT_PRESS);
}

bool Key_IsShortRelease(KEY_ID key_id){
    if (!Key_IsValidId(key_id)){
        return false;
    }

    if (!s_key_state[key_id].short_release_pending){
        return false;
    }

    s_key_state[key_id].short_release_pending = false;
    s_key_state[key_id].short_release_consumed = true;
    return true;
}

bool Key_IsLongPress(KEY_ID key_id){
    return Key_ConsumeIf(key_id, KEY_EVENT_LONG_PRESS);
}

bool Key_IsDoubleClick(KEY_ID key_id){
    return Key_ConsumeIf(key_id, KEY_EVENT_DOUBLE_CLICK);
}

void Key_ClearEvent(KEY_ID key_id){
    if (Key_IsValidId(key_id)){
        s_key_state[key_id].pending_event = KEY_EVENT_NONE;
        s_key_state[key_id].short_release_pending = false;
        s_key_state[key_id].short_release_consumed = false;
    }
}

void Key_ClearAllEvents(void){
    for (uint8_t i = 0U; i < (uint8_t)KEY_ID_MAX; i++){
        s_key_state[i].pending_event = KEY_EVENT_NONE;
        s_key_state[i].short_release_pending = false;
        s_key_state[i].short_release_consumed = false;
    }
}
