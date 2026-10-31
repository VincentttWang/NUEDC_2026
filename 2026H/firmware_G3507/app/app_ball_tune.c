/**
 * @file  app_ball_tune.c
 * @brief 滚球控制参数串口热更新的实现。
 */
#include "app_ball_tune.h"

#include "bsp_time.h"
#include "debug_uart.h"

#include <stddef.h>

/* 参数表上限。当前 ball_scurve 可调项已超过 56 个，保留后续扩展余量。 */
#define APP_BALL_TUNE_MAX_ENTRIES 64U
/* 单条命令的最大长度。超长直接丢弃整行，不做截断执行——半条命令比不执行危险。 */
#define APP_BALL_TUNE_LINE_MAX 64U
/* 单拍最多从 RX 取多少字节。限制它是为了不让粘贴的大段文本撑爆控制回调。 */
#define APP_BALL_TUNE_RX_CHUNK 32U
/* 2KB TX 缓冲在 115200 baud 下约 180ms 排空；开始列表前留出完整排空窗口。 */
#define APP_BALL_TUNE_LIST_START_DELAY_MS 200U

static const APP_BALL_TUNE_ENTRY *tune_table;
static uint8_t tune_count;
/* 进入任务时的快照，供 `d` 命令恢复。 */
static float tune_defaults[APP_BALL_TUNE_MAX_ENTRIES];

static char tune_line[APP_BALL_TUNE_LINE_MAX];
static uint8_t tune_len;
/* 本行已超长：继续吃字节直到换行，但整行作废。 */
static bool tune_overflow;
/* 完整列表必须分拍回显；56 行约 4KB，不能一次塞进 2KB Debug UART TX 环形缓冲。 */
static bool tune_list_active;
static uint8_t tune_list_index;
static uint32_t tune_list_start_ms;

/* ===== 小工具：不依赖 sscanf(%f)，省代码空间也更可控 ===== */

static char ToLower(char c){
    return ((c >= 'A') && (c <= 'Z')) ? (char)(c - 'A' + 'a') : c;
}

static bool IsSpace(char c){
    return (c == ' ') || (c == '\t');
}

/** 比较到词尾（空格或结束符）。 */
static bool TokenEquals(const char *token, const char *name){
    uint8_t i = 0U;
    while ((name[i] != '\0') && (token[i] != '\0') && !IsSpace(token[i])){
        if (ToLower(token[i]) != name[i]){ return false; }
        i++;
    }
    return (name[i] == '\0') && ((token[i] == '\0') || IsSpace(token[i]));
}

/** 跳过空白，返回下一个非空白字符位置；无则返回 NULL。 */
static const char *SkipSpace(const char *s){
    while ((*s != '\0') && IsSpace(*s)){ s++; }
    return (*s == '\0') ? NULL : s;
}

/** 跳过当前词再跳空白。 */
static const char *NextToken(const char *s){
    while ((*s != '\0') && !IsSpace(*s)){ s++; }
    return SkipSpace(s);
}

/**
 * @brief 解析十进制浮点，支持前导符号、小数点、可选指数。
 * @return 解析成功为 true；整个词必须都是合法数字字符。
 *
 * 不用 sscanf("%f")：它在 Cortex-M0+ 上会拉进一大块浮点格式化代码，
 * 而且对 "0.05abc" 这类输入会静默接受前缀——热更参数不能容忍静默。
 */
static bool ParseFloat(const char *s, float *out){
    bool negative = false;
    if ((*s == '+') || (*s == '-')){
        negative = (*s == '-');
        s++;
    }
    if ((*s == '\0') || IsSpace(*s)){ return false; }

    float value = 0.0f;
    bool has_digit = false;
    while ((*s >= '0') && (*s <= '9')){
        value = value * 10.0f + (float)(*s - '0');
        has_digit = true;
        s++;
    }
    if (*s == '.'){
        s++;
        float scale = 0.1f;
        while ((*s >= '0') && (*s <= '9')){
            value += scale * (float)(*s - '0');
            scale *= 0.1f;
            has_digit = true;
            s++;
        }
    }
    if (!has_digit){ return false; }

    if ((*s == 'e') || (*s == 'E')){
        s++;
        bool exponent_negative = false;
        if ((*s == '+') || (*s == '-')){
            exponent_negative = (*s == '-');
            s++;
        }
        int exponent = 0;
        bool has_exponent_digit = false;
        while ((*s >= '0') && (*s <= '9')){
            exponent = exponent * 10 + (int)(*s - '0');
            has_exponent_digit = true;
            s++;
        }
        if (!has_exponent_digit || (exponent > 30)){ return false; }
        for (int i = 0; i < exponent; i++){
            value = exponent_negative ? (value * 0.1f) : (value * 10.0f);
        }
    }

    /* 词必须干净结束，"0.05abc" 一律拒绝。 */
    if ((*s != '\0') && !IsSpace(*s)){ return false; }

    *out = negative ? -value : value;
    return true;
}

/* ===== 取值 / 设值 ===== */

static float EntryGet(const APP_BALL_TUNE_ENTRY *entry){
    if (entry->value != NULL){ return *entry->value; }
    return (entry->get != NULL) ? entry->get() : 0.0f;
}

static bool EntrySet(const APP_BALL_TUNE_ENTRY *entry, float value){
    if (entry->value != NULL){
        *entry->value = value;
        /* 有的项既在配置里、又需要通知驱动（例如步进增益）。 */
        if (entry->set != NULL){ return entry->set(value); }
        return true;
    }
    return (entry->set != NULL) ? entry->set(value) : false;
}

static const APP_BALL_TUNE_ENTRY *Find(const char *token, uint8_t *index){
    for (uint8_t i = 0U; i < tune_count; i++){
        if (TokenEquals(token, tune_table[i].name)){
            if (index != NULL){ *index = i; }
            return &tune_table[i];
        }
    }
    return NULL;
}

static void PrintEntry(const APP_BALL_TUNE_ENTRY *entry){
    DebugUart_Printf("[TUNE] p name=%s val=%.6f min=%.6f max=%.6f unit=%s\r\n",
                     entry->name, (double)EntryGet(entry),
                     (double)entry->min_value, (double)entry->max_value,
                     (entry->unit != NULL) ? entry->unit : "-");
}

void AppBallTune_PrintAll(void){
    tune_list_index = 0U;
    tune_list_start_ms = BSP_Time_GetMs();
    tune_list_active = (tune_table != NULL) && (tune_count > 0U);
}

bool AppBallTune_IsListing(void){
    return tune_list_active;
}

/** 每个控制拍最多回显一项；列表期间 H3 会暂停大行遥测。 */
static void ServiceList(void){
    if (!tune_list_active){ return; }
    if ((uint32_t)(BSP_Time_GetMs() - tune_list_start_ms) <
        APP_BALL_TUNE_LIST_START_DELAY_MS){
        return;
    }

    if (tune_list_index < tune_count){
        PrintEntry(&tune_table[tune_list_index]);
        tune_list_index++;
        return;
    }

    DebugUart_Printf("[TUNE] end n=%u\r\n", (unsigned)tune_count);
    tune_list_active = false;
}

void AppBallTune_Init(const APP_BALL_TUNE_ENTRY *table, uint8_t count){
    tune_table = table;
    tune_count = (count > APP_BALL_TUNE_MAX_ENTRIES)
                     ? (uint8_t)APP_BALL_TUNE_MAX_ENTRIES : count;
    for (uint8_t i = 0U; i < tune_count; i++){
        tune_defaults[i] = EntryGet(&tune_table[i]);
    }
    tune_len = 0U;
    tune_overflow = false;
    tune_list_active = false;
    tune_list_index = 0U;
    tune_list_start_ms = 0U;
}

/* ===== 命令执行 ===== */

static void HandleSet(const char *name_token){
    uint8_t index = 0U;
    const APP_BALL_TUNE_ENTRY *entry = Find(name_token, &index);
    if (entry == NULL){
        DebugUart_Printf("[TUNE] err reason=unknown\r\n");
        return;
    }
    const char *value_token = NextToken(name_token);
    float value = 0.0f;
    if ((value_token == NULL) || !ParseFloat(value_token, &value)){
        DebugUart_Printf("[TUNE] err name=%s reason=parse\r\n", entry->name);
        return;
    }
    /*
     * 越界**拒绝**而不是钳位。钳位会让操作者以为已经设进去了，
     * 而实际生效的是另一个值——整定时这种误解代价很高。
     */
    if ((value < entry->min_value) || (value > entry->max_value)){
        DebugUart_Printf("[TUNE] err name=%s reason=range min=%.6f max=%.6f\r\n",
                         entry->name, (double)entry->min_value,
                         (double)entry->max_value);
        return;
    }
    float old_value = EntryGet(entry);
    if (!EntrySet(entry, value)){
        DebugUart_Printf("[TUNE] err name=%s reason=reject\r\n", entry->name);
        return;
    }
    DebugUart_Printf("[TUNE] ok name=%s val=%.6f old=%.6f\r\n",
                     entry->name, (double)EntryGet(entry), (double)old_value);
}

static void HandleGet(const char *name_token){
    const APP_BALL_TUNE_ENTRY *entry = Find(name_token, NULL);
    if (entry == NULL){
        DebugUart_Printf("[TUNE] err reason=unknown\r\n");
        return;
    }
    PrintEntry(entry);
}

static void HandleDefault(void){
    for (uint8_t i = 0U; i < tune_count; i++){
        (void)EntrySet(&tune_table[i], tune_defaults[i]);
    }
    DebugUart_Printf("[TUNE] ok reason=default n=%u\r\n", (unsigned)tune_count);
}

static void Execute(char *line){
    const char *token = SkipSpace(line);
    if (token == NULL){ return; }

    if ((token[0] == '?') || TokenEquals(token, "list")){
        AppBallTune_PrintAll();
        return;
    }
    if (TokenEquals(token, "s") || TokenEquals(token, "set")){
        const char *name = NextToken(token);
        if (name == NULL){
            DebugUart_Printf("[TUNE] err reason=parse\r\n");
            return;
        }
        HandleSet(name);
        return;
    }
    if (TokenEquals(token, "g") || TokenEquals(token, "get")){
        const char *name = NextToken(token);
        if (name == NULL){
            DebugUart_Printf("[TUNE] err reason=parse\r\n");
            return;
        }
        HandleGet(name);
        return;
    }
    if (TokenEquals(token, "d") || TokenEquals(token, "default")){
        HandleDefault();
        return;
    }
    DebugUart_Printf("[TUNE] err reason=command\r\n");
}

void AppBallTune_Poll(void){
    if ((tune_table == NULL) || (tune_count == 0U)){ return; }

    uint8_t chunk[APP_BALL_TUNE_RX_CHUNK];
    uint16_t got = DebugUart_Read(chunk, (uint16_t)sizeof(chunk));
    for (uint16_t i = 0U; i < got; i++){
        char c = (char)chunk[i];
        if ((c == '\n') || (c == '\r')){
            if (tune_overflow){
                DebugUart_Printf("[TUNE] err reason=toolong\r\n");
            } else if (tune_len > 0U){
                tune_line[tune_len] = '\0';
                Execute(tune_line);
            }
            tune_len = 0U;
            tune_overflow = false;
            continue;
        }
        if (tune_len < (APP_BALL_TUNE_LINE_MAX - 1U)){
            tune_line[tune_len] = c;
            tune_len++;
        } else{
            /* 超长整行作废：执行半条命令比不执行更危险。 */
            tune_overflow = true;
        }
    }

    ServiceList();
}
