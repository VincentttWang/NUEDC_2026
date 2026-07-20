/**
 * @file  canmv_uart.c
 * @brief BSP CanMV UART 协议解析实现。
 */
#include "canmv_uart.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef struct {
    uint8_t begin;
    uint8_t byte_count;
} CANMV_TARGET_PARSE_CONFIG;

static const CANMV_TARGET_PARSE_CONFIG s_canmv_parse_config[CANMV_TARGET_MAX] = {
    [CANMV_TARGET_LASER] = {
        .begin = CANMV_LASER_BEGIN,
        .byte_count = CANMV_LASER_BYTE_COUNT,
    },
    [CANMV_TARGET_RECT] = {
        .begin = CANMV_RECT_BEGIN,
        .byte_count = CANMV_RECT_BYTE_COUNT,
    },
};

static uint8_t s_canmv_rx_buffer[CANMV_RX_BUFFER_LEN];
static uint8_t s_canmv_frame[CANMV_RX_BUFFER_LEN];
static uint8_t s_canmv_rx_count;
static bool s_canmv_receiving;
static bool s_canmv_target_seen[CANMV_TARGET_MAX];
static CANMV_TARGET_DATA s_canmv_target_data[CANMV_TARGET_MAX];

volatile uint32_t g_canmv_uart_rx_byte_count;
volatile uint32_t g_canmv_uart_valid_frame_count;
volatile uint32_t g_canmv_uart_drop_count;
volatile uint8_t g_canmv_uart_last_byte;

static bool CanMvUart_IsValidTarget(CANMV_TARGET target){
    return target < CANMV_TARGET_MAX;
}

static bool CanMvUart_CombineCoordinate(uint8_t high, uint8_t low, uint16_t *out){
    if (out == NULL){
        return false;
    }

    uint16_t big_endian = (uint16_t)(((uint16_t)high << 8) | low);
    uint16_t little_endian = (uint16_t)(((uint16_t)low << 8) | high);

    /*
     * 当前 K230 视觉坐标应落在图像尺寸附近。若按旧协议大端解析得到明显
     * 异常的五位数，而交换字节后落在合理范围，则兼容 K230 端小端发送。
     */
    if ((big_endian > CANMV_COORDINATE_SANITY_LIMIT) &&
        (little_endian <= CANMV_COORDINATE_SANITY_LIMIT)){
        *out = little_endian;
        return true;
    }

    if (big_endian <= CANMV_COORDINATE_SANITY_LIMIT){
        *out = big_endian;
        return true;
    }

    return false;
}

static bool CanMvUart_IsTargetEmpty(const CANMV_TARGET_DATA *data){
    for (uint8_t i = 0U; i < data->count; i++){
        if (data->value[i] != 0U){
            return false;
        }
    }

    return true;
}

static void CanMvUart_SetAllStatus(CANMV_STATUS status){
    if (status == CANMV_STATUS_FRAME_DROP){
        g_canmv_uart_drop_count++;
    }

    for (uint8_t i = 0U; i < (uint8_t)CANMV_TARGET_MAX; i++){
        s_canmv_target_data[i].status = status;
    }
}

static void CanMvUart_ClearTargetData(void){
    for (uint8_t i = 0U; i < (uint8_t)CANMV_TARGET_MAX; i++){
        memset(s_canmv_target_data[i].value, 0, sizeof(s_canmv_target_data[i].value));
        s_canmv_target_data[i].count = 0U;
        s_canmv_target_data[i].status = CANMV_STATUS_INIT;
        s_canmv_target_seen[i] = false;
    }
}

static void CanMvUart_ResetRxState(void){
    memset(s_canmv_rx_buffer, 0, sizeof(s_canmv_rx_buffer));
    s_canmv_rx_count = 0U;
    s_canmv_receiving = false;
}

static void CanMvUart_UpdateTargetStatus(CANMV_TARGET target){
    CANMV_TARGET_DATA *data = &s_canmv_target_data[target];

    if (CanMvUart_IsTargetEmpty(data)){
        data->status = s_canmv_target_seen[target] ? CANMV_STATUS_LOST : CANMV_STATUS_NOT_FOUND;
        return;
    }

    data->status = CANMV_STATUS_OK;
    s_canmv_target_seen[target] = true;
}

static bool CanMvUart_ParseTarget(CANMV_TARGET target){
    const CANMV_TARGET_PARSE_CONFIG *config = &s_canmv_parse_config[target];
    CANMV_TARGET_DATA *data = &s_canmv_target_data[target];
    uint8_t value_count = (uint8_t)(config->byte_count / 2U);

    if (value_count > CANMV_TARGET_VALUE_CAPACITY){
        value_count = CANMV_TARGET_VALUE_CAPACITY;
    }

    memset(data->value, 0, sizeof(data->value));
    data->count = value_count;

    for (uint8_t i = 0U; i < value_count; i++){
        uint8_t frame_index = (uint8_t)(config->begin + (i * 2U));
        if (!CanMvUart_CombineCoordinate(s_canmv_frame[frame_index],
                                         s_canmv_frame[frame_index + 1U],
                                         &data->value[i])){
            memset(data->value, 0, sizeof(data->value));
            data->count = 0U;
            data->status = CANMV_STATUS_FRAME_DROP;
            return false;
        }
    }

    CanMvUart_UpdateTargetStatus(target);
    return true;
}

static void CanMvUart_ParseFrame(void){
    for (uint8_t i = 0U; i < (uint8_t)CANMV_TARGET_MAX; i++){
        if (!CanMvUart_ParseTarget((CANMV_TARGET)i)){
            CanMvUart_SetAllStatus(CANMV_STATUS_FRAME_DROP);
            return;
        }
    }
}

BSP_STATUS CanMvUart_Init(void){
    CanMvUart_ResetRxState();
    memset(s_canmv_frame, 0, sizeof(s_canmv_frame));
    CanMvUart_ClearTargetData();
    g_canmv_uart_rx_byte_count = 0U;
    g_canmv_uart_valid_frame_count = 0U;
    g_canmv_uart_drop_count = 0U;
    g_canmv_uart_last_byte = 0U;

    DL_UART_Main_setRXFIFOThreshold(CANMV_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setRXInterruptTimeout(CANMV_UART_INST, 1U);
    DL_UART_Main_disableInterrupt(CANMV_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    DL_UART_Main_enableInterrupt(CANMV_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    NVIC_ClearPendingIRQ(CANMV_UART_IRQN);
    NVIC_EnableIRQ(CANMV_UART_IRQN);
    return BSP_STATUS_OK;
}

void CanMvUart_ProcessByte(uint8_t byte){
    g_canmv_uart_rx_byte_count++;
    g_canmv_uart_last_byte = byte;

    if (byte == CANMV_FRAME_START){
        /* 新帧头到来时立即重新同步，丢弃此前可能残缺的帧内容。 */
        CanMvUart_ResetRxState();
        s_canmv_receiving = true;
    }

    if (!s_canmv_receiving){
        /* 未见帧头前收到的字节不参与解析，标记为丢帧便于上层诊断。 */
        CanMvUart_SetAllStatus(CANMV_STATUS_FRAME_DROP);
        return;
    }

    if (s_canmv_rx_count >= CANMV_RX_BUFFER_LEN){
        /* 缓冲区满仍未等到帧尾，说明协议已经失步。 */
        CanMvUart_ResetRxState();
        CanMvUart_SetAllStatus(CANMV_STATUS_FRAME_DROP);
        return;
    }

    s_canmv_rx_buffer[s_canmv_rx_count++] = byte;

    if (byte != CANMV_FRAME_END){
        return;
    }

    if (s_canmv_rx_count != CANMV_MIN_FRAME_LEN){
        /* 固定帧必须长度精确匹配，避免把随机字节中的 0x12/0x5B 误判成有效帧。 */
        CanMvUart_ResetRxState();
        CanMvUart_SetAllStatus(CANMV_STATUS_FRAME_DROP);
        return;
    }

    /* 解析前复制完整帧，随后复位接收状态，保证下一帧可以马上开始接收。 */
    memcpy(s_canmv_frame, s_canmv_rx_buffer, sizeof(s_canmv_frame));
    CanMvUart_ResetRxState();
    g_canmv_uart_valid_frame_count++;
    CanMvUart_ParseFrame();
}

void CanMvUart_ProcessRx(void){
    while (!DL_UART_Main_isRXFIFOEmpty(CANMV_UART_INST)){
        uint8_t byte = DL_UART_Main_receiveData(CANMV_UART_INST);
        CanMvUart_ProcessByte(byte);
    }
}

BSP_STATUS CanMvUart_SendByte(uint8_t byte){
    while (DL_UART_isBusy(CANMV_UART_INST)){
        ;
    }

    DL_UART_Main_transmitData(CANMV_UART_INST, byte);
    return BSP_STATUS_OK;
}

BSP_STATUS CanMvUart_SendString(const char *str){
    if (str == NULL){
        return BSP_STATUS_NULL;
    }

    while (*str != '\0'){
        BSP_STATUS status = CanMvUart_SendByte((uint8_t)*str);

        if (status != BSP_STATUS_OK){
            return status;
        }

        str++;
    }

    return BSP_STATUS_OK;
}

CANMV_STATUS CanMvUart_GetStatus(CANMV_TARGET target){
    if (!CanMvUart_IsValidTarget(target)){
        return CANMV_STATUS_INIT;
    }

    return s_canmv_target_data[target].status;
}

uint8_t CanMvUart_GetData(CANMV_TARGET target, uint16_t *out, uint8_t max_count){
    if (!CanMvUart_IsValidTarget(target) || out == NULL || max_count == 0U){
        return 0U;
    }

    const CANMV_TARGET_DATA *data = &s_canmv_target_data[target];
    uint8_t copy_count = data->count;

    if (copy_count > max_count){
        copy_count = max_count;
    }

    for (uint8_t i = 0U; i < copy_count; i++){
        out[i] = data->value[i];
    }

    return copy_count;
}

const CANMV_TARGET_DATA *CanMvUart_GetTargetData(CANMV_TARGET target){
    if (!CanMvUart_IsValidTarget(target)){
        return NULL;
    }

    return &s_canmv_target_data[target];
}
