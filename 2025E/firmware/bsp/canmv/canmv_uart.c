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
static uint8_t s_canmv_expected_len;
static bool s_canmv_receiving_angle;
static bool s_canmv_target_seen[CANMV_TARGET_MAX];
static CANMV_TARGET_DATA s_canmv_target_data[CANMV_TARGET_MAX];

volatile uint32_t g_canmv_uart_rx_byte_count;
volatile uint32_t g_canmv_uart_valid_frame_count;
volatile uint32_t g_canmv_uart_angle_frame_count;
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
    s_canmv_expected_len = 0U;
    s_canmv_receiving_angle = false;
}

static void CanMvUart_ParseAngleFrame(void){
    CANMV_TARGET_DATA *data = &s_canmv_target_data[CANMV_TARGET_ANGLE];
    uint8_t checksum = 0U;
    uint8_t vision_status;

    for (uint8_t i = 0U; i < (CANMV_ANGLE_FRAME_LEN - 1U); i++){
        checksum = (uint8_t)(checksum + s_canmv_rx_buffer[i]);
    }
    if (checksum != s_canmv_rx_buffer[CANMV_ANGLE_FRAME_LEN - 1U]){
        data->status = CANMV_STATUS_FRAME_DROP;
        g_canmv_uart_drop_count++;
        return;
    }

    vision_status = s_canmv_rx_buffer[2];
    if (vision_status > CANMV_ANGLE_STATUS_LOST){
        data->status = CANMV_STATUS_FRAME_DROP;
        g_canmv_uart_drop_count++;
        return;
    }

    data->value[0] = (uint16_t)(((uint16_t)s_canmv_rx_buffer[4] << 8) |
                                s_canmv_rx_buffer[5]);
    data->value[1] = (uint16_t)(((uint16_t)s_canmv_rx_buffer[6] << 8) |
                                s_canmv_rx_buffer[7]);
    data->count = 2U;
    if (vision_status == CANMV_ANGLE_STATUS_VALID){
        data->status = CANMV_STATUS_OK;
        s_canmv_target_seen[CANMV_TARGET_ANGLE] = true;
    } else if ((vision_status == CANMV_ANGLE_STATUS_LOST) ||
               s_canmv_target_seen[CANMV_TARGET_ANGLE]){
        data->status = CANMV_STATUS_LOST;
    } else{
        data->status = CANMV_STATUS_NOT_FOUND;
    }
    g_canmv_uart_angle_frame_count++;
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
    /* 27 字节坐标帧只携带 laser/rect；角度帧由专用解析器处理。 */
    for (uint8_t i = 0U; i < (uint8_t)CANMV_TARGET_ANGLE; i++){
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
    g_canmv_uart_angle_frame_count = 0U;
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

    /*
     * 支持两种定长帧：坐标帧 0x12...0x5B（27 字节）和角度帧
     * 0xA5 0x5A status seq yaw_i16 pitch_i16 checksum（9 字节）。
     * 关键点 —— 进入接收态后按长度采集, **中途不再把载荷里的 0x12/0x5B 当作帧
     * 边界**。旧实现见到任意 0x12 就重同步, 只要坐标/角点字节里出现 0x12/0x5B
     * 就会把整帧打散, 导致含这些字节的目标(尤其静止靶重复发同一帧)永远收不到。
     * 对齐后每帧「满 27 字节->校验帧尾 0x5B->复位」自然衔接下一帧, 保持对齐。
     */
    if (!s_canmv_receiving){
        /* 空闲态: 识别两种帧头；其余为帧间噪声。 */
        if (byte == CANMV_FRAME_START){
            s_canmv_rx_count = 0U;
            s_canmv_rx_buffer[s_canmv_rx_count++] = byte;
            s_canmv_receiving = true;
            s_canmv_expected_len = CANMV_MIN_FRAME_LEN;
        } else if (byte == CANMV_ANGLE_FRAME_START0){
            s_canmv_rx_count = 0U;
            s_canmv_rx_buffer[s_canmv_rx_count++] = byte;
            s_canmv_receiving = true;
            s_canmv_receiving_angle = true;
        }
        return;
    }

    if (s_canmv_receiving_angle && (s_canmv_rx_count == 1U)){
        if (byte == CANMV_ANGLE_FRAME_START1){
            s_canmv_rx_buffer[s_canmv_rx_count++] = byte;
            s_canmv_expected_len = CANMV_ANGLE_FRAME_LEN;
            return;
        }
        CanMvUart_ResetRxState();
        if (byte == CANMV_FRAME_START){
            CanMvUart_ProcessByte(byte);
        }
        return;
    }

    /* 接收态: 载荷字节(含 0x12/0x5B)一律按数据存入。 */
    s_canmv_rx_buffer[s_canmv_rx_count++] = byte;

    if (s_canmv_rx_count < s_canmv_expected_len){
        return;   /* 帧未满, 继续接收 */
    }

    if (s_canmv_receiving_angle){
        CanMvUart_ParseAngleFrame();
        CanMvUart_ResetRxState();
        g_canmv_uart_valid_frame_count++;
    } else if (s_canmv_rx_buffer[CANMV_MIN_FRAME_LEN - 1U] == CANMV_FRAME_END){
        /* 长度到位且帧尾正确 => 有效帧。复位后下一字节即下一帧帧头, 天然对齐。 */
        memcpy(s_canmv_frame, s_canmv_rx_buffer, sizeof(s_canmv_frame));
        CanMvUart_ResetRxState();
        g_canmv_uart_valid_frame_count++;
        CanMvUart_ParseFrame();
    } else{
        /* 满长仍非帧尾 => 已失步(多半丢过字节)。丢弃回空闲, 等下一个对齐帧头。 */
        CanMvUart_ResetRxState();
        CanMvUart_SetAllStatus(CANMV_STATUS_FRAME_DROP);
    }
}

void CanMvUart_ProcessRx(void){
    while (!DL_UART_Main_isRXFIFOEmpty(CANMV_UART_INST)){
        uint8_t byte = DL_UART_Main_receiveData(CANMV_UART_INST);
        CanMvUart_ProcessByte(byte);
    }
}

#ifndef CANMV_UART_TX_TIMEOUT
#define CANMV_UART_TX_TIMEOUT 100000U
#endif

BSP_STATUS CanMvUart_SendByte(uint8_t byte){
    /*
     * 只等 TX FIFO 有空位, 不能等 DL_UART_isBusy: BUSY 位在 K230 持续串流(UART2 RX
     * 活动)时长期为忙, 会使本函数无限阻塞 —— 标定各页入口发 "LASER=x" 指令时全部卡死。
     * 加超时兜底, 防止 TX 异常时挂死。
     */
    uint32_t timeout = CANMV_UART_TX_TIMEOUT;
    while (DL_UART_isTXFIFOFull(CANMV_UART_INST)){
        if (timeout-- == 0U){
            return BSP_STATUS_TIMEOUT;
        }
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
