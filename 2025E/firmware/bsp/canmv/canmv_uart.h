/**
 * @file  canmv_uart.h
 * @brief BSP CanMV UART 协议解析接口。
 */
#ifndef CANMV_UART_H
#define CANMV_UART_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CANMV_UART_INST
#define CANMV_UART_INST UART2
#endif

#ifndef CANMV_UART_IRQN
#define CANMV_UART_IRQN UART2_INT_IRQn
#endif

#ifndef CANMV_RX_BUFFER_LEN
#define CANMV_RX_BUFFER_LEN 100U
#endif

#ifndef CANMV_TARGET_VALUE_CAPACITY
#define CANMV_TARGET_VALUE_CAPACITY 10U
#endif

#ifndef CANMV_COORDINATE_SANITY_LIMIT
#define CANMV_COORDINATE_SANITY_LIMIT 1000U
#endif

#define CANMV_FRAME_START 0x12U
#define CANMV_FRAME_END   0x5BU
#define CANMV_ANGLE_FRAME_START0 0xA5U
#define CANMV_ANGLE_FRAME_START1 0x5AU
#define CANMV_ANGLE_FRAME_LEN    9U
#define CANMV_ANGLE_SCALE        100.0f /* int16 单位为 0.01 deg */
#define CANMV_ANGLE_STATUS_VALID     0U
#define CANMV_ANGLE_STATUS_NOT_FOUND 1U
#define CANMV_ANGLE_STATUS_LOST      2U

#define CANMV_LASER_BEGIN      2U
#define CANMV_LASER_BYTE_COUNT 8U

#define CANMV_RECT_BEGIN      10U
#define CANMV_RECT_BYTE_COUNT 16U

#define CANMV_MIN_FRAME_LEN (CANMV_RECT_BEGIN + CANMV_RECT_BYTE_COUNT + 1U)

extern volatile uint32_t g_canmv_uart_rx_byte_count;
extern volatile uint32_t g_canmv_uart_valid_frame_count;
extern volatile uint32_t g_canmv_uart_angle_frame_count;
extern volatile uint32_t g_canmv_uart_drop_count;
extern volatile uint8_t g_canmv_uart_last_byte;

/**
 * @brief CanMV 目标数据类型。
 */
typedef enum {
    CANMV_TARGET_LASER = 0,
    CANMV_TARGET_RECT,
    /** 两个有符号 int16：yaw/pitch 角度误差，单位 0.01 deg。 */
    CANMV_TARGET_ANGLE,
    CANMV_TARGET_MAX
} CANMV_TARGET;

/**
 * @brief CanMV 目标解析状态。
 */
typedef enum {
    /** 尚未收到有效数据。 */
    CANMV_STATUS_INIT = -1,
    /** 目标数据有效。 */
    CANMV_STATUS_OK = 0,
    /** CanMV 明确返回未找到目标。 */
    CANMV_STATUS_NOT_FOUND = 1,
    /** 目标曾经有效但当前丢失。 */
    CANMV_STATUS_LOST = 2,
    /** 接收帧不完整或被丢弃。 */
    CANMV_STATUS_FRAME_DROP = 3,
} CANMV_STATUS;

/**
 * @brief CanMV 单类目标数据缓存。
 */
typedef struct {
    /** 目标数据数组，具体含义由 CANMV_TARGET 决定。 */
    uint16_t value[CANMV_TARGET_VALUE_CAPACITY];
    /** 当前有效数据数量。 */
    uint8_t count;
    /** 当前解析状态。 */
    CANMV_STATUS status;
} CANMV_TARGET_DATA;

/**
 * @brief 初始化 CanMV UART 接收和目标状态。
 */
BSP_STATUS CanMvUart_Init(void);

/**
 * @brief 处理单个 UART 字节。
 * @note 可在 UART 中断或轮询路径中调用。
 */
void CanMvUart_ProcessByte(uint8_t byte);

/**
 * @brief 清空 UART RX FIFO 并处理其中所有字节。
 */
void CanMvUart_ProcessRx(void);

/**
 * @brief 发送单字节到 CanMV。
 */
BSP_STATUS CanMvUart_SendByte(uint8_t byte);

/**
 * @brief 发送 C 字符串到 CanMV，不自动追加换行。
 */
BSP_STATUS CanMvUart_SendString(const char *str);

/**
 * @brief 获取指定目标的解析状态。
 */
CANMV_STATUS CanMvUart_GetStatus(CANMV_TARGET target);

/**
 * @brief 复制指定目标数据。
 * @param target 目标类型。
 * @param out 输出数组。
 * @param max_count 输出数组容量。
 * @return 实际复制的数据数量。
 */
uint8_t CanMvUart_GetData(CANMV_TARGET target, uint16_t *out, uint8_t max_count);

/**
 * @brief 获取指定目标数据缓存指针。
 * @return 缓存指针；非法目标返回 NULL。
 */
const CANMV_TARGET_DATA *CanMvUart_GetTargetData(CANMV_TARGET target);

#ifdef __cplusplus
}
#endif

#endif /* CANMV_UART_H */
