#ifndef TEST_STUB_CANMV_UART_H
#define TEST_STUB_CANMV_UART_H

#include <stdint.h>

#define CANMV_ANGLE_SCALE 100.0f

typedef enum {
    CANMV_TARGET_LASER = 0,
    CANMV_TARGET_RECT,
    CANMV_TARGET_ANGLE,
    CANMV_TARGET_MAX
} CANMV_TARGET;

typedef enum {
    CANMV_STATUS_INIT = -1,
    CANMV_STATUS_OK = 0,
    CANMV_STATUS_NOT_FOUND,
    CANMV_STATUS_LOST,
    CANMV_STATUS_FRAME_DROP
} CANMV_STATUS;

typedef struct {
    uint16_t value[10];
    uint8_t count;
    CANMV_STATUS status;
} CANMV_TARGET_DATA;

extern volatile uint32_t g_canmv_uart_angle_frame_count;
const CANMV_TARGET_DATA *CanMvUart_GetTargetData(CANMV_TARGET target);

#endif
