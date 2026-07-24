/**
 * @file  grayscale_sensor.c
 * @brief BSP 8 路光敏灰度传感器数字量实现。
 */
#include "grayscale_sensor.h"
#include <stddef.h>

typedef struct {
    /** SysConfig 生成的 GPIO 端口。 */
    GPIO_Regs *port;
    /** SysConfig 生成的 GPIO 引脚掩码。 */
    uint32_t pin;
} GRAYSCALE_SENSOR_HW_CONFIG;

/*
 * 逻辑通道顺序沿用旧 Digital[8] 的顺序：channel 0 对应物理 Tracking_8，
 * channel 7 对应物理 Tracking_1。这样上层巡线算法不需要关心板上排线方向。
 */
static const GRAYSCALE_SENSOR_HW_CONFIG s_grayscale_sensor_hw[GRAYSCALE_SENSOR_CHANNEL_COUNT] = {
    [GRAYSCALE_SENSOR_CHANNEL_0] = {
        .port = GRAYSCALE_SENSOR_8_PORT,
        .pin = GRAYSCALE_SENSOR_8_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_1] = {
        .port = GRAYSCALE_SENSOR_7_PORT,
        .pin = GRAYSCALE_SENSOR_7_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_2] = {
        .port = GRAYSCALE_SENSOR_6_PORT,
        .pin = GRAYSCALE_SENSOR_6_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_3] = {
        .port = GRAYSCALE_SENSOR_5_PORT,
        .pin = GRAYSCALE_SENSOR_5_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_4] = {
        .port = GRAYSCALE_SENSOR_4_PORT,
        .pin = GRAYSCALE_SENSOR_4_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_5] = {
        .port = GRAYSCALE_SENSOR_3_PORT,
        .pin = GRAYSCALE_SENSOR_3_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_6] = {
        .port = GRAYSCALE_SENSOR_2_PORT,
        .pin = GRAYSCALE_SENSOR_2_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_7] = {
        .port = GRAYSCALE_SENSOR_1_PORT,
        .pin = GRAYSCALE_SENSOR_1_PIN,
    },
};

uint8_t GrayscaleSensor_ReadSingle(GRAYSCALE_SENSOR_CHANNEL channel){
    if (channel >= GRAYSCALE_SENSOR_CHANNEL_MAX){
        return 0U;
    }

    const GRAYSCALE_SENSOR_HW_CONFIG *hw = &s_grayscale_sensor_hw[channel];
    uint8_t high_level = (DL_GPIO_readPins(hw->port, hw->pin) != 0U) ? 1U : 0U;

    /*
     * 多数数字灰度模块为低电平有效。这里统一在 BSP 层完成有效电平翻转，
     * 使调用者只看到抽象后的 0/1 状态。
     */
    if (GRAYSCALE_SENSOR_ACTIVE_LOW != 0U){
        return (uint8_t)(!high_level);
    }

    return high_level;
}

void GrayscaleSensor_Read(uint8_t digital_array[GRAYSCALE_SENSOR_CHANNEL_COUNT]){
    if (digital_array == NULL){
        return;
    }

    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++){
        digital_array[i] = GrayscaleSensor_ReadSingle((GRAYSCALE_SENSOR_CHANNEL)i);
    }
}

uint8_t GrayscaleSensor_ReadMask(void){
    uint8_t mask = 0U;

    /* bit i 对应逻辑通道 i，便于调试页用二进制形式快速观察 8 路状态。 */
    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++){
        if (GrayscaleSensor_ReadSingle((GRAYSCALE_SENSOR_CHANNEL)i) != 0U){
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}
