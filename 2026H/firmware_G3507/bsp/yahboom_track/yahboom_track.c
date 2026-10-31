/**
 * @file  yahboom_track.c
 * @brief BSP Yahboom 8 路循线模块 I2C 驱动实现（I2C0，阻塞式）。
 */
#include "yahboom_track.h"

#include "ti_msp_dl_config.h"
#include "bsp_time.h"

#include <stddef.h>

#define YAHBOOM_TRACK_I2C_INST       Gray_JY61P_I2C_INST
#define YAHBOOM_TRACK_I2C_TIMEOUT    100000U
#define YAHBOOM_TRACK_INIT_RETRY     100U

#define YAHBOOM_TRACK_REG_CALIBRATE  0x01U
#define YAHBOOM_TRACK_REG_DIGITAL    0x30U

#define YAHBOOM_TRACK_I2C_ERROR_IRQS \
    (DL_I2C_INTERRUPT_CONTROLLER_NACK | \
     DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

static uint32_t diag_read_fail;
static uint32_t diag_write_fail;
static BSP_STATUS diag_last;

static void YahboomTrack_ResetTransfer(void)
{
    DL_I2C_resetControllerTransfer(YAHBOOM_TRACK_I2C_INST);
    DL_I2C_flushControllerRXFIFO(YAHBOOM_TRACK_I2C_INST);
    DL_I2C_flushControllerTXFIFO(YAHBOOM_TRACK_I2C_INST);
}

static BSP_STATUS YahboomTrack_WaitIdle(void)
{
    uint32_t timeout = YAHBOOM_TRACK_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(YAHBOOM_TRACK_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U){
        if (timeout-- == 0U){
            YahboomTrack_ResetTransfer();
            return BSP_STATUS_TIMEOUT;
        }
    }
    if ((DL_I2C_getControllerStatus(YAHBOOM_TRACK_I2C_INST) &
         DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
        YahboomTrack_ResetTransfer();
        return BSP_STATUS_ERROR;
    }
    return BSP_STATUS_OK;
}

static BSP_STATUS YahboomTrack_WaitInterrupt(uint32_t done_irq)
{
    uint32_t timeout = YAHBOOM_TRACK_I2C_TIMEOUT;

    while (timeout-- != 0U){
        uint32_t pending = DL_I2C_getRawInterruptStatus(
            YAHBOOM_TRACK_I2C_INST, done_irq | YAHBOOM_TRACK_I2C_ERROR_IRQS);

        if ((pending & YAHBOOM_TRACK_I2C_ERROR_IRQS) != 0U){
            DL_I2C_clearInterruptStatus(YAHBOOM_TRACK_I2C_INST, pending);
            YahboomTrack_ResetTransfer();
            return BSP_STATUS_ERROR;
        }
        if ((pending & done_irq) != 0U){
            DL_I2C_clearInterruptStatus(YAHBOOM_TRACK_I2C_INST, done_irq);
            return BSP_STATUS_OK;
        }
    }
    YahboomTrack_ResetTransfer();
    return BSP_STATUS_TIMEOUT;
}

/* 标准寄存器读：TX 寄存器地址（不发 STOP），随后 repeated START 读 1 字节。 */
static BSP_STATUS YahboomTrack_ReadReg(uint8_t reg, uint8_t *value)
{
    BSP_STATUS st;

    if (value == NULL){ return BSP_STATUS_NULL; }

    st = YahboomTrack_WaitIdle();
    if (st != BSP_STATUS_OK){ return st; }

    DL_I2C_flushControllerRXFIFO(YAHBOOM_TRACK_I2C_INST);
    DL_I2C_flushControllerTXFIFO(YAHBOOM_TRACK_I2C_INST);
    DL_I2C_clearInterruptStatus(YAHBOOM_TRACK_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
        YAHBOOM_TRACK_I2C_ERROR_IRQS);

    DL_I2C_transmitControllerData(YAHBOOM_TRACK_I2C_INST, reg);
    DL_I2C_startControllerTransferAdvanced(YAHBOOM_TRACK_I2C_INST,
        YAHBOOM_TRACK_I2C_ADDR_7BIT, DL_I2C_CONTROLLER_DIRECTION_TX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);

    st = YahboomTrack_WaitInterrupt(DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    if (st != BSP_STATUS_OK){ return st; }

    DL_I2C_startControllerTransfer(YAHBOOM_TRACK_I2C_INST,
        YAHBOOM_TRACK_I2C_ADDR_7BIT, DL_I2C_CONTROLLER_DIRECTION_RX, 1U);
    st = YahboomTrack_WaitInterrupt(DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    if (st != BSP_STATUS_OK){ return st; }
    if (DL_I2C_isControllerRXFIFOEmpty(YAHBOOM_TRACK_I2C_INST)){
        YahboomTrack_ResetTransfer();
        return BSP_STATUS_ERROR;
    }

    *value = DL_I2C_receiveControllerData(YAHBOOM_TRACK_I2C_INST);
    return YahboomTrack_WaitIdle();
}

/* 标准寄存器写：START + 地址(W) + 寄存器 + 值 + STOP。 */
static BSP_STATUS YahboomTrack_WriteReg(uint8_t reg, uint8_t value)
{
    BSP_STATUS st = YahboomTrack_WaitIdle();
    if (st != BSP_STATUS_OK){ return st; }

    DL_I2C_flushControllerTXFIFO(YAHBOOM_TRACK_I2C_INST);
    DL_I2C_clearInterruptStatus(YAHBOOM_TRACK_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE | YAHBOOM_TRACK_I2C_ERROR_IRQS);
    DL_I2C_transmitControllerData(YAHBOOM_TRACK_I2C_INST, reg);
    DL_I2C_transmitControllerData(YAHBOOM_TRACK_I2C_INST, value);
    DL_I2C_startControllerTransfer(YAHBOOM_TRACK_I2C_INST,
        YAHBOOM_TRACK_I2C_ADDR_7BIT, DL_I2C_CONTROLLER_DIRECTION_TX, 2U);

    st = YahboomTrack_WaitInterrupt(DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    if (st != BSP_STATUS_OK){ return st; }
    return YahboomTrack_WaitIdle();
}

static uint8_t YahboomTrack_Normalize(uint8_t raw)
{
    uint8_t detected = 0U;

    for (uint8_t channel = 0U; channel < YAHBOOM_TRACK_CHANNEL_COUNT; channel++){
        uint8_t raw_bit = (uint8_t)(7U - channel);
        if ((raw & (uint8_t)(1U << raw_bit)) == 0U){
            detected |= (uint8_t)(1U << channel);
        }
    }
    return detected;
}

BSP_STATUS YahboomTrack_Init(void)
{
    uint8_t raw;
    BSP_STATUS st = BSP_STATUS_TIMEOUT;

    diag_read_fail  = 0U;
    diag_write_fail = 0U;
    diag_last       = BSP_STATUS_OK;

    for (uint8_t i = 0U; i < YAHBOOM_TRACK_INIT_RETRY; i++){
        st = YahboomTrack_ReadRaw(&raw);
        if (st == BSP_STATUS_OK){ return BSP_STATUS_OK; }
        BSP_DelayMs(1U);
    }
    return st;
}

BSP_STATUS YahboomTrack_ReadRaw(uint8_t *raw)
{
    BSP_STATUS st;

    if (raw == NULL){ return BSP_STATUS_NULL; }
    st = YahboomTrack_ReadReg(YAHBOOM_TRACK_REG_DIGITAL, raw);
    if (st != BSP_STATUS_OK){
        diag_read_fail++;
        diag_last = st;
    }
    return st;
}

BSP_STATUS YahboomTrack_ReadDetectedMask(uint8_t *mask)
{
    uint8_t raw;
    BSP_STATUS st;

    if (mask == NULL){ return BSP_STATUS_NULL; }
    st = YahboomTrack_ReadRaw(&raw);
    if (st == BSP_STATUS_OK){ *mask = YahboomTrack_Normalize(raw); }
    return st;
}

BSP_STATUS YahboomTrack_SetCalibration(bool enabled)
{
    BSP_STATUS st = YahboomTrack_WriteReg(YAHBOOM_TRACK_REG_CALIBRATE,
                                           enabled ? 1U : 0U);
    if (st != BSP_STATUS_OK){
        diag_write_fail++;
        diag_last = st;
    }
    return st;
}

void YahboomTrack_GetDiag(uint32_t *read_fail, uint32_t *write_fail,
                          int32_t *last_status)
{
    if (read_fail != NULL){ *read_fail = diag_read_fail; }
    if (write_fail != NULL){ *write_fail = diag_write_fail; }
    if (last_status != NULL){ *last_status = (int32_t)diag_last; }
}
