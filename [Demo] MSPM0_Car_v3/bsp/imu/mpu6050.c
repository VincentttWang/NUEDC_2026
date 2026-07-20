/**
 * @file  mpu6050.c
 * @brief MPU6050 六轴传感器硬件 I2C 驱动实现
 *
 * 共享 I2C0 总线 (与 JY61P 同一外设, 不同地址 0x68 vs 0x50)
 * 复用 wit_sdk.c 中验证过的 REPEATED START 硬件 I2C 模式
 *
 * 参考:
 *   - 立创 MSPM0G3507 MPU6050 驱动 (bsp_mpu6050.c)
 *   - TI E2E: "LP-MSPM0G3507: Repeated Start Condition Settings"
 */

#include "mpu6050.h"
#include "bsp_time.h"

#define MPU6050_I2C_INST     MPU6050_JY61P_Tracking_INST
#define MPU6050_I2C_TIMEOUT  100000U

static volatile bool s_mpu6050_present;

/* ── 等待 I2C 空闲 ── */
static bool MPU6050_WaitIdle(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(MPU6050_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U){
        if (timeout-- == 0U){
            DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
            return false;
        }
    }
    return true;
}

/* ── 写单个寄存器 ── */
static bool MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    uint32_t timeout;

    if (!MPU6050_WaitIdle()){ return false; }

    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    DL_I2C_transmitControllerData(MPU6050_I2C_INST, reg);
    DL_I2C_transmitControllerData(MPU6050_I2C_INST, value);

    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, MPU6050_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    delay_cycles(10U);

    timeout = MPU6050_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(MPU6050_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){ return false; }
    }

    if ((DL_I2C_getControllerStatus(MPU6050_I2C_INST)
         & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
        DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
        return false;
    }
    return MPU6050_WaitIdle();
}

/*
 * I2C 读寄存器 (REPEATED START):
 *   START + DEVADDR(W) + REG + REPEATED START + DEVADDR(R) + DATA + STOP
 */
static bool MPU6050_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint32_t timeout;

    if (data == NULL || len == 0U){ return true; }

    /* Phase 1: TX 寄存器地址, STOP_DISABLE */
    DL_I2C_flushControllerRXFIFO(MPU6050_I2C_INST);
    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);

    if (!MPU6050_WaitIdle()){ return false; }

    DL_I2C_transmitControllerData(MPU6050_I2C_INST, reg);
    DL_I2C_startControllerTransferAdvanced(MPU6050_I2C_INST, MPU6050_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(10U);

    timeout = MPU6050_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(MPU6050_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){ return false; }
    }

    if ((DL_I2C_getControllerStatus(MPU6050_I2C_INST)
         & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
        DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
        return false;
    }

    /* Phase 2: RX, 硬件自动 REPEATED START */
    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, MPU6050_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    delay_cycles(10U);

    for (i = 0U; i < len; i++){
        timeout = MPU6050_I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C_INST)){
            if (timeout-- == 0U){ return false; }
        }
        data[i] = DL_I2C_receiveControllerData(MPU6050_I2C_INST);
    }

    return MPU6050_WaitIdle();
}

/* ── 读取 16 位有符号值 (MPU6050 大端: buf[0]=MSB, buf[1]=LSB) ── */
static int16_t MPU6050_ParseI16(const uint8_t *buf)
{
    return (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

/* ═══════════════════ 公开 API ═══════════════════════════════════════════ */

bool MPU6050_IsPresent(void)
{
    return s_mpu6050_present;
}

void MPU6050_SetGyroFsr(uint8_t fsr)
{
    (void)MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, (uint8_t)(fsr << 3));
}

void MPU6050_SetAccelFsr(uint8_t fsr)
{
    (void)MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, (uint8_t)(fsr << 3));
}

bool MPU6050_Init(void)
{
    uint8_t who_am_i = 0;

    s_mpu6050_present = false;

    /* 复位 */
    if (!MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x80U)){
        DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
        return false;
    }
    BSP_DelayMs(100);

    /* 唤醒 */
    if (!MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00U)){ return false; }
    BSP_DelayMs(10);

    /* 配置量程: 陀螺 ±2000dps, 加速度 ±2g */
    MPU6050_SetGyroFsr(MPU6050_GYRO_FSR_2000);
    MPU6050_SetAccelFsr(MPU6050_ACCEL_FSR_2G);

    /* 采样率 50Hz (SMPLRT_DIV = 1000/50 - 1 = 19) */
    (void)MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 19U);

    /* 数字低通滤波器: DLPF_CFG=3 → 42Hz */
    (void)MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x03U);

    /* 关闭中断、I2C Master、FIFO */
    (void)MPU6050_WriteReg(MPU6050_REG_INT_ENABLE, 0x00U);
    (void)MPU6050_WriteReg(MPU6050_REG_USER_CTRL,  0x00U);
    (void)MPU6050_WriteReg(MPU6050_REG_FIFO_EN,    0x00U);

    /* 验证 WHO_AM_I */
    if (!MPU6050_ReadRegs(MPU6050_REG_WHO_AM_I, &who_am_i, 1U)){ return false; }
    if (who_am_i != 0x68U){ return false; }

    /* 时钟源: PLL with X-axis gyro */
    (void)MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01U);
    /* 所有轴使能 */
    (void)MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_2, 0x00U);

    s_mpu6050_present = true;
    return true;
}

void MPU6050_ReadGyro(short *gx, short *gy, short *gz)
{
    uint8_t buf[6];
    if (!MPU6050_ReadRegs(MPU6050_REG_GYRO_XOUT_H, buf, 6U)){
        if (gx){ *gx = 0; } if (gy){ *gy = 0; } if (gz){ *gz = 0; }
        return;
    }
    if (gx){ *gx = MPU6050_ParseI16(&buf[0]); }
    if (gy){ *gy = MPU6050_ParseI16(&buf[2]); }
    if (gz){ *gz = MPU6050_ParseI16(&buf[4]); }
}

void MPU6050_ReadAccel(short *ax, short *ay, short *az)
{
    uint8_t buf[6];
    if (!MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, buf, 6U)){
        if (ax){ *ax = 0; } if (ay){ *ay = 0; } if (az){ *az = 0; }
        return;
    }
    if (ax){ *ax = MPU6050_ParseI16(&buf[0]); }
    if (ay){ *ay = MPU6050_ParseI16(&buf[2]); }
    if (az){ *az = MPU6050_ParseI16(&buf[4]); }
}

float MPU6050_ReadTemp(void)
{
    uint8_t buf[2];
    if (!MPU6050_ReadRegs(MPU6050_REG_TEMP_OUT_H, buf, 2U)){ return 0.0f; }
    return (float)MPU6050_ParseI16(buf) / 340.0f + 36.53f;
}
