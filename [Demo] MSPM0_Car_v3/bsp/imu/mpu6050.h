/**
 * @file  mpu6050.h
 * @brief MPU6050 六轴传感器硬件 I2C 驱动 (共享 I2C0, 地址 0x68)
 *
 * 参考: 立创 MSPM0G3507 MPU6050 驱动 (bsp_mpu6050.c)
 *       MPU6050 Register Map (InvenSense)
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ── I2C 地址 ── */
#define MPU6050_I2C_ADDR_7BIT    0x68U

/* ── 寄存器地址 ── */
#define MPU6050_REG_WHO_AM_I     0x75U
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_REG_PWR_MGMT_2   0x6CU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_INT_ENABLE   0x38U
#define MPU6050_REG_USER_CTRL    0x6AU
#define MPU6050_REG_FIFO_EN      0x23U
#define MPU6050_REG_INT_PIN_CFG  0x37U

#define MPU6050_REG_ACCEL_XOUT_H 0x3BU   /* 加速度 6 字节: X/Z/Y, MSB first */
#define MPU6050_REG_TEMP_OUT_H   0x41U   /* 温度 2 字节 */
#define MPU6050_REG_GYRO_XOUT_H  0x43U   /* 陀螺仪 6 字节 */

/* ── 量程配置 ── */
#define MPU6050_GYRO_FSR_250     0U
#define MPU6050_GYRO_FSR_500     1U
#define MPU6050_GYRO_FSR_1000    2U
#define MPU6050_GYRO_FSR_2000    3U

#define MPU6050_ACCEL_FSR_2G     0U
#define MPU6050_ACCEL_FSR_4G     1U
#define MPU6050_ACCEL_FSR_8G     2U
#define MPU6050_ACCEL_FSR_16G    3U

/* ── API ── */
bool MPU6050_Init(void);
bool MPU6050_IsPresent(void);

void MPU6050_ReadGyro(short *gx, short *gy, short *gz);
void MPU6050_ReadAccel(short *ax, short *ay, short *az);
float MPU6050_ReadTemp(void);

/* 量程设置后需要重新读取数据才能反映变化 */
void MPU6050_SetGyroFsr(uint8_t fsr);
void MPU6050_SetAccelFsr(uint8_t fsr);

#endif /* MPU6050_H */
