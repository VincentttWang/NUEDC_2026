/**
 * @file  wit_sdk.h
 * @brief WitMotion IMU SDK 接口，支持串口/I2C/CAN 协议通信
 */
#ifndef WIT_SDK_H
#define WIT_SDK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "reg.h"


#define WIT_HAL_OK      (0)     /**< There is no error */
#define WIT_HAL_BUSY    (-1)    /**< Busy */
#define WIT_HAL_TIMEOUT (-2)    /**< Timed out */
#define WIT_HAL_ERROR   (-3)    /**< A generic error happens */
#define WIT_HAL_NOMEM   (-4)    /**< No memory */
#define WIT_HAL_EMPTY   (-5)    /**< The resource is empty */
#define WIT_HAL_INVAL   (-6)    /**< Invalid argument */

#define WIT_DATA_BUFF_SIZE  256

#define WIT_PROTOCOL_NORMAL 0
#define WIT_PROTOCOL_MODBUS 1
#define WIT_PROTOCOL_CAN    2
#define WIT_PROTOCOL_I2C    3


/* serial function */
typedef void (*SerialWrite)(uint8_t *p_ucData, uint32_t uiLen);
int32_t WitSerialWriteRegister(SerialWrite write_func);
void WitSerialDataIn(uint8_t ucData);

/* iic function */

/*
    i2c write function example

    int32_t WitI2cWrite(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen){
        i2c_start();
        i2c_send(ucAddr);
        if(i2c_wait_ask() != SUCCESS)return 0;
        i2c_send(ucReg);
        if(i2c_wait_ask() != SUCCESS)return 0;
        for(uint32_t i = 0; i < uiLen; i++){
            i2c_send(*p_ucVal++); 
            if(i2c_wait_ask() != SUCCESS)return 0;
        }
        i2c_stop();
        return 1;
    }
*/
typedef int32_t (*WitI2cWrite)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
/*
    i2c read function example

    int32_t WitI2cRead(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen){
        i2c_start();
        i2c_send(ucAddr);
        if(i2c_wait_ask() != SUCCESS)return 0;
        i2c_send(ucReg);
        if(i2c_wait_ask() != SUCCESS)return 0;
        
        i2c_start();
        i2c_send(ucAddr+1);
        for(uint32_t i = 0; i < uiLen; i++){
            if(i+1 == uiLen)*p_ucVal++ = i2c_read(0);  //last byte no ask
            else *p_ucVal++ = i2c_read(1);  //  ask
        }
        i2c_stop();
        return 1;
    }
*/
typedef int32_t (*WitI2cRead)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func);

/* can function */
typedef void (*CanWrite)(uint8_t ucStdId, uint8_t *p_ucData, uint32_t uiLen);
int32_t WitCanWriteRegister(CanWrite write_func);

/* Delayms function */
typedef void (*DelaymsCb)(uint16_t ucMs);
int32_t WitDelayMsRegister(DelaymsCb delayms_func);


void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen);


typedef void (*RegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);
int32_t WitRegisterCallBack(RegUpdateCb update_func);
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData);
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum);
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr);
void WitDeInit(void);



/**
  ******************************************************************************
  * @file    wit_c_sdk.h
  * @author  Wit
  * @version V1.0
  * @date    05-May-2022
  * @brief   This file provides all Configure sensor function.
  ******************************************************************************
  * @attention
  *
  *        http://wit-motion.cn/
  *
  ******************************************************************************
  */
int32_t WitStartAccCali(void);
int32_t WitStopAccCali(void);
int32_t WitStartMagCali(void);
int32_t WitStopMagCali(void);
int32_t WitSetUartBaud(int32_t uiBaudIndex);
int32_t WitSetBandwidth(int32_t uiBaudWidth);
int32_t WitSetOutputRate(int32_t uiRate);
int32_t WitSetContent(int32_t uiRsw);
int32_t WitSetCanBaud(int32_t uiBaudIndex);

void IT_JY61P(void);
void GYROSCOPE_DATA_Decoder(uint8_t *buf);
void JY61P_Init(UART_Regs *uart);

/* ── JY61P I2C 驱动 (替代 UART 方式) ── */
/*
 * 硬件 I2C, 外设 I2C0 (SysConfig 分配): PA0=SDA, PA1=SCL, 400kHz
 * 参考: MSPM0 SDK i2c_controller_rw_multibyte_fifo_poll
 */

/* ── JY61P I2C 地址与协议常量 ── */
#define JY61P_I2C_ADDR_7BIT          0x50U
#define JY61P_I2C_POLL_PERIOD_TICK   5U
#define JY61P_I2C_REG_ANGLE          0x3DU
#define JY61P_I2C_REG_GYRO           0x37U
#define JY61P_I2C_REG_UNLOCK         0x69U
#define JY61P_I2C_REG_SAVE           0x00U
#define JY61P_I2C_REG_CALSW          0x01U

void JY61P_I2C_Init(void);
void JY61P_I2C_Poll(void);
uint32_t JY61P_I2C_GetPollCount(void);
uint32_t JY61P_I2C_GetErrorCount(void);
uint32_t JY61P_I2C_GetNackCount(void);
uint32_t JY61P_I2C_GetTimeoutCount(void);

typedef struct {
    float x;
    float y;
    float z;
} WIT_VECTOR3F;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} WIT_ATTITUDE;

typedef struct {
    WIT_VECTOR3F acc_g;
    WIT_VECTOR3F gyro_deg_s;
    WIT_ATTITUDE attitude_deg;
    float temperature_c;
} WIT_IMU_DATA;

int32_t WitGetAcc(WIT_VECTOR3F *out);
int32_t WitGetGyro(WIT_VECTOR3F *out);
int32_t WitGetAttitude(WIT_ATTITUDE *out);
int32_t WitGetData(WIT_IMU_DATA *out);
	
char CheckRange(short sTemp,short sMin,short sMax);

extern int16_t sReg[REGSIZE];
extern double GyroscopeChannelData[10];
#ifdef __cplusplus
}
#endif

#endif /* __WIT_C_SDK_H */
