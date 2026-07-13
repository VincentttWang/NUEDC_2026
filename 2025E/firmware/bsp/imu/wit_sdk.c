#include "wit_sdk.h"
#include <stdio.h>
#include <string.h>

#define GYROSCOPE_BUFFER_SIZE    33
#define JY61P_FRAME_SIZE         11
#define GYROSCOPE_CH_DATA_SIZE   10

static SerialWrite p_WitSerialWriteFunc = NULL;
static WitI2cWrite p_WitI2cWriteFunc = NULL;
static WitI2cRead p_WitI2cReadFunc = NULL;
static CanWrite p_WitCanWriteFunc = NULL;
static RegUpdateCb p_WitRegUpdateCbFunc = NULL;
static DelaymsCb p_WitDelaymsFunc = NULL;

static uint8_t s_ucAddr = 0xff;
static uint8_t s_ucWitDataBuff[WIT_DATA_BUFF_SIZE];
static uint32_t s_uiWitDataCnt = 0, s_uiProtoclo = 0, s_uiReadRegIndex = 0;
int16_t sReg[REGSIZE];

uint8_t GyroscopeUsart3RxBuffer[33];
double GyroscopeChannelData[10] = {0};
#define FuncW 0x06
#define FuncR 0x03

static int16_t JY61P_ReadI16(const uint8_t *buf, uint8_t low_index);
static uint8_t JY61P_FrameChecksum(const uint8_t *buf);
static void JY61P_DecodeFrame(const uint8_t *buf);

static int16_t JY61P_ReadI16(const uint8_t *buf, uint8_t low_index)
{
    return (int16_t)(((uint16_t)buf[(uint8_t)(low_index + 1U)] << 8) | buf[low_index]);
}

static uint8_t JY61P_FrameChecksum(const uint8_t *buf)
{
    uint16_t sum = 0U;

    for(uint8_t i = 0U; i < (JY61P_FRAME_SIZE - 1U); i++){
        sum += buf[i];
    }

    return (uint8_t)(sum & 0xFFU);
}

static void JY61P_DecodeFrame(const uint8_t *buf)
{
    if((buf == NULL) || (buf[0] != 0x55U) || (buf[10] != JY61P_FrameChecksum(buf))){
        return;
    }

    switch(buf[1]){
        case WIT_ACC:
            GyroscopeChannelData[0] = (double)JY61P_ReadI16(buf, 2U) / 32768.0 * 16.0;
            GyroscopeChannelData[1] = (double)JY61P_ReadI16(buf, 4U) / 32768.0 * 16.0;
            GyroscopeChannelData[2] = (double)JY61P_ReadI16(buf, 6U) / 32768.0 * 16.0;
            GyroscopeChannelData[9] = (double)JY61P_ReadI16(buf, 8U) / 100.0;
            break;
        case WIT_GYRO:
            GyroscopeChannelData[3] = (double)JY61P_ReadI16(buf, 2U) / 32768.0 * 2000.0;
            GyroscopeChannelData[4] = (double)JY61P_ReadI16(buf, 4U) / 32768.0 * 2000.0;
            GyroscopeChannelData[5] = (double)JY61P_ReadI16(buf, 6U) / 32768.0 * 2000.0;
            break;
        case WIT_ANGLE:
            GyroscopeChannelData[6] = (double)JY61P_ReadI16(buf, 2U) / 32768.0 * 180.0;
            GyroscopeChannelData[7] = (double)JY61P_ReadI16(buf, 4U) / 32768.0 * 180.0;
            GyroscopeChannelData[8] = (double)JY61P_ReadI16(buf, 6U) / 32768.0 * 180.0;
            break;
        default:
            break;
    }
}

static const uint8_t __auchCRCHi[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};
static const uint8_t __auchCRCLo[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};


static uint16_t __CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint8_t uIndex;
    int i = 0;
    uchCRCHi = 0xFF;
    uchCRCLo = 0xFF;
    for (; i<usDataLen; i++)
    {
        uIndex = uchCRCHi ^ puchMsg[i];
        uchCRCHi = uchCRCLo ^ __auchCRCHi[uIndex];
        uchCRCLo = __auchCRCLo[uIndex] ;
    }
    return (uint16_t)(((uint16_t)uchCRCHi << 8) | (uint16_t)uchCRCLo) ;
}
static uint8_t __CaliSum(uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint8_t ucCheck = 0;
    for(i=0; i<len; i++) ucCheck += *(data + i);
    return ucCheck;
}
int32_t WitSerialWriteRegister(SerialWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    p_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}
static void CopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen)
{
    uint32_t uiReg1 = 0, uiReg2 = 0, uiReg1Len = 0, uiReg2Len = 0;
    uint16_t *p_usReg1Val = p_data;
    uint16_t *p_usReg2Val = p_data+3;
    
    uiReg1Len = 4;
    switch(ucIndex)
    {
        case WIT_ACC:   uiReg1 = AX;    uiReg1Len = 3;  uiReg2 = TEMP;  uiReg2Len = 1;  break;
        case WIT_ANGLE: uiReg1 = Roll;  uiReg1Len = 3;  uiReg2 = VERSION;  uiReg2Len = 1;  break;
        case WIT_TIME:  uiReg1 = YYMM;	break;
        case WIT_GYRO:  uiReg1 = GX;  uiLen = 3;break;
        case WIT_MAGNETIC: uiReg1 = HX;  uiLen = 3;break;
        case WIT_DPORT: uiReg1 = D0Status;  break;
        case WIT_PRESS: uiReg1 = PressureL;  break;
        case WIT_GPS:   uiReg1 = LonL;  break;
        case WIT_VELOCITY: uiReg1 = GPSHeight;  break;
        case WIT_QUATER:    uiReg1 = q0;  break;
        case WIT_GSA:   uiReg1 = SVNUM;  break;
        case WIT_REGVALUE:  uiReg1 = s_uiReadRegIndex;  break;
		default:
			return ;

    }
    if(uiLen == 3)
    {
        uiReg1Len = 3;
        uiReg2Len = 0;
    }
    if(uiReg1Len)
	{
		memcpy(&sReg[uiReg1], p_usReg1Val, uiReg1Len<<1);
		p_WitRegUpdateCbFunc(uiReg1, uiReg1Len);
	}
    if(uiReg2Len)
	{
		memcpy(&sReg[uiReg2], p_usReg2Val, uiReg2Len<<1);
		p_WitRegUpdateCbFunc(uiReg2, uiReg2Len);
	}
}

void WitSerialDataIn(uint8_t ucData)
{
    uint16_t usCRC16, usTemp, i, usData[4];
    uint8_t ucSum;

    if(p_WitRegUpdateCbFunc == NULL)return ;
    s_ucWitDataBuff[s_uiWitDataCnt++] = ucData;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(s_ucWitDataBuff[0] != 0x55)
            {
                s_uiWitDataCnt--;
                memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                return ;
            }
            if(s_uiWitDataCnt >= 11)
            {
                ucSum = __CaliSum(s_ucWitDataBuff, 10);
                if(ucSum != s_ucWitDataBuff[10])
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                usData[0] = ((uint16_t)s_ucWitDataBuff[3] << 8) | (uint16_t)s_ucWitDataBuff[2];
                usData[1] = ((uint16_t)s_ucWitDataBuff[5] << 8) | (uint16_t)s_ucWitDataBuff[4];
                usData[2] = ((uint16_t)s_ucWitDataBuff[7] << 8) | (uint16_t)s_ucWitDataBuff[6];
                usData[3] = ((uint16_t)s_ucWitDataBuff[9] << 8) | (uint16_t)s_ucWitDataBuff[8];
                CopeWitData(s_ucWitDataBuff[1], usData, 4);
                s_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_MODBUS:
            if(s_uiWitDataCnt > 2)
            {
                if(s_ucWitDataBuff[1] != FuncR)
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                if(s_uiWitDataCnt < (s_ucWitDataBuff[2] + 5))return ;
                usTemp = ((uint16_t)s_ucWitDataBuff[s_uiWitDataCnt-2] << 8) | s_ucWitDataBuff[s_uiWitDataCnt-1];
                usCRC16 = __CRC16(s_ucWitDataBuff, s_uiWitDataCnt-2);
                if(usTemp != usCRC16)
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                usTemp = s_ucWitDataBuff[2] >> 1;
                for(i = 0; i < usTemp; i++)
                {
                    sReg[i+s_uiReadRegIndex] = ((uint16_t)s_ucWitDataBuff[(i<<1)+3] << 8) | s_ucWitDataBuff[(i<<1)+4];
                }
                p_WitRegUpdateCbFunc(s_uiReadRegIndex, usTemp);
                s_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_CAN:
        case WIT_PROTOCOL_I2C:
        s_uiWitDataCnt = 0;
        break;
    }
    if(s_uiWitDataCnt == WIT_DATA_BUFF_SIZE)s_uiWitDataCnt = 0;
}
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func)
{
    if(!write_func)return WIT_HAL_INVAL;
    if(!read_func)return WIT_HAL_INVAL;
    p_WitI2cWriteFunc = write_func;
    p_WitI2cReadFunc = read_func;
    return WIT_HAL_OK;
}
int32_t WitCanWriteRegister(CanWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    p_WitCanWriteFunc = Write_func;
    return WIT_HAL_OK;
}
void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen)
{
	uint16_t usData[3];
    if(p_WitRegUpdateCbFunc == NULL)return ;
    if(ucLen < 8)return ;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_CAN:
            if(ucData[0] != 0x55)return ;
            usData[0] = ((uint16_t)ucData[3] << 8) | ucData[2];
            usData[1] = ((uint16_t)ucData[5] << 8) | ucData[4];
            usData[2] = ((uint16_t)ucData[7] << 8) | ucData[6];
            CopeWitData(ucData[1], usData, 3);
            break;
        case WIT_PROTOCOL_NORMAL:
        case WIT_PROTOCOL_MODBUS:
        case WIT_PROTOCOL_I2C:
            break;
    }
}
int32_t WitRegisterCallBack(RegUpdateCb update_func)
{
    if(!update_func)return WIT_HAL_INVAL;
    p_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData)
{
    uint16_t usCRC;
    uint8_t ucBuff[8];
    if(uiReg >= REGSIZE)return WIT_HAL_INVAL;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            p_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = s_ucAddr;
            ucBuff[1] = FuncW;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = usData >> 8;
            ucBuff[5] = usData & 0xff;
            usCRC = __CRC16(ucBuff, 6);
            ucBuff[6] = usCRC >> 8;
            ucBuff[7] = usCRC & 0xff;
            p_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(p_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            p_WitCanWriteFunc(s_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(p_WitI2cWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = usData & 0xff;
            ucBuff[1] = usData >> 8;
			if(p_WitI2cWriteFunc(s_ucAddr << 1, uiReg, ucBuff, 2) != 1)
			{
				//printf("i2c write fail\r\n");
			}
        break;
	default: 
            return WIT_HAL_INVAL;        
    }
    return WIT_HAL_OK;
}
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum)
{
    uint16_t usTemp, i;
    uint8_t ucBuff[8];
    if((uiReg + uiReadNum) >= REGSIZE)return WIT_HAL_INVAL;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(uiReadNum > 4)return WIT_HAL_INVAL;
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            p_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if((usTemp + 5) > WIT_DATA_BUFF_SIZE)return WIT_HAL_NOMEM;
            ucBuff[0] = s_ucAddr;
            ucBuff[1] = FuncR;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = uiReadNum >> 8;
            ucBuff[5] = uiReadNum & 0xff;
            usTemp = __CRC16(ucBuff, 6);
            ucBuff[6] = usTemp >> 8;
            ucBuff[7] = usTemp & 0xff;
            p_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(uiReadNum > 3)return WIT_HAL_INVAL;
            if(p_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            p_WitCanWriteFunc(s_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(p_WitI2cReadFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if(WIT_DATA_BUFF_SIZE < usTemp)return WIT_HAL_NOMEM;
            if(p_WitI2cReadFunc(s_ucAddr << 1, uiReg, s_ucWitDataBuff, usTemp) == 1)
            {
                if(p_WitRegUpdateCbFunc == NULL)return WIT_HAL_EMPTY;
                for(i = 0; i < uiReadNum; i++)
                {
                    sReg[i+uiReg] = ((uint16_t)s_ucWitDataBuff[(i<<1)+1] << 8) | s_ucWitDataBuff[i<<1];
                }
                p_WitRegUpdateCbFunc(uiReg, uiReadNum);
            }
			
            break;
		default: 
            return WIT_HAL_INVAL;
    }
    s_uiReadRegIndex = uiReg;

    return WIT_HAL_OK;
}
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr)
{
	if(uiProtocol > WIT_PROTOCOL_I2C)return WIT_HAL_INVAL;
    s_uiProtoclo = uiProtocol;
    s_ucAddr = ucAddr;
    s_uiWitDataCnt = 0;
    return WIT_HAL_OK;
}
void WitDeInit(void)
{
    p_WitSerialWriteFunc = NULL;
    p_WitI2cWriteFunc = NULL;
    p_WitI2cReadFunc = NULL;
    p_WitCanWriteFunc = NULL;
    p_WitRegUpdateCbFunc = NULL;
    s_ucAddr = 0xff;
    s_uiWitDataCnt = 0;
    s_uiProtoclo = 0;
}

int32_t WitDelayMsRegister(DelaymsCb delayms_func)
{
    if(!delayms_func)return WIT_HAL_INVAL;
    p_WitDelaymsFunc = delayms_func;
    return WIT_HAL_OK;
}

char CheckRange(short sTemp,short sMin,short sMax)
{
    if ((sTemp>=sMin)&&(sTemp<=sMax)) return 1;
    else return 0;
}
/*Acceleration calibration demo*/
int32_t WitStartAccCali(void)
{
/*
	First place the equipment horizontally, and then perform the following operations
*/
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	    return  WIT_HAL_ERROR;// unlock reg
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(CALSW, CALGYROACC) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
int32_t WitStopAccCali(void)
{
	if(WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*Magnetic field calibration*/
int32_t WitStartMagCali(void)
{
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(CALSW, CALMAGMM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
int32_t WitStopMagCali(void)
{
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Band*/
int32_t WitSetUartBaud(int32_t uiBaudIndex)
{
	if(!CheckRange(uiBaudIndex,WIT_BAUD_4800,WIT_BAUD_230400))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Can Band*/
int32_t WitSetCanBaud(int32_t uiBaudIndex)
{
	if(!CheckRange(uiBaudIndex,CAN_BAUD_1000000,CAN_BAUD_3000))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Bandwidth*/
int32_t WitSetBandwidth(int32_t uiBaudWidth)
{	
	if(!CheckRange(uiBaudWidth,BANDWIDTH_256HZ,BANDWIDTH_5HZ))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(BANDWIDTH, uiBaudWidth) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/*change output rate */
int32_t WitSetOutputRate(int32_t uiRate)
{	
	if(!CheckRange(uiRate,RRATE_02HZ,RRATE_NONE))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(RRATE, uiRate) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/*change WitSetContent */
int32_t WitSetContent(int32_t uiRsw)
{	
	if(!CheckRange(uiRsw,RSW_TIME,RSW_MASK))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(RSW, uiRsw) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

void IT_JY61P(void){
    if(GyroscopeUsart3RxBuffer[0] == 0x55)
    {
        uint16_t AccelerationSumData = 0, AngularSumData = 0, AngleSumData = 0;
        for(uint8_t index = 0; index < GYROSCOPE_BUFFER_SIZE/3-1; index++)
            AccelerationSumData += GyroscopeUsart3RxBuffer[index];

        for(uint8_t index = GYROSCOPE_BUFFER_SIZE/3; index < 2*GYROSCOPE_BUFFER_SIZE/3-1; index++)
            AngularSumData += GyroscopeUsart3RxBuffer[index];

        for(uint8_t index = 2*GYROSCOPE_BUFFER_SIZE/3; index < GYROSCOPE_BUFFER_SIZE-1; index++)
            AngleSumData += GyroscopeUsart3RxBuffer[index];

        if(GyroscopeUsart3RxBuffer[GYROSCOPE_BUFFER_SIZE/3-1] == (uint8_t)(AccelerationSumData & 0x00FF)
        && GyroscopeUsart3RxBuffer[2*GYROSCOPE_BUFFER_SIZE/3-1] == (uint8_t)(AngularSumData & 0x00FF)
        && GyroscopeUsart3RxBuffer[GYROSCOPE_BUFFER_SIZE-1] == (uint8_t)(AngleSumData & 0x00FF))
        {
            JY61P_DecodeFrame(GyroscopeUsart3RxBuffer);
            JY61P_DecodeFrame(&GyroscopeUsart3RxBuffer[11]);
            JY61P_DecodeFrame(&GyroscopeUsart3RxBuffer[22]);
        }
    }
    else
    {
        memset(GyroscopeUsart3RxBuffer, 0x00, sizeof(GyroscopeUsart3RxBuffer));
    }
}

void GYROSCOPE_DATA_Decoder(uint8_t *buf)
{
    if(buf == NULL){
        return;
    }

    /* JY61P 标准串口输出为 11 字节子帧：0x51 加速度、0x52 角速度、0x53 姿态角。 */
    JY61P_DecodeFrame(buf);
}

void JY61P_Init(UART_Regs *uart){
    (void)uart;
    memset(GyroscopeUsart3RxBuffer, 0x00, sizeof(GyroscopeUsart3RxBuffer));
}

int32_t WitGetAcc(WIT_VECTOR3F *out)
{
    if(out == NULL)return WIT_HAL_INVAL;

    out->x = (float)GyroscopeChannelData[0];
    out->y = (float)GyroscopeChannelData[1];
    out->z = (float)GyroscopeChannelData[2];
    return WIT_HAL_OK;
}

int32_t WitGetGyro(WIT_VECTOR3F *out)
{
    if(out == NULL)return WIT_HAL_INVAL;

    out->x = (float)GyroscopeChannelData[3];
    out->y = (float)GyroscopeChannelData[4];
    out->z = (float)GyroscopeChannelData[5];
    return WIT_HAL_OK;
}

int32_t WitGetAttitude(WIT_ATTITUDE *out)
{
    if(out == NULL)return WIT_HAL_INVAL;

    out->roll = (float)GyroscopeChannelData[6];
    out->pitch = (float)GyroscopeChannelData[7];
    out->yaw = (float)GyroscopeChannelData[8];
    return WIT_HAL_OK;
}

int32_t WitGetData(WIT_IMU_DATA *out)
{
    if(out == NULL)return WIT_HAL_INVAL;

    out->acc_g.x = (float)GyroscopeChannelData[0];
    out->acc_g.y = (float)GyroscopeChannelData[1];
    out->acc_g.z = (float)GyroscopeChannelData[2];
    out->gyro_deg_s.x = (float)GyroscopeChannelData[3];
    out->gyro_deg_s.y = (float)GyroscopeChannelData[4];
    out->gyro_deg_s.z = (float)GyroscopeChannelData[5];
    out->attitude_deg.roll = (float)GyroscopeChannelData[6];
    out->attitude_deg.pitch = (float)GyroscopeChannelData[7];
    out->attitude_deg.yaw = (float)GyroscopeChannelData[8];
    out->temperature_c = (float)GyroscopeChannelData[9];
    return WIT_HAL_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * JY61P 硬件 I2C 驱动实现 (替代 UART 方式)
 *
 * 参考: MSPM0 SDK i2c_controller_rw_multibyte_fifo_poll (官方 FIFO 轮询模式)
 *       MSPM0 SDK data_sensor_aggregator (多 I2C 传感器采集)
 *
 * 外设: I2C0 (SysConfig), PA0=SDA, PA1=SCL, 400kHz, 设备地址 0x50
 * 轮询: SysTick ISR 中每 5ms 调用一次 JY61P_I2C_Poll()
 * ═══════════════════════════════════════════════════════════════════════════ */

#include "bsp_time.h"

#define JY61P_I2C_INST          MPU6050_JY61P_Tracking_INST
#define JY61P_I2C_TIMEOUT       100000U

static volatile uint32_t s_jy61p_i2c_poll_count;
static volatile uint32_t s_jy61p_i2c_error_count;
static volatile uint32_t s_jy61p_i2c_nack_count;

/* ═══════════════════ 硬件 I2C 底层 (MSPM0 SDK FIFO 轮询模式) ═══════════════ */

static bool JY61P_I2C_WaitIdle(void)
{
    uint32_t timeout = JY61P_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(JY61P_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U){
        if (timeout-- == 0U){
            /* 超时: 总线可能被损坏设备拉死, 强制复位控制器并发送 STOP */
            DL_I2C_resetControllerTransfer(JY61P_I2C_INST);
            return false;
        }
    }
    return true;
}

/*
 * I2C 写: START + DEVADDR(W) + REG + DATA[0..N] + STOP
 */
static bool JY61P_I2C_WriteRegs(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint32_t timeout;

    if (!JY61P_I2C_WaitIdle()){ return false; }

    DL_I2C_flushControllerTXFIFO(JY61P_I2C_INST);
    DL_I2C_transmitControllerData(JY61P_I2C_INST, reg);
    for (i = 0U; i < len; i++){
        uint32_t tx_timeout = JY61P_I2C_TIMEOUT;
        while (DL_I2C_isControllerTXFIFOFull(JY61P_I2C_INST)){
            if (tx_timeout-- == 0U){ return false; }
        }
        DL_I2C_transmitControllerData(JY61P_I2C_INST, data[i]);
    }

    DL_I2C_startControllerTransfer(JY61P_I2C_INST, JY61P_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, (uint32_t)(len + 1U));
    delay_cycles(10U);

    timeout = JY61P_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(JY61P_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){ return false; }
    }

    if ((DL_I2C_getControllerStatus(JY61P_I2C_INST)
         & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
        DL_I2C_resetControllerTransfer(JY61P_I2C_INST);
        return false;
    }

    return JY61P_I2C_WaitIdle();
}

/*
 * I2C 读 (REPEATED START):
 *   Phase 1: TX reg addr (STOP_DISABLE) → Phase 2: RX data (auto REPEATED START)
 *
 *   参考: TI E2E 论坛 "LP-MSPM0G3507: Repeated Start Condition Settings"
 *   https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/
 *   arm-based-microcontrollers/f/arm-based-microcontrollers-forum/1416788/
 *
 *   协议序列 (与软件 I2C 完全一致):
 *   START + DEVADDR(W) + REG + REPEATED_START + DEVADDR(R) + DATA[0..N] + STOP
 */
static bool JY61P_I2C_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint32_t timeout;

    if (data == NULL || len == 0U){ return true; }

    /* Phase 1: TX register address, STOP_DISABLE 抑制 STOP */
    DL_I2C_flushControllerRXFIFO(JY61P_I2C_INST);
    DL_I2C_flushControllerTXFIFO(JY61P_I2C_INST);

    if (!JY61P_I2C_WaitIdle()){ return false; }

    DL_I2C_transmitControllerData(JY61P_I2C_INST, reg);
    DL_I2C_startControllerTransferAdvanced(JY61P_I2C_INST, JY61P_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,   /* <-- 关键: 不发 STOP */
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(10U);

    timeout = JY61P_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(JY61P_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){ s_jy61p_i2c_nack_count++; return false; }
    }

    if ((DL_I2C_getControllerStatus(JY61P_I2C_INST)
         & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
        s_jy61p_i2c_nack_count++;
        DL_I2C_resetControllerTransfer(JY61P_I2C_INST);
        return false;
    }

    /* Phase 2: RX, 硬件自动产生 REPEATED START (前一阶段未发 STOP) */
    DL_I2C_startControllerTransfer(JY61P_I2C_INST, JY61P_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    delay_cycles(10U);

    for (i = 0U; i < len; i++){
        timeout = JY61P_I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(JY61P_I2C_INST)){
            if (timeout-- == 0U){ s_jy61p_i2c_nack_count++; return false; }
        }
        data[i] = DL_I2C_receiveControllerData(JY61P_I2C_INST);
    }

    return JY61P_I2C_WaitIdle();
}
/* ═══════════════════ 应用层接口 ═══════════════════════════════════════════ */

static int16_t JY61P_I2C_ParseI16(const uint8_t *buf)
{
    return (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
}

static float JY61P_I2C_ParseAngle(int16_t raw)
{
    float angle = (float)raw / 32768.0f * 180.0f;
    if (angle > 180.0f)       { angle -= 360.0f; }
    else if (angle < -180.0f) { angle += 360.0f; }
    return angle;
}

void JY61P_I2C_Init(void)
{
    s_jy61p_i2c_poll_count  = 0U;
    s_jy61p_i2c_error_count = 0U;
    s_jy61p_i2c_nack_count  = 0U;
    /* 校准已固化在 JY61P flash 中, 上电无需重复执行 */
}

void JY61P_I2C_Poll(void)
{
    uint8_t buf[6];
    int16_t raw[3];

    s_jy61p_i2c_poll_count++;

    if (JY61P_I2C_ReadRegs(JY61P_I2C_REG_ANGLE, buf, 6U)){
        GyroscopeChannelData[6] = (double)JY61P_I2C_ParseAngle(JY61P_I2C_ParseI16(&buf[0]));
        GyroscopeChannelData[7] = (double)JY61P_I2C_ParseAngle(JY61P_I2C_ParseI16(&buf[2]));
        GyroscopeChannelData[8] = (double)JY61P_I2C_ParseAngle(JY61P_I2C_ParseI16(&buf[4]));
    } else{ s_jy61p_i2c_error_count++; }

    if (JY61P_I2C_ReadRegs(JY61P_I2C_REG_GYRO, buf, 6U)){
        raw[0] = JY61P_I2C_ParseI16(&buf[0]);
        raw[1] = JY61P_I2C_ParseI16(&buf[2]);
        raw[2] = JY61P_I2C_ParseI16(&buf[4]);
        GyroscopeChannelData[3] = (double)raw[0] / 32768.0 * 2000.0;
        GyroscopeChannelData[4] = (double)raw[1] / 32768.0 * 2000.0;
        GyroscopeChannelData[5] = (double)raw[2] / 32768.0 * 2000.0;
    } else{ s_jy61p_i2c_error_count++; }
}

uint32_t JY61P_I2C_GetPollCount(void)    { return s_jy61p_i2c_poll_count; }
uint32_t JY61P_I2C_GetErrorCount(void)   { return s_jy61p_i2c_error_count; }
uint32_t JY61P_I2C_GetNackCount(void)    { return s_jy61p_i2c_nack_count; }
uint32_t JY61P_I2C_GetTimeoutCount(void) { return s_jy61p_i2c_nack_count; }
