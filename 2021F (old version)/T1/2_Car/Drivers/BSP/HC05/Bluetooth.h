#ifndef BLUETOOTH_H
#define BLUETOOTH_H
/*
  ******************************************************************************
  * @file    Bluetooth.h
  * @brief   Header file for Bluetooth communication functions.
  ******************************************************************************
  * @attention
  *
  * This file is part of the EDC project for Car2.
  *
  ******************************************************************************
  */

#include "AllHeader.h"
#include "mode.h" // 需要用到相关的mode变量

#define BUF_LEN 256
#define UART_HANDLE &huart3// UART handle for Bluetooth communication

#ifdef __cplusplus
extern "C" {
#endif

void Bluetooth_Init(void);
void GetMessage(UART_HandleTypeDef *huart, uint16_t Size);
// This function processes the received data and updates the command_From_Bluetooth variable
//写在中断里面

#ifdef __cplusplus
}   // extern "C"
#endif

#endif // BLUETOOTH_H