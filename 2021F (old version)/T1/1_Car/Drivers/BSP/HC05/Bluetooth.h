#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "mode.h"//需要用到相关的mode变量

#define BUF_LEN 256
#define UART_HANDLE &huart3// UART handle for Bluetooth communication

#ifdef __cplusplus
extern "C" {
#endif

void Bluetooth_Init(void);
void SendTrace(LOC loc);
void SendBegin(void);
void SendContinue(void);

#ifdef __cplusplus
}   // extern "C"
#endif

#endif // BLUETOOTH_H