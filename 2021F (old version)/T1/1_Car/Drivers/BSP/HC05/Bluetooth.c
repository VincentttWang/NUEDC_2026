#include "Bluetooth.h"
#include "AllHeader.h"

// This file is part of the EDC project for Car1.

extern MODE mode;
char Bluetooth_Receive_Buffer[BUF_LEN] __attribute__((section(".sram_no_cache")));
char Bluetooth_Send_Buffer[BUF_LEN] __attribute__((section(".sram_no_cache")));
void Bluetooth_Init(void)
{
    // Initialize the Bluetooth module here
    // This could include setting up UART, configuring the module, etc.
    HAL_UARTEx_ReceiveToIdle_DMA(UART_HANDLE, (uint8_t *)Bluetooth_Receive_Buffer, sizeof(Bluetooth_Receive_Buffer));
    HAL_Delay(10);
}

void SendTrace(LOC loc)
{
   for(int i = 0; i < loc.n; i++)
   {
       Bluetooth_Send_Buffer[i] = loc.trace[i];
   }
    Bluetooth_Send_Buffer[loc.n] = 20; // 表示结束符
    HAL_UART_Transmit_DMA(UART_HANDLE, (uint8_t *)Bluetooth_Send_Buffer, loc.n);
    
}

void SendBegin(void)//发送开始送药的指令，实际上
{
  // Your code to send the beginning of a Bluetooth message goes here
    if(mode.loc.n == 2)
    {
        HAL_UART_Transmit(UART_HANDLE, (uint8_t *)"M", 1, HAL_MAX_DELAY);// Send 'M' to indicate the start of middle trace
        HAL_Delay(10);
        SendTrace(mode.loc);
    }
    else if(mode.loc.n == 4)
    {
        HAL_UART_Transmit(UART_HANDLE, (uint8_t *)"F", 1, HAL_MAX_DELAY);// Send 'F' to indicate the start of far trace
    }
}

void SendContinue(void)//发送继续送药的指令
{
    // Your code to send the continuation of a Bluetooth message goes here
    HAL_UART_Transmit(UART_HANDLE, (uint8_t *)"C", 1, HAL_MAX_DELAY);// Send 'C' to indicate continue
}
