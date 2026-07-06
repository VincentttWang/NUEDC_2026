#include "Bluetooth.h"
// This file is part of the EDC project for Car2

extern char command_From_Bluetooth; // 用于存储蓝牙接收到的命令
extern LOC target_loc; // 用于存储目标位置
extern short drug_change;
extern MODE mode; // 当前模式

char Bluetooth_Receive_Buffer[BUF_LEN] __attribute__((section(".sram_no_cache")));
char Bluetooth_Send_Buffer[BUF_LEN] __attribute__((section(".sram_no_cache")));
void Bluetooth_Init(void)
{
    // Initialize the Bluetooth module here
    // This could include setting up UART, configuring the module, etc.
    HAL_UARTEx_ReceiveToIdle_DMA(UART_HANDLE, (uint8_t *)Bluetooth_Receive_Buffer, sizeof(Bluetooth_Receive_Buffer));
    HAL_Delay(10);
}

void GetMessage(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Process the received data
    // This could include parsing commands, updating state, etc.
    // For example, you might want to parse the Bluetooth_Receive_Buffer here
    // and take appropriate actions based on the received commands.
    if (huart == UART_HANDLE) {
        switch(Size)
        {
            case 1:
                command_From_Bluetooth = Bluetooth_Receive_Buffer[0];
                if(mode.drug == WAIT_MODE && (command_From_Bluetooth == 'M' || command_From_Bluetooth == 'F'))
                {
                    drug_change = 1; // 设置标志位为1，表示需要进行药物模式的转化
                }
            break;
            case 2:
                target_loc = BStringToLoc(Bluetooth_Receive_Buffer, 2);
            break;
            default:
                // Handle other cases or ignore
                break;
        }
        // Restart receiving data
        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)Bluetooth_Receive_Buffer, sizeof(Bluetooth_Receive_Buffer));
    }
}