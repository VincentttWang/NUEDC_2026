#ifndef VISUAL_H
#define VISUAL_H

#include "AllHeader.h"

// 定义接受长度
#define USART_RX_BUF_LEN 25 
// 采样
#define MAX_SAMPLES 10
// 定义接受长度
#define USART_EYE_RX_LEN 25 

void setRoomArray(void);
void SampleGoalNumFromArr(uint8_t arr[]);
void ResetSampling();
void control(void);
// 控制函数
// 根据采样结果更新模式的方向
#endif // VISUAL_H