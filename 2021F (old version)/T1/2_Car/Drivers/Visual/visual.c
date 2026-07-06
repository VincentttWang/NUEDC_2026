#include "visual.h"
#include "mode.h"
// 刘1


// 接收状态和缓冲区
extern uint8_t USART_RX_BUF[USART_RX_BUF_LEN]; 
// 真正接受数据的函数
extern uint8_t arr[25];

extern MODE mode; // 全局模式变量
// 储存房间号的数组
uint8_t RoomArray[25] = {0} ;
// 房间数
int len ;
// 设置方向参数
DIR dir ;
// 取样得到目标房间
uint8_t goalRoomArray[100] = {0} ;
// 目标房间
int goalRoom = -1 ;

uint8_t sample_buffer[MAX_SAMPLES];
int sample_index = 0;
int sampling_done = 0; // 采样完成标志


// 串口调试
// 接收状态和缓冲区
extern uint8_t USART_EYE_RX_BUF[USART_EYE_RX_LEN]; 
// 真正接受数据的函数

// 刘欣

void setRoomArray(void)
{
    if (arr[0] != 0x12) return; // 帧头校验
	
    len = arr[1];

    if (arr[len + 2] != 0x5B) return; // 帧尾校验（+2是因为 arr[0] 是帧头，arr[1] 是长度）

    RoomArray[0] = len; // 保存数据个数
	
    for (int i = 0; i < len; i++) 
	{
        RoomArray[i + 1] = (int)arr[i + 2]; // 从 arr[2] 开始是数据段
    }
}

/*
num = room 
if 1
 dir = left
elif 2
 dir = right
else
  dir = forward
	num = room
  if ar[] has num
		if x_num < len(ar) / 2
*/
void control()
{
	// 单个数字
	if (RoomArray[0] == 1 && RoomArray[1] == 1)
	{
		mode.dir = LEFT ;
	}
	else if (RoomArray[0] == 1 &&RoomArray[1] == 2)
	{
		mode.dir = RIGHT ;
	}
	else 
	{
		mode.dir = FORWARD ;
		if (RoomArray[0]>0)
		{
      // 多个数字
      int isFound = 0;
			for (int j = 0 ; j < RoomArray[0] ; j ++)
			{
				if (goalRoom == RoomArray[j])
				{
          isFound = 1; // 找到目标房间
					if (j < RoomArray[0] / 2)
					{
						mode.dir = LEFT ;
					}
					else 
					{
						mode.dir = RIGHT ;
					}
				}
			}
      if (!isFound) // 如果没有找到目标房间
      {
        mode.dir = FORWARD ; // 默认前进
      }
		}
	}
}

// 重置采样
void ResetSampling()
{
    sample_index = 0;
    sampling_done = 0;
}

// 采样次数为10
void SampleGoalNumFromArr(uint8_t arr[])
{
    if (sampling_done) return; // 采样结束不再采样

    int len = arr[1];
	
    if (len == 1)
    {
        uint8_t val = arr[2]; // 检查是否超标
        if (val <= 9)
        {
			
            if (sample_index < MAX_SAMPLES )
            {
                sample_buffer[sample_index++] = val;
				
            }

            if (sample_index >= MAX_SAMPLES )
            {
                int freq[10] = {0};
                for (int i = 0; i < MAX_SAMPLES ; i++)
                {
                    freq[sample_buffer[i]]++;
                }

                int maxCount = 0;
                int maxNum = 0;
                for (int i = 0; i < 10; i++)
                {
                    if (freq[i] > maxCount)
                    {
                        maxCount = freq[i];
                        maxNum = i;
                    }
                }
                goalRoom = maxNum;    // 更新目标数字
                sampling_done = 1;    // 标记采样完成
            }
        }
    }
}
