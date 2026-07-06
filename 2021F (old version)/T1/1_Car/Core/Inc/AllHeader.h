#ifndef ALLHEADER_H
#define ALLHEADER_H

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// MS601M 传感器头文件
#include "atk_ms601m.h"
#include "atk_ms601m_uart.h"

// 感为灰度传感器头文件
#include "GanWei_i2c.h"
#include "gw_grayscale_sensor.h"

//巡线程序头文件
#include "tracking.h"
#include "tracking_delay.h"

// 电机驱动头文件
#include "tb6612fng.h"

// 霍尔编码器头文件
#include "Hall_Encoder.h"

// 内存管理头文件
#include "Mem.h"

// 电机和编码器初始化头文件
#include "Initialize.h"

//OLED显示屏头文件
#include "oled.h"
//状态机头文件,但是我的mode文件在逻辑上属于最下层，
//也就是实际上它时建立在以上文件的基础上写成的，包含了我在kinematics中写成的方向等等变量类型
//所以需要单独include，避免循环出错
#include "Bluetooth.h"
#include "visual.h"

#include "led.h"

#ifdef __cplusplus
extern "C" {
#endif

#endif /* ALLHEADER_H */