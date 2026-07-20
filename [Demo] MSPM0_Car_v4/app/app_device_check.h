/**
 * @file  app_device_check.h
 * @brief App 层设备检查页面和 IMU 调试数据入口。
 */
#ifndef APP_DEVICE_CHECK_H
#define APP_DEVICE_CHECK_H

#include <stdint.h>

/**
 * @brief 进入设备检查页面。
 */
void AppDeviceCheck_Run(void);

/**
 * @brief 处理 IMU 调试串口收到的单字节数据。
 * @param byte UART 接收字节。
 */
void AppDeviceCheck_ProcessImuByte(uint8_t byte);

#endif /* APP_DEVICE_CHECK_H */
