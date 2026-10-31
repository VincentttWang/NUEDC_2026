/**
 * @file  app_checks.h
 * @brief 外设自检任务描述符（挂在菜单树的 Device Check 子菜单下）。
 *
 * 均遵守 APP_TASK_DESC 契约，只靠 BACK 短按退出（不返回 DONE）。
 * JY61P、MPU6050 与两种 I2C 灰度模块共用 I2C0；阻塞式检查任务通过
 * on_enter/on_exit 挂起并恢复 JY61P，分时占用总线。Yaw A/B 页面仅消费 JY61P 快照，
 * 不调用底盘或电机接口。
 */
#ifndef APP_CHECKS_H
#define APP_CHECKS_H

#include "app_task.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const APP_TASK_DESC APP_CHK_GYRO_JY61P;    /**< JY61P 陀螺/姿态（I2C0）。 */
extern const APP_TASK_DESC APP_CHK_GRAY_I2C;      /**< Yahboom 8 路循线 I2C（I2C0，0x12）。 */
extern const APP_TASK_DESC APP_CHK_TB6612;        /**< TB6612 电机通道脉冲（主动）。 */
extern const APP_TASK_DESC APP_CHK_STEP_MOTOR;    /**< 摆杆步进七模式页（主动，含运行期水平点捕获）。 */
extern const APP_TASK_DESC APP_CHK_PIPE_SWING;    /**< 临时水管三档变节奏往复测试。 */
extern const APP_TASK_DESC APP_CHK_ENCODER;       /**< 霍尔编码器计数/速度/里程。 */
extern const APP_TASK_DESC APP_CHK_SPEED_PID;     /**< 双轮速度闭环 PID（主动，整定用）。 */
extern const APP_TASK_DESC APP_CHK_DUTY_SWEEP;    /**< 开环占空比扫描, 查各占空比下编码器读速。 */
extern const APP_TASK_DESC APP_CHK_RPI_UART;       /**< 树莓派球视觉 Rpi_UART/UART2 协议诊断。 */
extern const APP_TASK_DESC APP_CHK_ACCEL_CALIB;     /**< JY61P 加速度校准（读三轴+内置校准）。 */

#ifdef __cplusplus
}
#endif

#endif /* APP_CHECKS_H */
