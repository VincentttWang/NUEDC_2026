/**
 * @file  app_e_calibration_test.h
 * @brief E 题上板实测标定入口。
 */
#ifndef APP_E_CALIBRATION_TEST_H
#define APP_E_CALIBRATION_TEST_H

/** @brief 显示标定测试子菜单并运行选中的独立测试。 */
void AppE_CalibrationTest_RunMenu(void);

/** @brief 编码器实测距离、速度和脉冲计数测试。 */
void AppE_CalibrationTest_RunEncoder(void);

/** @brief IMU yaw、gyro-z 和相对转角测试。 */
void AppE_CalibrationTest_RunImu(void);

/** @brief 关闭视觉 bias 的静态几何前馈测试。 */
void AppE_CalibrationTest_RunFeedforward(void);

/** @brief 开启视觉 bias 的静态收敛方向和增益测试。 */
void AppE_CalibrationTest_RunVisionBias(void);

/** @brief 手动改变靶面圆相位的 F3 几何测试。 */
void AppE_CalibrationTest_RunCircle(void);

#endif /* APP_E_CALIBRATION_TEST_H */
