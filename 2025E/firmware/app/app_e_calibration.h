/**
 * @file  app_e_calibration.h
 * @brief E 题实机标定参数集中配置。
 */
#ifndef APP_E_CALIBRATION_H
#define APP_E_CALIBRATION_H

#include "middleware/auto_aim/auto_aim.h"

#include <stdint.h>

typedef struct {
    AUTO_AIM_CONFIG auto_aim;
    float aim_lock_yaw_deg;
    float aim_lock_pitch_deg;
    uint8_t aim_lock_confirm_frames;
    uint32_t e2_force_shot_ms;
    uint32_t e3_force_shot_ms;
    uint32_t laser_pulse_ms;
    uint8_t imu_fail_limit;
} APP_E_CALIBRATION_CONFIG;

/**
 * @brief 返回整车比赛标定配置。
 *
 * @note 实机标定时集中修改本函数，不要在任务状态机中散改常量。
 */
APP_E_CALIBRATION_CONFIG AppE_GetCalibrationConfig(void);

#endif /* APP_E_CALIBRATION_H */
