#ifndef TEST_STUB_GIMBAL_H
#define TEST_STUB_GIMBAL_H

#include "bsp_common.h"

BSP_STATUS Gimbal_SetAngle(float yaw_deg, float pitch_deg);
BSP_STATUS Gimbal_Stop(void);

#endif
