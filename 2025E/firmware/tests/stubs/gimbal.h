#ifndef TEST_STUB_GIMBAL_H
#define TEST_STUB_GIMBAL_H

#include "bsp_common.h"

typedef struct {
    float yaw_deg_s;
    float pitch_deg_s;
} GIMBAL_SPEED;

BSP_STATUS Gimbal_SetAngle(float yaw_deg, float pitch_deg, float yaw_rate_ff_deg_s);
BSP_STATUS Gimbal_SetSpeed(float yaw_deg_s, float pitch_deg_s);
BSP_STATUS Gimbal_HoldOnTargetLost(void);
GIMBAL_SPEED Gimbal_GetSpeed(void);
BSP_STATUS Gimbal_Stop(void);

#endif
