/**
 * @file  gimbal.c
 * @brief Middleware gimbal system implementation.
 */
#include "gimbal.h"
#include "bsp_time.h"
#include <stdbool.h>
#include <stddef.h>

static bool Gimbal_IsValidAxis(GIMBAL_AXIS axis){
    return axis < GIMBAL_AXIS_MAX;
}

static BSP_STATUS Gimbal_CombineStatus(BSP_STATUS current, BSP_STATUS next){
    return (current == BSP_STATUS_OK) ? next : current;
}

BSP_STATUS Gimbal_Init(void){
    BSP_STATUS status = StepMotor_Init();
    if (status != BSP_STATUS_OK){
        return status;
    }

    GIMBAL_LIMIT limit = {
        .pitch_min_deg = GIMBAL_PITCH_MIN_DEG,
        .pitch_max_deg = GIMBAL_PITCH_MAX_DEG,
    };

    return Gimbal_SetLimit(&limit);
}

BSP_STATUS Gimbal_SetSpeed(float yaw_deg_s, float pitch_deg_s){
    /* 切换速度前先积分上一段开环运动，避免估计位置在变速点丢失时间。 */
    (void)Gimbal_Update();

    BSP_STATUS status = StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_YAW, yaw_deg_s);
    BSP_STATUS pitch_status = StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_PITCH, pitch_deg_s);

    return Gimbal_CombineStatus(status, pitch_status);
}

BSP_STATUS Gimbal_Stop(void){
    return StepMotor_StopAll();
}

BSP_STATUS Gimbal_Update(void){
    return StepMotor_UpdateAllState(BSP_Time_GetMs());
}

BSP_STATUS Gimbal_GetStatus(GIMBAL_STATUS *out){
    if (out == NULL){
        return BSP_STATUS_NULL;
    }

    out->angle = Gimbal_GetAngle();
    out->speed = Gimbal_GetSpeed();
    out->limit = Gimbal_GetLimit();
    return BSP_STATUS_OK;
}

GIMBAL_ANGLE Gimbal_GetAngle(void){
    GIMBAL_ANGLE angle = {
        .yaw_deg = StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_YAW),
        .pitch_deg = StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_PITCH),
    };

    return angle;
}

GIMBAL_SPEED Gimbal_GetSpeed(void){
    GIMBAL_SPEED speed = {
        .yaw_deg_s = StepMotor_GetSpeed(STEP_MOTOR_CHANNEL_YAW),
        .pitch_deg_s = StepMotor_GetSpeed(STEP_MOTOR_CHANNEL_PITCH),
    };

    return speed;
}

void Gimbal_ResetPosition(void){
    StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_YAW);
    StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_PITCH);
}

void Gimbal_ResetAxisPosition(GIMBAL_AXIS axis){
    if (!Gimbal_IsValidAxis(axis)){
        return;
    }

    STEP_MOTOR_CHANNEL channel = (axis == GIMBAL_AXIS_YAW) ?
        STEP_MOTOR_CHANNEL_YAW : STEP_MOTOR_CHANNEL_PITCH;

    StepMotor_ResetEstimatedPosition(channel);
}

BSP_STATUS Gimbal_SetLimit(const GIMBAL_LIMIT *limit){
    if (limit == NULL){
        return BSP_STATUS_NULL;
    }

    if (limit->pitch_min_deg > limit->pitch_max_deg){
        return BSP_STATUS_INVALID_ARG;
    }

    STEP_MOTOR_POSITION_LIMIT pitch_limit = {
        .min_deg = limit->pitch_min_deg,
        .max_deg = limit->pitch_max_deg,
    };

    return StepMotor_SetPitchLimit(&pitch_limit);
}

GIMBAL_LIMIT Gimbal_GetLimit(void){
    STEP_MOTOR_POSITION_LIMIT pitch_limit = StepMotor_GetPitchLimit();
    GIMBAL_LIMIT limit = {
        .pitch_min_deg = pitch_limit.min_deg,
        .pitch_max_deg = pitch_limit.max_deg,
    };

    return limit;
}
