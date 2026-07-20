/**
 * @file  chassis.c
 * @brief Middleware chassis system implementation.
 */
#include "chassis.h"
#include <stddef.h>

static CHASSIS_DUTY s_chassis_duty;

static BSP_STATUS Chassis_CombineStatus(BSP_STATUS current, BSP_STATUS next){
    return (current == BSP_STATUS_OK) ? next : current;
}

static void Chassis_ClearDuty(void){
    s_chassis_duty.left_percent = 0.0f;
    s_chassis_duty.right_percent = 0.0f;
}

BSP_STATUS Chassis_Init(void){
    Chassis_ClearDuty();

    BSP_STATUS status = TB6612FNG_Init();
    status = Chassis_CombineStatus(status, HallEncoder_Init());
    return status;
}

BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent){
    BSP_STATUS status = TB6612FNG_SetDuty(TB6612FNG_CHANNEL_LEFT, left_percent);
    BSP_STATUS right_status = TB6612FNG_SetDuty(TB6612FNG_CHANNEL_RIGHT, right_percent);

    status = Chassis_CombineStatus(status, right_status);

    if (status == BSP_STATUS_OK){
        s_chassis_duty.left_percent = TB6612FNG_GetDuty(TB6612FNG_CHANNEL_LEFT);
        s_chassis_duty.right_percent = TB6612FNG_GetDuty(TB6612FNG_CHANNEL_RIGHT);
    }

    return status;
}

BSP_STATUS Chassis_Stop(CHASSIS_STOP_MODE mode){
    BSP_STATUS status = BSP_STATUS_OK;

    switch (mode){
        case CHASSIS_STOP_MODE_BRAKE:
            status = TB6612FNG_BrakeAll();
            break;
        case CHASSIS_STOP_MODE_COAST:
            status = TB6612FNG_CoastAll();
            break;
        default:
            return BSP_STATUS_INVALID_ARG;
    }

    if (status == BSP_STATUS_OK){
        Chassis_ClearDuty();
    }

    return status;
}

BSP_STATUS Chassis_Brake(void){
    return Chassis_Stop(CHASSIS_STOP_MODE_BRAKE);
}

BSP_STATUS Chassis_Coast(void){
    return Chassis_Stop(CHASSIS_STOP_MODE_COAST);
}

BSP_STATUS Chassis_Update(void){
    HallEncoder_UpdateSample();
    return BSP_STATUS_OK;
}

void Chassis_HandleEncoderIrq(uint32_t gpio_status){
    HallEncoder_HandleGpioIrq(gpio_status);
}

void Chassis_HandleEncoderTimerIrq(void){
    HallEncoder_UpdateSample();
}

BSP_STATUS Chassis_GetStatus(CHASSIS_STATUS *out){
    if (out == NULL){
        return BSP_STATUS_NULL;
    }

    out->duty = Chassis_GetDuty();
    out->speed_mps = HallEncoder_GetSpeed();
    out->distance_m = HallEncoder_GetDistance();
    out->encoder_dir = HallEncoder_GetDir();
    out->left_output = TB6612FNG_GetOutputStatus(TB6612FNG_CHANNEL_LEFT);
    out->right_output = TB6612FNG_GetOutputStatus(TB6612FNG_CHANNEL_RIGHT);
    return BSP_STATUS_OK;
}

CHASSIS_DUTY Chassis_GetDuty(void){
    return s_chassis_duty;
}

float Chassis_GetSpeed(void){
    return HallEncoder_GetSpeed();
}

float Chassis_GetDistance(void){
    return HallEncoder_GetDistance();
}

HALL_ENCODER_DIR Chassis_GetEncoderDir(void){
    return HallEncoder_GetDir();
}

void Chassis_ResetDistance(void){
    HallEncoder_ResetDistance();
}
