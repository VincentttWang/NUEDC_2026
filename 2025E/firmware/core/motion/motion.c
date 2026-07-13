#include "motion.h"

#include <stddef.h>

static MOTION_STATE s_motion_state;

static float Motion_Abs(float value){
    return (value < 0.0f) ? -value : value;
}

static MOTION_COMMAND Motion_MakeCommand(MOTION_MODE mode, float duty_percent){
    MOTION_COMMAND command = {
        .mode = mode,
        .duty_percent = Motion_Abs(duty_percent),
    };

    return command;
}

static void Motion_UpdateState(const MOTION_COMMAND *command,
                               BSP_STATUS status){
    if (command != NULL){
        s_motion_state.mode = command->mode;
        s_motion_state.command = *command;
    }

    s_motion_state.duty = Chassis_GetDuty();
    s_motion_state.line_output = LineTracking_GetOutput();
    s_motion_state.last_status = status;
}

void Motion_Init(void){
    s_motion_state.mode = MOTION_MODE_IDLE;
    s_motion_state.command = Motion_MakeCommand(MOTION_MODE_IDLE, 0.0f);
    s_motion_state.duty = Chassis_GetDuty();
    s_motion_state.line_output = LineTracking_GetOutput();
    s_motion_state.last_status = BSP_STATUS_OK;
}

BSP_STATUS Motion_Apply(const MOTION_COMMAND *command, float dt_s){
    if (command == NULL){
        Motion_UpdateState(NULL, BSP_STATUS_NULL);
        return BSP_STATUS_NULL;
    }

    BSP_STATUS status = BSP_STATUS_OK;
    float duty = Motion_Abs(command->duty_percent);

    switch (command->mode){
        case MOTION_MODE_IDLE:
            status = BSP_STATUS_OK;
            break;
        case MOTION_MODE_BRAKE:
            status = Chassis_Brake();
            break;
        case MOTION_MODE_COAST:
            status = Chassis_Coast();
            break;
        case MOTION_MODE_STRAIGHT:
            status = Chassis_SetDuty(duty, duty);
            break;
        case MOTION_MODE_BACKWARD:
            status = Chassis_SetDuty(-duty, -duty);
            break;
        case MOTION_MODE_SPIN_LEFT:
            status = Chassis_SetDuty(-duty, duty);
            break;
        case MOTION_MODE_SPIN_RIGHT:
            status = Chassis_SetDuty(duty, -duty);
            break;
        case MOTION_MODE_LINE_FOLLOW:
            status = LineTracking_Update(dt_s);
            break;
        default:
            status = BSP_STATUS_INVALID_ARG;
            break;
    }

    Motion_UpdateState(command, status);
    return status;
}

BSP_STATUS Motion_Stop(void){
    MOTION_COMMAND command = Motion_CommandBrake();
    BSP_STATUS status = Chassis_Brake();

    command.mode = MOTION_MODE_IDLE;
    Motion_UpdateState(&command, status);
    return status;
}

MOTION_STATE Motion_GetState(void){
    return s_motion_state;
}

MOTION_COMMAND Motion_CommandBrake(void){
    return Motion_MakeCommand(MOTION_MODE_BRAKE, 0.0f);
}

MOTION_COMMAND Motion_CommandCoast(void){
    return Motion_MakeCommand(MOTION_MODE_COAST, 0.0f);
}

MOTION_COMMAND Motion_CommandStraight(float duty_percent){
    return Motion_MakeCommand(MOTION_MODE_STRAIGHT, duty_percent);
}

MOTION_COMMAND Motion_CommandBackward(float duty_percent){
    return Motion_MakeCommand(MOTION_MODE_BACKWARD, duty_percent);
}

MOTION_COMMAND Motion_CommandSpinLeft(float duty_percent){
    return Motion_MakeCommand(MOTION_MODE_SPIN_LEFT, duty_percent);
}

MOTION_COMMAND Motion_CommandSpinRight(float duty_percent){
    return Motion_MakeCommand(MOTION_MODE_SPIN_RIGHT, duty_percent);
}

MOTION_COMMAND Motion_CommandLineFollow(void){
    return Motion_MakeCommand(MOTION_MODE_LINE_FOLLOW, 0.0f);
}
