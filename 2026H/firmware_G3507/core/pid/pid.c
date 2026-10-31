#include "pid.h"

#include <stddef.h>

static float PID_Limit(float value, float limit){
    if (limit <= 0.0f){
        return value;
    }

    if (value > limit){
        return limit;
    }

    if (value < -limit){
        return -limit;
    }

    return value;
}

void PID_Init(PID_CONTROLLER *pid, const PID_CONFIG *config){
    if ((pid == NULL) || (config == NULL)){
        return;
    }

    pid->config = *config;
    PID_Reset(pid);
}

void PID_Reset(PID_CONTROLLER *pid){
    if (pid == NULL){
        return;
    }

    pid->state.target = 0.0f;
    pid->state.feedback = 0.0f;
    pid->state.error = 0.0f;
    pid->state.last_error = 0.0f;
    pid->state.prev_error = 0.0f;
    pid->state.integral = 0.0f;
    pid->state.derivative = 0.0f;
    pid->state.increment = 0.0f;
    pid->state.output = 0.0f;
}

float PID_Update(PID_CONTROLLER *pid, float target, float feedback, float dt_s){
    if (pid == NULL){
        return 0.0f;
    }

    if (dt_s <= 0.0f){
        return pid->state.output;
    }

    float last_output = pid->state.output;
    float old_error = pid->state.error;
    float old_last_error = pid->state.last_error;

    pid->state.target = target;
    pid->state.feedback = feedback;
    pid->state.prev_error = old_last_error;
    pid->state.last_error = old_error;
    pid->state.error = target - feedback;

    if (pid->config.mode == PID_MODE_INCREMENTAL){
        /* 预留：增量式路径当前工程无调用者。量纲与位置式一致:
         * ki*e*dt 对应积分、kd*(e-2*le+pe)/dt 对应二阶差分微分。 */
        pid->state.derivative = (pid->state.error -
                                 2.0f * pid->state.last_error +
                                 pid->state.prev_error) / dt_s;
        pid->state.increment =
            pid->config.kp * (pid->state.error - pid->state.last_error) +
            pid->config.ki * pid->state.error * dt_s +
            pid->config.kd * pid->state.derivative;
        pid->state.output += pid->state.increment;
    } else{
        pid->state.integral += pid->state.error * dt_s;
        pid->state.integral = PID_Limit(pid->state.integral,
                                        pid->config.integral_limit);
        pid->state.derivative =
            (pid->state.error - pid->state.last_error) / dt_s;
        pid->state.output =
            pid->config.kp * pid->state.error +
            pid->config.ki * pid->state.integral +
            pid->config.kd * pid->state.derivative;
    }

    pid->state.output = PID_Limit(pid->state.output,
                                  pid->config.output_limit);
    pid->state.increment = pid->state.output - last_output;

    return pid->state.output;
}
