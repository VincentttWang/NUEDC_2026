#include "filter.h"

float Filter_LowpassEma(float state, float sample, float alpha){
    return state + alpha * (sample - state);
}

float Filter_Deadband(float value, float threshold){
    if (threshold <= 0.0f){
        return value;
    }

    if ((value > -threshold) && (value < threshold)){
        return 0.0f;
    }

    return value;
}
