#include "kinematics.h"

#include <math.h>

/* 判定 ω≈0(直线)的角速度阈值，rad/s。 */
#define KINEMATICS_OMEGA_EPS 1.0e-6f

static float Kinematics_Abs(float value){
    return (value < 0.0f) ? -value : value;
}

float Kinematics_Clamp(float value, float min_value, float max_value){
    if (min_value > max_value){
        float temp = min_value;
        min_value = max_value;
        max_value = temp;
    }

    if (value > max_value){
        return max_value;
    }

    if (value < min_value){
        return min_value;
    }

    return value;
}

float Kinematics_NormalizeAngleDeg(float angle_deg){
    while (angle_deg >= 180.0f){
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f){
        angle_deg += 360.0f;
    }

    return angle_deg;
}

float Kinematics_AngleDiffDeg(float target_deg, float current_deg){
    return Kinematics_NormalizeAngleDeg(target_deg - current_deg);
}

KINEMATICS_DIFFERENTIAL_OUTPUT Kinematics_DifferentialMix(float forward,
                                                          float turn,
                                                          float output_limit){
    KINEMATICS_DIFFERENTIAL_OUTPUT output = {
        .left = forward + turn,
        .right = forward - turn,
    };

    if (output_limit <= 0.0f){
        return output;
    }

    float max_abs = Kinematics_Abs(output.left);
    float right_abs = Kinematics_Abs(output.right);

    if (right_abs > max_abs){
        max_abs = right_abs;
    }

    if (max_abs > output_limit){
        float scale = output_limit / max_abs;
        output.left *= scale;
        output.right *= scale;
    }

    return output;
}

/* ═══════════════ 两轮差速运动学模型 ═══════════════ */

KINEMATICS_VELOCITY Kinematics_WheelToBody(float left_mps, float right_mps,
                                           float track_width_m){
    KINEMATICS_VELOCITY vel = {
        .linear_mps = (left_mps + right_mps) * 0.5f,
        .angular_deg_s = 0.0f,
    };

    if (track_width_m > 0.0f){
        float omega_rad_s = (right_mps - left_mps) / track_width_m;
        vel.angular_deg_s = KINEMATICS_RAD_TO_DEG(omega_rad_s);
    }

    return vel;
}

KINEMATICS_WHEEL_SPEED Kinematics_BodyToWheel(float linear_mps, float angular_deg_s,
                                              float track_width_m){
    float omega_rad_s = KINEMATICS_DEG_TO_RAD(angular_deg_s);
    float half_track = track_width_m * 0.5f;
    KINEMATICS_WHEEL_SPEED wheel = {
        .left_mps = linear_mps - omega_rad_s * half_track,
        .right_mps = linear_mps + omega_rad_s * half_track,
    };

    return wheel;
}

float Kinematics_WheelLinearToAngular(float linear_mps, float wheel_radius_m){
    if (wheel_radius_m <= 0.0f){
        return 0.0f;
    }
    return linear_mps / wheel_radius_m;
}

float Kinematics_WheelAngularToLinear(float angular_rad_s, float wheel_radius_m){
    return angular_rad_s * wheel_radius_m;
}

float Kinematics_TurnRadius(float linear_mps, float angular_deg_s){
    float omega_rad_s = KINEMATICS_DEG_TO_RAD(angular_deg_s);

    if (Kinematics_Abs(omega_rad_s) < KINEMATICS_OMEGA_EPS){
        return KINEMATICS_TURN_RADIUS_STRAIGHT;
    }

    return linear_mps / omega_rad_s;
}

KINEMATICS_POSE Kinematics_IntegratePose(KINEMATICS_POSE pose, KINEMATICS_VELOCITY vel,
                                         float dt_s){
    if (dt_s <= 0.0f){
        return pose;
    }

    /* 中点欧拉：用半步航向作位移方向，转弯时比前向欧拉更准；直线(ω=0)自然退化。 */
    float heading_mid_deg = pose.heading_deg + vel.angular_deg_s * dt_s * 0.5f;
    float heading_mid_rad = KINEMATICS_DEG_TO_RAD(heading_mid_deg);
    float distance = vel.linear_mps * dt_s;

    pose.x_m += distance * cosf(heading_mid_rad);
    pose.y_m += distance * sinf(heading_mid_rad);
    pose.heading_deg = Kinematics_NormalizeAngleDeg(
        pose.heading_deg + vel.angular_deg_s * dt_s);

    return pose;
}

