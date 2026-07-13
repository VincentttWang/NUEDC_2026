#include "middleware/auto_aim/auto_aim.h"

#include "canmv_uart.h"
#include "wit_sdk.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static float s_distance_m;
static WIT_IMU_DATA s_imu;
static CANMV_TARGET_DATA s_angle_data;
static float s_command_yaw;
static float s_command_pitch;
static int s_command_count;

volatile uint32_t g_canmv_uart_angle_frame_count;

float Chassis_GetDistance(void){
    return s_distance_m;
}

int WitGetData(WIT_IMU_DATA *out){
    *out = s_imu;
    return WIT_HAL_OK;
}

const CANMV_TARGET_DATA *CanMvUart_GetTargetData(CANMV_TARGET target){
    return (target == CANMV_TARGET_ANGLE) ? &s_angle_data : NULL;
}

BSP_STATUS Gimbal_SetAngle(float yaw_deg, float pitch_deg){
    s_command_yaw = yaw_deg;
    s_command_pitch = pitch_deg;
    s_command_count++;
    return BSP_STATUS_OK;
}

BSP_STATUS Gimbal_Stop(void){
    return BSP_STATUS_OK;
}

static int Near(float actual, float expected, float tolerance){
    return fabsf(actual - expected) <= tolerance;
}

int main(void){
    AUTO_AIM_CONFIG config = AutoAim_DefaultConfig();
    config.start_pose = (KINEMATICS_POSE){0.0f, 0.0f, 0.0f};
    config.encoder_lateral_offset_m = 0.1f;
    AutoAim_Init(&config);

    s_imu.attitude_deg.yaw = 20.0f;
    assert(AutoAim_Start(AUTO_AIM_MODE_CENTER) == BSP_STATUS_OK);

    s_distance_m = 0.1f;
    s_imu.gyro_deg_s.z = 90.0f;
    s_angle_data.status = CANMV_STATUS_OK;
    s_angle_data.count = 2U;
    s_angle_data.value[0] = (uint16_t)(int16_t)100;
    s_angle_data.value[1] = (uint16_t)(int16_t)-50;
    g_canmv_uart_angle_frame_count = 1U;
    assert(AutoAim_Update(0.1f) == BSP_STATUS_OK);

    AUTO_AIM_STATE state = AutoAim_GetState();
    assert(state.imu_valid);
    assert(state.vision_valid);
    assert(Near(state.pose.heading_deg, 0.0f, 0.001f));
    assert(state.last_center_delta_m > state.last_wheel_delta_m);
    assert(Near(state.vision_bias.yaw_bias_deg, -0.035f, 0.001f));
    assert(Near(state.vision_bias.pitch_bias_deg, -0.0175f, 0.001f));
    assert(s_command_count == 1);
    assert(Near(s_command_yaw, state.yaw_command_deg, 0.001f));
    assert(Near(s_command_pitch, state.pitch_command_deg, 0.001f));

    AutoAim_AnchorCorner(LOCALIZATION_CORNER_B, 1.0f);
    state = AutoAim_GetState();
    assert(Near(state.pose.x_m, 0.5f, 0.001f));
    assert(Near(state.lap_progress, 0.25f, 0.001f));
    assert(AutoAim_Stop() == BSP_STATUS_OK);
    puts("auto aim tests passed");
    return 0;
}
