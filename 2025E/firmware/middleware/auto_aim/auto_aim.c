#include "auto_aim.h"

#include "canmv_uart.h"
#include "chassis.h"
#include "gimbal.h"
#include "wit_sdk.h"

#include <stddef.h>

static AUTO_AIM_CONFIG s_config;
static AUTO_AIM_STATE s_state;
static float s_last_wheel_distance_m;
static float s_heading_offset_deg;
static bool s_initialized;

AUTO_AIM_CONFIG AutoAim_DefaultConfig(void){
    AUTO_AIM_CONFIG config = {
        /* 默认从 A 点沿 A->C 行驶；实机起点改变时只需覆盖此配置。 */
        .start_pose = {-0.5f, 0.0f, -90.0f},
        .track_side_m = AUTO_AIM_DEFAULT_TRACK_SIDE_M,
        .encoder_lateral_offset_m = AUTO_AIM_DEFAULT_ENCODER_LATERAL_OFFSET_M,
        .vision_bias_ki = AUTO_AIM_DEFAULT_VISION_BIAS_KI,
        .vision_bias_limit_deg = AUTO_AIM_DEFAULT_VISION_BIAS_LIMIT_DEG,
        .vision_yaw_sign = AUTO_AIM_DEFAULT_VISION_YAW_SIGN,
        .vision_pitch_sign = AUTO_AIM_DEFAULT_VISION_PITCH_SIGN,
        .circle_phase0_deg = AUTO_AIM_DEFAULT_CIRCLE_PHASE0_DEG,
        .solver = AimSolver_DefaultConfig(),
    };
    return config;
}

void AutoAim_Init(const AUTO_AIM_CONFIG *config){
    s_config = (config == NULL) ? AutoAim_DefaultConfig() : *config;
    s_state = (AUTO_AIM_STATE){0};
    AimVisionBias_Init(&s_state.vision_bias,
                       s_config.vision_bias_ki,
                       s_config.vision_bias_limit_deg);
    AimSolver_Init(&s_config.solver);
    s_initialized = true;
}

static void AutoAim_EnsureInitialized(void){
    if (!s_initialized){
        AutoAim_Init(NULL);
    }
}

BSP_STATUS AutoAim_Start(AUTO_AIM_MODE mode){
    WIT_IMU_DATA imu = {0};

    AutoAim_EnsureInitialized();
    if ((mode != AUTO_AIM_MODE_CENTER) && (mode != AUTO_AIM_MODE_CIRCLE)){
        return BSP_STATUS_INVALID_ARG;
    }

    s_state = (AUTO_AIM_STATE){0};
    s_state.active = true;
    s_state.mode = mode;
    AimVisionBias_Init(&s_state.vision_bias,
                       s_config.vision_bias_ki,
                       s_config.vision_bias_limit_deg);

    s_last_wheel_distance_m = Chassis_GetDistance();
    Localization_Init(s_config.start_pose,
                      s_config.track_side_m,
                      s_last_wheel_distance_m);

    if (WitGetData(&imu) == WIT_HAL_OK){
        s_heading_offset_deg = Kinematics_AngleDiffDeg(s_config.start_pose.heading_deg,
                                                       imu.attitude_deg.yaw);
        s_state.imu_valid = true;
    } else{
        s_heading_offset_deg = 0.0f;
        s_state.imu_valid = false;
    }
    s_state.vision_frame_count = g_canmv_uart_angle_frame_count;
    return s_state.imu_valid ? BSP_STATUS_OK : BSP_STATUS_NOT_READY;
}

static bool AutoAim_ReadNewVision(float *yaw_error_deg,
                                  float *pitch_error_deg){
    uint32_t frame_count = g_canmv_uart_angle_frame_count;
    const CANMV_TARGET_DATA *data;

    if (frame_count == s_state.vision_frame_count){
        return false;
    }
    s_state.vision_frame_count = frame_count;
    data = CanMvUart_GetTargetData(CANMV_TARGET_ANGLE);
    if ((data == NULL) || (data->status != CANMV_STATUS_OK) || (data->count < 2U)){
        return false;
    }

    *yaw_error_deg = (float)(int16_t)data->value[0] / CANMV_ANGLE_SCALE;
    *pitch_error_deg = (float)(int16_t)data->value[1] / CANMV_ANGLE_SCALE;
    return true;
}

BSP_STATUS AutoAim_Update(float dt_s){
    WIT_IMU_DATA imu = {0};
    float wheel_distance_m;
    float heading_deg;
    float yaw_error_deg = 0.0f;
    float pitch_error_deg = 0.0f;

    AutoAim_EnsureInitialized();
    if (!s_state.active){
        return BSP_STATUS_NOT_READY;
    }
    if (dt_s <= 0.0f){
        return BSP_STATUS_INVALID_ARG;
    }
    if (dt_s > 0.1f){
        dt_s = 0.1f;
    }

    if (WitGetData(&imu) != WIT_HAL_OK){
        s_state.imu_valid = false;
        return BSP_STATUS_NOT_READY;
    }
    s_state.imu_valid = true;

    wheel_distance_m = Chassis_GetDistance();
    s_state.last_wheel_delta_m = wheel_distance_m - s_last_wheel_distance_m;
    s_last_wheel_distance_m = wheel_distance_m;
    s_state.last_center_delta_m = Localization_CorrectWheelDelta(
        s_state.last_wheel_delta_m,
        imu.gyro_deg_s.z,
        s_config.encoder_lateral_offset_m,
        dt_s);
    heading_deg = Kinematics_NormalizeAngleDeg(imu.attitude_deg.yaw +
                                                s_heading_offset_deg);
    Localization_UpdateDelta(s_state.last_center_delta_m, heading_deg);
    s_state.pose = Localization_GetPose();
    s_state.lap_progress = Localization_GetLapProgress();

    s_state.vision_valid = AutoAim_ReadNewVision(&yaw_error_deg, &pitch_error_deg);
    if (s_state.vision_valid){
        s_state.vision_yaw_error_deg = yaw_error_deg;
        s_state.vision_pitch_error_deg = pitch_error_deg;
    }
    AimVisionBias_Update(&s_state.vision_bias,
                         s_config.vision_yaw_sign * yaw_error_deg,
                         s_config.vision_pitch_sign * pitch_error_deg,
                         s_state.vision_valid,
                         dt_s);

    if (s_state.mode == AUTO_AIM_MODE_CIRCLE){
        float phase_deg = AimSolver_ProgressToPhaseDeg(s_state.lap_progress,
                                                       s_config.circle_phase0_deg);
        s_state.feedforward = AimSolver_SolveCircle(s_state.pose, phase_deg);
    } else{
        s_state.feedforward = AimSolver_SolveCenter(s_state.pose);
    }

    s_state.yaw_command_deg = s_state.feedforward.yaw_cmd_deg +
                              s_state.vision_bias.yaw_bias_deg;
    s_state.pitch_command_deg = s_state.feedforward.pitch_cmd_deg +
                                s_state.vision_bias.pitch_bias_deg;
    return Gimbal_SetAngle(s_state.yaw_command_deg, s_state.pitch_command_deg);
}

void AutoAim_AnchorCorner(LOCALIZATION_CORNER corner, float travelled_m){
    if (!s_state.active){
        return;
    }
    Localization_ResetToCornerAtTravel(corner, travelled_m);
    s_state.pose = Localization_GetPose();
    s_state.lap_progress = Localization_GetLapProgress();
}

BSP_STATUS AutoAim_Stop(void){
    s_state.active = false;
    s_state.vision_valid = false;
    return Gimbal_Stop();
}

AUTO_AIM_STATE AutoAim_GetState(void){
    return s_state;
}
