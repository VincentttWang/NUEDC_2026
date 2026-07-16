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
static float s_v_ratef_lpf;       /* 速率前馈用的低通 v 状态 (m/s), 去里程混叠尖峰 */
static bool s_initialized;

AUTO_AIM_CONFIG AutoAim_DefaultConfig(void){
    AUTO_AIM_CONFIG config = {
        /* 默认从 A 点沿 A->C 行驶；实机起点改变时只需覆盖此配置。 */
        .start_pose = {-0.5f, 0.0f, -90.0f},
        .track_side_m = AUTO_AIM_DEFAULT_TRACK_SIDE_M,
        .encoder_lateral_offset_m = AUTO_AIM_DEFAULT_ENCODER_LATERAL_OFFSET_M,
        .vision_yaw = {
            .gain0 = AUTO_AIM_DEFAULT_VISION_YAW_GAIN0,
            .gain_min = AUTO_AIM_DEFAULT_VISION_YAW_GAIN_MIN,
            .limit_deg = AUTO_AIM_DEFAULT_VISION_YAW_LIMIT_DEG,
        },
        .vision_pitch = {
            .gain0 = AUTO_AIM_DEFAULT_VISION_PITCH_GAIN0,
            .gain_min = AUTO_AIM_DEFAULT_VISION_PITCH_GAIN_MIN,
            .limit_deg = AUTO_AIM_DEFAULT_VISION_PITCH_LIMIT_DEG,
        },
        .vision_yaw_startup_limit_deg = AUTO_AIM_DEFAULT_VISION_YAW_STARTUP_LIMIT_DEG,
        .vision_pitch_startup_limit_deg = AUTO_AIM_DEFAULT_VISION_PITCH_STARTUP_LIMIT_DEG,
        .vision_yaw_sign = AUTO_AIM_DEFAULT_VISION_YAW_SIGN,
        .vision_pitch_sign = AUTO_AIM_DEFAULT_VISION_PITCH_SIGN,
        .circle_phase0_deg = AUTO_AIM_DEFAULT_CIRCLE_PHASE0_DEG,
        .pitch_rate_lead_s = AUTO_AIM_DEFAULT_PITCH_RATE_LEAD_S,
        .rate_ff_v_tau_s = AUTO_AIM_DEFAULT_RATE_FF_V_TAU_S,
        .gyro_z_sign = AUTO_AIM_DEFAULT_GYRO_Z_SIGN,
        .solver = AimSolver_DefaultConfig(),
    };
    return config;
}

/* 起步对齐轴增益 = 该轴慢跟踪增益换上独立的起步限幅。 */
static AIM_VISION_GAIN AutoAim_StartupGain(AIM_VISION_GAIN axis, float limit_deg){
    axis.limit_deg = limit_deg;
    return axis;
}

/* 重置两个视觉校正器 (bias 与帧计数清零, 从首帧快收敛重新开始)。 */
static void AutoAim_ResetVisionBias(void){
    AimVisionBias_Init(&s_state.vision_bias,
                       s_config.vision_yaw,
                       s_config.vision_pitch);
    AimVisionBias_Init(&s_state.startup_bias,
                       AutoAim_StartupGain(s_config.vision_yaw,
                                           s_config.vision_yaw_startup_limit_deg),
                       AutoAim_StartupGain(s_config.vision_pitch,
                                           s_config.vision_pitch_startup_limit_deg));
}

void AutoAim_Init(const AUTO_AIM_CONFIG *config){
    s_config = (config == NULL) ? AutoAim_DefaultConfig() : *config;
    s_state = (AUTO_AIM_STATE){0};
    AutoAim_ResetVisionBias();
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
    s_v_ratef_lpf = 0.0f;
    /* bias 与帧计数清零: 每次任务从首帧快收敛重新开始。 */
    AutoAim_ResetVisionBias();
    s_state.startup_align = false;

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
    s_state.vision_correction_enabled = true;
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

    /* 陀螺 yaw-rate 取符号使其与航向变化率同号 (实测本机 gz 反号)。
     * ω 同时用于车心速度重构与速率前馈, 符号错会让两者都反向。 */
    float omega_deg_s = s_config.gyro_z_sign * imu.gyro_deg_s.z;

    wheel_distance_m = Chassis_GetDistance();
    s_state.last_wheel_delta_m = wheel_distance_m - s_last_wheel_distance_m;
    s_last_wheel_distance_m = wheel_distance_m;
    s_state.last_center_delta_m = Localization_CorrectWheelDelta(
        s_state.last_wheel_delta_m,
        omega_deg_s,
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
    /* 视觉帧路由: 起步对齐窗口内喂大量程 startup_bias (吸收静态总偏差),
     * 窗口外喂慢跟踪 vision_bias (只管运动中漂移)。 */
    if (s_state.vision_correction_enabled){
        if (s_state.startup_align){
            AimVisionBias_Update(&s_state.startup_bias,
                                 s_config.vision_yaw_sign * yaw_error_deg,
                                 s_config.vision_pitch_sign * pitch_error_deg,
                                 s_state.vision_valid);
        } else{
            AimVisionBias_Update(&s_state.vision_bias,
                                 s_config.vision_yaw_sign * yaw_error_deg,
                                 s_config.vision_pitch_sign * pitch_error_deg,
                                 s_state.vision_valid);
        }
    }

    /* 确定本周期实际瞄准的世界目标点 (靶心或圆点), 供静态角与速率前馈共用。 */
    AIM_SOLVER_CONFIG solver_cfg = AimSolver_GetConfig();
    AIM_POINT2F target_xy;
    float target_dz;
    if (s_state.mode == AUTO_AIM_MODE_CIRCLE){
        float phase_deg = AimSolver_ProgressToPhaseDeg(s_state.lap_progress,
                                                       s_config.circle_phase0_deg);
        AimSolver_CirclePoint(phase_deg, &target_xy, &target_dz);
    } else{
        target_xy = solver_cfg.target_center;
        target_dz = solver_cfg.height_diff_m;
    }
    s_state.feedforward = AimSolver_Solve(s_state.pose, target_xy, target_dz);

    /* 速率前馈: 车运动引起的云台角速度需求 (支点基准, 含 mount 甩臂项)。
     * yaw_rate 直接作为 velff 的解析速度前馈下发 (闭式运动学, 无差分噪声,
     * 免疫角点 snap 命令跳变); pitch 侧仍经 pitch lead 叠加 (默认 0)。 */
    s_state.gyro_z_deg_s = omega_deg_s;   /* 原始 ω: 遥测/上报与车心重构用 */
    s_state.v_center_mps = s_state.last_center_delta_m / dt_s;
    /* v 单独低通: 去掉里程更新慢于/差拍于循环导致的 v=Δs/dt 混叠尖峰(0↔~1m/s)。
     * ω 不低通: 低噪声真实自转, 速度内环需实时 1:1 反转补偿(含蛇形), 滤了引入相位滞后。 */
    float v_ratef = s_state.v_center_mps;
    if (s_config.rate_ff_v_tau_s > 0.0f){
        float alpha = dt_s / (dt_s + s_config.rate_ff_v_tau_s);
        s_v_ratef_lpf += alpha * (s_state.v_center_mps - s_v_ratef_lpf);
        v_ratef = s_v_ratef_lpf;
    } else{
        s_v_ratef_lpf = s_state.v_center_mps;
    }
    AIM_RATE_FF rate = AimFusion_MotionRate(s_state.pose, target_xy, target_dz,
                                            v_ratef, omega_deg_s,
                                            solver_cfg.mount_x_m,
                                            solver_cfg.mount_y_m);
    s_state.yaw_rate_ff_deg_s = rate.yaw_rate_deg_s;
    s_state.pitch_rate_ff_deg_s = rate.pitch_rate_deg_s;

    /* yaw 命令 = 静态几何角 + 起步对齐校正 + 视觉慢校正; yaw 执行延迟补偿由 gimbal
     * 层 velff 内建, 此处不叠加 yaw lead。pitch 保留 pitch lead (默认 0)。 */
    s_state.yaw_command_deg = s_state.feedforward.yaw_cmd_deg +
                              s_state.startup_bias.yaw_bias_deg +
                              s_state.vision_bias.yaw_bias_deg;
    s_state.pitch_command_deg = s_state.feedforward.pitch_cmd_deg +
                                s_state.startup_bias.pitch_bias_deg +
                                s_state.vision_bias.pitch_bias_deg +
                                rate.pitch_rate_deg_s * s_config.pitch_rate_lead_s;
    return Gimbal_SetAngle(s_state.yaw_command_deg, s_state.pitch_command_deg,
                           s_state.yaw_rate_ff_deg_s);
}

void AutoAim_SetStartupAlign(bool active){
    if (!s_state.active){
        return;
    }
    if (s_state.startup_align && !active &&
        (s_state.startup_bias.frame_count > 0U)){
        /* 对齐完成且吸收过帧: 慢 bias 跳过快收敛段, 直接稳态小增益跟漂移。 */
        AimVisionBias_SetSteadyGain(&s_state.vision_bias);
    }
    s_state.startup_align = active;
}

void AutoAim_SetStartupBias(float yaw_bias_deg, float pitch_bias_deg){
    if (!s_state.active){
        return;
    }

    /*
     * 这是纯视觉闭环已经物理对准后的控制器交接值，不是未经验证的单帧误差，
     * 因而不复用积分器的小量程限幅。frame_count 标记该基准有效，使关闭
     * startup_align 时 steady bias 直接使用稳态小增益，避免再次快速修正。
     */
    s_state.startup_bias.yaw_bias_deg = yaw_bias_deg;
    s_state.startup_bias.pitch_bias_deg = pitch_bias_deg;
    s_state.startup_bias.frame_count = 1U;
}

void AutoAim_SetVisionCorrectionEnabled(bool enabled){
    if (!s_state.active){
        return;
    }
    s_state.vision_correction_enabled = enabled;
}

void AutoAim_ResetVisionCorrection(void){
    if (!s_state.active){
        return;
    }
    AutoAim_ResetVisionBias();
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
