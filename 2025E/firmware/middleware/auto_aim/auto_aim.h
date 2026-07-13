/**
 * @file  auto_aim.h
 * @brief Middleware 层几何前馈自动瞄准协调器。
 */
#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include "aim_fusion/aim_fusion.h"
#include "aim_solver/aim_solver.h"
#include "bsp_common.h"
#include "localization/localization.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AUTO_AIM_DEFAULT_TRACK_SIDE_M 1.0f
#define AUTO_AIM_DEFAULT_ENCODER_LATERAL_OFFSET_M 0.0f
#define AUTO_AIM_DEFAULT_VISION_BIAS_KI 0.35f
#define AUTO_AIM_DEFAULT_VISION_BIAS_LIMIT_DEG 3.0f
#define AUTO_AIM_DEFAULT_VISION_YAW_SIGN -1.0f
#define AUTO_AIM_DEFAULT_VISION_PITCH_SIGN 1.0f
#define AUTO_AIM_DEFAULT_CIRCLE_PHASE0_DEG 0.0f

typedef enum {
    AUTO_AIM_MODE_CENTER = 0,
    AUTO_AIM_MODE_CIRCLE
} AUTO_AIM_MODE;

typedef struct {
    KINEMATICS_POSE start_pose;
    float track_side_m;
    /** 编码轮位于车体左侧为正、右侧为负。 */
    float encoder_lateral_offset_m;
    float vision_bias_ki;
    float vision_bias_limit_deg;
    float vision_yaw_sign;
    float vision_pitch_sign;
    float circle_phase0_deg;
    AIM_SOLVER_CONFIG solver;
} AUTO_AIM_CONFIG;

typedef struct {
    bool active;
    bool imu_valid;
    bool vision_valid;
    AUTO_AIM_MODE mode;
    KINEMATICS_POSE pose;
    AIM_SOLVER_RESULT feedforward;
    AIM_VISION_BIAS vision_bias;
    float yaw_command_deg;
    float pitch_command_deg;
    float vision_yaw_error_deg;
    float vision_pitch_error_deg;
    float last_wheel_delta_m;
    float last_center_delta_m;
    float lap_progress;
    uint32_t vision_frame_count;
} AUTO_AIM_STATE;

AUTO_AIM_CONFIG AutoAim_DefaultConfig(void);
void AutoAim_Init(const AUTO_AIM_CONFIG *config);
BSP_STATUS AutoAim_Start(AUTO_AIM_MODE mode);
BSP_STATUS AutoAim_Update(float dt_s);
void AutoAim_AnchorCorner(LOCALIZATION_CORNER corner, float travelled_m);
BSP_STATUS AutoAim_Stop(void);
AUTO_AIM_STATE AutoAim_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* AUTO_AIM_H */
