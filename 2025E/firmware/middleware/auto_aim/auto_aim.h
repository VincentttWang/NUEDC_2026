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
/* 陀螺 yaw-rate 符号: 使 ω 与航向变化率同号 (左转 yaw 增大时 ω>0)。
 * 实测本机 gz 与航向反号, 故默认 -1; 用于速率前馈与车心速度重构。 */
#define AUTO_AIM_DEFAULT_GYRO_Z_SIGN -1.0f
/* pitch 速率前馈 lead 时间, 单位 s: pitch 位置指令叠加 rate·lead 补偿位置模式执行延迟。
 * 0 = 纯静态前馈。(yaw 的执行延迟已由 gimbal 层 velff 内建, 故 yaw 无此 lead。) */
#define AUTO_AIM_DEFAULT_PITCH_RATE_LEAD_S 0.0f
/* 速率前馈运动输入(ω 与 v)低通时间常数, 单位 s: 仅平滑"速率前馈用的 ω 和 v"。
 *  - ω: 直线蛇形是绕 0 的振荡, 低通后被平均掉, 不灌进命令放大成 yaw 抖; 转弯(持续 ω)保留。
 *  - v: 单轮里程更新慢于循环时 v=Δs/dt 会混叠尖峰(0↔~1m/s), 经平移项 v·sinΔβ/Rh 灌成命令
 *       每拍大抖; 低通恢复平滑真实车速。
 * 车心里程重构(积分)与遥测仍用原始 ω/Δs, 不受影响。0 = 不低通。fc≈1/(2π·τ); τ=0.133 -> ~1.2Hz。 */
#define AUTO_AIM_DEFAULT_RATE_FF_MOTION_TAU_S 0.0f

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
    /** pitch 速率前馈 lead 时间, 单位 s。(yaw 执行延迟由 gimbal velff 内建, 无 yaw lead。) */
    float pitch_rate_lead_s;
    /** 速率前馈运动输入(ω 与 v)低通时间常数 (s); 0=不低通。抑制蛇形/里程混叠被前馈放大成 yaw 抖。 */
    float rate_ff_motion_tau_s;
    /** 陀螺 yaw-rate 符号 (使 ω 与航向变化率同号); 实测本机反号取 -1。 */
    float gyro_z_sign;
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
    /** 车心前向速度 (= last_center_delta/dt), 单位 m/s。 */
    float v_center_mps;
    /** 车体 yaw 角速度 (陀螺), 单位 deg/s。 */
    float gyro_z_deg_s;
    /** yaw 速率前馈量, 单位 deg/s。 */
    float yaw_rate_ff_deg_s;
    /** pitch 速率前馈量, 单位 deg/s。 */
    float pitch_rate_ff_deg_s;
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
