/**
 * @file  aim_fusion.h
 * @brief Core 层瞄准控制融合: 静态前馈角 + 运动速率前馈 + 视觉慢校正 bias。
 *
 * 职责划分:
 *   - aim_solver: 纯几何 (位姿+靶心 → 静态角);
 *   - aim_fusion: 控制融合 (静态角 + 车运动 + 视觉误差 → 最终指令 + 速率前馈)。
 *
 * 速率前馈 (车运动引起的云台角速度需求, 固定靶点):
 *   yaw_rate   = RAD2DEG( v·sin(Δβ)/Rh ) − ω     (平移掠过 − 车体自转)
 *   pitch_rate = RAD2DEG( Δz·v·cos(Δβ)/(Rh²+Δz²) )
 * 其中 Δβ = 世界方位 − 车航向, ω 取自陀螺 yaw-rate (逆时针为正), v 为车心前向速度。
 * 速率项忽略 mount 偏移 (二阶小量, 静态角已含之)。
 *
 * 本模块为纯计算, 不依赖任何硬件/中间层, 可单独单元测试。
 */
#ifndef AIM_FUSION_H
#define AIM_FUSION_H

#include "aim_solver/aim_solver.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 水平距离下限保护, 单位 m (防 Rh→0 除零)。 */
#define AIM_FUSION_RH_EPS 1.0e-4f
/** yaw 速率前馈钳位, 单位 deg/s (极近靶防护; 本题 Rh≥0.5m 通常触不到)。 */
#define AIM_FUSION_MAX_YAW_RATE_DEG_S 300.0f

/**
 * @brief 视觉低增益 bias 校正器 (自 aim_solver 迁入)。
 *
 * 把 K230 回传的"靶心相对图像中心的角度误差"积分成一个缓变 bias, 叠加到前馈
 * 角上, 消除里程漂移与机械标定误差。视觉无效时 bias 保持不动 (前馈照常)。
 */
typedef struct {
    /** yaw bias, 单位 deg。 */
    float yaw_bias_deg;
    /** pitch bias, 单位 deg。 */
    float pitch_bias_deg;
    /** 校正积分增益, 单位 (deg bias) / (deg error · s)。宜取小值 (慢校正)。 */
    float ki;
    /** bias 绝对值限幅, 单位 deg。 */
    float bias_limit_deg;
} AIM_VISION_BIAS;

/**
 * @brief 运动速率前馈 (车运动引起的云台角速度需求)。
 */
typedef struct {
    /** yaw 轴角速度需求, 单位 deg/s。 */
    float yaw_rate_deg_s;
    /** pitch 轴角速度需求, 单位 deg/s。 */
    float pitch_rate_deg_s;
} AIM_RATE_FF;

/**
 * @brief 融合输出: 位置指令 (静态前馈 + 视觉 bias) + 速率前馈。
 */
typedef struct {
    /** yaw 位置指令 (给 F32C 位置模式), 单位 deg。 */
    float yaw_cmd_deg;
    /** pitch 位置指令, 单位 deg。 */
    float pitch_cmd_deg;
    /** yaw 速率前馈 (S6 决定是否叠加为超前), 单位 deg/s。 */
    float yaw_rate_ff_deg_s;
    /** pitch 速率前馈, 单位 deg/s。 */
    float pitch_rate_ff_deg_s;
} AIM_FUSION_OUTPUT;

/**
 * @brief 初始化视觉 bias 校正器。
 * @param ki 校正积分增益 (宜取小值)。
 * @param limit bias 绝对值限幅, 单位 deg。
 */
void AimVisionBias_Init(AIM_VISION_BIAS *bias, float ki, float limit);

/**
 * @brief 更新视觉 bias。
 * @param bias 校正器。
 * @param yaw_err_deg 靶心相对图像中心的水平角误差, 单位 deg。
 * @param pitch_err_deg 靶心相对图像中心的竖直角误差, 单位 deg。
 * @param valid 本帧视觉是否有效; false 时 bias 保持不动。
 * @param dt_s 控制周期, 单位 s。
 */
void AimVisionBias_Update(AIM_VISION_BIAS *bias,
                          float yaw_err_deg,
                          float pitch_err_deg,
                          bool valid,
                          float dt_s);

/**
 * @brief 计算车运动引起的云台角速度需求 (速率前馈)。
 * @param pose 车在世界系的位姿。
 * @param target_xy 世界系目标水平点 (靶心或圆点)。
 * @param dz_m 目标相对云台出光点的竖直高差, 单位 m。
 * @param v_center_mps 车心前向速度, 单位 m/s (可由 last_center_delta/dt 求)。
 * @param omega_deg_s 车体绕 +Z 角速度 (陀螺 yaw-rate), 单位 deg/s, 逆时针为正。
 * @return 速率前馈; yaw 轴已按 AIM_FUSION_MAX_YAW_RATE_DEG_S 钳位。
 */
AIM_RATE_FF AimFusion_MotionRate(KINEMATICS_POSE pose,
                                 AIM_POINT2F target_xy,
                                 float dz_m,
                                 float v_center_mps,
                                 float omega_deg_s);

/**
 * @brief 组装最终指令 = 静态前馈角 + 视觉 bias, 并携带速率前馈。
 * @param feedforward aim_solver 的静态解算结果。
 * @param bias 视觉 bias; NULL 时按 0 处理。
 * @param rate 速率前馈。
 */
AIM_FUSION_OUTPUT AimFusion_Combine(AIM_SOLVER_RESULT feedforward,
                                    const AIM_VISION_BIAS *bias,
                                    AIM_RATE_FF rate);

#ifdef __cplusplus
}
#endif

#endif /* AIM_FUSION_H */
