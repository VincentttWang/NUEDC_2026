/**
 * @file  aim_fusion.h
 * @brief Core 层瞄准控制融合: 静态前馈角 + 运动速率前馈 + 视觉慢校正 bias。
 *
 * 职责划分:
 *   - aim_solver: 纯几何 (位姿+靶心 → 静态角);
 *   - aim_fusion: 控制融合 (静态角 + 车运动 + 视觉误差 → 最终指令 + 速率前馈)。
 *
 * 速率前馈 (车运动引起的云台角速度需求, 固定靶点), 以【云台支点】为基准:
 *   支点世界速度 v_g = v·x̂(航向) + ω × r_mount   (车身自转把支点甩出的横向速度)
 *   yaw_rate   = RAD2DEG( 视线垂向分量(v_g)/Rh ) − ω   (视线转速 − 车体自转)
 *   pitch_rate = RAD2DEG( Δz·视线径向分量(v_g)/(Rh²+Δz²) )
 * 其中 ω 取自陀螺 yaw-rate (逆时针为正), v 为车心前向速度, Rh 从支点起算。
 * mount 不可忽略: 支点在轮轴前 12cm 时, 转弯 ω×r ≈ 0.27m/s 与车速同量级,
 * 忽略会在近靶转弯段引入 ~15°/s 的速率误差。mount=0 退化为原车心公式。
 *
 * 本模块为纯计算, 不依赖任何硬件/中间层, 可单独单元测试。
 */
#ifndef AIM_FUSION_H
#define AIM_FUSION_H

#include "aim_solver/aim_solver.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 水平距离下限保护, 单位 m (防 Rh→0 除零)。 */
#define AIM_FUSION_RH_EPS 1.0e-4f
/** yaw 速率前馈钳位, 单位 deg/s (极近靶防护; 本题 Rh≥0.5m 通常触不到)。 */
#define AIM_FUSION_MAX_YAW_RATE_DEG_S 300.0f

/**
 * @brief 视觉 bias 校正器: 按帧衰减增益 (KF 式增益调度的退化形态)。
 *
 * 把 K230 回传的"靶心相对光轴的角度误差"吸收成缓变 bias, 叠加到前馈角上,
 * 消除里程漂移与机械标定误差。测量质量控制(平滑/野值/丢帧预测)在 K230 侧
 * Kalman 完成, 本层只决定"信任程度":
 *   第 n 个有效帧增益 g(n) = max(gain_min, gain0/(1+n)), n 从 0 起;
 *   首帧吃掉 gain0 比例的残差(快收敛, 供 4s 限时任务), 稳态小增益跟踪慢漂移。
 * 视觉无效时 bias 冻结 (前馈照常, 帧计数不清零); 任务启动时清零重新快收敛。
 */
/**
 * @brief 单轴衰减增益参数。yaw/pitch 分开配置:
 *        pitch 视觉误差是"光束角"而校正加在"电机角"上, 减速比放大等效环增益,
 *        且 pitch 电机行程 (±15°) 远小于 yaw, 增益与限幅都应独立取值。
 */
typedef struct {
    /** 首帧增益 (0~1]。 */
    float gain0;
    /** 稳态最小增益。 */
    float gain_min;
    /** bias 绝对值限幅, 单位 deg。 */
    float limit_deg;
} AIM_VISION_GAIN;

typedef struct {
    /** yaw bias, 单位 deg。 */
    float yaw_bias_deg;
    /** pitch bias, 单位 deg。 */
    float pitch_bias_deg;
    /** yaw 轴增益/限幅。 */
    AIM_VISION_GAIN yaw;
    /** pitch 轴增益/限幅。 */
    AIM_VISION_GAIN pitch;
    /** 已吸收的有效帧计数 (增益调度用; 一帧含两轴测量, 计数共用)。 */
    uint32_t frame_count;
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
 * @brief 初始化视觉 bias 校正器 (bias 与帧计数清零)。
 * @param yaw yaw 轴增益/限幅。
 * @param pitch pitch 轴增益/限幅。
 */
void AimVisionBias_Init(AIM_VISION_BIAS *bias, AIM_VISION_GAIN yaw,
                        AIM_VISION_GAIN pitch);

/**
 * @brief 按帧更新视觉 bias: bias += g(n)·err (仅在新有效帧时调用生效)。
 * @param bias 校正器。
 * @param yaw_err_deg 靶心相对光轴的水平角误差, 单位 deg。
 * @param pitch_err_deg 靶心相对光轴的竖直角误差, 单位 deg。
 * @param valid 本帧视觉是否有效; false 时 bias 冻结 (帧计数不清零)。
 */
void AimVisionBias_Update(AIM_VISION_BIAS *bias,
                          float yaw_err_deg,
                          float pitch_err_deg,
                          bool valid);

/**
 * @brief 直接进入稳态增益 (跳过快收敛段): 把帧计数推到 g(n)<=gain_min 处。
 *        用于起步对齐完成后, 慢跟踪 bias 不再对首帧做大步吸收。
 */
void AimVisionBias_SetSteadyGain(AIM_VISION_BIAS *bias);

/**
 * @brief 计算车运动引起的云台角速度需求 (速率前馈, 以云台支点为基准)。
 * @param pose 车在世界系的位姿 (定位原点 = 轮轴中点)。
 * @param target_xy 世界系目标水平点 (靶心或圆点)。
 * @param dz_m 目标相对云台出光点的竖直高差, 单位 m。
 * @param v_center_mps 车心前向速度, 单位 m/s (可由 last_center_delta/dt 求)。
 * @param omega_deg_s 车体绕 +Z 角速度 (陀螺 yaw-rate), 单位 deg/s, 逆时针为正。
 * @param mount_x_m 云台 yaw 支点相对定位原点的车体前向偏移, 单位 m (前为正)。
 * @param mount_y_m 云台 yaw 支点相对定位原点的车体左向偏移, 单位 m (左为正)。
 * @return 速率前馈; yaw 轴已按 AIM_FUSION_MAX_YAW_RATE_DEG_S 钳位。
 */
AIM_RATE_FF AimFusion_MotionRate(KINEMATICS_POSE pose,
                                 AIM_POINT2F target_xy,
                                 float dz_m,
                                 float v_center_mps,
                                 float omega_deg_s,
                                 float mount_x_m,
                                 float mount_y_m);

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
