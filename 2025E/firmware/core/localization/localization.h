/**
 * @file  localization.h
 * @brief Core 层车体位姿估计 (航位推算 + 路标重置)。
 *
 * 融合策略:
 *   - 位置: 编码器累计里程增量沿 IMU 航向积分 (航位推算 dead reckoning);
 *   - 航向: 直接取 IMU 融合航向 (比双轮里程差分航向漂移小得多);
 *   - 校正: 巡线状态机每确认一个直角 (已知世界坐标), 把估计位置 snap 到该
 *           角点, 清除单段里程累积误差 —— 四个角 A/B/C/D 是绝对位置基准。
 *
 * 世界系与 aim_solver 一致 (原点 = AB 中点, +X 沿 A→B, +Y 指向靶外侧, +Z 向上)。
 * 本模块为纯计算, 不直接读硬件: 里程与航向由调用方每周期传入。
 */
#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "common/core_types.h"
#include "kinematics/kinematics.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 正方形赛道边长默认值, 单位 m。 */
#define LOCALIZATION_DEFAULT_TRACK_SIDE_M 1.00f

/*
 * 渐变锚定系数: 角点校正不再一帧 snap 位姿, 而是把"目标角点 − 当前估计"存为待修正量,
 * 由每次 UpdateDelta 施加该比例 (指数收敛)。1.0 = 立即(旧突变行为); 越小越平滑但收敛越慢。
 * 目的: 低占空比轮打滑使里程累积 5~9cm 误差, anchor 一帧 snap 会让几何 yaw 指令阶跃、
 * 云台被猛甩过冲; 摊平成 ~5~8 拍(~100~200ms)的平滑追赶即可消除该阶跃。
 */
#ifndef LOCALIZATION_ANCHOR_BLEND_ALPHA
#define LOCALIZATION_ANCHOR_BLEND_ALPHA 0.35f
#endif

/**
 * @brief 赛道角点标识 (世界坐标由边长推出)。
 *
 * 布局 (世界系, 原点 AB 中点):
 *   A = (-side/2, 0)      B = (+side/2, 0)      靠靶的 AB 边
 *   C = (-side/2, -side)  D = (+side/2, -side)  远离靶的 CD 边
 */
typedef enum {
    LOCALIZATION_CORNER_A = 0,
    LOCALIZATION_CORNER_B,
    LOCALIZATION_CORNER_C,
    LOCALIZATION_CORNER_D,
    LOCALIZATION_CORNER_COUNT
} LOCALIZATION_CORNER;

/**
 * @brief 位姿估计器状态。
 */
typedef struct {
    /** 当前估计位姿。 */
    KINEMATICS_POSE pose;
    /** 赛道边长, 单位 m。 */
    float track_side_m;
    /** 单圈周长 (= 4·边长), 单位 m。 */
    float perimeter_m;
    /** 上次传入的累计里程, 单位 m (用于求增量)。 */
    float last_distance_m;
    /** 自 Init 起累计行进里程, 单位 m (用于圈进度)。 */
    float travelled_m;
    /** 最近一次车心位移增量, 单位 m (供 S3 求 v_center=Δ/dt 与遥测)。 */
    float last_center_delta_m;
    /** 渐变锚定待施加的位置修正 (x/y, m): anchor 存, UpdateDelta 逐帧摊平, 防位姿突变。 */
    float pending_corr_x_m;
    float pending_corr_y_m;
    /** 是否已初始化。 */
    bool initialized;
} LOCALIZATION_STATE;

/**
 * @brief 初始化位姿估计器。
 * @param start_pose 已知起点位姿 (发挥部分起点由赛题规定, 位姿已知)。
 * @param track_side_m 赛道边长, 单位 m; <=0 时用默认值。
 * @param initial_distance_m 当前编码器累计里程读数 (作为增量基准)。
 */
void Localization_Init(KINEMATICS_POSE start_pose,
                       float track_side_m,
                       float initial_distance_m);

/**
 * @brief 用最新里程与航向推进一步。
 * @param distance_m 编码器累计里程当前读数, 单位 m。
 * @param heading_deg IMU 融合航向, 单位 deg (世界系)。
 */
void Localization_Update(float distance_m, float heading_deg);

/**
 * @brief 用已经换算到车体中心的位移增量推进一步。
 * @param center_delta_m 本周期车体中心前向位移，单位 m，可正可负。
 * @param heading_deg IMU 融合航向，单位 deg（世界系）。
 *
 * @note 发挥任务优先使用本接口。调用方可先用
 *       Localization_CorrectWheelDelta() 修正单轮编码器在转弯时的弧长差。
 */
void Localization_UpdateDelta(float center_delta_m, float heading_deg);

/**
 * @brief 把单个编码轮的位移换算为车体中心位移。
 * @param wheel_delta_m 编码轮本周期位移，单位 m。
 * @param gyro_z_deg_s 车体绕 +Z 的角速度，单位 deg/s，逆时针为正。
 * @param encoder_lateral_offset_m 编码轮相对车体中心的横向偏移，单位 m；
 *        位于车体左侧为正，右侧为负。
 * @param dt_s 本周期时长，单位 s。
 * @return 估算的车体中心位移，单位 m。
 */
float Localization_CorrectWheelDelta(float wheel_delta_m,
                                     float gyro_z_deg_s,
                                     float encoder_lateral_offset_m,
                                     float dt_s);

/**
 * @brief 单轮里程一步推进: 先车心修正, 再航位推算 (组合入口)。
 * @param wheel_delta_m 编码轮本周期位移, 单位 m。
 * @param gyro_z_deg_s 车体绕 +Z 的角速度, 单位 deg/s, 逆时针为正。
 * @param encoder_lateral_offset_m 编码轮相对车体中心的横向偏移, 单位 m;
 *        位于车体左侧为正, 右侧为负。
 * @param heading_deg IMU 融合航向, 单位 deg (世界系)。
 * @param dt_s 本周期时长, 单位 s。
 * @note = Localization_CorrectWheelDelta() + Localization_UpdateDelta(),
 *       S5 传感器接线单次调用即可, 避免两步误用。
 */
void Localization_UpdateFromWheel(float wheel_delta_m,
                                  float gyro_z_deg_s,
                                  float encoder_lateral_offset_m,
                                  float heading_deg,
                                  float dt_s);

/**
 * @brief 路标校正: 已到达某已知角点, 把估计位置 snap 到该角世界坐标。
 * @param corner 角点标识。
 * @note 只重置 x/y, 不动航向 (航向由 IMU 提供, 无需重置)。
 */
void Localization_ResetToCorner(LOCALIZATION_CORNER corner);

/**
 * @brief 路标校正并同步重锚累计路程。
 * @param corner 已确认到达的角点。
 * @param travelled_m 从任务起点到该角点的理论累计路程，单位 m。
 */
void Localization_ResetToCornerAtTravel(LOCALIZATION_CORNER corner,
                                        float travelled_m);

/**
 * @brief 获取角点世界坐标。
 */
CORE_POINT2F Localization_CornerPoint(LOCALIZATION_CORNER corner);

/**
 * @brief 获取当前估计位姿。
 */
KINEMATICS_POSE Localization_GetPose(void);

/**
 * @brief 获取自 Init 起累计行进里程, 单位 m。
 */
float Localization_GetTravelled(void);

/**
 * @brief 获取当前圈内进度, 范围 [0,1)。
 * @note = fmod(travelled, perimeter) / perimeter, 供 E3 画圆相位使用。
 */
float Localization_GetLapProgress(void);

/**
 * @brief 获取已完成圈数 (向下取整)。
 */
uint32_t Localization_GetLapCount(void);

/**
 * @brief 获取估计器完整状态快照。
 */
LOCALIZATION_STATE Localization_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* LOCALIZATION_H */
