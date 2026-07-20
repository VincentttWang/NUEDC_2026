/**
 * @file  aim_solver.h
 * @brief Core 层激光瞄准几何前馈解算器 (2D 世界 + 标量高差)。
 *
 * 设计目标: 把"车在哪里 + 靶心在哪里"这两个【已知量】直接解算成云台
 * yaw/pitch 绝对角, 作为前馈量下发给云台位置模式。视觉只作低增益 bias
 * 校正 (见 aim_fusion 的 AIM_VISION_BIAS), 因此:
 *   - 运动中激光始终指向 (前馈连续、零滞后), 不依赖同时识别激光光斑;
 *   - 视觉丢失时纯前馈继续, 激光不断 (满足发挥部分"连续发光"硬约束);
 *   - E3 画圆相位由里程直接参数化, 车 1 圈 = 画 1 圈, 天然同步。
 *
 * 坐标系 (世界系, 单位 m, 右手系):
 *   - 原点 O: AB 边中点 (贴近靶的那条边);
 *   - +X: 沿 AB, 由 A 指向 B;
 *   - +Y: 由 AB 指向靶面外侧 (靶在 +Y, 正方形赛道在 -Y)。
 * 车位姿用 KINEMATICS_POSE 表示 (x_m, y_m, heading_deg), heading 为车体 +X
 * (前向) 相对世界 +X 的夹角, 由 IMU 融合航向提供。
 *
 * 竖直方向 (2D 简化): 整车 pitch 摆幅小且慢, 无需把竖直坐标建成三维世界系。
 * 竖直依赖坍缩为单一标量 height_diff_m = (靶心离地高 - 云台出光点离地高)。
 * pitch 由"水平距离 + 该高差"直接给出: pitch = atan2(height_diff, horizontal)。
 * 位姿与靶心均为 2D 水平量。
 *
 * 本模块为纯计算, 不依赖任何硬件/中间层, 可单独单元测试。
 */
#ifndef AIM_SOLVER_H
#define AIM_SOLVER_H

#include "kinematics/kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 世界布局默认值 (依赖实机标定, 下面为按赛题名义尺寸给的起调值) ===== */
/** 正方形赛道边长, 单位 m (赛题 100cm)。 */
#define AIM_SOLVER_DEFAULT_TRACK_SIDE_M 1.00f
/** 靶面到 AB 边的外沿距离, 单位 m (赛题 50cm); 派生靶心 Y。 */
#define AIM_SOLVER_DEFAULT_TARGET_OFFSET_M 0.50f
/** 靶心沿 AB (世界 X) 的水平位置, 单位 m。默认对准 AB 中点 (0)。 */
#define AIM_SOLVER_DEFAULT_TARGET_X_M 0.00f
/** 竖直高差 = 靶心离地高 - 云台出光点离地高, 单位 m。机械上尽量趋 0。 */
#define AIM_SOLVER_DEFAULT_HEIGHT_DIFF_M 0.10f
/** 云台 yaw 轴支点相对车体原点的前向偏移 (车体 +X), 单位 m。 */
#define AIM_SOLVER_DEFAULT_MOUNT_X_M 0.00f
/** 云台 yaw 轴支点相对车体原点的左向偏移 (车体 +Y), 单位 m。 */
#define AIM_SOLVER_DEFAULT_MOUNT_Y_M 0.00f
/** yaw 机械零位补偿: yaw_cmd=0 时激光相对车体 +X 前向的夹角, 单位 deg。 */
#define AIM_SOLVER_DEFAULT_YAW_ZERO_OFFSET_DEG 0.0f
/** 激光束线相对 yaw 转轴的垂直横向偏移, 单位 m (沿出光方向看, 光束在转轴
 *  左侧为正)。光束不过转轴时命中条件是 yaw = bearing - asin(ℓ/r), 修正量
 *  ∝1/距离 (0.03m 在 0.5m 处 ≈3.4°, 在 1.8m 处 ≈1°), 机械尺量填入。 */
#define AIM_SOLVER_DEFAULT_LASER_LATERAL_OFFSET_M 0.0f
/** pitch 机械零位补偿: 加到解算 pitch 上, 单位 deg。 */
#define AIM_SOLVER_DEFAULT_PITCH_ZERO_OFFSET_DEG 0.0f
/** pitch 光束角/电机角传动比 k (光束每 1° 电机角变化多少度)。pitch 有减速时
 *  k>1: 几何仰角是【光束角】, 而指令/零位是【电机角】, 命令必须按 1/k 折算:
 *    pitch_cmd = pitch_zero + atan2(dz, r)/k
 *  k 填 1 时退化为旧"电机角=光束角"模型 (斜率错 k 倍 → 随距离漂移, 实测 ±3°)。
 *  标定: 两个距离(0.5m/1.6m)分别让激光竖直正中靶心, 记电机角 m1/m2,
 *  k = [atan2(dz,r1) − atan2(dz,r2)] / (m1 − m2)。 */
#define AIM_SOLVER_DEFAULT_PITCH_BEAM_PER_MOTOR 1.0f
/** 激光束线相对 pitch 转轴的竖直偏移, 单位 m (束线在转轴上方为正); 见结构体注释。 */
#define AIM_SOLVER_DEFAULT_LASER_VERTICAL_OFFSET_M 0.0f
/** E3 画圆半径, 单位 m (赛题 6cm 红色圆弧)。 */
#define AIM_SOLVER_DEFAULT_CIRCLE_RADIUS_M 0.06f

/** 水平距离退化保护下限, 单位 m (防 atan2/除零病态)。 */
#define AIM_SOLVER_HORIZONTAL_EPS 1.0e-4f

/**
 * @brief 世界系水平点, 单位 m (竖直方向由 height_diff 标量承接, 不在此结构中)。
 */
typedef struct {
    float x_m;
    float y_m;
} AIM_POINT2F;

/**
 * @brief 瞄准解算配置 (世界布局 + 云台机械安装参数)。
 */
typedef struct {
    /** 靶心水平位置 (世界系 x/y)。 */
    AIM_POINT2F target_center;
    /** 竖直高差 = 靶心离地高 - 云台出光点离地高, 单位 m。 */
    float height_diff_m;
    /** 云台 yaw 轴支点相对车体原点的前向偏移 (车体 +X), 单位 m。 */
    float mount_x_m;
    /** 云台 yaw 轴支点相对车体原点的左向偏移 (车体 +Y), 单位 m。 */
    float mount_y_m;
    /** yaw 机械零位补偿, 单位 deg。 */
    float yaw_zero_offset_deg;
    /** 激光束线相对 yaw 转轴的垂直横向偏移, 单位 m (左正; 见宏注释)。 */
    float laser_lateral_offset_m;
    /** pitch 机械零位补偿, 单位 deg (电机角)。 */
    float pitch_zero_offset_deg;
    /** pitch 光束角/电机角传动比 k (>0; 1=直驱)。见宏注释。 */
    float pitch_beam_per_motor;
    /** 激光束线相对 pitch 转轴的竖直偏移, 单位 m (束线在转轴上方为正)。
     *  yaw 偏轴的竖直版: 束线不过 pitch 轴时命中仰角 = atan2(dz,r) − asin(ℓp/r),
     *  误差 ∝1/r (实测 pitch 漂移 ~-1.9°·m/r 即 ~3cm 级)。 */
    float laser_vertical_offset_m;
    /** E3 画圆半径, 单位 m。 */
    float circle_radius_m;
} AIM_SOLVER_CONFIG;

/**
 * @brief 瞄准解算结果。
 */
typedef struct {
    /** 云台 yaw 目标角 (车体系, 供位置模式), 归一化到 [-180,180), 单位 deg。 */
    float yaw_cmd_deg;
    /** 云台 pitch 目标角 (供位置模式), 单位 deg。 */
    float pitch_cmd_deg;
    /** 云台支点到目标点的直线距离 (含高差), 单位 m。 */
    float range_m;
    /** 云台支点到目标点的水平投影距离, 单位 m。 */
    float horizontal_m;
} AIM_SOLVER_RESULT;

/**
 * @brief 获取默认解算配置 (按赛题名义尺寸)。
 */
AIM_SOLVER_CONFIG AimSolver_DefaultConfig(void);

/**
 * @brief 初始化解算器。
 * @param config 配置指针; NULL 时用默认配置。
 */
void AimSolver_Init(const AIM_SOLVER_CONFIG *config);

/**
 * @brief 获取当前解算配置。
 */
AIM_SOLVER_CONFIG AimSolver_GetConfig(void);

/**
 * @brief 运行时设置靶心水平坐标 (供上位机/标定流程写入)。
 */
void AimSolver_SetTargetCenter(AIM_POINT2F target_center);

/**
 * @brief 由车位姿 + 世界目标水平点 + 竖直高差解算云台角 (前馈核心)。
 * @param pose 车在世界系的位姿。
 * @param target_xy 世界系目标水平点。
 * @param dz_m 目标相对云台出光点的竖直高差, 单位 m。
 * @return 解算结果。
 */
AIM_SOLVER_RESULT AimSolver_Solve(KINEMATICS_POSE pose,
                                  AIM_POINT2F target_xy,
                                  float dz_m);

/**
 * @brief 直接瞄准靶心 (基本要求 (2)(3) 与发挥 (1)(2))。
 */
AIM_SOLVER_RESULT AimSolver_SolveCenter(KINEMATICS_POSE pose);

/**
 * @brief 计算靶面 (Y = 靶心 Y 的竖直平面) 上, 相位 phase_deg 处的圆点。
 * @param phase_deg 圆周相位, 0° 取靶心正右方 (世界 +X), 90° 取正上方, 逆时针为正。
 * @param out_xy    输出圆点水平坐标 (世界系)。
 * @param out_dz    输出圆点相对云台出光点的竖直高差, 单位 m。
 */
void AimSolver_CirclePoint(float phase_deg, AIM_POINT2F *out_xy, float *out_dz);

/**
 * @brief 瞄准靶面 6cm 圆上相位 phase_deg 的点 (发挥 (3) 画圆)。
 */
AIM_SOLVER_RESULT AimSolver_SolveCircle(KINEMATICS_POSE pose, float phase_deg);

/**
 * @brief 里程圈进度 → 圆周相位。
 * @param lap_progress 当前圈内进度, [0,1)。
 * @param phase0_deg 相位起点偏移, 单位 deg。
 * @return 圆周相位, 单位 deg。
 */
float AimSolver_ProgressToPhaseDeg(float lap_progress, float phase0_deg);

#ifdef __cplusplus
}
#endif

#endif /* AIM_SOLVER_H */
