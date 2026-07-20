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
/* 视觉 bias 衰减增益 (逐轴独立): 第 n 个有效帧 g(n)=max(gain_min, gain0/(1+n))。
 * gain0 高 -> 首帧快收敛(4s 限时任务); gain_min 低 -> 稳态慢跟踪不引入抖动。
 * 测量质量控制(平滑/野值/丢帧预测)在 K230 侧 Kalman 完成, MCU 侧不再门控。
 * pitch 独立取值且更保守: 视觉误差是"光束角"而校正加在"电机角"上, pitch 减速比
 * 把等效环增益放大数倍(电机 1°≈光束数度), 增益需按 1/减速比缩小防过冲;
 * pitch 电机行程仅 home±15°, 限幅也相应收紧。 */
#define AUTO_AIM_DEFAULT_VISION_YAW_GAIN0 0.5f
#define AUTO_AIM_DEFAULT_VISION_YAW_GAIN_MIN 0.02f
#define AUTO_AIM_DEFAULT_VISION_YAW_LIMIT_DEG 3.0f
#define AUTO_AIM_DEFAULT_VISION_PITCH_GAIN0 0.25f
#define AUTO_AIM_DEFAULT_VISION_PITCH_GAIN_MIN 0.01f
#define AUTO_AIM_DEFAULT_VISION_PITCH_LIMIT_DEG 2.0f
/* 起步静态对齐校正器限幅 (逐轴): 倒计时静止窗口内一次性吸收"静态总偏差"
 * (零位/摆车/安装, 光轴统一假设下含相机-激光偏差), 量程独立于慢跟踪 bias。
 * 增益复用上面各轴的 gain0/gain_min。 */
#define AUTO_AIM_DEFAULT_VISION_YAW_STARTUP_LIMIT_DEG 10.0f
#define AUTO_AIM_DEFAULT_VISION_PITCH_STARTUP_LIMIT_DEG 5.0f
/* 弯中毒帧 ω 门控: |陀螺 yaw-rate| 超此阈值时视觉帧只观测不吸收 (慢 bias 冻结)。
 * 弯中帧含跟踪瞬态+测量延迟, 高稳态增益(0.1)下 8 帧即可把 bias 打到限幅, 再被
 * 近靶 1/r 放大成 ~10° 尖峰 (实测教训)。转弯 ω≈80-120, 直线蛇形 <30, 40 分界清晰。
 * 0 = 不门控。出弯后再滞留 HOLD 帧, 挡掉延迟(~60-100ms)拖尾的弯中测量。
 * HOLD 由 5→2: 遥测实测出弯尖峰(visErrY 峰值 ~12°)落在 ω 落回后的前几帧, HOLD=5 会把
 * 整段尖峰冻住、到第 6 帧才恢复视觉(此时已衰减), 尖峰全程得不到修正; HOLD=2 让视觉在
 * 峰值帧即接管。远边出弯 1/r 放大小、增益已衰减, 提前接管安全 (弯中本身仍由阈值挡住)。 */
#define AUTO_AIM_DEFAULT_VISION_FREEZE_OMEGA_DEG_S 40.0f
#define AUTO_AIM_VISION_FREEZE_HOLD_FRAMES 2U
/* 画圆相位限速 (deg/s): 角点 snap 会让 lap_progress 跳变 → 圆相位突跳 → 靶纸
 * 留径向划痕。限速把跳变摊平成短暂追赶 (名义相位速率 ~360°/17s ≈ 21°/s,
 * 限 90°/s 时 30° 的 snap 跳变 ~0.4s 内追平, 划痕变成弧内小弯)。 */
#define AUTO_AIM_CIRCLE_PHASE_SLEW_DEG_S 90.0f
#define AUTO_AIM_DEFAULT_VISION_YAW_SIGN -1.0f
#define AUTO_AIM_DEFAULT_VISION_PITCH_SIGN 1.0f
#define AUTO_AIM_DEFAULT_CIRCLE_PHASE0_DEG 0.0f
/* 陀螺 yaw-rate 符号: 使 ω 与航向变化率同号 (左转 yaw 增大时 ω>0)。
 * 实测本机 gz 与航向反号, 故默认 -1; 用于速率前馈与车心速度重构。 */
#define AUTO_AIM_DEFAULT_GYRO_Z_SIGN -1.0f
/* IMU 融合航向的陀螺前推时间, 单位 s: heading_used = 融合航向 + ω·lead。
 * JY61P 融合航向在快速旋转时滞后 (弯中 ω≈100°/s 时每 10ms 滞后 = 1° 命令误差,
 * 实测弯中 visErr 均值反算 τ≈25ms)。前推同时改善弯中定位积分。0 = 不前推。
 * 整定: 看弯中(st=2) visErr 均值 -> 0; 过大表现为弯中反向偏。 */
#define AUTO_AIM_DEFAULT_HEADING_GYRO_LEAD_S 0.0f
/* pitch 速率前馈 lead 时间, 单位 s: pitch 位置指令叠加 rate·lead 补偿位置模式执行延迟。
 * 0 = 纯静态前馈。(yaw 的执行延迟已由 gimbal 层 velff 内建, 故 yaw 无此 lead。) */
#define AUTO_AIM_DEFAULT_PITCH_RATE_LEAD_S 0.0f
/* 速率前馈 v 输入低通时间常数, 单位 s: 仅平滑"速率前馈用的 v"。
 * 单轮里程更新(10ms)慢于/差拍于控制环时 v=Δs/dt 会混叠尖峰(0↔~1m/s), 经视线平移项灌成
 * 速度指令每拍大抖; 低通恢复平滑真实车速。ω 不低通: 它是低噪声的真实自转测量, velff
 * 速度内环需要实时 1:1 反转补偿(含蛇形), 滤了反而引入转弯前馈相位滞后。
 * 车心里程重构(积分)与遥测仍用原始 Δs, 不受影响。0 = 不低通。fc≈1/(2π·τ); τ=0.133 -> ~1.2Hz。 */
#define AUTO_AIM_DEFAULT_RATE_FF_V_TAU_S 0.0f
/* pitch 纯视觉伺服 (仅 CENTER 模式): 弃用几何 pitch 前馈, 改由视觉 pitch 误差经 PID
 * 直接伺服 pitch 位置指令。理由: yaw 随车运动大幅变化, 必须几何前馈(连续零滞后);
 * pitch 几乎不动(靶高近固定, 只随距离变 ~1-3°), 几何前馈收益小却引入光束模型系统误差
 * (实测 visErrP 常负漂)。纯视觉去掉该模型误差, 让 pitch 直接收敛到靶心。
 * 复用 aim_track 角度环 pitch PID (KP=2/KI=0.8/KD=0), 按需把 P 适当调大加快收敛。
 * PID 输出为角速度(deg/s), 积分进 pitch 位置指令(等效 aim_track 的"速度积分成位置")。 */
#define AUTO_AIM_DEFAULT_PITCH_VISION_ONLY   true
#define AUTO_AIM_DEFAULT_PITCH_VISION_KP     3.2f   /* aim_track 2.0 调大 (~1.6×) 加快收敛 */
#define AUTO_AIM_DEFAULT_PITCH_VISION_KI     0.8f
#define AUTO_AIM_DEFAULT_PITCH_VISION_KD     0.0f
#define AUTO_AIM_DEFAULT_PITCH_VISION_I_LIMIT 25.0f

typedef enum {
    AUTO_AIM_MODE_CENTER = 0,
    AUTO_AIM_MODE_CIRCLE
} AUTO_AIM_MODE;

typedef struct {
    KINEMATICS_POSE start_pose;
    float track_side_m;
    /** 编码轮位于车体左侧为正、右侧为负。 */
    float encoder_lateral_offset_m;
    /** 视觉 bias yaw 轴增益/限幅 (慢跟踪)。 */
    AIM_VISION_GAIN vision_yaw;
    /** 视觉 bias pitch 轴增益/限幅 (慢跟踪; 因减速比与行程, 独立且更保守)。 */
    AIM_VISION_GAIN vision_pitch;
    /** 起步静态对齐限幅, 逐轴, 单位 deg (增益复用各轴 vision_* 的 gain0/gain_min)。 */
    float vision_yaw_startup_limit_deg;
    float vision_pitch_startup_limit_deg;
    /** 弯中毒帧 ω 门控阈值, 单位 deg/s; 0=不门控 (见宏注释)。 */
    float vision_freeze_omega_deg_s;
    float vision_yaw_sign;
    float vision_pitch_sign;
    float circle_phase0_deg;
    /** pitch 速率前馈 lead 时间, 单位 s。(yaw 执行延迟由 gimbal velff 内建, 无 yaw lead。) */
    float pitch_rate_lead_s;
    /** 速率前馈 v 输入低通时间常数 (s); 0=不低通。抑制单轮里程混叠尖峰灌进速度前馈。 */
    float rate_ff_v_tau_s;
    /** 陀螺 yaw-rate 符号 (使 ω 与航向变化率同号); 实测本机反号取 -1。 */
    float gyro_z_sign;
    /** IMU 融合航向陀螺前推时间 (s), 补快速旋转时的融合滞后; 0=不前推。 */
    float heading_gyro_lead_s;
    /** CENTER 模式 pitch 是否纯视觉伺服 (弃用几何 pitch 前馈, 消除光束模型系统误差)。 */
    bool pitch_vision_only;
    /** 纯视觉 pitch 伺服 PID: 视觉 pitch 误差(deg)->pitch 角速度(deg/s); 复用 aim_track 角度环。 */
    float pitch_vision_kp;
    float pitch_vision_ki;
    float pitch_vision_kd;
    /** I 项积分限幅 (deg·s)。 */
    float pitch_vision_i_limit;
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
    /** 起步静态对齐校正器 (倒计时窗口吸收静态总偏差, 任务开跑后冻结)。 */
    AIM_VISION_BIAS startup_bias;
    /** 起步对齐窗口是否激活 (激活期间视觉帧喂 startup_bias 而非 vision_bias)。 */
    bool startup_align;
    /** 是否允许视觉误差写入校正器；关闭时仍接收/上报视觉帧。 */
    bool vision_correction_enabled;
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
    /** 车体俯仰/侧倾姿态 (IMU 融合), 单位 deg; 供遥测诊断 pitch 漂移与姿态耦合。 */
    float body_pitch_deg;
    float body_roll_deg;
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

/**
 * @brief 开/关起步静态对齐窗口 (车静止的倒计时阶段开, 起步前关)。
 *        窗口内视觉帧以衰减增益喂 startup_bias (量程 vision_*_startup_limit_deg),
 *        一次性吸收静态总偏差; 关闭时若已吸收过帧, 慢跟踪 bias 直接进入稳态
 *        小增益 (快收敛已由对齐完成)。窗口内视觉未锁定则无副作用 (纯前馈起跑)。
 */
void AutoAim_SetStartupAlign(bool active);
/**
 * @brief 写入由静态纯视觉对准测得的前馈基准偏置。
 * @param yaw_bias_deg 已对准 yaw 角减去起点几何前馈 yaw 角，单位 deg。
 * @param pitch_bias_deg 已对准 pitch 角减去起点几何前馈 pitch 角，单位 deg。
 * @note 该接口用于控制器交接，不经过视觉积分器限幅；调用方必须先完成可靠锁定。
 */
void AutoAim_SetStartupBias(float yaw_bias_deg, float pitch_bias_deg);
/**
 * @brief 允许/禁止视觉误差写入 bias；禁用不影响视觉接收和几何前馈。
 * @note 用于启动几何预定位阶段，避免云台大角度过渡的瞬态误差污染静态校正。
 */
void AutoAim_SetVisionCorrectionEnabled(bool enabled);
/** @brief 清零 startup/steady 两套视觉校正器，保留当前任务和定位状态。 */
void AutoAim_ResetVisionCorrection(void);
void AutoAim_AnchorCorner(LOCALIZATION_CORNER corner, float travelled_m);
BSP_STATUS AutoAim_Stop(void);
AUTO_AIM_STATE AutoAim_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* AUTO_AIM_H */
