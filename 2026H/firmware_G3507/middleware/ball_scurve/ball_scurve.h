/**
 * @file  ball_scurve.h
 * @brief 纯五次 S 曲线滚球点到点控制（纯计算，不访问硬件）。
 *
 * 与 `middleware/ball_balance` 的区别是**控制哲学**，不是参数：
 *
 *   ball_balance : 反馈主导。每拍由当前误差直接算倾角，用非线性调度和制动
 *                  截停来收敛。视觉时延直接进入闭环相位。
 *   ball_scurve  : 前馈主导。进入时按当前实测状态一次性规划一条五次剖面，
 *                  之后倾角主要由 `a_ref(t)/K_G` 给出，反馈只纠正剖面跟踪误差。
 *
 * 选择前馈主导的物理依据：球对倾角是**无阻尼双积分器**，位置扰动响应
 * 幅值 = K_G·α/ω²。球以速度 v 通过波长 λ 的表面起伏时 ω = 2πv/λ，故
 * 表面缺陷的影响按 **1/v²** 衰减——v = 60 mm/s 时 λ = 20 mm、幅值 0.5° 的
 * 起伏只造成 0.17 mm 位置扰动。**滚动中的球基本不受凹槽缺陷影响**，
 * 模型可信，所以应当把权重放在前馈而不是高增益反馈上。
 *
 * 低速段由可选抖动或单向脱困突破静摩擦；两者互斥，均可关闭以恢复纯 S 曲线基线。
 *
 * 正方向约定：水管正倾角使球向正方向加速。调用方须保证视觉 x/v 与之一致。
 */
#ifndef BALL_SCURVE_H
#define BALL_SCURVE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 五次剖面归一化系数个数，仅供内部使用。 */
#define BALL_SCURVE_COEFFICIENTS 3U

typedef struct {
    /* ===== 对象模型 ===== */
    /**
     * 滚球增益 K_G = (5/7)·g，单位 mm/s² per rad。
     * 纯滚动实心球 1 + I/(mr²) = 7/5，故 ẍ = (5/7)·g·sinθ。
     * 该值是物理常数而非整定参数；只有管身扭转会让它偏小几个百分点。
     */
    float rolling_acceleration_gain_mm_s2;

    /* ===== 剖面规划 ===== */
    /** 剖面峰值加速度上限，mm/s²。决定所需峰值倾角 asin(a/K_G)。 */
    float max_acceleration_mm_s2;
    /** 剖面峰值速度上限，mm/s。取太小会拉长低速段，反而更容易被俘获。 */
    float max_velocity_mm_s;
    /** 规划时长的下限/上限，s。上限同时兜住"目标极近时时长趋零"。 */
    float min_duration_s;
    float max_duration_s;
    /**
     * 加速度前馈预览时间，s。用已知轨迹在 t+preview 的 a_ref 提前驱动水管，
     * 补偿步进位置环的一阶滞后；0 = 关闭预览，严格使用本拍 a_ref。
     */
    float acceleration_preview_s;

    /* ===== 剖面跟踪反馈 ===== */
    /** 位置反馈增益，deg/mm。由 wn²/K_G 换算。 */
    float kp_deg_per_mm;
    /** 速度反馈增益，deg/(mm/s)。由 2ζwn/K_G 换算。 */
    float kd_deg_per_mm_s;
    /**
     * 反馈分量的限幅，deg。**只限反馈，不限前馈**——前馈是模型算出的
     * 必要驱动，把它一起夹掉会让剖面根本走不动。
     */
    float feedback_limit_deg;

    /* ===== MOVE / BRAKE / HOLD 增益调度 ===== */
    /** 0 = 严格使用上面的固定 kp/kd；>0 = 启用连续增益调度。 */
    float gain_schedule_enabled;
    /** 制动段和保持段的目标增益。MOVE 继续使用 kp_deg_per_mm/kd_deg_per_mm_s。 */
    float brake_kp_deg_per_mm;
    float brake_kd_deg_per_mm_s;
    float hold_kp_deg_per_mm;
    float hold_kd_deg_per_mm_s;
    /** 停止距离模型：d = v*delay + v^2/(2*acceleration)。 */
    float brake_delay_s;
    float brake_acceleration_mm_s2;
    /** d_stop/|e_target| 从 start 到 full 时，BRAKE 权重由 0 平滑升到 1。 */
    float brake_blend_start_ratio;
    float brake_blend_full_ratio;
    /** BRAKE 权重的一阶平滑时间常数。 */
    float brake_blend_tau_s;
    /** HOLD 进入条件、退出滞回和进入驻留。 */
    float hold_enter_error_mm;
    float hold_enter_speed_mm_s;
    float hold_enter_dwell_s;
    float hold_exit_error_mm;
    float hold_exit_speed_mm_s;
    /** HOLD 权重的一阶平滑时间常数。 */
    float hold_blend_tau_s;
    /**
     * 速度测量权重：可信测量在 full_age 前权重为 1，之后线性降至 floor；
     * V_VALID 不可信时直接使用 floor。应用层的 200 ms 超时保护仍优先。
     */
    float velocity_full_weight_age_ms;
    float velocity_floor_weight_age_ms;
    float velocity_untrusted_weight;
    /** 剖面结束且 MOVING=0 时的速度反馈权重，抑制静止差分测速噪声。 */
    float stationary_velocity_weight;

    /* ===== 末端固定目标捕获 ===== */
    /** 实际目标误差进入该范围后，开始把移动参考平滑拉向固定目标。0 = 关闭。 */
    float capture_enter_error_mm;
    /** 实际目标误差进入该范围后，固定目标参考完全接管。 */
    float capture_full_error_mm;
    /** 捕获混合权重的一阶时间常数，s。 */
    float capture_blend_tau_s;

    /* ===== 前馈修正 ===== */
    /**
     * 滚动阻力前馈幅值，deg，按 sign(v_ref) 施加。
     *
     * 不加这一项会有**系统性欠冲**：滚阻在整个行程内恒定反向，欠冲量
     * = 0.5·K_G·θ_roll·T²。θ_roll = 0.06°、T = 1.2 s 时是 5.3 mm，
     * 占 ±10 mm 判据的一半以上。
     *
     * 默认 0：首次上板时先测出实际欠冲量，再反解 θ_roll 填回来
     * （θ_roll = 2·欠冲量/(K_G·T²)），比拍脑袋填一个值可靠。
     */
    float rolling_resistance_deg;
    /** 施加滚阻前馈所需的最小 |v_ref|，mm/s。低于此值不施加，避免末端换号抖动。 */
    float rolling_ff_speed_deadband_mm_s;
    /**
     * 动力学水平点相对查表 0° 的偏置，deg。
     *
     * 查表的 0° 是**几何名义水平**，不是"球不动"的那个角。二者之差来自
     * 上电参考位重复性、机构装配和水管弯曲。全部倾角指令都叠加本值。
     * 未标定前填 0，此时它的真实值会表现为剖面全程的单向漂移。
     */
    float level_bias_deg;

    /* ===== 输出限制 ===== */
    /** 水管角物理可达范围，deg（由编码器软限位经查表换算）。 */
    float angle_min_deg;
    float angle_max_deg;
    /** 输出斜率限制，deg/s。须覆盖剖面所需的 dθ_ff/dt，否则前馈会被削顶。 */
    float angle_rate_limit_deg_s;

    /* ===== 抖动：破静摩擦死区 ===== */
    /*
     * 为什么需要它——纯 PD 的稳态残差有一个**与整定无关的数学下限**：
     *
     *     残差 = θ_stick / Kp
     *
     * 球停住的条件是 |净倾角| < 脱离角，此时控制器再怎么推也推不动。
     * 2026-07-31 实测（COM16，75 s 无扰动）：球停在 −36.9 mm、目标 −50 mm，
     * 控制器持续给出 −0.620° 而球一动不动 ⇒ **θ_stick ≈ 0.62°**，
     * 代入 Kp=0.04711 °/mm 得下限 13.2 mm，与实测 13.1 mm 完全吻合。
     * 也就是说：**不破静摩擦，±10 mm 判据在这台机器上数学上不可达。**
     *
     * 抖动为什么代价极低——球对倾角是双积分器，位置响应幅值 = K_G·A/ω²，
     * 随频率按 ω⁻² 衰减。管子实打实抖 ±0.6°，球只晃约 0.5 mm。
     *
     * 幅值必须按**实际**水管角定，而指令要考虑步进位置环的一阶衰减
     * （τ = 1/STEP_MOTOR_SERVO_KP，当前 0.1 s）：
     *
     *     实际幅值 = 指令幅值 / sqrt(1 + (2πf·τ)²)
     *
     *   f      指令      实际     电机峰值转速    球纹波
     *   1.5Hz  ±0.82°   ±0.6°     47 deg/s      0.83 mm
     *   2.0Hz  ±0.97°   ±0.6°     74 deg/s      0.46 mm   ← 当前取值
     *   3.0Hz  ±1.28°   ±0.6°    146 deg/s ✗   0.21 mm    （超 120 限速）
     *
     * ⚠ angle_rate_limit_deg_s 必须 > 2πf·A，否则抖动会被斜率限制削顶。
     *   2 Hz / ±0.97° 需要 12.2 deg/s。
     */
    /** 抖动指令幅值，deg。**0 = 关闭**。 */
    float dither_amplitude_deg;
    float dither_frequency_hz;
    /**
     * 触发抖动所需的最小目标误差，mm。误差已经够小就别抖——
     * 抖动会带来约 0.5 mm 纹波，在判据内注入纹波是净亏损。
     */
    float dither_min_error_mm;
    /** 触发抖动所需的最大球速，mm/s。球还在滚就不需要破静摩擦。 */
    float dither_max_speed_mm_s;
    /** 上述条件需连续满足多久才起振，s。防止移动末段的瞬时低速误触发。 */
    float dither_dwell_s;

    /* ===== 小误差静止积分：补偿水平零偏与微小静摩擦 ===== */
    /** 积分增益，deg/(mm*s)。**0 = 关闭**。 */
    float hold_integral_ki_deg_per_mm_s;
    /** 小于等于该误差时不积分，mm。 */
    float hold_integral_min_error_mm;
    /** 仅在该误差范围内积分，mm；更大误差交给单向脱困。 */
    float hold_integral_max_error_mm;
    /** 允许开始/继续积分的最大球速，mm/s。 */
    float hold_integral_max_speed_mm_s;
    /** 达到该速度或 moving=true 后开始快速清零，mm/s。 */
    float hold_integral_release_speed_mm_s;
    /** 积分额外倾角的清零速率，deg/s。 */
    float hold_integral_release_rate_deg_s;
    /** 球运动后按速度追加的反向退积分增益，(deg/s)/(mm/s)=deg/mm。 */
    float hold_integral_motion_comp_deg_per_mm;

    /* ===== 单向脱困：破静摩擦的另一条路线 =====
     *
     * 与抖动的根本区别在于**能量方向**：
     *
     *   抖动   —— 左右往复，一半的功用在把球往远离目标的方向推，
     *              且突破瞬间球的速度方向是随机的（取决于在哪个相位挣脱）。
     *   单向脱困 —— 只朝目标方向渐增倾角，突破瞬间球必然朝目标走。
     *
     * 代价是它依赖**可靠的释放判据**：撤销太晚就过冲，撤销太早又缩回死区。
     * 抖动不需要释放判据（误差进判据自动停振），这是抖动唯一的结构优势。
     *
     * ⚠ 与抖动**互斥**。breakout_max_angle_deg > 0 时抖动被强制置 0，
     *   因为两者叠加会让脱困方向变得不确定，等于两个机制互相拆台。
     *
     * 大误差仍不用积分器处理；积分只服务于上面的有界小误差区间。
     * 两者互斥，避免额外倾角叠加。
     */
    /** 脱困额外倾角上限，deg。**0 = 关闭单向脱困（回退到抖动）**。
     *  实测静摩擦脱离角 θ_stick ≈ 0.62°，取 1.0° 留 60% 余量。
     *  不建议一开始超过 1.2°——顶太高会在突破瞬间给出过量驱动。 */
    float breakout_max_angle_deg;
    /** 倾角渐增速率，deg/s。决定"多久能突破"：0.8°/s 时约 0.8 s 爬到 0.62°。 */
    float breakout_ramp_rate_deg_s;
    /** 撤销速率，deg/s。远大于渐增速率——检测到动了就要赶紧撤，
     *  但仍走斜坡而不是瞬间归零，避免指令角阶跃。 */
    float breakout_release_rate_deg_s;
    /** 触发所需的最小目标误差，mm。 */
    float breakout_min_error_mm;
    /** 触发所需的最大球速，mm/s。球还在滚就不是卡住。 */
    float breakout_max_speed_mm_s;
    /** 触发条件需连续满足多久，s。 */
    float breakout_dwell_s;
    /**
     * 判定"已脱困"的速度阈值，mm/s。
     *
     * ⚠ 不能用 v != 0 —— 视觉速度有量化（1 mm/s 台阶）和 62 ms 滤波延迟，
     *   噪声就能满足 v != 0，会在球还没真动时就撤销倾角，形成"爬升-撤销"循环。
     */
    float breakout_release_speed_mm_s;
    /** 释放条件需连续满足多久，s。 */
    float breakout_release_dwell_s;

    /* ===== 稳定判据（只影响遥测与状态，不参与控制律）===== */
    float settled_position_mm;
    float settled_speed_mm_s;
    float settled_time_s;

    /** 跟踪位置误差阈值，mm。0 = 不按位置误差触发重规划。 */
    float replan_error_mm;
    /** 跟踪速度误差阈值，mm/s。0 = 不按速度误差触发重规划。 */
    float replan_velocity_error_mm_s;
    /** 误差需连续超限多久才重规划，s。 */
    float replan_dwell_s;
    /** 两次重规划的最小间隔，s，防止每拍重启剖面。 */
    float replan_cooldown_s;
} BALL_SCURVE_CONFIG;

typedef struct {
    /* 规划起点状态（规划时刻冻结）。 */
    float x0_mm;
    float v0_mm_s;
    float a0_mm_s2;
    /* 归一化五次系数，作用于 s = t/T 的 s³/s⁴/s⁵ 项。 */
    float c[BALL_SCURVE_COEFFICIENTS];
    float duration_s;
    float elapsed_s;
    float target_mm;
    bool  active;

    /* 输出斜率限制与稳定判据的状态。 */
    float last_angle_deg;
    bool  have_last_angle;
    float settled_elapsed_s;
    bool  settled;

    /* 抖动状态。相位连续累加，避免起停时产生角度阶跃。 */
    float dither_phase_rad;
    float dither_stuck_elapsed_s;
    bool  dither_on;

    /* 小误差静止积分状态。 */
    float hold_integral_deg;
    bool  hold_integral_on;

    /* 单向脱困状态 */
    float breakout_angle_deg;           /**< 当前额外倾角，**带符号** */
    float breakout_stuck_elapsed_s;     /**< 卡住条件已持续多久 */
    float breakout_release_elapsed_s;   /**< 释放条件已持续多久 */
    bool  breakout_on;

    /* 增益调度状态。权重连续变化，状态切换不直接制造输出阶跃。 */
    float brake_blend;
    float hold_blend;
    float hold_enter_elapsed_s;
    bool  hold_mode;
    float capture_blend;

    /* 受控重规划状态。 */
    float replan_error_elapsed_s;
    float replan_cooldown_elapsed_s;
} BALL_SCURVE_CONTROLLER;

typedef struct {
    float x_mm;
    float velocity_mm_s;
    /**
     * 车辆沿钢球正坐标方向的规划加速度，mm/s²。
     * 正值产生正倾角 atan(a/g)，用于抵消车体加速造成的反向惯性力；
     * 静止、匀速以及不需要运动补偿的任务传 0。
     */
    float vehicle_acceleration_mm_s2;
    /** 由编码器经查表得到的**实际**水管角，用于斜率限制的续接。 */
    float actual_angle_deg;
    /** 视觉速度是否通过 V_VALID 与连续帧恢复判据。 */
    bool  velocity_trusted;
    /** 视觉预测器确认球正在移动；比单拍速度阈值更抗量化噪声。 */
    bool  moving;
    /** 当前有效测量龄，ms。 */
    float measurement_age_ms;
    float dt_s;
} BALL_SCURVE_INPUT;

typedef enum {
    BALL_SCURVE_GAIN_MOVE = 0,
    BALL_SCURVE_GAIN_BRAKE,
    BALL_SCURVE_GAIN_HOLD
} BALL_SCURVE_GAIN_MODE;

typedef struct {
    /** 最终下发的水管角指令，deg（已含 level_bias、限幅与斜率限制）。 */
    float angle_deg;

    /* 剖面参考量。 */
    float x_ref_mm;
    float v_ref_mm_s;
    float a_ref_mm_s2;
    float acceleration_preview_mm_s2; /**< 用于前馈的 t+preview 加速度 */

    /* 指令分解，便于遥测中直接看清每一项的贡献。 */
    float feedforward_deg;   /**< asin(a_ref(t+preview)/K_G) */
    float actuator_lead_deg; /**< 预览前馈相对本拍 a_ref 前馈的超前量 */
    float rolling_ff_deg;    /**< 滚阻前馈 */
    float feedback_deg;      /**< 限幅后的 PD 分量 */
    float dither_deg;        /**< 本拍注入的抖动量（已含符号） */
    float hold_integral_deg; /**< 小误差静止积分产生的额外倾角 */
    float breakout_deg;      /**< 本拍的单向脱困倾角（已含符号，朝目标为正） */
    float breakout_stuck_s;  /**< 卡住条件已连续满足多久，s（遥测用，判误触发） */
    float breakout_release_s;/**< 释放条件已连续满足多久，s（遥测用，判释放是否太晚） */

    float position_error_mm;
    float velocity_error_mm_s;
    float effective_kp_deg_per_mm;
    float effective_kd_deg_per_mm_s;
    float brake_blend;
    float hold_blend;
    float capture_blend;
    float stopping_distance_mm;
    float closing_velocity_mm_s;
    float velocity_weight;
    float profile_time_s;
    float profile_duration_s;

    bool  profile_active;    /**< 剖面尚未走完 */
    bool  saturated;         /**< 总指令被物理角度范围夹住 */
    bool  feedback_clipped;  /**< PD 分量被 feedback_limit_deg 夹住 */
    bool  rate_limited;      /**< 本拍被输出斜率限制削过 */
    bool  dither_on;         /**< 抖动正在注入 */
    bool  hold_integral_on;  /**< 小误差积分本拍正在累积 */
    bool  breakout_on;       /**< 单向脱困正在介入（与 dither_on 互斥） */
    bool  replanned;         /**< 本拍因跟踪误差执行了受控重规划 */
    bool  settled;
    BALL_SCURVE_GAIN_MODE gain_mode;
} BALL_SCURVE_OUTPUT;

/** 初始化并清空全部状态；不产生任何输出。 */
void BallScurve_Init(BALL_SCURVE_CONTROLLER *controller);

/**
 * @brief 清空动态状态，并令下一拍的斜率限制从给定实际水管角续接。
 * @note 进入任务或视觉恢复后必须调用，否则第一拍会从上一次的陈旧角度做斜率限制。
 */
void BallScurve_Reset(BALL_SCURVE_CONTROLLER *controller, float current_angle_deg);

/**
 * @brief 按当前实测状态规划一条到 target_mm 的五次剖面（末端 v=0、a=0）。
 * @return 规划成功为 true；参数非法（K_G/时长/加速度上限非正）为 false。
 *
 * 时长由「峰值加速度」「峰值速度」「时长下限」三者取严，再迭代收紧到
 * 真实峰值加速度不超上限——起点带初速时闭式公式只是近似，必须校核。
 */
bool BallScurve_PlanTo(BALL_SCURVE_CONTROLLER *controller,
                       const BALL_SCURVE_CONFIG *config,
                       float x_now_mm,
                       float v_now_mm_s,
                       float target_mm);

/** 执行一次剖面推进 + 前馈/反馈合成。剖面走完后退化为绕目标点的纯 PD。 */
bool BallScurve_Update(BALL_SCURVE_CONTROLLER *controller,
                       const BALL_SCURVE_CONFIG *config,
                       const BALL_SCURVE_INPUT *input,
                       BALL_SCURVE_OUTPUT *output);

/** 剖面是否已走完（不代表球已停稳，稳定看 output.settled）。 */
bool BallScurve_IsProfileFinished(const BALL_SCURVE_CONTROLLER *controller);

/**
 * @brief 按剖面时长公式估算一次移动需要的时间，s。
 * @note 供上层做序列时间预算，不改变控制器状态。
 */
float BallScurve_EstimateDuration(const BALL_SCURVE_CONFIG *config,
                                  float distance_mm);

#ifdef __cplusplus
}
#endif

#endif /* BALL_SCURVE_H */
