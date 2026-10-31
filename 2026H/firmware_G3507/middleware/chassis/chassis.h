/**
 * @file  chassis.h
 * @brief Middleware 层底盘组合服务接口。
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include "bsp_common.h"
#include "hall_encoder.h"
#include "tb6612fng.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 每轮速度环 = 前馈 + PI 残差修正。
 *
 * 前馈由 Device Check「Duty Sweep」实测的「占空比 -> 稳态轮速」曲线反解得到
 * (2026-07-29 抬轮空载标定，左右轮分别最小二乘拟合，10%~80% 区间残差 < 2.4%)：
 *     左 duty% = 81.85*v + 1.03      右 duty% = 77.25*v + 1.70
 * offset 取目标速度的符号（反向实测斜率/截距与正向相差 < 4%，由 PI 补齐）。
 * 前馈承担主出力，PI 只补残差，积分因此稳定维持在小值。
 *
 * ⚠ 该曲线是**抬轮空载**标定。落地后负载更大、同占空比对应更低速度，PI 需补一个正
 *   偏置；上地面后应重扫 Duty Sweep 重新拟合这四个常数。
 */
#define CHASSIS_FF_LEFT_GAIN         81.85f
#define CHASSIS_FF_LEFT_OFFSET        1.03f
#define CHASSIS_FF_RIGHT_GAIN        77.25f
#define CHASSIS_FF_RIGHT_OFFSET       1.70f

/*
 * PI 残差修正增益(位置式: setpoint=目标轮速 m/s, feedback=实测轮速, output=占空比修正量 %)。
 * kp 由 200 降到 60：编码器 20ms 窗的量化台阶是 0.0295 m/s/计数，kp=200 时单个计数
 * 跳变就产生 5.9% 占空比抖动（实测静止时 σ=6.9%、摆幅 -17%~+25%），降到 60 后约 1.8%。
 * 前馈接管主出力后 kp 不再需要承担建立稳态出力的职责。
 * @note KI 不得为 0（CHASSIS_SPEED_INTEGRAL_LIMIT 由其派生）。
 */
#define CHASSIS_SPEED_KP             60.0f
#define CHASSIS_SPEED_KI             60.0f
#define CHASSIS_SPEED_KD             0.0f

/** PI 修正量的绝对值限幅（前馈之外的余量）；前馈+修正的总和由 TB6612 再限到 ±100%。 */
#define CHASSIS_SPEED_OUTPUT_LIMIT   40.0f

/*
 * 积分限幅必须按 output_limit/ki 派生：PID_Limit 限的是积分累加量**本身**，而输出中的
 * 积分项是 ki*integral，故越过本值后积分项已单独占满修正限幅，继续累积纯属过度累积，
 * 只会拖长退绕时间。
 * 旧配置 ki=17 / integral_limit=96 相差 16 倍，实测把目标顶到不可达值 11.9s 后积分冲到
 * 10.7（饱和参考线的 1.8 倍），清零目标后 55s 才退绕完、64.8s 观测窗内轮子始终未停稳。
 */
#define CHASSIS_SPEED_INTEGRAL_LIMIT (CHASSIS_SPEED_OUTPUT_LIMIT / CHASSIS_SPEED_KI)

/*
 * 速度环反馈的一阶低通(EMA)平滑系数，alpha = 1 - exp(-2*pi*fc*dt)。
 * 取 fc ≈ 6 Hz、dt = 10 ms → alpha ≈ 0.31（`control-plan.md` §2.4 要求 5~8 Hz）。
 * 目的：压制编码器 20ms 窗的量化跳变（台阶 0.0295 m/s，在 0.26 m/s 工作点即 11% 噪声）。
 * 只作用于**速度环反馈**；`HallEncoder_GetSpeed()` 与 `[SPD]` 遥测的 l/r 仍是原始值，
 * 便于诊断。取 1.0f 即直通（做滤波前后 A/B 时改这一个值即可）。
 *
 * ⚠ 本系数**按拍生效**，改控制周期必须重算，否则截止频率按比例漂移：
 *   dt=20ms → 0.53（历史值）、dt=10ms → 0.31（当前）。
 *   反馈源仍是 50Hz 更新的测速窗，本滤波在 100Hz 控制拍上跑，等效连续时间
 *   截止频率不变，同时顺带平滑了阶梯输入。
 */
#define CHASSIS_SPEED_FEEDBACK_ALPHA 0.31f

/*
 * 目标速度绝对值低于该阈值即视为**停止指令**：速度环在起转死区（实测 10% 占空比）内无法
 * 收敛——小占空比不产生运动，没有反馈，积分就退不下去，残留积分会驱动轮子长时间爬行
 * （实测目标清零后 60s 仍在 0.03~0.06 m/s 抽搐）。故直接复位该轮 PID 并主动刹车。
 * 阈值取远低于最小可用目标（编码器量化台阶 0.0295 m/s，低于此的目标本就不可分辨）。
 */
#define CHASSIS_SPEED_ZERO_TARGET_MPS 0.01f

/**
 * @brief 底盘停止模式。
 */
typedef enum {
    /** 关闭 PWM，让电机自然滑行。 */
    CHASSIS_STOP_MODE_COAST = 0,
    /** IN1/IN2 同时制动，主动刹车。 */
    CHASSIS_STOP_MODE_BRAKE
} CHASSIS_STOP_MODE;

/**
 * @brief 底盘控制模式。
 */
typedef enum {
    /** 开环：直接下发占空比(默认)。 */
    CHASSIS_CONTROL_DUTY = 0,
    /** 闭环：每轮速度环 PID 跟踪目标轮速。 */
    CHASSIS_CONTROL_SPEED
} CHASSIS_CONTROL_MODE;

/**
 * @brief 左右轮占空比输出。
 */
typedef struct {
    /** 左轮占空比，范围通常为 [-100, 100]。 */
    float left_percent;
    /** 右轮占空比，范围通常为 [-100, 100]。 */
    float right_percent;
} CHASSIS_DUTY;

/**
 * @brief 初始化底盘相关 BSP 外设。
 */
BSP_STATUS Chassis_Init(void);

/**
 * @brief 设置左右轮占空比并立即输出到底盘电机(开环)。切回 CHASSIS_CONTROL_DUTY 模式。
 * @param left_percent 左轮占空比，正负表示方向。
 * @param right_percent 右轮占空比，正负表示方向。
 */
BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent);

/**
 * @brief 设置左右轮目标线速度并进入速度闭环模式(CHASSIS_CONTROL_SPEED)。
 * @param left_mps  左轮目标线速度，m/s(负为倒转)。
 * @param right_mps 右轮目标线速度，m/s。
 * @note 仅设定目标; 实际出力由 Chassis_UpdateSpeedControl() 周期驱动。从开环切入时复位 PID。
 */
void Chassis_SetWheelSpeed(float left_mps, float right_mps);

/**
 * @brief 设置车体直行目标线速度(两轮同速), 进入速度闭环。
 * @param body_mps 目标线速度，m/s。
 * @note 差速转向需 track_width(暂缺), 故此处仅直行; 后续可用 core/kinematics BodyToWheel 扩展。
 */
void Chassis_SetSpeed(float body_mps);

/**
 * @brief 速度闭环单步更新: 闭环模式下跑两轮 PID 并出力; 开环模式为空操作。
 * @param dt_s 控制周期，s。
 * @return 出力状态; 开环模式恒返回 BSP_STATUS_OK。
 * @note 须由控制周期任务周期调用(当前接在 App_ControlTick, 20ms)。
 */
BSP_STATUS Chassis_UpdateSpeedControl(float dt_s);

/**
 * @brief 获取当前底盘控制模式。
 */
CHASSIS_CONTROL_MODE Chassis_GetControlMode(void);

/**
 * @brief 获取指定轮当前目标线速度(m/s), 仅闭环模式有意义。
 */
float Chassis_GetWheelSpeedTarget(HALL_ENCODER_ID wheel);

/**
 * @brief 获取指定轮速度环 PID 的积分累加值, 仅闭环模式有意义。
 * @note 诊断/整定用: 输出中的积分项为 CHASSIS_SPEED_KI * 本值, 故本值达到
 *       CHASSIS_SPEED_INTEGRAL_LIMIT (= OUTPUT_LIMIT/KI) 时积分项已单独占满修正限幅。
 *       该轮目标为停止、或切回开环时都会复位为 0。
 */
float Chassis_GetWheelSpeedIntegral(HALL_ENCODER_ID wheel);

/**
 * @brief 按指定模式停止底盘。
 */
BSP_STATUS Chassis_Stop(CHASSIS_STOP_MODE mode);

/**
 * @brief 主动刹停左右电机。
 */
BSP_STATUS Chassis_Brake(void);

/**
 * @brief 关闭左右电机输出，使其自然滑行。
 */
BSP_STATUS Chassis_Coast(void);

/**
 * @brief 获取最近设置的左右轮占空比。
 */
CHASSIS_DUTY Chassis_GetDuty(void);

/**
 * @brief 获取车体估算线速度(左右轮均值)，单位 m/s。
 */
float Chassis_GetSpeed(void);

/**
 * @brief 获取指定轮估算线速度，单位 m/s。
 */
float Chassis_GetWheelSpeed(HALL_ENCODER_ID wheel);

/**
 * @brief 获取车体累计距离(左右轮均值)，单位 m。
 */
float Chassis_GetDistance(void);

/**
 * @brief 获取指定轮累计距离，单位 m。
 */
float Chassis_GetWheelDistance(HALL_ENCODER_ID wheel);

/**
 * @brief 清零两轮编码器累计距离。
 */
void Chassis_ResetDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
