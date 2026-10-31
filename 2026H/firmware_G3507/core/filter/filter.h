/**
 * @file  filter.h
 * @brief Core 层通用信号调理算法：一阶低通(EMA)与中心死区。
 *
 * 纯算法模块，不依赖硬件。均为无状态自由函数，滤波状态由调用方持有
 * （与 core/kinematics 的 Clamp/DifferentialMix 同风格）。当前消费者为
 * middleware/line_follow，用于抑制数字灰度量化跳变引起的转向尖峰/蛇形。
 */
#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一阶低通滤波(指数移动平均)单步更新：new = state + alpha * (sample - state)。
 * @param state  上一拍滤波值(调用方持有)。
 * @param sample 本拍原始输入。
 * @param alpha  平滑系数，有效范围 (0, 1]；越小越平滑越滞后，取 1 则直通 sample。
 * @return 本拍滤波值，供下一拍作为 state 传入。
 */
float Filter_LowpassEma(float state, float sample, float alpha);

/**
 * @brief 中心死区：|value| < threshold 时归零，否则原样返回。
 * @param value     输入值。
 * @param threshold 死区半宽；<= 0 时不处理，原样返回。
 * @return 处理后的值。
 */
float Filter_Deadband(float value, float threshold);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */
