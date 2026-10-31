/**
 * @file  hall_encoder.h
 * @brief BSP 霍尔编码器接口(左右双轮)。
 */
#ifndef HALL_ENCODER_H
#define HALL_ENCODER_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 编码器标识(对应左右轮)。
 */
typedef enum {
    HALL_ENCODER_LEFT = 0,
    HALL_ENCODER_RIGHT,
    HALL_ENCODER_COUNT
} HALL_ENCODER_ID;

/*
 * 引脚映射(SysConfig 生成板级名)。每轮方案: A 相上升沿中断计数 + 读 B 相电平定方向。
 *   右轮: A=ENC_R_A(PA28, 中断), B=ENC_R_B(PA2, 读向);
 *   左轮: A=ENC_L_A(PA22, 中断), B=ENC_L_B(PA25, 读向)。
 * 两轮 A 相均在 GPIOA, 中断聚合到 INT_GROUP1(GPIOA_INT_IRQn)。
 */
#define HALL_ENCODER_R_A_PORT           Motor_IO_ENC_R_A_PORT
#define HALL_ENCODER_R_A_PIN            Motor_IO_ENC_R_A_PIN
#define HALL_ENCODER_R_B_PORT           Motor_IO_ENC_R_B_PORT
#define HALL_ENCODER_R_B_PIN            Motor_IO_ENC_R_B_PIN
#define HALL_ENCODER_L_A_PORT           Motor_IO_ENC_L_A_PORT
#define HALL_ENCODER_L_A_PIN            Motor_IO_ENC_L_A_PIN
#define HALL_ENCODER_L_B_PORT           Motor_IO_ENC_L_B_PORT
#define HALL_ENCODER_L_B_PIN            Motor_IO_ENC_L_B_PIN
#define HALL_ENCODER_GPIO_IRQN          GPIOA_INT_IRQn
#define HALL_ENCODER_SAMPLE_TIMER       TIMER_0_INST
#define HALL_ENCODER_SAMPLE_TIMER_IRQN  TIMER_0_INST_INT_IRQN

/* 里程换算参数: 距离 = 计数 / (PPR·减速比) · π·轮径 · 标定系数。两轮同型号, 共用。 */
#define HALL_ENCODER_PI                 3.1415926f
#define HALL_ENCODER_PPR                13.0f
#define HALL_ENCODER_REDUCTION_RATIO    28.0f
#define HALL_ENCODER_WHEEL_DIAMETER_M   0.065f
/*
 * 测速窗口，秒。**必须与 SysConfig 里 TIMER_0(TIMA1) 的实际周期一致**。
 *
 * ⚠ 本值刻意**不跟随控制周期**：控制环为对齐视觉已过采样到 10ms，而测速窗每缩短
 *   一半，速度量化台阶就翻一倍（20ms 窗 0.0295 m/s，10ms 窗 0.0590 m/s）。
 *   在 0.26 m/s 巡航点前者是 11% 噪声、后者 23%，后者会把速度环推回整改前的状态。
 *   编码器测速与控制拍因此解耦：控制环 100Hz 读，测速窗保持 50Hz 更新，
 *   每个速度值被连读两拍。底盘速度环是纯 PI(KD=0)，阶梯输入不会激发微分尖峰。
 *   要真正提高速度分辨率应做 A 相双边沿或 AB 四倍频，而不是缩短窗口。
 */
#define HALL_ENCODER_SAMPLE_PERIOD_S    0.02f
#define HALL_ENCODER_DISTANCE_SCALE     1.05f

/*
 * 每轮"整车前进为正"的计数符号。左右轮镜像安装, 同样的电气解码在整车前进时符号可能相反;
 * 用本符号把两轮统一到"整车前进为正"。上板验证: 整车前进时两轮 speed 应同为正, 某轮相反则翻其符号。
 */
#define HALL_ENCODER_LEFT_DIR_SIGN      (-1)
#define HALL_ENCODER_RIGHT_DIR_SIGN     (+1)

/**
 * @brief 霍尔编码器方向。
 */
typedef enum {
    HALL_ENCODER_DIR_FORWARD = 0,
    HALL_ENCODER_DIR_REVERSE
} HALL_ENCODER_DIR;

/**
 * @brief 初始化左右编码器计数/采样状态, 使能 GPIO 与采样定时器中断。
 */
BSP_STATUS HallEncoder_Init(void);

/**
 * @brief 更新一次速度采样(结算左右两轮)。
 * @note 通常由固定周期定时器中断调用。
 */
void HallEncoder_UpdateSample(void);

/**
 * @brief 获取指定轮上一采样窗口的编码器增量计数(非自复位起的累计, 累计里程用 GetDistance)。
 */
int32_t HallEncoder_GetCount(HALL_ENCODER_ID id);

/**
 * @brief 获取指定轮最近判断的编码器方向。
 */
HALL_ENCODER_DIR HallEncoder_GetDir(HALL_ENCODER_ID id);

/**
 * @brief 获取指定轮估算线速度，单位 m/s。
 */
float HallEncoder_GetSpeed(HALL_ENCODER_ID id);

/**
 * @brief 获取指定轮累计距离估计，单位 m。
 */
float HallEncoder_GetDistance(HALL_ENCODER_ID id);

/**
 * @brief 清零两轮计数、速度和距离状态。
 */
void HallEncoder_Reset(void);

/**
 * @brief 只清零两轮累计距离，不清零编码器原始计数。
 */
void HallEncoder_ResetDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* HALL_ENCODER_H */
