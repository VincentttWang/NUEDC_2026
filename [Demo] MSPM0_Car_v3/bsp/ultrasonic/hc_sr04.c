/**
 * @file  hc_sr04.c
 * @brief HC-SR04 超声波测距驱动实现 (GPIO 轮询 + SysTick 精确定时)
 *
 * 原理:
 *   1. Trig 发 ~15μs 高电平脉冲
 *   2. 模块自动发 8 个 40kHz 超声波
 *   3. Echo 变高, 持续时间与距离成正比
 *   4. 距离 (cm) = 脉宽 (μs) / 58
 *
 * 定时: 使用 SysTick CVR (32MHz 递减计数, 周期 1ms) + BSP_Time_GetMs()
 *       实现跨 ms 边界的微秒级脉宽测量
 */

#include "hc_sr04.h"
#include "bsp_time.h"
#include <ti/devices/msp/msp.h>

/* ═══════════════════ 内部实现 ═══════════════════════════════════════════ */

/*
 * 带超时的 Echo 电平检测
 * level: 0=等待低, 1=等待高
 * 返回: true=检测到目标电平, false=超时
 */
static bool SR04_WaitEcho(uint8_t level, uint32_t timeout_cycles)
{
    uint32_t pin_mask = SR04_ECHO_PIN;

    while (timeout_cycles > 0U){
        uint32_t val = DL_GPIO_readPins(SR04_ECHO_PORT, pin_mask) & pin_mask;
        if (level == 0U){
            if (val == 0U){ return true; }
        } else{
            if (val != 0U){ return true; }
        }
        timeout_cycles--;
    }
    return false;
}

/*
 * 使用 SysTick 硬件计数器 (CVR) 测量微秒级时间
 * CVR 为 32MHz 递减计数器, 每 32000 个 tick 重载一次 (1ms 周期)
 * 分辨率: 31.25ns
 */
static uint32_t SR04_ReadSysTickUs(void)
{
    return (32000U - SysTick->VAL) / 32U;  /* 转换为 μs (0~999) */
}

/* ═══════════════════ 公开 API ═══════════════════════════════════════════ */

float SR04_GetDistance(void)
{
    uint32_t start_ms, end_ms, start_tick, end_tick;
    uint32_t elapsed_us;

    /* 确保模块就绪 (Echo 为低) */
    if (!SR04_WaitEcho(0U, SR04_TIMEOUT_CYCLES)){
        return -1.0f;
    }

    /* 发 Trig 脉冲 ~15μs */
    DL_GPIO_setPins(SR04_TRIG_PORT, SR04_TRIG_PIN);
    DL_Common_delayCycles(480U);  /* 15μs @ 32MHz */
    DL_GPIO_clearPins(SR04_TRIG_PORT, SR04_TRIG_PIN);

    /* 等 Echo 上升沿 */
    if (!SR04_WaitEcho(1U, SR04_TIMEOUT_CYCLES)){
        return -1.0f;
    }

    /* 记录起始时间 (跨 ms 边界处理) */
    start_ms   = BSP_Time_GetMs();
    start_tick = SR04_ReadSysTickUs();

    /* 等 Echo 下降沿 */
    if (!SR04_WaitEcho(0U, SR04_TIMEOUT_CYCLES)){
        return -1.0f;
    }

    /* 计算脉宽 = (结束ms - 起始ms)*1000 + (结束μs - 起始μs) */
    end_ms   = BSP_Time_GetMs();
    end_tick = SR04_ReadSysTickUs();

    elapsed_us = (end_ms - start_ms) * 1000U;
    if (end_tick >= start_tick){
        elapsed_us += (end_tick - start_tick);
    } else{
        /* SysTick 在测量期间重载 (每 1ms), 补偿 1000μs */
        elapsed_us += (1000U + end_tick - start_tick);
    }

    /* 距离 (cm) = 声速 × 脉宽 / 2 ≈ 脉宽(μs) / 58 */
    return (float)elapsed_us / 58.0f;
}

float SR04_GetDistanceAvg(uint8_t count)
{
    float sum = 0.0f;
    uint8_t valid = 0U;

    if (count == 0U){ count = 1U; }

    for (uint8_t i = 0U; i < count; i++){
        float d = SR04_GetDistance();
        if (d > 0.0f){
            sum += d;
            valid++;
        }
    }

    return (valid > 0U) ? (sum / (float)valid) : -1.0f;
}
