/**
 * @file  line_follow.h
 * @brief Middleware 层巡线运行状态服务接口。
 */
#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include "bsp_common.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LINE_FOLLOW_SENSOR_COUNT 8U

/**
 * @brief 巡线传感器状态快照。
 */
typedef struct {
    /** 8 路传感器最近一次数字状态，通道顺序与 GrayscaleSensor 逻辑通道一致。 */
    uint8_t value[LINE_FOLLOW_SENSOR_COUNT];
    /** 8 路传感器位掩码，bit0 对应第 0 路，bit 置位规则由 LineFollow_BuildMask() 决定。 */
    uint8_t mask;
} LINE_FOLLOW_SENSOR_STATE;

/**
 * @brief 巡线运行状态。
 */
typedef struct {
    /** 最近一次传感器状态。 */
    LINE_FOLLOW_SENSOR_STATE sensor;
    /** 边线或阶段计数，由上层任务按规则维护；middleware 只保存数值。 */
    int32_t edge_count;
    /** 当前是否处于转弯状态。 */
    bool turning;
} LINE_FOLLOW_STATE;

/**
 * @brief 初始化巡线传感器和运行状态。
 */
BSP_STATUS LineFollow_Init(void);

/**
 * @brief 清空巡线运行状态。
 */
void LineFollow_Reset(void);

/**
 * @brief 更新传感器状态和派生状态。
 */
BSP_STATUS LineFollow_Update(void);

/**
 * @brief 只更新传感器快照。
 */
BSP_STATUS LineFollow_UpdateSensor(void);

/**
 * @brief 获取巡线状态快照。
 */
BSP_STATUS LineFollow_GetState(LINE_FOLLOW_STATE *out);

/**
 * @brief 获取最近一次传感器状态。
 */
BSP_STATUS LineFollow_GetSensor(LINE_FOLLOW_SENSOR_STATE *out);

/**
 * @brief 获取最近一次传感器位掩码。
 */
uint8_t LineFollow_GetSensorMask(void);

/**
 * @brief 获取指定通道传感器值。
 */
uint8_t LineFollow_GetSensorValue(uint8_t index);

/**
 * @brief 获取当前有效传感器数量。
 */
uint8_t LineFollow_GetActiveCount(void);

/**
 * @brief 判断有效传感器数量是否落在指定闭区间。
 */
bool LineFollow_IsActiveCountInRange(uint8_t min_count, uint8_t max_count);

/**
 * @brief 判断是否未检测到线。
 */
bool LineFollow_IsEmpty(void);

/**
 * @brief 判断是否满足半线检测条件。
 */
bool LineFollow_IsHalfDetected(void);

/**
 * @brief 判断是否满足十字检测条件。
 */
bool LineFollow_IsCrossDetected(void);

/**
 * @brief 判断中间传感器是否检测到线。
 */
bool LineFollow_IsCenterActive(void);

/**
 * @brief 获取边线或阶段计数。
 */
int32_t LineFollow_GetEdgeCount(void);

/**
 * @brief 设置边线或阶段计数。
 */
void LineFollow_SetEdgeCount(int32_t edge_count);

/**
 * @brief 将边线或阶段计数加一。
 */
void LineFollow_IncrementEdge(void);

/**
 * @brief 清零边线或阶段计数。
 */
void LineFollow_ResetEdge(void);

/**
 * @brief 获取转弯状态。
 */
bool LineFollow_IsTurning(void);

/**
 * @brief 设置转弯状态。
 */
void LineFollow_SetTurning(bool turning);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOW_H */
