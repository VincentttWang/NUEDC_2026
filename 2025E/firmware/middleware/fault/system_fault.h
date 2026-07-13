/**
 * @file  system_fault.h
 * @brief Middleware 层系统故障处理接口。
 */
#ifndef SYSTEM_FAULT_H
#define SYSTEM_FAULT_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SYSTEM_FAULT_MESSAGE_LEN 64U

/**
 * @brief 系统故障码。
 */
typedef enum {
    SYSTEM_FAULT_NONE = 0,
    SYSTEM_FAULT_UNKNOWN,
    SYSTEM_FAULT_INVALID_ARG,
    SYSTEM_FAULT_ALLOC_FAILED,
    SYSTEM_FAULT_STATE_ERROR,
    SYSTEM_FAULT_HARDWARE
} SYSTEM_FAULT_CODE;

/**
 * @brief 系统故障信息快照。
 */
typedef struct {
    /** 当前故障码。 */
    SYSTEM_FAULT_CODE code;
    /** 当前故障说明，以 '\0' 结尾。 */
    char message[SYSTEM_FAULT_MESSAGE_LEN];
} SYSTEM_FAULT_INFO;

/**
 * @brief 设置当前故障信息。
 */
BSP_STATUS SystemFault_Set(SYSTEM_FAULT_CODE code, const char *message);

/**
 * @brief 获取当前故障信息。
 */
BSP_STATUS SystemFault_Get(SYSTEM_FAULT_INFO *out);

/**
 * @brief 获取当前故障码。
 */
SYSTEM_FAULT_CODE SystemFault_GetCode(void);

/**
 * @brief 获取当前故障消息字符串。
 */
const char *SystemFault_GetMessage(void);

/**
 * @brief 清除当前故障信息。
 */
void SystemFault_Clear(void);

/**
 * @brief 执行故障处理流程，设置故障、停止底盘和云台并显示故障页。
 */
void SystemFault_Handler(SYSTEM_FAULT_CODE code, const char *message);

/**
 * @brief 停机等待，用于不可恢复故障。
 */
void SystemFault_Halt(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_FAULT_H */
