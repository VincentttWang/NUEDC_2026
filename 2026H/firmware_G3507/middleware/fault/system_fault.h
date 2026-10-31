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
 * @brief 执行故障处理流程，设置故障、刹停底盘并显示故障页。
 * @note 需在 Chassis_Init()/Ui_Init() 之后调用：内部会刹停底盘并渲染错误页；
 *       若在基础外设初始化前触发，刹车/显示可能无效（不会崩溃，仅无输出）。
 * @note 内部经 SystemFault_Halt() 进入死循环并关中断，函数不会返回。
 */
void SystemFault_Handler(SYSTEM_FAULT_CODE code, const char *message);

/**
 * @brief 停机等待，用于不可恢复故障。
 * @note 刹停底盘并渲染错误页后 __disable_irq() 并死循环，函数不会返回；
 *       依赖 Chassis_Init()/Ui_Init() 已完成（见 SystemFault_Handler 说明）。
 */
void SystemFault_Halt(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_FAULT_H */
