/**
 * @file  system_fault.c
 * @brief Middleware system fault handling implementation.
 */
#include "system_fault.h"
#include "chassis.h"
#include "gimbal.h"
#include "ui.h"
#include <stddef.h>

static SYSTEM_FAULT_INFO s_system_fault_info = {
    .code = SYSTEM_FAULT_NONE,
    .message = "",
};

static const char *SystemFault_DefaultMessage(const char *message){
    return (message == NULL) ? "Fault" : message;
}

static const char *SystemFault_CodeText(SYSTEM_FAULT_CODE code){
    switch (code){
        case SYSTEM_FAULT_NONE:
            return "NONE";
        case SYSTEM_FAULT_INVALID_ARG:
            return "ARG";
        case SYSTEM_FAULT_ALLOC_FAILED:
            return "ALLOC";
        case SYSTEM_FAULT_STATE_ERROR:
            return "STATE";
        case SYSTEM_FAULT_HARDWARE:
            return "HW";
        case SYSTEM_FAULT_UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

static void SystemFault_CopyMessage(char *dst, const char *src){
    const char *message = SystemFault_DefaultMessage(src);
    uint8_t i = 0U;

    while (i < (SYSTEM_FAULT_MESSAGE_LEN - 1U) && message[i] != '\0'){
        dst[i] = message[i];
        i++;
    }

    dst[i] = '\0';
}

BSP_STATUS SystemFault_Set(SYSTEM_FAULT_CODE code, const char *message){
    s_system_fault_info.code = code;
    SystemFault_CopyMessage(s_system_fault_info.message, message);
    return BSP_STATUS_OK;
}

BSP_STATUS SystemFault_Get(SYSTEM_FAULT_INFO *out){
    if (out == NULL){
        return BSP_STATUS_NULL;
    }

    *out = s_system_fault_info;
    return BSP_STATUS_OK;
}

SYSTEM_FAULT_CODE SystemFault_GetCode(void){
    return s_system_fault_info.code;
}

const char *SystemFault_GetMessage(void){
    return s_system_fault_info.message;
}

void SystemFault_Clear(void){
    s_system_fault_info.code = SYSTEM_FAULT_NONE;
    s_system_fault_info.message[0] = '\0';
}

void SystemFault_Handler(SYSTEM_FAULT_CODE code, const char *message){
    (void)SystemFault_Set(code, message);
    SystemFault_Halt();
}

void SystemFault_Halt(void){
    /* 故障停机优先让执行机构进入安全状态，再刷新错误页。 */
    (void)Chassis_Brake();
    (void)Gimbal_Stop();

    Ui_RenderStatusPage("System Fault",
                        UI_STATUS_ERROR,
                        s_system_fault_info.message,
                        SystemFault_CodeText(s_system_fault_info.code));

    while (1){
        ;
    }
}
