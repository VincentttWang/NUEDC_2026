/**
 * @file  bsp_common.h
 * @brief BSP 层公共状态类型定义。
 */
#ifndef BSP_COMMON_H
#define BSP_COMMON_H

#include <stdint.h>

/**
 * @brief BSP 层通用返回状态。
 */
typedef enum {
    /** 操作成功。 */
    BSP_STATUS_OK = 0,
    /** 通用错误。 */
    BSP_STATUS_ERROR = -1,
    /** 空指针或空资源。 */
    BSP_STATUS_NULL = -2,
    /** 参数非法。 */
    BSP_STATUS_INVALID_ARG = -3,
    /** 操作超时。 */
    BSP_STATUS_TIMEOUT = -4,
    /** 资源忙。 */
    BSP_STATUS_BUSY = -5,
    /** 资源或外设尚未准备好。 */
    BSP_STATUS_NOT_READY = -6,
} BSP_STATUS;

#endif /* BSP_COMMON_H */
