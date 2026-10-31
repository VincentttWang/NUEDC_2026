/**
 * @file app_track_tune.h
 * @brief H2/H5/H6 赛道切换参数的 RAM 调试配置。
 */
#ifndef APP_TRACK_TUNE_H
#define APP_TRACK_TUNE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_TRACK_TUNE_PARAM_COUNT 13U

#define APP_TRACK_TUNE_OP_GET            0x01U
#define APP_TRACK_TUNE_OP_SET            0x02U
#define APP_TRACK_TUNE_OP_GET_ALL        0x03U
#define APP_TRACK_TUNE_OP_RESET_DEFAULTS 0x04U

#define APP_TRACK_TUNE_STATUS_OK         0x00U
#define APP_TRACK_TUNE_STATUS_UNKNOWN_ID 0x01U
#define APP_TRACK_TUNE_STATUS_BUSY       0x02U
#define APP_TRACK_TUNE_STATUS_BAD_OP     0x03U

#define APP_TRACK_TUNE_ID_LIST_END 0xFFFFU

typedef struct {
    uint16_t s1_end_mm;
    uint16_t s2_end_mm;
    uint16_t s3_end_mm;
    uint16_t s4_heading_end_mm;
    uint16_t s3_gyro_recover_mm;
    uint16_t lap_stop_mm;
    uint16_t finish_arm_margin_mm;
    uint16_t loaded_decel_warning_mm;
    uint16_t loaded_odom_arrival_mm;
    uint16_t h2_odom_fallback_mm;
    uint16_t h2_s2_ff_x1000;
    uint16_t loaded_s2_ff_x1000;
    uint16_t s4_ff_x1000;
} APP_TRACK_TUNE_CONFIG;

/** 恢复编译期默认值；配置只保存在 RAM 中。 */
void AppTrackTune_Init(void);

/**
 * 处理 UART2 调参请求。SET/RESET 仅在 writes_allowed=true 时执行。
 * 推荐范围和参数间约束由树莓派 UI 负责，固件仅校验参数 ID。
 */
void AppTrackTune_Poll(bool writes_allowed);

/** 复制当前完整配置，供任务进入时冻结为本次运行快照。 */
void AppTrackTune_GetSnapshot(APP_TRACK_TUNE_CONFIG *config);

#ifdef __cplusplus
}
#endif

#endif /* APP_TRACK_TUNE_H */
