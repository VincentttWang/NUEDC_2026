/**
 * @file app_track_tune.c
 * @brief UART2 赛道调参命令处理与 13 项 RAM 配置表。
 */
#include "app_track_tune.h"

#include "rpi_uart.h"

#include <stddef.h>

typedef struct {
    uint16_t id;
    uint16_t offset;
    uint16_t default_value;
    uint8_t priority;
} TRACK_TUNE_ITEM;

#define TRACK_TUNE_ITEM_DEF(param_id, member, value, level) \
    { (param_id), (uint16_t)offsetof(APP_TRACK_TUNE_CONFIG, member), (value), (level) }

static const TRACK_TUNE_ITEM tune_items[APP_TRACK_TUNE_PARAM_COUNT] = {
    TRACK_TUNE_ITEM_DEF(0x0101U, s1_end_mm,                 1500U, 1U),
    TRACK_TUNE_ITEM_DEF(0x0102U, s2_end_mm,                 3071U, 1U),
    TRACK_TUNE_ITEM_DEF(0x0103U, s3_end_mm,                 4641U, 1U),
    TRACK_TUNE_ITEM_DEF(0x0104U, s4_heading_end_mm,         6212U, 1U),
    TRACK_TUNE_ITEM_DEF(0x0105U, s3_gyro_recover_mm,         300U, 1U),
    TRACK_TUNE_ITEM_DEF(0x0110U, lap_stop_mm,               6242U, 2U),
    TRACK_TUNE_ITEM_DEF(0x0111U, finish_arm_margin_mm,       400U, 2U),
    TRACK_TUNE_ITEM_DEF(0x0112U, loaded_decel_warning_mm,    250U, 2U),
    TRACK_TUNE_ITEM_DEF(0x0113U, loaded_odom_arrival_mm,    6342U, 2U),
    TRACK_TUNE_ITEM_DEF(0x0114U, h2_odom_fallback_mm,       6442U, 2U),
    TRACK_TUNE_ITEM_DEF(0x0120U, h2_s2_ff_x1000,           1100U, 3U),
    TRACK_TUNE_ITEM_DEF(0x0121U, loaded_s2_ff_x1000,       1000U, 3U),
    TRACK_TUNE_ITEM_DEF(0x0122U, s4_ff_x1000,              1000U, 3U),
};

static APP_TRACK_TUNE_CONFIG active_config;
static bool list_active;
static uint8_t list_seq;
static uint8_t list_index;

static uint16_t *TrackTune_ValuePtr(const TRACK_TUNE_ITEM *item){
    return (uint16_t *)((uint8_t *)&active_config + item->offset);
}

static const TRACK_TUNE_ITEM *TrackTune_Find(uint16_t id){
    for (uint8_t i = 0U; i < APP_TRACK_TUNE_PARAM_COUNT; i++){
        if (tune_items[i].id == id){
            return &tune_items[i];
        }
    }
    return NULL;
}

static void TrackTune_ResetDefaults(void){
    for (uint8_t i = 0U; i < APP_TRACK_TUNE_PARAM_COUNT; i++){
        *TrackTune_ValuePtr(&tune_items[i]) = tune_items[i].default_value;
    }
}

void AppTrackTune_Init(void){
    TrackTune_ResetDefaults();
    list_active = false;
    list_seq = 0U;
    list_index = 0U;
}

void AppTrackTune_GetSnapshot(APP_TRACK_TUNE_CONFIG *config){
    if (config != NULL){
        *config = active_config;
    }
}

static void TrackTune_SendList(void){
    while (list_active){
        if (list_index < APP_TRACK_TUNE_PARAM_COUNT){
            const TRACK_TUNE_ITEM *item = &tune_items[list_index];
            if (!RpiUart_SendConfigResponse(
                    list_seq, APP_TRACK_TUNE_STATUS_OK, item->id,
                    *TrackTune_ValuePtr(item), item->priority)){
                return;
            }
            list_index++;
        } else{
            if (!RpiUart_SendConfigResponse(
                    list_seq, APP_TRACK_TUNE_STATUS_OK,
                    APP_TRACK_TUNE_ID_LIST_END, APP_TRACK_TUNE_PARAM_COUNT, 0U)){
                return;
            }
            list_active = false;
        }
    }
}

void AppTrackTune_Poll(bool writes_allowed){
    RPI_UART_CONFIG_REQUEST_FRAME request;

    TrackTune_SendList();
    if (list_active){
        return;
    }

    while (RpiUart_GetConfigRequest(&request)){
        const TRACK_TUNE_ITEM *item = TrackTune_Find(request.param_id);
        if (request.op == APP_TRACK_TUNE_OP_GET){
            if (item == NULL){
                (void)RpiUart_SendConfigResponse(
                    request.seq, APP_TRACK_TUNE_STATUS_UNKNOWN_ID,
                    request.param_id, 0U, 0U);
            } else{
                (void)RpiUart_SendConfigResponse(
                    request.seq, APP_TRACK_TUNE_STATUS_OK, item->id,
                    *TrackTune_ValuePtr(item), item->priority);
            }
        } else if (request.op == APP_TRACK_TUNE_OP_SET){
            if (!writes_allowed){
                (void)RpiUart_SendConfigResponse(
                    request.seq, APP_TRACK_TUNE_STATUS_BUSY,
                    request.param_id, (item != NULL) ? *TrackTune_ValuePtr(item) : 0U,
                    (item != NULL) ? item->priority : 0U);
            } else if (item == NULL){
                (void)RpiUart_SendConfigResponse(
                    request.seq, APP_TRACK_TUNE_STATUS_UNKNOWN_ID,
                    request.param_id, 0U, 0U);
            } else{
                *TrackTune_ValuePtr(item) = request.value;
                (void)RpiUart_SendConfigResponse(
                    request.seq, APP_TRACK_TUNE_STATUS_OK, item->id,
                    request.value, item->priority);
            }
        } else if (request.op == APP_TRACK_TUNE_OP_GET_ALL){
            list_active = true;
            list_seq = request.seq;
            list_index = 0U;
            TrackTune_SendList();
            return;
        } else if (request.op == APP_TRACK_TUNE_OP_RESET_DEFAULTS){
            if (!writes_allowed){
                (void)RpiUart_SendConfigResponse(
                    request.seq, APP_TRACK_TUNE_STATUS_BUSY,
                    APP_TRACK_TUNE_ID_LIST_END, APP_TRACK_TUNE_PARAM_COUNT, 0U);
            } else{
                TrackTune_ResetDefaults();
                (void)RpiUart_SendConfigResponse(
                    request.seq, APP_TRACK_TUNE_STATUS_OK,
                    APP_TRACK_TUNE_ID_LIST_END, APP_TRACK_TUNE_PARAM_COUNT, 0U);
            }
        } else{
            (void)RpiUart_SendConfigResponse(
                request.seq, APP_TRACK_TUNE_STATUS_BAD_OP,
                request.param_id, 0U, 0U);
        }
    }
}
