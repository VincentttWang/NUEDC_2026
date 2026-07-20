/**
 * @file  app_debug_cmd.c
 * @brief debug 串口上行命令处理实现 (实时 PID 调参)。
 */
#include "app_debug_cmd.h"

#include "bsp/debug_uart/debug_uart.h"
#include "gimbal_tracking.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define APP_DEBUG_CMD_LINE_MAX 48U

/* 组行缓冲: ISR 单写, 线程满行时读取。 */
static volatile char s_line[APP_DEBUG_CMD_LINE_MAX];
static volatile uint8_t s_len;
static volatile bool s_ready;

void AppDebugCmd_FeedByte(uint8_t byte){
    if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n')){
        if ((s_len > 0U) && !s_ready){
            s_line[s_len] = '\0';
            s_ready = true;      /* 交给线程处理; 处理完才收下一行 */
        }
        s_len = 0U;
    } else if (s_len < (APP_DEBUG_CMD_LINE_MAX - 1U)){
        s_line[s_len++] = (char)byte;
    }
}

void AppDebugCmd_EmitConfig(void){
    GIMBAL_TRACKING_CONFIG c = GimbalTracking_GetConfig();
    DebugUart_Printf(
        "[CFG] ykp=%.3f yki=%.3f ykd=%.3f pkp=%.3f pki=%.3f pkd=%.3f olim=%.1f\r\n",
        (double)c.yaw_pid.kp, (double)c.yaw_pid.ki, (double)c.yaw_pid.kd,
        (double)c.pitch_pid.kp, (double)c.pitch_pid.ki, (double)c.pitch_pid.kd,
        (double)c.yaw_pid.output_limit);
}

void AppDebugCmd_Poll(void){
    char line[APP_DEBUG_CMD_LINE_MAX];
    uint8_t i;

    if (!s_ready){
        return;
    }
    for (i = 0U; i < APP_DEBUG_CMD_LINE_MAX; i++){
        line[i] = s_line[i];
        if (line[i] == '\0'){
            break;
        }
    }
    line[APP_DEBUG_CMD_LINE_MAX - 1U] = '\0';
    s_ready = false;             /* 允许接收下一行 */

    if ((line[0] == 's') && (line[1] == ' ')){
        char *key = &line[2];
        char *sp = strchr(key, ' ');
        if (sp != NULL){
            *sp = '\0';
            GimbalTracking_SetGain(key, strtof(sp + 1, NULL));
            AppDebugCmd_EmitConfig();
        }
    } else if ((line[0] == 'g') || (line[0] == '?')){
        AppDebugCmd_EmitConfig();
    }
}
