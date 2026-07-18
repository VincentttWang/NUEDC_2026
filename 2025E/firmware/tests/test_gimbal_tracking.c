#include "middleware/gimbal_tracking/gimbal_tracking.h"
#include "gimbal.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

volatile uint32_t g_canmv_uart_angle_frame_count;

static CANMV_TARGET_DATA s_angle;
static uint32_t s_now_ms;
static float s_yaw_speed;
static float s_pitch_speed;
static unsigned s_hold_count;

uint32_t BSP_Time_GetMs(void){
    return s_now_ms;
}

const CANMV_TARGET_DATA *CanMvUart_GetTargetData(CANMV_TARGET target){
    return (target == CANMV_TARGET_ANGLE) ? &s_angle : NULL;
}

CANMV_STATUS CanMvUart_GetStatus(CANMV_TARGET target){
    return (target == CANMV_TARGET_ANGLE) ? s_angle.status : CANMV_STATUS_INIT;
}

uint8_t CanMvUart_GetData(CANMV_TARGET target, uint16_t *out, uint8_t max_count){
    (void)target;
    (void)out;
    (void)max_count;
    return 0U;
}

BSP_STATUS Gimbal_SetAngle(float yaw_deg, float pitch_deg, float yaw_rate_ff_deg_s){
    (void)yaw_deg;
    (void)pitch_deg;
    (void)yaw_rate_ff_deg_s;
    return BSP_STATUS_OK;
}

BSP_STATUS Gimbal_SetSpeed(float yaw_deg_s, float pitch_deg_s){
    s_yaw_speed = yaw_deg_s;
    s_pitch_speed = pitch_deg_s;
    return BSP_STATUS_OK;
}

BSP_STATUS Gimbal_HoldOnTargetLost(void){
    s_hold_count++;
    s_yaw_speed = 0.0f;
    return BSP_STATUS_OK;
}

GIMBAL_SPEED Gimbal_GetSpeed(void){
    GIMBAL_SPEED speed = {s_yaw_speed, s_pitch_speed};
    return speed;
}

BSP_STATUS Gimbal_Stop(void){
    s_yaw_speed = 0.0f;
    s_pitch_speed = 0.0f;
    return BSP_STATUS_OK;
}

static int Near(float actual, float expected){
    return fabsf(actual - expected) <= 0.001f;
}

int main(void){
    GimbalTracking_Init(NULL);

    /* P+I 首拍(积分很小)+ 两轴独立方向: yaw=-(6*err + 2*err*dt), pitch=+(2*err + 0.8*err*dt)。 */
    GimbalTracking_Reset();
    assert(GimbalTracking_TrackAngleErrors(1.0f, -1.0f, 0.02f) == BSP_STATUS_OK);
    assert(Near(s_yaw_speed, -(6.0f * 1.0f + 2.0f * (1.0f * 0.02f))));
    assert(Near(s_pitch_speed, (2.0f * -1.0f + 0.8f * (-1.0f * 0.02f))));

    /* 无死区: 小误差(旧 0.75° 死区内)也持续输出、指向减小误差方向, 由 I 收敛到中心。 */
    GimbalTracking_Reset();
    assert(GimbalTracking_TrackAngleErrors(0.5f, -0.5f, 0.02f) == BSP_STATUS_OK);
    assert(s_yaw_speed < 0.0f);
    assert(s_pitch_speed < 0.0f);
    assert(Near(s_yaw_speed, -(6.0f * 0.5f + 2.0f * (0.5f * 0.02f))));
    assert(Near(s_pitch_speed, (2.0f * -0.5f + 0.8f * (-0.5f * 0.02f))));

    /* I 项在持续同号误差下累积, 第二拍 |输出| 应大于第一拍(验证积分生效)。 */
    GimbalTracking_Reset();
    GimbalTracking_TrackAngleErrors(1.0f, 0.0f, 0.02f);
    float first_yaw = s_yaw_speed;
    GimbalTracking_TrackAngleErrors(1.0f, 0.0f, 0.02f);
    assert(fabsf(s_yaw_speed) > fabsf(first_yaw));

    /* 大误差受各轴输出限幅约束(积分限幅后仍饱和)。 */
    GimbalTracking_Reset();
    assert(GimbalTracking_TrackAngleErrors(100.0f, 100.0f, 0.02f) == BSP_STATUS_OK);
    assert(Near(s_yaw_speed, -120.0f));
    assert(Near(s_pitch_speed, 60.0f));

    /* 专用 UpdateAngle 不得回退旧像素协议；无角度帧时进入安全保持。 */
    GimbalTracking_Reset();
    s_now_ms = 10U;
    assert(GimbalTracking_UpdateAngle(0.02f) == BSP_STATUS_NOT_READY);
    assert(s_hold_count > 0U);

    puts("gimbal tracking tests passed");
    return 0;
}
