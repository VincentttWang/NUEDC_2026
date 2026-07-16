#include "middleware/auto_aim/auto_aim.h"

#include "canmv_uart.h"
#include "wit_sdk.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static float s_distance_m;
static WIT_IMU_DATA s_imu;
static CANMV_TARGET_DATA s_angle_data;
static float s_command_yaw;
static float s_command_pitch;
static int s_command_count;

volatile uint32_t g_canmv_uart_angle_frame_count;

float Chassis_GetDistance(void){
    return s_distance_m;
}

int WitGetData(WIT_IMU_DATA *out){
    *out = s_imu;
    return WIT_HAL_OK;
}

const CANMV_TARGET_DATA *CanMvUart_GetTargetData(CANMV_TARGET target){
    return (target == CANMV_TARGET_ANGLE) ? &s_angle_data : NULL;
}

BSP_STATUS Gimbal_SetAngle(float yaw_deg, float pitch_deg, float yaw_rate_ff_deg_s){
    (void)yaw_rate_ff_deg_s;
    s_command_yaw = yaw_deg;
    s_command_pitch = pitch_deg;
    s_command_count++;
    return BSP_STATUS_OK;
}

BSP_STATUS Gimbal_Stop(void){
    return BSP_STATUS_OK;
}

static int Near(float actual, float expected, float tolerance){
    return fabsf(actual - expected) <= tolerance;
}

int main(void){
    AUTO_AIM_CONFIG config = AutoAim_DefaultConfig();
    config.start_pose = (KINEMATICS_POSE){0.0f, 0.0f, 0.0f};
    config.encoder_lateral_offset_m = 0.1f;
    config.gyro_z_sign = 1.0f;   /* 本测试直接用正 gz, 不叠符号 */
    AutoAim_Init(&config);

    s_imu.attitude_deg.yaw = 20.0f;
    assert(AutoAim_Start(AUTO_AIM_MODE_CENTER) == BSP_STATUS_OK);

    s_distance_m = 0.1f;
    s_imu.gyro_deg_s.z = 90.0f;
    s_angle_data.status = CANMV_STATUS_OK;
    s_angle_data.count = 2U;
    s_angle_data.value[0] = (uint16_t)(int16_t)100;
    s_angle_data.value[1] = (uint16_t)(int16_t)-50;
    g_canmv_uart_angle_frame_count = 1U;
    assert(AutoAim_Update(0.1f) == BSP_STATUS_OK);

    AUTO_AIM_STATE state = AutoAim_GetState();
    assert(state.imu_valid);
    assert(state.vision_valid);
    assert(Near(state.pose.heading_deg, 0.0f, 0.001f));
    assert(state.last_center_delta_m > state.last_wheel_delta_m);
    /* 首帧逐轴增益: yaw g0=0.5 -> 0.5*(-1.0); pitch g0=0.25 -> 0.25*(-0.5) */
    assert(Near(state.vision_bias.yaw_bias_deg, -0.5f, 0.001f));
    assert(Near(state.vision_bias.pitch_bias_deg, -0.125f, 0.001f));
    assert(s_command_count == 1);
    assert(Near(s_command_yaw, state.yaw_command_deg, 0.001f));
    assert(Near(s_command_pitch, state.pitch_command_deg, 0.001f));

    /* 起步对齐窗口: 视觉帧喂大量程 startup_bias, 慢 bias 冻结不动。 */
    AutoAim_SetStartupAlign(true);
    s_angle_data.value[0] = (uint16_t)(int16_t)200;   /* +2.00° */
    s_angle_data.value[1] = (uint16_t)(int16_t)100;   /* +1.00° */
    g_canmv_uart_angle_frame_count = 2U;
    assert(AutoAim_Update(0.1f) == BSP_STATUS_OK);
    state = AutoAim_GetState();
    /* startup 首帧逐轴: yaw 0.5*(-2.0)=-1.0, pitch 0.25*(+1.0)=+0.25 */
    assert(Near(state.startup_bias.yaw_bias_deg, -1.0f, 0.001f));
    assert(Near(state.startup_bias.pitch_bias_deg, 0.25f, 0.001f));
    assert(Near(state.vision_bias.yaw_bias_deg, -0.5f, 0.001f));   /* 慢 bias 未动 */
    AutoAim_SetStartupAlign(false);
    /* 对齐完成: 慢 bias 跳到稳态小增益段 (g0/gmin=25 -> frame_count>=25)。 */
    state = AutoAim_GetState();
    assert(state.vision_bias.frame_count >= 25U);
    assert(!state.startup_align);

    /* 几何预定位阶段: 视觉仍可见，但禁止写入两套 bias。 */
    float frozen_yaw_bias = state.vision_bias.yaw_bias_deg;
    float frozen_pitch_bias = state.vision_bias.pitch_bias_deg;
    AutoAim_SetVisionCorrectionEnabled(false);
    s_angle_data.value[0] = (uint16_t)(int16_t)-500;
    s_angle_data.value[1] = (uint16_t)(int16_t)300;
    g_canmv_uart_angle_frame_count = 3U;
    assert(AutoAim_Update(0.1f) == BSP_STATUS_OK);
    state = AutoAim_GetState();
    assert(state.vision_valid);
    assert(!state.vision_correction_enabled);
    assert(Near(state.vision_bias.yaw_bias_deg, frozen_yaw_bias, 0.001f));
    assert(Near(state.vision_bias.pitch_bias_deg, frozen_pitch_bias, 0.001f));

    /* 预定位结束可原地清零校正器，不破坏任务/定位状态。 */
    AutoAim_ResetVisionCorrection();
    AutoAim_SetVisionCorrectionEnabled(true);
    state = AutoAim_GetState();
    assert(state.active);
    assert(state.vision_correction_enabled);
    assert(Near(state.startup_bias.yaw_bias_deg, 0.0f, 0.001f));
    assert(Near(state.startup_bias.pitch_bias_deg, 0.0f, 0.001f));
    assert(Near(state.vision_bias.yaw_bias_deg, 0.0f, 0.001f));
    assert(Near(state.vision_bias.pitch_bias_deg, 0.0f, 0.001f));

    /* 纯视觉锁定后的控制器交接可直接写入已验证的几何基准偏置。 */
    AutoAim_SetStartupAlign(true);
    AutoAim_SetStartupBias(12.5f, -3.25f);
    AutoAim_SetStartupAlign(false);
    state = AutoAim_GetState();
    assert(Near(state.startup_bias.yaw_bias_deg, 12.5f, 0.001f));
    assert(Near(state.startup_bias.pitch_bias_deg, -3.25f, 0.001f));
    assert(state.startup_bias.frame_count == 1U);
    assert(state.vision_bias.frame_count >= 25U);

    AutoAim_AnchorCorner(LOCALIZATION_CORNER_B, 1.0f);
    state = AutoAim_GetState();
    assert(Near(state.pose.x_m, 0.5f, 0.001f));
    assert(Near(state.lap_progress, 0.25f, 0.001f));
    assert(AutoAim_Stop() == BSP_STATUS_OK);
    puts("auto aim tests passed");
    return 0;
}
