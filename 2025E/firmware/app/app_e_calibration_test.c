#include "app_e_calibration_test.h"

#include "app_e_calibration.h"
#include "bsp_time.h"
#include "canmv_uart.h"
#include "chassis.h"
#include "gimbal.h"
#include "hall_encoder.h"
#include "key.h"
#include "kinematics/kinematics.h"
#include "middleware/auto_aim/auto_aim.h"
#include "ui.h"
#include "wit_sdk.h"

#include <stdbool.h>
#include <stdio.h>

#define CAL_TEST_LOOP_DELAY_MS 20U
/* 前馈瞄准闭环用更高控制率 (~200Hz), yaw 更跟手。 */
#define CAL_TEST_AIM_LOOP_DELAY_MS 5U
/* OLED 刷新与控制解耦: 屏幕每 ~50ms 更新一次, 避免 200Hz 阻塞式 I2C 刷屏
 * 拖垮控制周期 (甚至把 I2C 逼入错误态反复超时, 表现为卡死)。 */
#define CAL_TEST_DISPLAY_INTERVAL_MS 50U
#define CAL_TEST_CIRCLE_PHASE_STEP_DEG 10.0f

typedef enum {
    CAL_TEST_ENCODER = 0,
    CAL_TEST_IMU,
    CAL_TEST_FEEDFORWARD,
    CAL_TEST_VISION_BIAS,
    CAL_TEST_CIRCLE,
    CAL_TEST_COUNT
} CAL_TEST_ITEM;

static const char *const s_cal_test_items[CAL_TEST_COUNT] = {
    "Encoder distance",
    "IMU heading",
    "Geo feedforward",
    "Vision bias",
    "F3 circle phase",
};

static bool CalTest_IsBack(void){
    return Key_IsLongPress(KEY_ID_CALIB);
}

static void CalTest_SetLaser(bool enabled){
    (void)CanMvUart_SendString(enabled ? "LASER=1\n" : "LASER=0\n");
}

static void CalTest_RenderMenu(CAL_TEST_ITEM selected){
    UI_LIST_PAGE page = {
        .title = "Calibration",
        .items = s_cal_test_items,
        .item_count = CAL_TEST_COUNT,
        .selected_index = (uint8_t)selected,
        .first_visible_index = 0U,
    };
    Ui_RenderListPage(&page);
}

void AppE_CalibrationTest_RunEncoder(void){
    char line0[24];
    char line1[24];
    char line2[24];

    Key_ClearAllEvents();
    CalTest_SetLaser(false);
    (void)Chassis_Coast();
    HallEncoder_Reset();
    Ui_RenderLines("Cal Encoder", "Move measured dist", "Dist:0.000m",
                   "Speed:0.000m/s", "Count:0", "K3:reset", "K2 long:back");

    while (!CalTest_IsBack()){
        snprintf(line0, sizeof(line0), "Dist:%0.3fm", HallEncoder_GetDistance());
        snprintf(line1, sizeof(line1), "Speed:%0.3fm/s", HallEncoder_GetSpeed());
        snprintf(line2, sizeof(line2), "Count:%ld", (long)HallEncoder_GetCount());
        Ui_UpdateContentLine(1U, line0);
        Ui_UpdateContentLine(2U, line1);
        Ui_UpdateContentLine(3U, line2);

        if (Key_IsShortPress(KEY_ID_ENTER)){
            HallEncoder_Reset();
            Key_ClearAllEvents();
        }
        BSP_DelayMs(CAL_TEST_LOOP_DELAY_MS);
    }

    (void)Chassis_Brake();
    Key_ClearAllEvents();
}

void AppE_CalibrationTest_RunImu(void){
    char line0[24];
    char line1[24];
    char line2[24];
    WIT_IMU_DATA imu = {0};
    float zero_yaw_deg = 0.0f;

    (void)WitGetData(&imu);
    zero_yaw_deg = imu.attitude_deg.yaw;
    Key_ClearAllEvents();
    CalTest_SetLaser(false);
    (void)Chassis_Brake();
    Ui_RenderLines("Cal IMU", "Turn known angle", "Yaw:0.00", "Delta:0.00",
                   "GyroZ:0.00", "K3:set zero", "K2 long:back");

    while (!CalTest_IsBack()){
        if (WitGetData(&imu) == WIT_HAL_OK){
            snprintf(line0, sizeof(line0), "Yaw:%0.2f", imu.attitude_deg.yaw);
            snprintf(line1, sizeof(line1), "Delta:%0.2f",
                     Kinematics_AngleDiffDeg(imu.attitude_deg.yaw, zero_yaw_deg));
            snprintf(line2, sizeof(line2), "GyroZ:%0.2f", imu.gyro_deg_s.z);
        } else{
            snprintf(line0, sizeof(line0), "Yaw:ERROR");
            snprintf(line1, sizeof(line1), "Delta:--");
            snprintf(line2, sizeof(line2), "GyroZ:--");
        }
        Ui_UpdateContentLine(1U, line0);
        Ui_UpdateContentLine(2U, line1);
        Ui_UpdateContentLine(3U, line2);

        if (Key_IsShortPress(KEY_ID_ENTER)){
            zero_yaw_deg = imu.attitude_deg.yaw;
            Key_ClearAllEvents();
        }
        BSP_DelayMs(CAL_TEST_LOOP_DELAY_MS);
    }
    Key_ClearAllEvents();
}

static void CalTest_RunAim(bool enable_vision_bias){
    char line0[24];
    char line1[24];
    char line2[24];
    APP_E_CALIBRATION_CONFIG calibration = AppE_GetCalibrationConfig();
    uint32_t last_ms = BSP_Time_GetMs();
    uint32_t last_display_ms = last_ms;
    bool laser_on = false;

    if (!enable_vision_bias){
        /* 纯几何模式: 两轴增益全置 0, 视觉帧到达也不吸收。 */
        calibration.auto_aim.vision_yaw.gain0 = 0.0f;
        calibration.auto_aim.vision_yaw.gain_min = 0.0f;
        calibration.auto_aim.vision_pitch.gain0 = 0.0f;
        calibration.auto_aim.vision_pitch.gain_min = 0.0f;
    }

    Key_ClearAllEvents();
    CalTest_SetLaser(false);
    (void)Chassis_Brake();
    /*
     * 不在此阻塞等待 pitch 归位: 本测试聚焦 yaw 稳定, 与 pitch 无关。
     * Gimbal_SetAngle 在 pitch 未就绪时会自动跳过 pitch、照常驱动 yaw;
     * 开机 Gimbal_StartupElevatePitch() 已让 F32C 自主抬升 pitch。
     * 之前这里调 Gimbal_EnsurePitchReady() 会阻塞轮询 pitch 反馈 (云台断电时
     * 吃满 2s、且超时依赖 SysTick), 表现为进入本页时"卡死"。
     */
    AutoAim_Init(&calibration.auto_aim);
    if (AutoAim_Start(AUTO_AIM_MODE_CENTER) != BSP_STATUS_OK){
        Ui_RenderStatusPage("Calibration", UI_STATUS_ERROR, "IMU not ready", "K2 long:back");
        while (!CalTest_IsBack()){
            BSP_DelayMs(CAL_TEST_LOOP_DELAY_MS);
        }
        Key_ClearAllEvents();
        return;
    }

    Ui_RenderLines(enable_vision_bias ? "Cal Vision" : "Cal Geometry",
                   enable_vision_bias ? "Bias enabled" : "Bias disabled",
                   "Y:0.0 H:0.0", "Gz:0.0 Rff:0.0", "Bias:0.0,0.0",
                   "K3:laser OFF", "K2 long:back");

    while (!CalTest_IsBack()){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;
        last_ms = now_ms;
        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        (void)AutoAim_Update(dt_s);

        /* OLED 刷新与控制解耦: 每 ~50ms 刷一次屏, 控制仍按 5ms 跑。 */
        if ((uint32_t)(now_ms - last_display_ms) >= CAL_TEST_DISPLAY_INTERVAL_MS){
            last_display_ms = now_ms;
            AUTO_AIM_STATE state = AutoAim_GetState();
            /* yaw 稳定诊断: 指令yaw / 世界航向 / 陀螺yaw-rate / 速率前馈量。 */
            snprintf(line0, sizeof(line0), "Y:%0.1f H:%0.1f",
                     state.yaw_command_deg, state.pose.heading_deg);
            snprintf(line1, sizeof(line1), "Gz:%0.1f Rff:%0.1f",
                     state.gyro_z_deg_s, state.yaw_rate_ff_deg_s);
            snprintf(line2, sizeof(line2), "Bias:%0.1f,%0.1f",
                     state.vision_bias.yaw_bias_deg, state.vision_bias.pitch_bias_deg);
            Ui_UpdateContentLine(1U, line0);
            Ui_UpdateContentLine(2U, line1);
            Ui_UpdateContentLine(3U, line2);
        }

        if (Key_IsShortPress(KEY_ID_ENTER)){
            laser_on = !laser_on;
            CalTest_SetLaser(laser_on);
            Ui_UpdateContentLine(4U, laser_on ? "K3:laser ON" : "K3:laser OFF");
            Key_ClearAllEvents();
        }
        BSP_DelayMs(CAL_TEST_AIM_LOOP_DELAY_MS);
    }

    CalTest_SetLaser(false);
    (void)AutoAim_Stop();
    Key_ClearAllEvents();
}

void AppE_CalibrationTest_RunFeedforward(void){
    CalTest_RunAim(false);
}

void AppE_CalibrationTest_RunVisionBias(void){
    CalTest_RunAim(true);
}

void AppE_CalibrationTest_RunCircle(void){
    char line0[24];
    char line1[24];
    APP_E_CALIBRATION_CONFIG calibration = AppE_GetCalibrationConfig();
    float phase_deg = calibration.auto_aim.circle_phase0_deg;
    bool laser_on = false;

    Key_ClearAllEvents();
    CalTest_SetLaser(false);
    (void)Chassis_Brake();
    Gimbal_EnsurePitchReady();
    AimSolver_Init(&calibration.auto_aim.solver);
    Ui_RenderLines("Cal F3 Circle", "K1/K4:phase", "Phase:0.0",
                   "Cmd:0.0,0.0", "K3:laser OFF", "Step:10deg", "K2 long:back");

    while (!CalTest_IsBack()){
        if (Key_IsShortPress(KEY_ID_UP)){
            phase_deg = Kinematics_NormalizeAngleDeg(phase_deg + CAL_TEST_CIRCLE_PHASE_STEP_DEG);
            Key_ClearAllEvents();
        } else if (Key_IsShortPress(KEY_ID_DOWN)){
            phase_deg = Kinematics_NormalizeAngleDeg(phase_deg - CAL_TEST_CIRCLE_PHASE_STEP_DEG);
            Key_ClearAllEvents();
        }

        AIM_SOLVER_RESULT result = AimSolver_SolveCircle(calibration.auto_aim.start_pose,
                                                         phase_deg);
        (void)Gimbal_SetAngle(result.yaw_cmd_deg, result.pitch_cmd_deg, 0.0f);
        snprintf(line0, sizeof(line0), "Phase:%0.1f", phase_deg);
        snprintf(line1, sizeof(line1), "Cmd:%0.1f,%0.1f",
                 result.yaw_cmd_deg, result.pitch_cmd_deg);
        Ui_UpdateContentLine(1U, line0);
        Ui_UpdateContentLine(2U, line1);

        if (Key_IsShortPress(KEY_ID_ENTER)){
            laser_on = !laser_on;
            CalTest_SetLaser(laser_on);
            Ui_UpdateContentLine(3U, laser_on ? "K3:laser ON" : "K3:laser OFF");
            Key_ClearAllEvents();
        }
        BSP_DelayMs(CAL_TEST_LOOP_DELAY_MS);
    }

    CalTest_SetLaser(false);
    (void)Gimbal_Stop();
    Key_ClearAllEvents();
}

void AppE_CalibrationTest_RunMenu(void){
    CAL_TEST_ITEM selected = CAL_TEST_ENCODER;

    Key_ClearAllEvents();
    CalTest_RenderMenu(selected);
    while (!CalTest_IsBack()){
        if (Key_IsShortPress(KEY_ID_DOWN)){
            selected = (CAL_TEST_ITEM)(((uint8_t)selected + 1U) % (uint8_t)CAL_TEST_COUNT);
            Key_ClearAllEvents();
            CalTest_RenderMenu(selected);
        } else if (Key_IsShortPress(KEY_ID_UP)){
            selected = (CAL_TEST_ITEM)(((uint8_t)selected + (uint8_t)CAL_TEST_COUNT - 1U) %
                                       (uint8_t)CAL_TEST_COUNT);
            Key_ClearAllEvents();
            CalTest_RenderMenu(selected);
        } else if (Key_IsShortPress(KEY_ID_ENTER)){
            Key_ClearAllEvents();
            switch (selected){
            case CAL_TEST_ENCODER:
                AppE_CalibrationTest_RunEncoder();
                break;
            case CAL_TEST_IMU:
                AppE_CalibrationTest_RunImu();
                break;
            case CAL_TEST_FEEDFORWARD:
                AppE_CalibrationTest_RunFeedforward();
                break;
            case CAL_TEST_VISION_BIAS:
                AppE_CalibrationTest_RunVisionBias();
                break;
            case CAL_TEST_CIRCLE:
                AppE_CalibrationTest_RunCircle();
                break;
            default:
                break;
            }
            CalTest_RenderMenu(selected);
        }
        BSP_DelayMs(CAL_TEST_LOOP_DELAY_MS);
    }
    CalTest_SetLaser(false);
    Key_ClearAllEvents();
}
