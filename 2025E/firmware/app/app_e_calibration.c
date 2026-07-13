#include "app_e_calibration.h"

APP_E_CALIBRATION_CONFIG AppE_GetCalibrationConfig(void){
    AUTO_AIM_CONFIG aim = AutoAim_DefaultConfig();

    /* 1. 场地与规定起点：世界系原点=AB 中点，+X=A->B，+Y 指向靶。 */
    aim.start_pose = (KINEMATICS_POSE){
        .x_m = -0.500f,
        .y_m = 0.000f,
        .heading_deg = -90.0f, /* 默认从 A 沿 A->C。 */
    };
    aim.track_side_m = 1.000f; /* 填黑线中心线之间的实测平均边长。 */

    /* 2. 单轮编码器：编码轮在车体左侧为正、右侧为负。 */
    aim.encoder_lateral_offset_m = 0.000f;

    /* 3. 靶心世界坐标与云台转轴相对车体定位原点的安装位置。 */
    aim.solver.target_center = (AIM_VEC3F){
        .x_m = 0.000f,
        .y_m = 0.500f,
        .z_m = 0.250f,
    };
    aim.solver.mount_x_m = 0.000f; /* 车体前方为正。 */
    aim.solver.mount_y_m = 0.000f; /* 车体左侧为正。 */
    aim.solver.mount_z_m = 0.150f;

    /* 4. 云台机械零位：增大 yaw offset 会减小 yaw 指令；pitch 反之。 */
    aim.solver.yaw_zero_offset_deg = 0.000f;
    aim.solver.pitch_zero_offset_deg = 0.000f;

    /* 5. 视觉慢校正：若某轴误差越校越大，只翻转对应 sign。 */
    aim.vision_bias_ki = 0.100f;       /* 从小增益开始实机调试。 */
    aim.vision_bias_limit_deg = 3.000f;
    aim.vision_yaw_sign = -1.000f;
    aim.vision_pitch_sign = 1.000f;

    /* 6. F3：0°=靶心右侧，90°=上方。 */
    aim.solver.circle_radius_m = 0.060f;
    aim.circle_phase0_deg = 0.000f;

    APP_E_CALIBRATION_CONFIG config = {
        .auto_aim = aim,
        .aim_lock_yaw_deg = 0.800f,
        .aim_lock_pitch_deg = 0.800f,
        .aim_lock_confirm_frames = 8U,
        .e2_force_shot_ms = 1800U,
        .e3_force_shot_ms = 3800U,
        .laser_pulse_ms = 20U,
        .imu_fail_limit = 10U,
    };
    return config;
}
