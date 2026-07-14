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

    /* 2. 单轮编码器：编码轮在车体左侧为正、右侧为负。
     *    实测：编码轮在右轮、轮距 14cm → -轮距/2 = -0.070m。
     *    转弯时用陀螺把单轮里程重构到车心, 消除弯中位置漂移(近弯 B 放大明显)。 */
    aim.encoder_lateral_offset_m = -0.070f;
    /* 陀螺 yaw-rate 符号：实测本机 gz 与航向反号 → -1（左转时 head 增而 gz 负）。
     * 影响速率前馈与车心重构，符号错会让转弯前馈反向、越快越滞后。 */
    aim.gyro_z_sign = -1.000f;

    /* 3. 靶心世界水平坐标与云台转轴相对车体定位原点的安装位置。 */
    aim.solver.target_center = (AIM_POINT2F){
        .x_m = 0.000f,
        .y_m = 0.500f,
    };
    /* 竖直高差 = 靶心离地高 - 云台出光点离地高 (机械上尽量趋 0)。
     * 名义: 靶心 0.25m - 出光点 0.15m = 0.10m。 */
    aim.solver.height_diff_m = 0.100f;
    aim.solver.mount_x_m = 0.000f; /* 车体前方为正。 */
    aim.solver.mount_y_m = 0.000f; /* 车体左侧为正。 */

    /* 4. 云台机械零位：增大 yaw offset 会减小 yaw 指令；pitch 反之。
     *    pitch_zero_offset = "激光水平(0 仰角)时的 pitch 电机角"。
     *    solver 输出 pitch_cmd = atan2(高差,水平距离) + 该 offset; 电机限位 [135,165](home150±15)。
     *    绕场几何仰角 ~3.6°(远角)~11.3°(近AB), offset=143 → 指令落 ~147~154° 居中不钳位。
     *    标定: 让车静瞄靶, 增/减该值直到激光竖直方向正中靶心 (增大→pitch 指令增大;
     *    激光是随之上抬还是下压取决于机械装配, 观察后决定加减方向)。 */
    aim.solver.yaw_zero_offset_deg = 0.000f;
    aim.solver.pitch_zero_offset_deg = 143.000f;

    /* 5. 视觉慢校正：若某轴误差越校越大，只翻转对应 sign。 */
    aim.vision_bias_ki = 0.100f;       /* 从小增益开始实机调试。 */
    aim.vision_bias_limit_deg = 3.000f;
    aim.vision_yaw_sign = -1.000f;
    aim.vision_pitch_sign = 1.000f;

    /* 6. F3：0°=靶心右侧，90°=上方。 */
    aim.solver.circle_radius_m = 0.060f;
    aim.circle_phase0_deg = 0.000f;

    /* 7. 速率前馈 lead 时间(s)：补偿转弯时位置伺服速度滞后 + IMU 融合航向滞后。
     *    yaw 用低延迟陀螺 −ω 项提前 τ 秒量, 只在转弯(ω 大)起作用、不影响直线。
     *    实机: 位置环 Kv≈4.3/s → 转弯 ~130°/s 滞后 ~30°; lead≈1/Kv 可补。
     *    起调 0.12(补~15°)；转弯仍滞后→加大(先抬 F32C 位置环KP, 再抬 lead 到 0.15~0.18)，
     *    出现转弯末尾过冲/激光甩过头 或 直线被蛇形放大→减小。 */
    aim.yaw_rate_lead_s = 0.120f;
    aim.pitch_rate_lead_s = 0.000f;

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
