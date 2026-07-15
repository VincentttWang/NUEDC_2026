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
    /* 竖直高差 = 靶心离地高 - 云台出光点离地高。
     * 实测: 靶心 0.25m - 出光点 0.22m = 0.03m。 */
    aim.solver.height_diff_m = 0.030f;
    aim.solver.mount_x_m = 0.000f; /* 车体前方为正。 */
    aim.solver.mount_y_m = 0.000f; /* 车体左侧为正。 */

    /* 4. 云台机械零位：增大 yaw offset 会减小 yaw 指令；pitch 反之。
     *    pitch_zero_offset = "激光水平(0 仰角)时的 pitch 电机角"。
     *    solver 输出 pitch_cmd = atan2(高差,水平距离) + 该 offset; 电机限位 [135,165](home150±15)。
     *    实测标定 = 140 (光束正中靶心); pitch 轴有减速比, 电机每度对应光束多度,
     *    故只需比 home 低 ~10° 即可下压到位。命令落 ~141~143° 仍在 [135,165] 内。
     *    重标: 让车静瞄靶, 增/减该值直到激光竖直方向正中靶心 (增大→pitch 指令增大→光束上抬)。 */
    aim.solver.yaw_zero_offset_deg = 0.000f;
    aim.solver.pitch_zero_offset_deg = 140.000f;

    /* 5. 视觉慢校正：若某轴误差越校越大，只翻转对应 sign。 */
    aim.vision_bias_ki = 0.100f;       /* 从小增益开始实机调试。 */
    aim.vision_bias_limit_deg = 3.000f;
    aim.vision_yaw_sign = -1.000f;
    aim.vision_pitch_sign = 1.000f;

    /* 6. F3：0°=靶心右侧，90°=上方。 */
    aim.solver.circle_radius_m = 0.060f;
    aim.circle_phase0_deg = 0.000f;

    /* 7. pitch 速率前馈 lead 时间(s)：补偿 pitch 位置模式的伺服速度滞后。
     *    Δz 仅 3cm, pitch 全程只变 ~2°、变化极慢, 故默认 0 即可。
     *    (yaw 的转弯执行滞后已由 gimbal 层 velff 内建, 不再用 lead, 避免重复前馈。) */
    aim.pitch_rate_lead_s = 0.000f;
    /* 速率前馈运动输入(ω 与 v)低通时间常数(s): 平均掉直线蛇形(绕0)与里程混叠尖峰(v 0↔1m/s),
     * 不让速率前馈把它们放大成 yaw 抖; 转弯(持续大 ω/v)仍保留(略延迟)。0.133 -> ~1.2Hz。
     * 抬大更稳直线但转弯前馈更迟, 反之亦然。 */
    aim.rate_ff_motion_tau_s = 0.133f;

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
