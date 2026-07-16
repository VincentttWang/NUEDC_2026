#include "app_e_calibration.h"

APP_E_CALIBRATION_CONFIG AppE_GetCalibrationConfig(void){
    AUTO_AIM_CONFIG aim = AutoAim_DefaultConfig();

    /* 1. 场地与规定起点：世界系原点=AB 中点，+X=A->B，+Y 指向靶。
     *    起点位姿指【轮轴中点】(定位原点), 不是车前沿!
     *    摆车方式 = 前沿对齐 AB 线、车头朝 A->C (-Y): 前沿到轮轴实测 0.18m,
     *    故轴心在 AB 线后方 +Y 侧 0.18m 处。此项漏配会让起点前馈错 ~10°,
     *    且被起步视觉对齐冻结进 bias、第一个角点 snap 后全圈倒灌(实测教训)。
     *    若摆车基准改变(如按轴心对线), 同步改 y_m。 */
    aim.start_pose = (KINEMATICS_POSE){
        .x_m = -0.500f,
        .y_m = 0.180f,
        .heading_deg = -90.0f, /* 从 A 沿 A->C。 */
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
    /* 云台 yaw 支点安装偏移 (定位原点 = 轮轴中点):
     * 实测支点在轮轴前 12cm → mount_x = +0.12。静态解与速率前馈均消费此值;
     * 近靶处(~0.5m)12cm 臂长最坏折 ~10° 级指向差, 且随位姿变化, 静态标零吸收不掉。
     * 注: 陀螺仪在轮轴后 2.5cm —— 无需补偿: 航向/角速度是刚体全局量, 与安装位置
     * 无关(只有加速度计读数受位置影响, 本工程不直接消费加计), 勿为此加"修正"。 */
    aim.solver.mount_x_m = 0.120f; /* 车体前方为正。 */
    aim.solver.mount_y_m = 0.000f; /* 车体左侧为正。 */

    /* 4. 云台机械零位：增大 yaw offset 会减小 yaw 指令；pitch 反之。
     *    pitch_zero_offset = "激光水平(0 仰角)时的 pitch 电机角"。
     *    solver 输出 pitch_cmd = atan2(高差,水平距离) + 该 offset; 电机限位 [135,165](home150±15)。
     *    实测标定 = 140 (光束正中靶心); pitch 轴有减速比, 电机每度对应光束多度,
     *    故只需比 home 低 ~10° 即可下压到位。命令落 ~141~143° 仍在 [135,165] 内。
     *    重标: 让车静瞄靶, 增/减该值直到激光竖直方向正中靶心 (增大→pitch 指令增大→光束上抬)。 */
    aim.solver.yaw_zero_offset_deg = 0.000f;
    /* 激光束线到 yaw 转轴的垂直横向距离 (m): 沿出光方向看, 光束在转轴左侧为正。
     * 机械上激光不过转轴 → 命中修正 asin(ℓ/r) 随距离变化 (0.035m: 远边 1.8m
     * 处 ≈1.1°, 近边 0.55m 处 ≈3.7°)。幅值实测 3.5cm; 符号由数据判定为左(+):
     * 历次起步锚定恒为负(均值~-6°), 与 ℓ=+0.035 在起点贡献 -3.2° 同向。
     * ⚠ 上板验证: 若近边 visErr 反而变差 → 翻成 -0.035。 */
    aim.solver.laser_lateral_offset_m = 0.035f;
    aim.solver.pitch_zero_offset_deg = 140.000f;

    /* 5. 视觉 bias (衰减增益, 逐轴): 第 n 个有效帧增益 g(n)=max(gain_min, gain0/(1+n))。
     *    yaw: gain0=0.5 -> 首帧吃掉一半残差, ~3 帧完成主收敛;
     *         gain_min=0.02 @~30fps -> 稳态时间常数 ~1.7s, 跟漂移不引入抖动。
     *    pitch 独立且更保守: 视觉误差是"光束角"而校正加在"电机角"上, pitch 减速比把
     *         等效环增益放大数倍(电机 1°≈光束数度) -> gain 按 1/减速比缩;
     *         若实测 pitch 收敛振荡/过冲 -> 再减半 gain0; 收敛慢 -> 加大。
     *         行程仅 home±15°, 限幅收紧到 2°。
     *    测量质量(平滑/野值/丢帧预测/延迟前推)由 K230 侧 Kalman 负责, 此处无门控。
     *    上板首验: 若某轴误差越校越大(激光越修越偏), 只翻转对应 sign。 */
    /* 视觉稳态增益按实测残差节奏整定 (2026-07-16):
     *   yaw gain_min=0.10 @30fps τ≈0.35s —— 残差随边变化(~2-3s/边), 0.05 追有滞后
     *   (直线段 std 3.9°), 抬到 0.10 收紧; 若出现与视觉噪声合拍的抖动则回落。
     *   pitch gain_min=0.03 τ≈1.1s —— 0.01 太慢, 实测 seg 级 -3.7° 持续残留。
     * 量程 8° 覆盖起步锚定的摆车级残差。诊断需要"纯观测"时把增益临时置 0 即可
     * (起步交接 SetStartupBias 是直写, 不经增益, 不受影响)。 */
    aim.vision_yaw = (AIM_VISION_GAIN){
        .gain0 = 0.500f, .gain_min = 0.100f, .limit_deg = 8.000f,
    };
    aim.vision_pitch = (AIM_VISION_GAIN){
        .gain0 = 0.250f, .gain_min = 0.030f, .limit_deg = 2.000f,
    };
    /* 起步静态对齐限幅(逐轴): 倒计时窗口内一次性吸收静态总偏差 (含光轴统一假设下
     * 的相机-激光偏差, yaw 实测基线 ~9° 故给 ±10°; pitch 行程小给 ±5°)。开跑即冻结。
     * ⚠ 若上板发现激光稳定偏离靶心一个常量 → 光轴统一假设不成立(相机居中≠激光
     * 命中), 需回头做 S10 光轴标定(K230 IDEAL_* 或光斑检测)。
     * 若对齐期间校正量打到限幅且激光越转越偏 → 翻对应 vision_*_sign。 */
    aim.vision_yaw_startup_limit_deg = 10.000f;
    aim.vision_pitch_startup_limit_deg = 5.000f;
    /* 弯中毒帧 ω 门控 (2026-07-16 实测教训): 高稳态增益(0.1)下弯中瞬态帧 8 帧
     * 即把 bias 打到限幅, 近靶边被 1/r 放大成 ~10° 尖峰。|gz|>40°/s 冻结慢 bias
     * (转弯 80-120, 直线蛇形 <30, 分界清晰), 出弯再滞留 5 帧挡延迟拖尾。 */
    aim.vision_freeze_omega_deg_s = 40.000f;
    /* yaw 符号实测定为 +1 (2026-07-15 两次运行差分定罪: 原 -1 下加大校正量
     * 误差同向增大 d(visErr)/d(bias)≈-1.2, 即校正反向、bias 必然单向贴限幅)。
     * 注意: 对准伺服(gimbal_tracking.yaw_output_sign)与本符号是两条独立链,
     * 伺服收敛不能为此符号作证。pitch 符号尚未经同法验证, 待实测。 */
    aim.vision_yaw_sign = 1.000f;
    aim.vision_pitch_sign = 1.000f;

    /* 6. F3：0°=靶心右侧，90°=上方。 */
    aim.solver.circle_radius_m = 0.060f;
    aim.circle_phase0_deg = 0.000f;

    /* 7. pitch 速率前馈 lead 时间(s)：补偿 pitch 位置模式的伺服速度滞后。
     *    Δz 仅 3cm, pitch 全程只变 ~2°、变化极慢, 故默认 0 即可。
     *    (yaw 的转弯执行滞后已由 gimbal 层 velff 内建, 不再用 lead, 避免重复前馈。) */
    aim.pitch_rate_lead_s = 0.000f;
    /* 速率前馈 v 输入低通时间常数(s): 只滤 v (单轮里程 v=Δs/dt 混叠尖峰 0↔1m/s)。
     * ω 不滤: 蛇形是车身真实自转, velff 速度内环实时 1:1 反转补偿才能把激光钉在世界系;
     * 滤 ω 只会给转弯前馈加相位滞后。0.133 -> ~1.2Hz。 */
    aim.rate_ff_v_tau_s = 0.133f;

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
