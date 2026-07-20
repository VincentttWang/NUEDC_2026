#include "app_e_calibration.h"

APP_E_CALIBRATION_CONFIG AppE_GetCalibrationConfig(void){
    AUTO_AIM_CONFIG aim = AutoAim_DefaultConfig();

    /* 1. 场地与规定起点：世界系原点=AB 中点，+X=A->B，+Y 指向靶。
     *    起点位姿指【轮轴中点】(定位原点), 不是车前沿!
     *    ★发挥部分规定摆位(原题原文): "将小车放置在 AB 段轨迹上, 前沿投影与 AC 线对齐"。
     *    场地: A(-0.5,0) B(0.5,0) 为最靠靶的上边; C(-0.5,-1) D(0.5,-1); 逆时针 A->C->D->B->A。
     *    "前沿投影与 AC 线对齐" => 车头前沿(那条边)竖直压在 AC 线(x=-0.5)上 => 车身沿 AB
     *    平放、从 A 向 B 延伸(+X)、车头朝 -X(heading 180°); 启动后在 A 处左拐进入 A->C 首边。
     *    前沿到轮轴实测 0.18m, heading 180° 时轮轴在前沿后方(+X)0.18m:
     *        x = -0.5 + 0.18 = -0.32, y = 0 (车身压 AB 线, y=0)。
     *    (旧值 -0.5,+0.18,-90° = 车头朝 -Y、车身伸向靶侧 +Y(方块外), 违反"放置在 AB 段轨迹上",
     *     且起点前馈朝向错 90°。改回前请同步改巡线"起步拐角不计边"逻辑。)
     *    起步拐角: 见 app_e_task.c AppE_RunLineTask 的 initial_start_turn(首个拐角不计边/不锚定)。
     *    若摆车基准改变, 同步改本项。 */
    aim.start_pose = (KINEMATICS_POSE){
        .x_m = -0.320f,
        .y_m = 0.000f,
        .heading_deg = 180.0f, /* 车头朝 -X(沿 B->A 向), 车身在 AB 上; 起步左拐进 A->C。 */
    };
    aim.track_side_m = 1.000f; /* 填黑线中心线之间的实测平均边长。 */

    /* 2. 单轮编码器：编码轮在车体左侧为正、右侧为负。
     *    实测：编码轮在右轮、轮距 14cm → -轮距/2 = -0.070m。
     *    转弯时用陀螺把单轮里程重构到车心, 消除弯中位置漂移(近弯 B 放大明显)。 */
    aim.encoder_lateral_offset_m = -0.070f;
    /* 陀螺 yaw-rate 符号：实测本机 gz 与航向反号 → -1（左转时 head 增而 gz 负）。
     * 影响速率前馈与车心重构，符号错会让转弯前馈反向、越快越滞后。 */
    aim.gyro_z_sign = -1.000f;
    /* IMU 融合航向陀螺前推 (s): JY61P 融合输出在快速旋转时滞后, 弯中 ω·τ 直接
     * 变 yaw 命令误差 (脱靶主因之一)。实测剂量-效果线性: 0→-2.2°, 25ms→-1.15°,
     * 50ms→+0.89°, 过零点 ≈38ms。整定: 弯中(st=2) visErr 均值 → 0。 */
    aim.heading_gyro_lead_s = 0.038f;

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
     *    pitch 完整模型: pitch_cmd = z0 + [atan2(高差,r) − asin(ℓp/r)]/k
     *    (z0=光束0仰角时电机角, k=光束/电机传动比, ℓp=束线-pitch轴竖直偏距)。
     *    电机限位 [135,165](home150±15); 当前命令域 ~138.5~140.2, 余量充足。
     *    重标走静态两距离法 (见下方 pitch 三元组注释), 单点重标只能动 z0。 */
    aim.solver.yaw_zero_offset_deg = 0.000f;
    /* 激光束线到 yaw 转轴的垂直横向距离 (m): 沿出光方向看, 光束在转轴左侧为正。
     * 机械上激光不过转轴 → 命中修正 asin(ℓ/r) 随距离变化 (0.035m: 远边 1.8m
     * 处 ≈1.1°, 近边 0.55m 处 ≈3.7°)。幅值实测 3.5cm; 符号由数据判定为左(+):
     * 历次起步锚定恒为负(均值~-6°), 与 ℓ=+0.035 在起点贡献 -3.2° 同向。
     * ⚠ 上板验证: 若近边 visErr 反而变差 → 翻成 -0.035。 */
    aim.solver.laser_lateral_offset_m = 0.035f;
    /* pitch 零位与传动比 (2026-07-16 静态两距离标定: Aim track 锁定后读 OLED
     * 的 Pit 电机角): 0.5m 命中 m1=140.14, 1.5m 命中 m2=138.67。
     * 含 ℓp=-0.04 偏轴项的命中光束角 beam(0.5)=8.03° / beam(1.5)=2.67°:
     *   k = Δbeam/Δm = 5.35/1.47 = 3.64 (光束°/电机°)
     *   z0 双点独立反解均为 137.94 (自洽 ✓)
     * (z0, k, ℓp) 是联合标定的三元组, 任何一个变动需整套重标。
     * ⚠ z0 不跨上电存活: pitch 角度基准 = 上电瞬间机械静止位, 断电后头部垂落
     * 位置不同则整个基准漂移 (实测跨会话 ±6°)。起步视觉对齐会自动吸收
     * (startup pitch bias 五六度属正常, 不是 bug); 仅"视觉未锁定的降级起步"
     * 会带着该漂移跑, 竖直可能偏数厘米, 为降级模式已知代价。 */
    aim.solver.pitch_zero_offset_deg = 137.940f;
    aim.solver.pitch_beam_per_motor = 3.640f;
    /* 激光束线到 pitch 转轴的竖直距离 (m, 束线在轴上方为正): yaw 偏轴的竖直版,
     * 误差 ∝1/r。尺量 |ℓp|≈4cm; 符号实测判定为负 (+0.04 时近距漂移 -3.68→-4.07
     * 反而变大, 2026-07-16)。相机-轴距离不进模型 (视觉误差是激光系相对量)。 */
    aim.solver.laser_vertical_offset_m = -0.040f;

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
