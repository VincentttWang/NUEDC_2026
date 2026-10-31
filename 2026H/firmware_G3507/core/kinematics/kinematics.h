/**
 * @file  kinematics.h
 * @brief Core 层二维运动学、差速混控与两轮差速运动学模型接口。
 *
 * 纯算法模块，不读传感器/不驱动执行器。当前消费者包括 middleware/line_follow、
 * middleware/chassis 与 middleware/straight_drive；其余类型/宏/函数标注为
 * 「预留」——已实现并保留，但当前工程暂无调用者，供后续路径/姿态/动作策略模块复用。
 *
 * 「两轮差速运动学模型」一节(WheelToBody/BodyToWheel/轮速换算/TurnRadius/IntegratePose)
 * 建立 轮速 ↔ 车体(v, ω) ↔ 位姿 的速度层几何关系。车体参数(轮距/轮半径)一律传参，
 * 不在本模块固化；约定见该节。当前巡线走占空比空间，尚无消费者，整节属预留备用。
 */
#ifndef KINEMATICS_H
#define KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

/* 角度/弧度换算宏。供下方「两轮差速运动学模型」内部换算使用(公开 API 角度用 deg)。 */
#define KINEMATICS_PI 3.1415926f
#define KINEMATICS_DEG_TO_RAD(deg) ((deg) * (KINEMATICS_PI / 180.0f))
#define KINEMATICS_RAD_TO_DEG(rad) ((rad) * (180.0f / KINEMATICS_PI))

/* ω≈0(直线)时 Kinematics_TurnRadius 返回的转弯半径哨兵，单位 m。 */
#define KINEMATICS_TURN_RADIUS_STRAIGHT 1.0e6f

/**
 * @brief 相对运动方向判定结果。
 * @note 预留：当前工程暂无调用者，供后续路径/动作策略模块表达方向。
 */
typedef enum {
    KINEMATICS_DIR_LEFT = 0,
    KINEMATICS_DIR_RIGHT,
    KINEMATICS_DIR_FORWARD,
    KINEMATICS_DIR_BACKWARD,
    KINEMATICS_DIR_UNSTABLE
} KINEMATICS_DIR;

/**
 * @brief 欧拉姿态角，单位 deg。
 * @note 预留：当前工程暂无调用者。
 */
typedef struct {
    float yaw;
    float pitch;
    float roll;
} KINEMATICS_ATTITUDE;

/**
 * @brief 二维平面位姿。
 * @note 预留：差速运动学模型 Kinematics_IntegratePose 的位姿类型；当前工程暂无调用者。
 */
typedef struct {
    /** X 坐标，单位 m。 */
    float x_m;
    /** Y 坐标，单位 m。 */
    float y_m;
    /** 航向角，单位 deg。 */
    float heading_deg;
} KINEMATICS_POSE;

/**
 * @brief 底盘平面速度指令。
 * @note 预留：差速运动学模型 WheelToBody/IntegratePose 的车体速度类型；当前工程暂无调用者。
 */
typedef struct {
    /** 线速度，单位 m/s。 */
    float linear_mps;
    /** 角速度，单位 deg/s(逆时针为正)。 */
    float angular_deg_s;
} KINEMATICS_VELOCITY;

/**
 * @brief 差速左右轮混控输出。
 */
typedef struct {
    /** 左轮输出，具体单位由调用方定义。 */
    float left;
    /** 右轮输出，具体单位由调用方定义。 */
    float right;
} KINEMATICS_DIFFERENTIAL_OUTPUT;

/**
 * @brief 差速左右轮线速度(差速运动学模型的物理量输出，区别于占空比混控 DIFFERENTIAL_OUTPUT)。
 * @note 预留：当前工程暂无调用者。
 */
typedef struct {
    /** 左轮着地点线速度，单位 m/s。 */
    float left_mps;
    /** 右轮着地点线速度，单位 m/s。 */
    float right_mps;
} KINEMATICS_WHEEL_SPEED;

/**
 * @brief 将数值限制在指定闭区间内。
 */
float Kinematics_Clamp(float value, float min_value, float max_value);

/**
 * @brief 将角度归一化到 [-180, 180)。
 * @param angle_deg 输入角度，单位 deg。
 * @return 归一化后的角度，单位 deg。
 * @note 预留：当前工程暂无调用者（仅被 Kinematics_AngleDiffDeg 内部使用）。
 */
float Kinematics_NormalizeAngleDeg(float angle_deg);

/**
 * @brief 计算目标角到当前角的最短角度误差。
 * @return target_deg - current_deg 的归一化结果，单位 deg，范围 [-180, 180)。
 * @note 预留：当前工程暂无调用者。
 */
float Kinematics_AngleDiffDeg(float target_deg, float current_deg);

/**
 * @brief 将前进量和转向量混合为左右轮输出。
 * @param forward 前进分量。
 * @param turn 转向分量。
 * @param output_limit 左右轮输出绝对值限幅；<= 0 时不启用限幅。
 * @return 左右轮混控输出。
 */
KINEMATICS_DIFFERENTIAL_OUTPUT Kinematics_DifferentialMix(float forward,
                                                          float turn,
                                                          float output_limit);

/* ═══════════ 两轮差速运动学模型(预留，纯函数，车体参数传入) ═══════════
 * 约定：线速度 m/s；车体航向角速度 ω 取 deg/s(逆时针为正，与陀螺 gz 同向由调用方保证)；
 *       轮自转角速度取 rad/s(直接配 v=r·ω)；位姿 x/y 为 m、航向 deg。
 * 记号：v_l/v_r 左右轮线速度，v 车体线速度，L 轮距，r 轮半径。 */

/**
 * @brief 正运动学：左右轮线速度 → 车体速度。
 * @param left_mps      左轮着地点线速度，m/s。
 * @param right_mps     右轮着地点线速度，m/s。
 * @param track_width_m 轮距(两轮着地点间距) L，m；<= 0 时角速度按 0 处理。
 * @return 车体速度：linear_mps=(v_l+v_r)/2；angular_deg_s=(v_r−v_l)/L(逆时针为正)。
 * @note 预留：当前工程暂无调用者。
 */
KINEMATICS_VELOCITY Kinematics_WheelToBody(float left_mps, float right_mps,
                                           float track_width_m);

/**
 * @brief 逆运动学：车体速度 → 左右轮线速度。
 * @param linear_mps    车体前进线速度 v，m/s。
 * @param angular_deg_s 车体航向角速度 ω(逆时针为正)，deg/s。
 * @param track_width_m 轮距 L，m。
 * @return 左右轮着地点线速度：left=v−ω·L/2；right=v+ω·L/2(m/s)。
 * @note 预留：当前工程暂无调用者。
 */
KINEMATICS_WHEEL_SPEED Kinematics_BodyToWheel(float linear_mps, float angular_deg_s,
                                              float track_width_m);

/**
 * @brief 轮线速度 → 轮自转角速度：ω = v / r。
 * @param linear_mps     轮着地点线速度，m/s。
 * @param wheel_radius_m 轮半径 r，m；<= 0 时返回 0。
 * @return 轮自转角速度，rad/s。
 * @note 预留：当前工程暂无调用者。
 */
float Kinematics_WheelLinearToAngular(float linear_mps, float wheel_radius_m);

/**
 * @brief 轮自转角速度 → 轮线速度：v = r · ω。
 * @param angular_rad_s  轮自转角速度，rad/s。
 * @param wheel_radius_m 轮半径 r，m。
 * @return 轮着地点线速度，m/s。
 * @note 预留：当前工程暂无调用者。
 */
float Kinematics_WheelAngularToLinear(float angular_rad_s, float wheel_radius_m);

/**
 * @brief 瞬时转弯半径：R = v / ω。
 * @param linear_mps    车体线速度 v，m/s。
 * @param angular_deg_s 车体航向角速度 ω，deg/s。
 * @return 转弯半径 m；|ω| 近 0(直线)时返回 KINEMATICS_TURN_RADIUS_STRAIGHT。
 * @note 预留：当前工程暂无调用者。
 */
float Kinematics_TurnRadius(float linear_mps, float angular_deg_s);

/**
 * @brief 里程推算(航位推算)：按车体速度积分一步位姿(中点欧拉，直线自然退化)。
 * @param pose 当前位姿(x_m/y_m/heading_deg)。
 * @param vel  车体速度(linear_mps/angular_deg_s)。
 * @param dt_s 积分步长，s；<= 0 时原样返回 pose。
 * @return 更新后的位姿，heading 归一化到 [-180, 180)。
 * @note 预留：当前工程暂无调用者。
 */
KINEMATICS_POSE Kinematics_IntegratePose(KINEMATICS_POSE pose, KINEMATICS_VELOCITY vel,
                                         float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */
