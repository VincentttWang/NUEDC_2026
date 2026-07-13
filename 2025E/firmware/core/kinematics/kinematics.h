/**
 * @file  kinematics.h
 * @brief Core 层二维运动学和差速混控计算接口。
 */
#ifndef KINEMATICS_H
#define KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#define KINEMATICS_PI 3.1415926f
#define KINEMATICS_DEG_TO_RAD(deg) ((deg) * (KINEMATICS_PI / 180.0f))
#define KINEMATICS_RAD_TO_DEG(rad) ((rad) * (180.0f / KINEMATICS_PI))

#define DEG_TO_RAD(deg) KINEMATICS_DEG_TO_RAD(deg)
#define RAD_TO_DEG(rad) KINEMATICS_RAD_TO_DEG(rad)

/**
 * @brief 相对运动方向判定结果。
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
 */
typedef struct {
    float yaw;
    float pitch;
    float roll;
} KINEMATICS_ATTITUDE;

/**
 * @brief 二维平面位姿。
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
 */
typedef struct {
    /** 线速度，单位 m/s。 */
    float linear_mps;
    /** 角速度，单位 deg/s。 */
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

typedef KINEMATICS_ATTITUDE RotationAngles;

/**
 * @brief 将数值限制在指定闭区间内。
 */
float Kinematics_Clamp(float value, float min_value, float max_value);

/**
 * @brief 将角度归一化到 [-180, 180]。
 * @param angle_deg 输入角度，单位 deg。
 * @return 归一化后的角度，单位 deg。
 */
float Kinematics_NormalizeAngleDeg(float angle_deg);

/**
 * @brief 计算目标角到当前角的最短角度误差。
 * @return target_deg - current_deg 的归一化结果，单位 deg。
 */
float Kinematics_AngleDiffDeg(float target_deg, float current_deg);

/**
 * @brief 计算二维点间欧氏距离。
 * @return 距离，单位 m。
 */
float Kinematics_Distance2D(float x0_m, float y0_m, float x1_m, float y1_m);

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

/**
 * @brief 清零二维位姿。
 * @param pose 位姿对象。
 */
void Kinematics_PoseInit(KINEMATICS_POSE *pose);

/**
 * @brief 根据线速度和航向角积分更新二维位姿。
 * @param pose 待更新位姿对象。
 * @param linear_mps 线速度，单位 m/s。
 * @param heading_deg 当前航向角，单位 deg。
 * @param dt_s 积分周期，单位 s。
 */
void Kinematics_PoseUpdate(KINEMATICS_POSE *pose,
                           float linear_mps,
                           float heading_deg,
                           float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */
