/**
 * @file  chassis.c
 * @brief Middleware 层底盘组合服务实现。
 */
#include "chassis.h"
#include "filter/filter.h"
#include "kinematics/kinematics.h"
#include "pid/pid.h"
#include <stdbool.h>
#include <stddef.h>

static CHASSIS_DUTY chassis_duty;

/* 速度闭环状态: 每轮一个位置式 PID + 目标轮速; 模式默认开环。 */
static CHASSIS_CONTROL_MODE chassis_mode;
static PID_CONTROLLER chassis_speed_pid[HALL_ENCODER_COUNT];
static float chassis_speed_target[HALL_ENCODER_COUNT];
/* 速度环反馈的低通状态(每轮一个); 只用于控制, 不影响 HallEncoder_GetSpeed 的原始值。 */
static float chassis_speed_feedback[HALL_ENCODER_COUNT];

static BSP_STATUS Chassis_CombineStatus(BSP_STATUS current, BSP_STATUS next){
    return (current == BSP_STATUS_OK) ? next : current;
}

static void Chassis_ClearDuty(void){
    chassis_duty.left_percent = 0.0f;
    chassis_duty.right_percent = 0.0f;
}

/* 回读 BSP 实际生效的占空比(已含限幅/刹车归零), 作为对外的 duty 记录。 */
static void Chassis_RefreshDuty(void){
    chassis_duty.left_percent = TB6612FNG_GetDuty(TB6612FNG_CHANNEL_LEFT);
    chassis_duty.right_percent = TB6612FNG_GetDuty(TB6612FNG_CHANNEL_RIGHT);
}

/* 原始出力: 直接下发占空比并刷新记录, 不改控制模式(供开环 SetDuty 与闭环 UpdateSpeedControl 共用)。 */
static BSP_STATUS Chassis_ApplyDuty(float left_percent, float right_percent){
    BSP_STATUS status = TB6612FNG_SetDuty(TB6612FNG_CHANNEL_LEFT, left_percent);
    BSP_STATUS right_status = TB6612FNG_SetDuty(TB6612FNG_CHANNEL_RIGHT, right_percent);

    status = Chassis_CombineStatus(status, right_status);

    if (status == BSP_STATUS_OK){
        Chassis_RefreshDuty();
    }

    return status;
}

/* 用默认增益初始化两轮速度环并复位为开环、目标清零。 */
static void Chassis_SpeedControlInit(void){
    PID_CONFIG cfg = {
        .kp = CHASSIS_SPEED_KP,
        .ki = CHASSIS_SPEED_KI,
        .kd = CHASSIS_SPEED_KD,
        .integral_limit = CHASSIS_SPEED_INTEGRAL_LIMIT,
        .output_limit = CHASSIS_SPEED_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };

    for (uint8_t id = 0U; id < (uint8_t)HALL_ENCODER_COUNT; id++){
        PID_Init(&chassis_speed_pid[id], &cfg);
        chassis_speed_target[id] = 0.0f;
        chassis_speed_feedback[id] = 0.0f;
    }
    chassis_mode = CHASSIS_CONTROL_DUTY;
}

/* 退出速度闭环: 切回开环、清目标并复位 PID(供 SetDuty/Stop 调用), 不留残留积分。 */
static void Chassis_ExitSpeedControl(void){
    chassis_mode = CHASSIS_CONTROL_DUTY;
    for (uint8_t id = 0U; id < (uint8_t)HALL_ENCODER_COUNT; id++){
        chassis_speed_target[id] = 0.0f;
        chassis_speed_feedback[id] = 0.0f;
        PID_Reset(&chassis_speed_pid[id]);
    }
}

/*
 * 速度前馈: 由实测「占空比 -> 稳态轮速」曲线反解(标定值见 chassis.h)。
 * offset 取目标速度符号, 使正反向都落在各自的线性段上。
 */
static float Chassis_SpeedFeedforward(HALL_ENCODER_ID wheel, float target_mps){
    float gain = (wheel == HALL_ENCODER_LEFT) ? CHASSIS_FF_LEFT_GAIN
                                              : CHASSIS_FF_RIGHT_GAIN;
    float offset = (wheel == HALL_ENCODER_LEFT) ? CHASSIS_FF_LEFT_OFFSET
                                                : CHASSIS_FF_RIGHT_OFFSET;

    if (target_mps > 0.0f){
        return (gain * target_mps) + offset;
    }
    return (gain * target_mps) - offset;
}

/*
 * 单轮速度环一步。
 * 返回 true 表示应按 *duty_out 驱动; 返回 false 表示目标为停止, 调用方须刹车
 * (此时该轮 PID 已复位, 不留积分)。
 */
static bool Chassis_UpdateWheel(HALL_ENCODER_ID wheel, float dt_s, float *duty_out){
    float target = chassis_speed_target[wheel];

    if ((target < CHASSIS_SPEED_ZERO_TARGET_MPS) &&
        (target > -CHASSIS_SPEED_ZERO_TARGET_MPS)){
        PID_Reset(&chassis_speed_pid[wheel]);
        chassis_speed_feedback[wheel] = 0.0f;
        return false;
    }

    /* 反馈先过一阶低通再进 PI, 压量化跳变(前馈走目标值, 不受该滞后影响)。 */
    chassis_speed_feedback[wheel] = Filter_LowpassEma(chassis_speed_feedback[wheel],
                                                      HallEncoder_GetSpeed(wheel),
                                                      CHASSIS_SPEED_FEEDBACK_ALPHA);

    float correction = PID_Update(&chassis_speed_pid[wheel], target,
                                  chassis_speed_feedback[wheel], dt_s);

    *duty_out = Chassis_SpeedFeedforward(wheel, target) + correction;
    return true;
}

BSP_STATUS Chassis_Init(void){
    Chassis_ClearDuty();
    Chassis_SpeedControlInit();

    BSP_STATUS status = TB6612FNG_Init();
    status = Chassis_CombineStatus(status, HallEncoder_Init());
    return status;
}

BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent){
    Chassis_ExitSpeedControl();
    return Chassis_ApplyDuty(left_percent, right_percent);
}

void Chassis_SetWheelSpeed(float left_mps, float right_mps){
    if (chassis_mode != CHASSIS_CONTROL_SPEED){
        /* 从开环切入闭环: 复位 PID, 避免旧积分/微分残留造成跳变。 */
        PID_Reset(&chassis_speed_pid[HALL_ENCODER_LEFT]);
        PID_Reset(&chassis_speed_pid[HALL_ENCODER_RIGHT]);
        chassis_mode = CHASSIS_CONTROL_SPEED;
    }
    chassis_speed_target[HALL_ENCODER_LEFT] = left_mps;
    chassis_speed_target[HALL_ENCODER_RIGHT] = right_mps;
}

void Chassis_SetSpeed(float body_mps){
    Chassis_SetWheelSpeed(body_mps, body_mps);
}

BSP_STATUS Chassis_UpdateSpeedControl(float dt_s){
    if (chassis_mode != CHASSIS_CONTROL_SPEED){
        return BSP_STATUS_OK;   /* 开环: 不干预出力。 */
    }

    float left_duty = 0.0f;
    float right_duty = 0.0f;
    bool left_drive = Chassis_UpdateWheel(HALL_ENCODER_LEFT, dt_s, &left_duty);
    bool right_drive = Chassis_UpdateWheel(HALL_ENCODER_RIGHT, dt_s, &right_duty);

    /* 目标为停止的轮走主动刹车, 不靠 PI 渐近爬到零(死区内爬不下去)。 */
    BSP_STATUS status = left_drive
        ? TB6612FNG_SetDuty(TB6612FNG_CHANNEL_LEFT, left_duty)
        : TB6612FNG_Brake(TB6612FNG_CHANNEL_LEFT);
    BSP_STATUS right_status = right_drive
        ? TB6612FNG_SetDuty(TB6612FNG_CHANNEL_RIGHT, right_duty)
        : TB6612FNG_Brake(TB6612FNG_CHANNEL_RIGHT);

    status = Chassis_CombineStatus(status, right_status);

    if (status == BSP_STATUS_OK){
        Chassis_RefreshDuty();
    }

    return status;
}

CHASSIS_CONTROL_MODE Chassis_GetControlMode(void){
    return chassis_mode;
}

float Chassis_GetWheelSpeedTarget(HALL_ENCODER_ID wheel){
    return chassis_speed_target[wheel];
}

float Chassis_GetWheelSpeedIntegral(HALL_ENCODER_ID wheel){
    return chassis_speed_pid[wheel].state.integral;
}

BSP_STATUS Chassis_Stop(CHASSIS_STOP_MODE mode){
    BSP_STATUS status = BSP_STATUS_OK;

    Chassis_ExitSpeedControl();

    switch (mode){
        case CHASSIS_STOP_MODE_BRAKE:
            status = TB6612FNG_BrakeAll();
            break;
        case CHASSIS_STOP_MODE_COAST:
            status = TB6612FNG_CoastAll();
            break;
        default:
            return BSP_STATUS_INVALID_ARG;
    }

    if (status == BSP_STATUS_OK){
        Chassis_ClearDuty();
    }

    return status;
}

BSP_STATUS Chassis_Brake(void){
    return Chassis_Stop(CHASSIS_STOP_MODE_BRAKE);
}

BSP_STATUS Chassis_Coast(void){
    return Chassis_Stop(CHASSIS_STOP_MODE_COAST);
}

CHASSIS_DUTY Chassis_GetDuty(void){
    return chassis_duty;
}

float Chassis_GetSpeed(void){
    /*
     * 车体线速度 = 左右轮线速度均值(Kinematics_WheelToBody 的 linear 分量)。
     * 角速度分量需 track_width(暂缺, 见 core/kinematics 约定), 故传 0 只取线速度。
     */
    KINEMATICS_VELOCITY body = Kinematics_WheelToBody(
        HallEncoder_GetSpeed(HALL_ENCODER_LEFT),
        HallEncoder_GetSpeed(HALL_ENCODER_RIGHT),
        0.0f);
    return body.linear_mps;
}

float Chassis_GetWheelSpeed(HALL_ENCODER_ID wheel){
    return HallEncoder_GetSpeed(wheel);
}

float Chassis_GetDistance(void){
    return (HallEncoder_GetDistance(HALL_ENCODER_LEFT) +
            HallEncoder_GetDistance(HALL_ENCODER_RIGHT)) * 0.5f;
}

float Chassis_GetWheelDistance(HALL_ENCODER_ID wheel){
    return HallEncoder_GetDistance(wheel);
}

void Chassis_ResetDistance(void){
    HallEncoder_ResetDistance();
}
