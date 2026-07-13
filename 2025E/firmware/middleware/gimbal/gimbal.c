/**
 * @file  gimbal.c
 * @brief Middleware 层云台服务实现 — 双 F32C 无刷 (yaw/pitch)。
 *
 * 云台两轴均由 F32C 无刷驱动 (UART3) 控制:
 *   - yaw   轴 = BLDC 地址 1 (GIMBAL_YAW_ADDR)
 *   - pitch 轴 = BLDC 地址 2 (GIMBAL_PITCH_ADDR)
 * 对外保持与原步进实现一致的"速度指令 (deg/s) + 开环角度估计"语义:
 *   - Gimbal_SetSpeed 将 yaw 的 deg/s 换算成 RPM 下发到无刷速度闭环;
 *   - Gimbal_GetAngle 返回按下发速度积分得到的开环估计角 (供 E3 扫描等使用);
 *   - pitch 轴保持位置模式并施加软件角度限位, yaw 轴不限位 (可连续多圈旋转)。
 *
 * 上电策略 (避免开机空转 / 缩短启动):
 *   - yaw 惰性使能: 开机不使能, 首次收到非零 yaw 速度指令时才使能。
 *     (速度环锁 0 会主动驱动电机去消除偏差, 若开机就使能会在 0 附近抖/爬。)
 *   - pitch 非阻塞抬升: 开机由 Gimbal_StartupElevatePitch() 发起位置模式抬升后
 *     立即返回, 不阻塞菜单; 进入 E2/E3 瞄准前调用 Gimbal_EnsurePitchReady()
 *     阻塞确认到位 (保持位置模式)。E1 循迹不用激光, 不受影响。
 */
#include "gimbal.h"
#include "bsp_time.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ===== 轴 → 无刷地址映射 ===== */
#ifndef GIMBAL_YAW_ADDR
#define GIMBAL_YAW_ADDR   BLDC_ADDR_1
#endif
#ifndef GIMBAL_PITCH_ADDR
#define GIMBAL_PITCH_ADDR BLDC_ADDR_2
#endif

/*
 * deg/s → RPM 换算:
 *   rpm = deg_s * GIMBAL_GEAR_RATIO / 6.0   (360 deg/s = 60 rpm, 直驱比 1:1)
 * GIMBAL_GEAR_RATIO = 电机轴转速 / 云台轴转速, 需按实际减速比在实机上标定。
 */
#ifndef GIMBAL_GEAR_RATIO
#define GIMBAL_GEAR_RATIO 1.0f
#endif
#define GIMBAL_DEG_S_TO_RPM(deg_s) ((deg_s) * GIMBAL_GEAR_RATIO / 6.0f)

/*
 * yaw 位置模式参数 (供前馈瞄准 Gimbal_SetAngle 使用):
 *   位置模式必须先设运动速度, 否则目标角不生效。yaw 惯量小, 可给较快的运动速度。
 *   yaw 不设角度限位 (min==max 取消限位), 允许连续多圈旋转。
 */
#ifndef GIMBAL_YAW_POS_SPEED
#define GIMBAL_YAW_POS_SPEED 80      /* yaw 位置模式运动速度 (RPM) */
#endif
#ifndef GIMBAL_YAW_POS_ACC
#define GIMBAL_YAW_POS_ACC   60      /* yaw 位置模式加速度 (转/s²) */
#endif

/*
 * pitch 开机抬升: 目标工作角复用无刷驱动的启动角常量 (0.1° 单位, 默认 150°)。
 * 位置模式定位到位后切回速度模式。到位判定容差与阻塞超时如下。
 */
#ifndef GIMBAL_PITCH_HOME_X10
#define GIMBAL_PITCH_HOME_X10 BLDC_PITCH_INIT_X10
#endif
#define GIMBAL_PITCH_HOME_TOL_X10     30      /* 3° 容差 */
#define GIMBAL_PITCH_HOME_TIMEOUT_MS  2000U
#define GIMBAL_PITCH_HOME_POLL_MS     50U

/* 归位后 pitch 限位收紧到 home±range (归位阶段仍用全量程从 0° travel 到 home)。 */
#define GIMBAL_PITCH_RANGE_X10        150     /* ±30° */
#define GIMBAL_PITCH_HOME_DEG         ((float)GIMBAL_PITCH_HOME_X10 / 10.0f)
#define GIMBAL_PITCH_RANGE_DEG        ((float)GIMBAL_PITCH_RANGE_X10 / 10.0f)

/*
 * pitch 控制模式: pitch 归位后【始终工作在位置模式】。
 *   HOMING → 开机位置模式抬升中, 尚未就绪 (不接受追踪指令);
 *   READY  → 就绪; 追踪 = 逐帧移动位置设定点, 丢失 = 设定点回 home。
 * 位置模式带保持力矩、精确到位, 且受硬件角度限位约束, 天然使限位硬优先于追踪。
 */
typedef enum {
    PITCH_HOMING = 0,
    PITCH_READY,
} PITCH_MODE;

typedef struct {
    float yaw_deg_s;
    float pitch_deg_s;          /* 追踪环给出的 pitch 角速度 (deg/s), 积分成设定点 */
    float yaw_deg;
    float pitch_deg;            /* pitch 当前角(度) = 位置设定点; 供显示 */
    float pitch_target_deg;     /* pitch 位置设定点 (下发给电机位置环) */
    float pitch_min_deg;
    float pitch_max_deg;
    int16_t last_pitch_rpm;
    int16_t last_yaw_rpm;
    int32_t last_pitch_angle_x10;  /* 上次下发的 pitch 位置(0.1°), 用于去重 */
    int32_t last_yaw_angle_x10;    /* 上次下发的 yaw 位置(0.1°), 用于去重 (位置模式) */
    uint32_t last_update_ms;
    uint32_t pitch_ctrl_ms;        /* pitch 设定点积分的上次时间戳 */
    bool yaw_enabled;      /* yaw 惰性使能标志 */
    bool yaw_pos_mode;     /* yaw 当前处于多圈位置模式 (前馈瞄准); false=速度模式 */
    bool initialized;
} GIMBAL_CONTEXT;

static GIMBAL_CONTEXT s_gimbal;

/* 将换算后的 RPM 下发到某轴; 仅在数值变化时发送, 降低 UART3 负载。 */
static void Gimbal_SendAxisRpm(uint8_t addr, int16_t rpm, int16_t *last_rpm){
    if (rpm == *last_rpm){
        return;
    }
    *last_rpm = rpm;
    BLDC_SetSpeed(addr, rpm);
}

/*
 * pitch 生命周期状态: 独立于 s_gimbal, 不被 Gimbal_Init() 重置。
 * 因为 pitch 抬升依赖"以上电时的机械静止位为 0°", 只能在开机执行一次,
 * 不能因为 Yaw 速度测试等重新调用 Gimbal_Init() 而重复归零 (会累加超程)。
 */
static bool s_pitch_home_started;  /* 已发起开机位置模式抬升 */
static PITCH_MODE s_pitch_mode;    /* HOMING / READY (初值 HOMING=未就绪) */

/* pitch 是否已就绪 (抬升完成, 可接受速度/park 指令)。 */
static bool Gimbal_PitchReady(void){
    return s_pitch_mode != PITCH_HOMING;
}

static bool Gimbal_IsValidAxis(GIMBAL_AXIS axis){
    return axis < GIMBAL_AXIS_MAX;
}

/* pitch 对应的反馈数据结构体 (随地址映射自动选择, 便于日后改映射)。 */
static volatile BLDC_MotorData_t *Gimbal_PitchMotor(void){
    return (GIMBAL_PITCH_ADDR == BLDC_ADDR_1) ? &BLDC_Motor1 : &BLDC_Motor2;
}

static float Gimbal_ClampPitch(float pitch_deg){
    if (pitch_deg < s_gimbal.pitch_min_deg){
        return s_gimbal.pitch_min_deg;
    }
    if (pitch_deg > s_gimbal.pitch_max_deg){
        return s_gimbal.pitch_max_deg;
    }
    return pitch_deg;
}

/* 发起 pitch 位置模式抬升到工作角 (非阻塞)。 */
static void Gimbal_StartPitchHoming(void){
    /*
     * 位置模式必须先给运动速度 (BLDC_PITCH_SPEED), 否则目标角不生效;
     * 以当前物理位置为 0°, 施加 [MIN,MAX] 限位后正转到工作角。
     */
    BLDC_Enable(GIMBAL_PITCH_ADDR);
    BLDC_SetMode(GIMBAL_PITCH_ADDR, MODE_MULTI_POS);
    BLDC_SetSpeed(GIMBAL_PITCH_ADDR, BLDC_PITCH_SPEED);
    BLDC_SetAcc(GIMBAL_PITCH_ADDR, BLDC_PITCH_ACC);
    BLDC_ClearMultiAngle(GIMBAL_PITCH_ADDR);
    BLDC_SetAngleLimit(GIMBAL_PITCH_ADDR, BLDC_PITCH_MIN_X10, BLDC_PITCH_MAX_X10);
    Gimbal_PitchMotor()->data_ready = 0U;
    BLDC_SetMultiAngle(GIMBAL_PITCH_ADDR, GIMBAL_PITCH_HOME_X10);

    s_pitch_home_started = true;
    s_pitch_mode = PITCH_HOMING;
}

/* 阻塞等待 pitch 到位 (或超时), 然后切回速度模式并对齐开环估计角。 */
static void Gimbal_FinishPitchHoming(void){
    volatile BLDC_MotorData_t *pitch = Gimbal_PitchMotor();
    uint32_t start_ms = BSP_Time_GetMs();

    while ((BSP_Time_GetMs() - start_ms) < GIMBAL_PITCH_HOME_TIMEOUT_MS){
        int32_t diff;

        BLDC_ReqFeedback(GIMBAL_PITCH_ADDR, FB_MULTI_ANGLE);
        BSP_DelayMs(GIMBAL_PITCH_HOME_POLL_MS);

        diff = (int32_t)pitch->multi_angle - (int32_t)GIMBAL_PITCH_HOME_X10;
        if (diff < 0){
            diff = -diff;
        }
        if ((pitch->data_ready != 0U) && (diff <= GIMBAL_PITCH_HOME_TOL_X10)){
            break;
        }
    }

    /* 归位完成后收紧硬件角度限位到 home±range (归位阶段用全量程 travel 到 home)。 */
    BLDC_SetAngleLimit(GIMBAL_PITCH_ADDR,
                       GIMBAL_PITCH_HOME_X10 - GIMBAL_PITCH_RANGE_X10,
                       GIMBAL_PITCH_HOME_X10 + GIMBAL_PITCH_RANGE_X10);

    /* 保持位置模式 (追踪与保持都用位置环); 位置设定点/估计角对齐到工作角。 */
    s_gimbal.pitch_deg_s = 0.0f;
    s_gimbal.pitch_target_deg = (float)GIMBAL_PITCH_HOME_X10 / 10.0f;
    s_gimbal.pitch_deg = s_gimbal.pitch_target_deg;
    s_gimbal.last_pitch_angle_x10 = GIMBAL_PITCH_HOME_X10;
    s_gimbal.pitch_ctrl_ms = BSP_Time_GetMs();
    s_gimbal.last_update_ms = BSP_Time_GetMs();
    s_pitch_mode = PITCH_READY;
}

BSP_STATUS Gimbal_Init(void){
    /* 无刷底层驱动初始化 (UART3 唤醒 + 接收中断), 幂等。 */
    BLDC_Init();

    s_gimbal.yaw_deg_s = 0.0f;
    s_gimbal.pitch_deg_s = 0.0f;
    s_gimbal.yaw_deg = 0.0f;
    /* 若 pitch 已在开机抬升到工作角, 保持估计角/设定点一致 (不因重新 Init 归零)。 */
    s_gimbal.pitch_deg = Gimbal_PitchReady() ? ((float)GIMBAL_PITCH_HOME_X10 / 10.0f) : 0.0f;
    s_gimbal.pitch_target_deg = s_gimbal.pitch_deg;
    /* pitch 限位收紧到 home±range (上电归位后即以 home 为中心)。 */
    s_gimbal.pitch_min_deg = GIMBAL_PITCH_HOME_DEG - GIMBAL_PITCH_RANGE_DEG;
    s_gimbal.pitch_max_deg = GIMBAL_PITCH_HOME_DEG + GIMBAL_PITCH_RANGE_DEG;
    s_gimbal.last_pitch_rpm = 0;
    s_gimbal.last_yaw_rpm = 0;
    s_gimbal.last_pitch_angle_x10 = (int32_t)(s_gimbal.pitch_deg * 10.0f);
    s_gimbal.last_yaw_angle_x10 = 0;
    s_gimbal.last_update_ms = BSP_Time_GetMs();
    s_gimbal.pitch_ctrl_ms = s_gimbal.last_update_ms;
    s_gimbal.yaw_enabled = false;
    s_gimbal.yaw_pos_mode = false;
    s_gimbal.initialized = true;

    /*
     * 开机不使能任何一轴:
     *   - yaw 首次收到非零速度指令时惰性使能 (见 Gimbal_SetSpeed);
     *   - pitch 由 Gimbal_StartupElevatePitch() / Gimbal_EnsurePitchReady() 管理。
     */
    return BSP_STATUS_OK;
}

void Gimbal_StartupElevatePitch(void){
    /* 非阻塞: 仅发起 pitch 位置模式抬升, 立即返回, 不阻塞开机菜单。 */
    Gimbal_StartPitchHoming();
}

void Gimbal_EnsurePitchReady(void){
    if (Gimbal_PitchReady()){
        return;
    }
    if (!s_pitch_home_started){
        Gimbal_StartPitchHoming();
    }
    Gimbal_FinishPitchHoming();
}

/* ===== 轴驱动辅助 (把 SetSpeed 的多职责拆成单元, 清晰化胶水逻辑) ===== */

/* yaw: 惰性使能 + deg/s→rpm 下发 + 记录速度。 */
static void Gimbal_DriveYaw(float yaw_deg_s){
    s_gimbal.yaw_deg_s = yaw_deg_s;

    /* yaw 惰性使能: 仅首次非零速度时使能 (避免开机速度环锁 0 抖动), 使能后保持。 */
    if (!s_gimbal.yaw_enabled && (yaw_deg_s != 0.0f)){
        BLDC_Enable(GIMBAL_YAW_ADDR);
        BLDC_SetMode(GIMBAL_YAW_ADDR, MODE_SPEED);
        s_gimbal.yaw_enabled = true;
        s_gimbal.yaw_pos_mode = false;
    } else if (s_gimbal.yaw_enabled && s_gimbal.yaw_pos_mode && (yaw_deg_s != 0.0f)){
        /* 之前用过前馈位置模式, 现在回到速度指令: 切回速度模式。 */
        BLDC_SetMode(GIMBAL_YAW_ADDR, MODE_SPEED);
        s_gimbal.yaw_pos_mode = false;
        s_gimbal.last_yaw_rpm = 0;   /* 失效缓存, 强制重发 */
    }
    if (s_gimbal.yaw_enabled && !s_gimbal.yaw_pos_mode){
        Gimbal_SendAxisRpm(GIMBAL_YAW_ADDR, (int16_t)GIMBAL_DEG_S_TO_RPM(yaw_deg_s),
                           &s_gimbal.last_yaw_rpm);
    }
}

/* yaw: 确保处于多圈位置模式 (前馈瞄准使用), 首次进入时使能并配置运动速度/加速度。 */
static void Gimbal_EnsureYawPositionMode(void){
    if (!s_gimbal.yaw_enabled){
        BLDC_Enable(GIMBAL_YAW_ADDR);
        s_gimbal.yaw_enabled = true;
    }
    if (!s_gimbal.yaw_pos_mode){
        BLDC_SetMode(GIMBAL_YAW_ADDR, MODE_MULTI_POS);
        BLDC_SetSpeed(GIMBAL_YAW_ADDR, GIMBAL_YAW_POS_SPEED);
        BLDC_SetAcc(GIMBAL_YAW_ADDR, GIMBAL_YAW_POS_ACC);
        BLDC_SetAngleLimit(GIMBAL_YAW_ADDR, 0, 0);   /* min==max: 取消限位, 允许多圈 */
        s_gimbal.yaw_pos_mode = true;
    }
}

/* yaw 位置指令去重下发: 仅目标角变化时发送多圈位置。 */
static void Gimbal_SendYawAngle(int32_t angle_x10){
    if (angle_x10 == s_gimbal.last_yaw_angle_x10){
        return;
    }
    s_gimbal.last_yaw_angle_x10 = angle_x10;
    BLDC_SetMultiAngle(GIMBAL_YAW_ADDR, angle_x10);
}

/* pitch 位置指令去重下发: 仅目标角变化时发送多圈位置, 降低 UART3 负载。 */
static void Gimbal_SendPitchAngle(int32_t angle_x10){
    if (angle_x10 == s_gimbal.last_pitch_angle_x10){
        return;
    }
    s_gimbal.last_pitch_angle_x10 = angle_x10;
    BLDC_SetMultiAngle(GIMBAL_PITCH_ADDR, angle_x10);
}

/*
 * pitch 位置模式追踪: 把跟踪环给出的角速度(deg/s)积分成位置设定点, 钳到 [min,max]
 * 后下发多圈位置。位置模式带保持力矩、精确到位, 且受硬件角度限位约束, 天然满足
 * "限位硬优先于追踪", 无需速度模式的最小驱动地板/反馈校正等补丁。未就绪则不下发。
 */
static void Gimbal_DrivePitchTrack(float pitch_deg_s){
    if (!Gimbal_PitchReady()){
        s_gimbal.pitch_deg_s = 0.0f;   /* 仍在归位: 不干扰 */
        return;
    }

    uint32_t now_ms = BSP_Time_GetMs();
    float dt_s = (float)(now_ms - s_gimbal.pitch_ctrl_ms) / 1000.0f;
    s_gimbal.pitch_ctrl_ms = now_ms;
    if (dt_s <= 0.0f){
        dt_s = 0.001f;
    } else if (dt_s > 0.1f){
        dt_s = 0.1f;   /* 限制长间隔(如刚进入)导致的设定点跳变 */
    }

    s_gimbal.pitch_deg_s = pitch_deg_s;
    s_gimbal.pitch_target_deg =
        Gimbal_ClampPitch(s_gimbal.pitch_target_deg + pitch_deg_s * dt_s);
    s_gimbal.pitch_deg = s_gimbal.pitch_target_deg;
    Gimbal_SendPitchAngle((int32_t)(s_gimbal.pitch_target_deg * 10.0f));
}

/* pitch 丢失目标: 停在当前位置 (位置模式自带保持力矩, 维持丢失瞬间的设定点即可)。 */
static void Gimbal_DrivePitchHold(void){
    if (!Gimbal_PitchReady()){
        return;
    }
    s_gimbal.pitch_deg_s = 0.0f;
    s_gimbal.pitch_ctrl_ms = BSP_Time_GetMs();   /* 重置 dt 基准, 避免恢复时跳变 */
    /* 不改 pitch_target_deg / 不重发: 电机保持在丢失瞬间的设定点。 */
}

/* ===== 对外速度/停止/归位接口 (薄封装, 编排上面的辅助) ===== */

BSP_STATUS Gimbal_SetSpeed(float yaw_deg_s, float pitch_deg_s){
    /* yaw 速度模式; pitch 位置模式(把角速度积分成位置设定点)。 */
    (void)Gimbal_Update();
    Gimbal_DriveYaw(yaw_deg_s);
    Gimbal_DrivePitchTrack(pitch_deg_s);
    return BSP_STATUS_OK;
}

/* pitch 绝对位置指令: 直接把设定点置为目标角 (钳位后下发), 供前馈瞄准使用。 */
static void Gimbal_DrivePitchAbsolute(float pitch_deg){
    if (!Gimbal_PitchReady()){
        return;
    }
    s_gimbal.pitch_deg_s = 0.0f;
    s_gimbal.pitch_target_deg = Gimbal_ClampPitch(pitch_deg);
    s_gimbal.pitch_deg = s_gimbal.pitch_target_deg;
    s_gimbal.pitch_ctrl_ms = BSP_Time_GetMs();   /* 重置积分基准, 避免混用时跳变 */
    Gimbal_SendPitchAngle((int32_t)(s_gimbal.pitch_target_deg * 10.0f));
}

BSP_STATUS Gimbal_SetAngle(float yaw_deg, float pitch_deg){
    /*
     * 前馈瞄准接口: 两轴均按【绝对目标角】走位置模式。
     *   - yaw: 多圈位置模式, 目标角来自 aim_solver 的几何解算 (车体系);
     *   - pitch: 位置模式, 直接置设定点。
     * 与 Gimbal_SetSpeed 互斥使用: 一个任务内应固定用其一, 需切换时先 Stop。
     * 用于运动中连续瞄准与画圆: 前馈连续、零滞后、不依赖视觉识别激光光斑。
     */
    Gimbal_EnsureYawPositionMode();
    Gimbal_SendYawAngle((int32_t)(yaw_deg * 10.0f));
    s_gimbal.yaw_deg = yaw_deg;          /* 位置模式: 估计角直接跟踪设定点 */
    s_gimbal.yaw_deg_s = 0.0f;

    Gimbal_DrivePitchAbsolute(pitch_deg);
    return BSP_STATUS_OK;
}

BSP_STATUS Gimbal_Stop(void){
    /* yaw 停转; pitch 位置模式自带保持力矩, 维持当前设定点即可, 不额外下发。 */
    Gimbal_DriveYaw(0.0f);
    s_gimbal.pitch_deg_s = 0.0f;
    s_gimbal.pitch_ctrl_ms = BSP_Time_GetMs();   /* 重置 dt 基准, 避免恢复时跳变 */
    return BSP_STATUS_OK;
}

BSP_STATUS Gimbal_HoldOnTargetLost(void){
    /* 丢失目标: yaw 停转; pitch 停在当前位置并保持 (未就绪则整体停)。 */
    (void)Gimbal_Update();
    if (!Gimbal_PitchReady()){
        return Gimbal_Stop();
    }
    Gimbal_DrivePitchHold();
    return BSP_STATUS_OK;
}

BSP_STATUS Gimbal_Update(void){
    uint32_t now_ms = BSP_Time_GetMs();
    float dt_s = (float)(now_ms - s_gimbal.last_update_ms) / 1000.0f;

    s_gimbal.last_update_ms = now_ms;

    if (dt_s <= 0.0f){
        return BSP_STATUS_OK;
    }
    s_gimbal.yaw_deg += s_gimbal.yaw_deg_s * dt_s;
    /* pitch 由位置设定点驱动 (见 Gimbal_DrivePitchTrack), 不在此开环积分。 */

    return BSP_STATUS_OK;
}

BSP_STATUS Gimbal_GetStatus(GIMBAL_STATUS *out){
    if (out == NULL){
        return BSP_STATUS_NULL;
    }

    out->angle = Gimbal_GetAngle();
    out->speed = Gimbal_GetSpeed();
    out->limit = Gimbal_GetLimit();
    return BSP_STATUS_OK;
}

GIMBAL_ANGLE Gimbal_GetAngle(void){
    GIMBAL_ANGLE angle = {
        .yaw_deg = s_gimbal.yaw_deg,
        .pitch_deg = s_gimbal.pitch_deg,
    };

    return angle;
}

GIMBAL_SPEED Gimbal_GetSpeed(void){
    GIMBAL_SPEED speed = {
        .yaw_deg_s = s_gimbal.yaw_deg_s,
        .pitch_deg_s = s_gimbal.pitch_deg_s,
    };

    return speed;
}

void Gimbal_ResetPosition(void){
    s_gimbal.yaw_deg = 0.0f;
    s_gimbal.pitch_deg = 0.0f;
}

void Gimbal_ResetAxisPosition(GIMBAL_AXIS axis){
    if (!Gimbal_IsValidAxis(axis)){
        return;
    }

    if (axis == GIMBAL_AXIS_YAW){
        s_gimbal.yaw_deg = 0.0f;
    } else{
        s_gimbal.pitch_deg = 0.0f;
    }
}

BSP_STATUS Gimbal_SetLimit(const GIMBAL_LIMIT *limit){
    if (limit == NULL){
        return BSP_STATUS_NULL;
    }

    if (limit->pitch_min_deg > limit->pitch_max_deg){
        return BSP_STATUS_INVALID_ARG;
    }

    s_gimbal.pitch_min_deg = limit->pitch_min_deg;
    s_gimbal.pitch_max_deg = limit->pitch_max_deg;
    s_gimbal.pitch_deg = Gimbal_ClampPitch(s_gimbal.pitch_deg);

    return BSP_STATUS_OK;
}

GIMBAL_LIMIT Gimbal_GetLimit(void){
    GIMBAL_LIMIT limit = {
        .pitch_min_deg = s_gimbal.pitch_min_deg,
        .pitch_max_deg = s_gimbal.pitch_max_deg,
    };

    return limit;
}
