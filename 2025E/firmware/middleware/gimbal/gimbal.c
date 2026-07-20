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
 * yaw 电机物理转向适配 (前馈瞄准 Gimbal_SetAngle 专用):
 *   +1 = 电机正转方向与几何 yaw 约定 (逆时针为正) 一致;
 *   -1 = 相反 (实测本机为相反, 故默认 -1)。
 * 逻辑 yaw 约定与开环估计角保持不变, 仅翻转下发给 F32C 的物理转向。
 * 视觉跟踪方向由 gimbal_tracking 的 yaw_output_sign 独立整定, 不受此影响。
 */
#ifndef GIMBAL_YAW_DIR
#define GIMBAL_YAW_DIR (-1.0f)
#endif

/* ===== yaw 速度前馈: MCU 位置外环 + F32C 速度内环 (Gimbal_SetAngle 唯一执行路径) =====
 * ω_cmd = 解析速率前馈(上层运动学闭式解, 经参数传入) + KP·(目标角 − 预估反馈角)。
 *   - 解析前馈替代命令求导: 无差分噪声, 免疫角点 snap 命令跳变(跳变改由 P 项平滑吸收);
 *   - 反馈一步预估: multi_angle 是上拍请求的(滞后~1环), 用上拍下发 ω 外推到当前,
 *     消转弯时 ~4° 的陈旧假误差, 为抬 KP 解锁;
 *   - ΔΣ 量化整形: 1RPM=6°/s 太粗(KP=4 下等效 ±1.5° 位置死区), 舍入误差累进下一拍,
 *     ~30Hz 交替下发相邻 RPM, 平均速度分辨率 ~0.2°/s。
 * yaw 前馈已由此内建, 故 auto_aim 不再另加 yaw_rate_lead。
 * ⚠ 速度模式无位置保持, MCU 若停更会失控空转(滑环下不撞机械, 但激光乱扫); 任务正常退出
 *   由 Gimbal_Stop 发 0 停住。 */
#ifndef GIMBAL_YAW_VELFF_KP
#define GIMBAL_YAW_VELFF_KP 7.0f            /* 位置外环增益 (deg/s per deg); 6→7 压出弯执行滞后(act-cmd 尖峰达 6~11°); 可试 6~8 */
#endif
#ifndef GIMBAL_YAW_RATE_FF_GAIN
/*
 * 解析速率前馈增益：补偿 F32C 速度链约 25~30ms 的等效执行延迟。
 * 实测高速段(60~120deg/s)平均滞后约 2.6deg，1.15 可在不提高位置 Kp 的情况下
 * 增强转弯动态响应；若弯后超前/回摆降到 1.10，仍滞后可逐步试到 1.20。
 */
#define GIMBAL_YAW_RATE_FF_GAIN 1.10f
#endif
#ifndef GIMBAL_YAW_VELFF_MAX_DEG_S
#define GIMBAL_YAW_VELFF_MAX_DEG_S 300.0f  /* ω 指令限幅, 兼作初始阶跃/失控缓冲 */
#endif
/* 链路自愈周期: 周期性重发使能+速度模式+当前 RPM。使能/模式/速度帧若在上电初期丢失,
 * 去重机制下饱和恒定的 RPM 永不重发 → 云台冻结(实测刷机后首跑冻到第一个弯才恢复)。
 * 自愈把丢帧后果从"永久冻结"压到"最多一个周期"。总线占用 ~6 帧/s, 可忽略。 */
#ifndef GIMBAL_YAW_LINK_REFRESH_MS
#define GIMBAL_YAW_LINK_REFRESH_MS 500U
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

/* 归位后 pitch 限位收紧到 home±range (归位阶段仍用全量程从 0° travel 到 home)。
 * 视觉追踪时目标竖直偏差随车-靶距离变化可超 ±15°, 窗口太窄会顶限位把 pitch 钳死。
 * 放宽到 ±30° -> [120,180] 覆盖更大俯仰。硬件极限 [0,180]: home(150)+30=180 正好到上限,
 * 这是以 home 为中心【对称】放宽的最大值。若还需更大【俯角】(向 <120° 方向), 需改用
 * 非对称限位(下压到如 90°, 上仍 ≤180°), 而不是继续加大对称 range(会超硬件上限)。 */
#define GIMBAL_PITCH_RANGE_X10        300     /* ±30° -> [120,180] */
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
    uint32_t last_update_ms;
    uint32_t pitch_ctrl_ms;        /* pitch 设定点积分的上次时间戳 */
    bool yaw_enabled;      /* yaw 惰性使能标志 */
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

/*
 * yaw 连续解缠累计命令角 (逻辑值, 可多圈): 不被 Gimbal_Init 重置, 始终跟踪从上电起
 * 真实下发的多圈位置。滑环支持多圈旋转, 靶转到车后方(过 ±180)时命令就近累积、
 * 不发生 360° 突跳, 云台平滑跟随。
 */
static float s_yaw_cmd_unwrapped_deg;

/* yaw 速度前馈状态: 速度模式已配置标志、时间戳、上拍下发的逻辑 ω(反馈一步预估用)、
 * ΔΣ 量化整形的舍入误差累计、链路自愈时间戳、上电就近对齐待执行标志。
 * 均属 yaw 生命周期状态, 不被 Gimbal_Init 重置。 */
static bool s_yaw_speed_mode_set;
static uint32_t s_yaw_velff_ms;
static float s_yaw_velff_prev_omega_deg_s;
static float s_yaw_velff_rpm_err;
static uint32_t s_yaw_link_refresh_ms;
static bool s_yaw_align_pending;

/* 归一化到 [-180,180), 处理任意范围输入 (供 yaw 解缠求就近增量)。 */
static float Gimbal_NormalizeDeg(float angle){
    while (angle >= 180.0f){ angle -= 360.0f; }
    while (angle < -180.0f){ angle += 360.0f; }
    return angle;
}

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

/* yaw 对应的反馈数据结构体 (速度前馈位置外环读多圈角用)。 */
static volatile BLDC_MotorData_t *Gimbal_YawMotor(void){
    return (GIMBAL_YAW_ADDR == BLDC_ADDR_1) ? &BLDC_Motor1 : &BLDC_Motor2;
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
    s_gimbal.last_update_ms = BSP_Time_GetMs();
    s_gimbal.pitch_ctrl_ms = s_gimbal.last_update_ms;
    s_gimbal.yaw_enabled = false;
    s_gimbal.initialized = true;

    /*
     * 开机不使能任何一轴:
     *   - yaw 首次收到非零速度/前馈指令时惰性使能;
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

/* yaw: 惰性使能 + deg/s→rpm 下发 + 记录速度 (速度指令 Gimbal_SetSpeed 用)。 */
static void Gimbal_DriveYaw(float yaw_deg_s){
    s_gimbal.yaw_deg_s = yaw_deg_s;

    /* yaw 惰性使能: 仅首次非零速度时使能 (避免开机速度环锁 0 抖动), 使能后保持。 */
    if (!s_gimbal.yaw_enabled && (yaw_deg_s != 0.0f)){
        BLDC_Enable(GIMBAL_YAW_ADDR);
        BLDC_SetMode(GIMBAL_YAW_ADDR, MODE_SPEED);
        s_gimbal.yaw_enabled = true;
    }
    if (s_gimbal.yaw_enabled){
        Gimbal_SendAxisRpm(GIMBAL_YAW_ADDR, (int16_t)GIMBAL_DEG_S_TO_RPM(yaw_deg_s),
                           &s_gimbal.last_yaw_rpm);
    }
}

/* yaw: 确保处于速度模式 (速度前馈使用), 首次进入时使能并配置。 */
static void Gimbal_EnsureYawSpeedMode(void){
    if (!s_gimbal.yaw_enabled){
        BLDC_Enable(GIMBAL_YAW_ADDR);
        s_gimbal.yaw_enabled = true;
    }
    if (!s_yaw_speed_mode_set){
        BLDC_SetMode(GIMBAL_YAW_ADDR, MODE_SPEED);
        s_yaw_speed_mode_set = true;
        s_gimbal.last_yaw_rpm = 0;   /* 失效缓存, 强制重发 */
    }
}

/*
 * yaw 速度前馈级联 (Gimbal_SetAngle 的 yaw 执行):
 *   ω_cmd = 解析速率前馈(参数传入) + Kp·(目标角 − 一步预估反馈角);
 *   限幅后经 ΔΣ 量化整形按转速下发 F32C 速度环。目标角做滑环解缠。
 */
static void Gimbal_DriveYawVelFF(float yaw_deg, float yaw_rate_ff_deg_s){
    Gimbal_EnsureYawSpeedMode();

    /* 目标角解缠(滑环多圈)。 */
    s_yaw_cmd_unwrapped_deg += Gimbal_NormalizeDeg(yaw_deg - s_yaw_cmd_unwrapped_deg);
    float yaw_target_deg = s_yaw_cmd_unwrapped_deg;

    uint32_t now_ms = BSP_Time_GetMs();
    float dt_s = (float)(now_ms - s_yaw_velff_ms) / 1000.0f;
    s_yaw_velff_ms = now_ms;
    if ((dt_s <= 0.0f) || (dt_s > 0.1f)){
        /* 首拍/长间隔: 预估基准与量化累计失效, 复位; 命令角待就近对齐反馈。 */
        dt_s = 0.03f;
        s_yaw_velff_prev_omega_deg_s = 0.0f;
        s_yaw_velff_rpm_err = 0.0f;
        s_yaw_align_pending = true;
    }

    /* 链路自愈: 周期性重发使能+速度模式, 并强制重发当前 RPM(失效去重缓存)。
     * 防止上电初期丢帧 + ω 饱和恒定 RPM 被去重卡死 → 云台永久冻结。 */
    if ((uint32_t)(now_ms - s_yaw_link_refresh_ms) >= GIMBAL_YAW_LINK_REFRESH_MS){
        s_yaw_link_refresh_ms = now_ms;
        BLDC_Enable(GIMBAL_YAW_ADDR);
        BLDC_SetMode(GIMBAL_YAW_ADDR, MODE_SPEED);
        s_gimbal.last_yaw_rpm = INT16_MIN;   /* 哨兵: 本拍必重发速度 */
    }

    /* 位置外环反馈一步预估: multi_angle 是上拍请求的(滞后~1环), 用上拍下发的 ω
     * 外推到当前, 消运动中的陈旧假误差。随后请求下拍反馈。 */
    float yaw_fb_deg = GIMBAL_YAW_DIR * (float)Gimbal_YawMotor()->multi_angle / 10.0f;
    BLDC_ReqFeedback(GIMBAL_YAW_ADDR, FB_MULTI_ANGLE);

    /* 上电/长间隔就近对齐: 电机可能带着上一会话的多圈角(独立供电不掉电), 而 MCU
     * 解缠基准已清零 → 位置误差可达数百度, 会整圈扫掠。收到首个有效反馈后把解缠
     * 命令角对齐到反馈的最短路一侧, 初始误差 ≤180°(滑环允许任意圈数, 对齐无代价)。 */
    if (s_yaw_align_pending && (Gimbal_YawMotor()->data_ready != 0U)){
        s_yaw_align_pending = false;
        s_yaw_cmd_unwrapped_deg = yaw_fb_deg +
            Gimbal_NormalizeDeg(yaw_target_deg - yaw_fb_deg);
        yaw_target_deg = s_yaw_cmd_unwrapped_deg;
    }
    float yaw_est_deg = yaw_fb_deg + s_yaw_velff_prev_omega_deg_s * dt_s;

    float omega = GIMBAL_YAW_RATE_FF_GAIN * yaw_rate_ff_deg_s +
                  GIMBAL_YAW_VELFF_KP * (yaw_target_deg - yaw_est_deg);
    if (omega > GIMBAL_YAW_VELFF_MAX_DEG_S){
        omega = GIMBAL_YAW_VELFF_MAX_DEG_S;
    } else if (omega < -GIMBAL_YAW_VELFF_MAX_DEG_S){
        omega = -GIMBAL_YAW_VELFF_MAX_DEG_S;
    }
    s_yaw_velff_prev_omega_deg_s = omega;

    /* 逻辑角速度 -> 物理转速(RPM), 经 GIMBAL_YAW_DIR 适配; ΔΣ 量化整形:
     * 舍入误差累进下一拍, 相邻 RPM 交替使平均转速达到亚 RPM 分辨率。 */
    float rpm_f = GIMBAL_YAW_DIR * GIMBAL_DEG_S_TO_RPM(omega) + s_yaw_velff_rpm_err;
    int16_t rpm = (int16_t)(rpm_f >= 0.0f ? rpm_f + 0.5f : rpm_f - 0.5f);
    s_yaw_velff_rpm_err = rpm_f - (float)rpm;
    Gimbal_SendAxisRpm(GIMBAL_YAW_ADDR, rpm, &s_gimbal.last_yaw_rpm);

    s_gimbal.yaw_deg = yaw_deg;     /* 逻辑估计角(归一化, 供显示) */
    s_gimbal.yaw_deg_s = omega;
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

BSP_STATUS Gimbal_SetAngle(float yaw_deg, float pitch_deg, float yaw_rate_ff_deg_s){
    /*
     * 前馈瞄准接口 (运动中连续瞄准/画圆, 前馈连续零滞后, 不依赖视觉识别光斑):
     *   - yaw: 速度前馈级联 (解析速率前馈 + MCU 位置外环 + F32C 速度内环);
     *   - pitch: 位置模式, 直接置绝对设定点。
     * 与 Gimbal_SetSpeed 互斥使用: 一个任务内应固定用其一, 需切换时先 Stop。
     */
    Gimbal_DriveYawVelFF(yaw_deg, yaw_rate_ff_deg_s);
    Gimbal_DrivePitchAbsolute(pitch_deg);
    return BSP_STATUS_OK;
}

BSP_STATUS Gimbal_SetPitchDeg(float pitch_deg){
    /* 只置 pitch 绝对目标角(位置模式, 受软件限位), 不动 yaw。供 F1 进入抬升相机看靶等用;
     * 之后 GimbalTracking 无目标时保持该角, 锁定后经视觉 servo 回靶心。 */
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
    /* yaw 必须显式发 0: 速度模式下不发就保持最后 RPM 永转
     * (实测: 低帧率视觉丢失后云台单向不停旋转的根因)。 */
    Gimbal_DriveYaw(0.0f);
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

int16_t Gimbal_GetYawCommandRpm(void){
    return s_gimbal.last_yaw_rpm;
}

float Gimbal_ReadYawFeedbackDeg(void){
    volatile BLDC_MotorData_t *yaw = Gimbal_YawMotor();
    uint32_t start_count = yaw->multi_angle_frame_count;
    uint32_t start_ms = BSP_Time_GetMs();

    /* 请求并等待"新"反馈帧 (按帧计数判新, 避免读到陈旧角); 典型 2~5ms 到达。 */
    BLDC_ReqFeedback(GIMBAL_YAW_ADDR, FB_MULTI_ANGLE);
    while ((yaw->multi_angle_frame_count == start_count) &&
           ((BSP_Time_GetMs() - start_ms) < 100U)){
        BSP_DelayMs(5U);
    }
    return GIMBAL_YAW_DIR * (float)yaw->multi_angle / 10.0f;
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
