/**
 * @file  app_checks.c
 * @brief 外设自检任务实现：姿态、灰度、驱动与编码器诊断。
 */
#include "app_checks.h"
#include "app_ball_scurve_task.h"
#include "app_fmt.h"

#include "ui.h"
#include "key.h"
#include "bsp_time.h"
#include "bsp_common.h"
#include "debug_uart.h"
#include "rpi_uart.h"
#include "chassis.h"
#include "yahboom_track.h"
#include "hall_encoder.h"
#include "step_motor.h"
#include "wit_sdk.h"
#include "kinematics/kinematics.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 自检刷屏节流周期。 */
#define CHK_UI_PERIOD_MS 200U
#define JY61P_G_TO_M_S2  9.80665f

/* 把字符串拷入 buf，返回长度（不终止），便于随后接 AppFmt_* 拼数字。 */
static uint8_t PutStr(char *buf, const char *s){
    uint8_t i = 0U;
    while (s[i] != '\0'){
        buf[i] = s[i];
        i++;
    }
    return i;
}

/* ============================ 灰度 I2C（Yahboom，I2C0） ============================ */
/* 与 JY61P 共 I2C0：进入时挂起 JY61P、退出时恢复。 */

static uint32_t gi_last_ui;
static uint32_t gi_ok_cnt;    /* 累计读取成功次数。 */
static uint32_t gi_err_cnt;   /* 累计读取失败次数。 */

static void ChkGrayI2c_Enter(void){
    JY61P_I2C_SetSuspended(true);    /* 让出 I2C0 */
    (void)YahboomTrack_Init();       /* 探测在线；同时清零驱动内部诊断计数 */
    gi_ok_cnt     = 0U;
    gi_err_cnt    = 0U;
    gi_last_ui    = 0U;
}

static APP_TASK_STATUS ChkGrayI2c_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();
    if ((now - gi_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    gi_last_ui = now;

    uint8_t mask = 0U;
    BSP_STATUS st = YahboomTrack_ReadDetectedMask(&mask);
    if (st == BSP_STATUS_OK){ gi_ok_cnt++; } else { gi_err_cnt++; }

    char bits[YAHBOOM_TRACK_CHANNEL_COUNT + 1U];
    uint8_t active = 0U;
    for (uint8_t i = 0U; i < YAHBOOM_TRACK_CHANNEL_COUNT; i++){
        bool on = ((mask & (uint8_t)(1U << i)) != 0U);   /* bit0=X1 */
        bits[i] = on ? '1' : '0';
        if (on){
            active++;
        }
    }
    bits[YAHBOOM_TRACK_CHANNEL_COUNT] = '\0';

    char l2[16];
    char l3[16];
    char l4[16];
    uint8_t n;

    if (st == BSP_STATUS_OK){
        n = PutStr(l2, "act ");
        AppFmt_I32(&l2[n], (int32_t)active);
    } else {
        (void)PutStr(l2, "READ FAIL");
        l2[9] = '\0';
    }

    /* 累计成功/失败次数：便于判断是偶发瞬态（err 少）还是持续故障（err 涨得快）。 */
    n = PutStr(l3, "ok ");
    AppFmt_I32(&l3[n], (int32_t)gi_ok_cnt);
    while (l3[n] != '\0'){ n++; }
    n += PutStr(&l3[n], " er ");
    AppFmt_I32(&l3[n], (int32_t)gi_err_cnt);

    /* 诊断：W=写寄存器累计失败 R=读寄存器累计失败 s=最近失败码(超时=-4 / NACK=-1)。 */
    uint32_t wr_fail = 0U;
    uint32_t rd_fail = 0U;
    int32_t  last_status = 0;
    YahboomTrack_GetDiag(&rd_fail, &wr_fail, &last_status);   /* 注意形参序为 read, write */
    n = PutStr(l4, "W");
    AppFmt_I32(&l4[n], (int32_t)wr_fail);
    while (l4[n] != '\0'){ n++; }
    n += PutStr(&l4[n], " R");
    AppFmt_I32(&l4[n], (int32_t)rd_fail);
    while (l4[n] != '\0'){ n++; }
    n += PutStr(&l4[n], " s");
    AppFmt_I32(&l4[n], last_status);

    Ui_RenderLines("Chk Gray I2C", bits, l2, l3, l4, "BACK: exit", NULL);
    return APP_TASK_RUNNING;
}

static void ChkGrayI2c_Exit(void){
    JY61P_I2C_SetSuspended(false);   /* 归还 I2C0 给 JY61P */
}

/* ============================ 陀螺仪 JY61P ============================ */

static uint32_t gj_last_ui;

static void ChkGyroJy_Enter(void){
    JY61P_I2C_SetSuspended(false);   /* 确保 JY61P 占用 I2C0 */
    JY61P_I2C_Init();
    gj_last_ui = 0U;
}

static APP_TASK_STATUS ChkGyroJy_Tick(float dt){
    (void)dt;
    JY61P_I2C_Poll();                /* 每控制周期推进 I2C 状态机 */

    uint32_t now = BSP_Time_GetMs();
    if ((now - gj_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    gj_last_ui = now;

    char l1[20];
    char l2[20];
    char l3[20];
    char l4[20];
    uint8_t n;

    WIT_IMU_DATA imu;
    if (WitGetData(&imu) == WIT_HAL_OK){
        n = PutStr(l1, "Gz ");
        AppFmt_Fixed(&l1[n], imu.gyro_deg_s.z, 1);
        n = PutStr(l2, "Yaw ");
        AppFmt_Fixed(&l2[n], imu.attitude_deg.yaw, 1);
        /* H3 longitudinal direction: JY61P Y axis; sensor-positive points rearward. */
        n = PutStr(l3, "Ay m/s2 ");
        AppFmt_Fixed(&l3[n], imu.acc_g.y * JY61P_G_TO_M_S2, 2);
    } else {
        (void)PutStr(l1, "Gz --"); l1[5] = '\0';
        (void)PutStr(l2, "Yaw --"); l2[6] = '\0';
        (void)PutStr(l3, "Ay m/s2 --"); l3[10] = '\0';
    }
    n = PutStr(l4, "err ");
    AppFmt_I32(&l4[n], (int32_t)JY61P_I2C_GetErrorCount());

    Ui_RenderLines("Chk Gyro JY61P", l1, l2, l3, l4, "BACK: exit", NULL);
    return APP_TASK_RUNNING;
}

/* ============================ 加速度校准 JY61P ============================ */

/*
 * 软件零偏校准：静置采集加速度样本，取均值作为 Ax/Ay 零偏，Az 保留符号归一。
 *
 * 为什么不直接用 JY61P 内置 CALGYROACC：传感器安装方向不确定（Z 轴可能朝下），
 * 内置校准假定传感器水平朝上（Z=+1g），校准后 Az 读数不对。软件侧只关心
 * "静止时 Ax≈0、Ay≈0"，不依赖传感器自身的方向假设。
 */
#define ACC_ZERO_COLLECT_MS  3000U    /* 采集窗口，ms */
#define ACC_ZERO_MIN_SAMPLES   20U    /* 最少有效样本数，不足则放弃 */

static uint32_t ga_last_ui;
static uint32_t ga_collect_start_ms;
static bool     ga_collecting;
static float    ga_sum_x, ga_sum_y, ga_sum_z;
static uint32_t ga_sample_count;

static void ChkAccCal_Enter(void){
    JY61P_I2C_SetSuspended(false);
    JY61P_I2C_Init();
    ga_last_ui = 0U;
    ga_collecting = false;
    ga_collect_start_ms = 0U;
    ga_sum_x = ga_sum_y = ga_sum_z = 0.0f;
    ga_sample_count = 0U;
}

static APP_TASK_STATUS ChkAccCal_Tick(float dt){
    (void)dt;
    JY61P_I2C_Poll();

    uint32_t now = BSP_Time_GetMs();

    JY61P_I2C_SAMPLE sample;
    bool has_data = JY61P_I2C_GetSnapshot(&sample);

    /* 采集中：每个新样本累加。不在 UI 节流内做，每拍都可能来新样本。 */
    if (ga_collecting && has_data){
        ga_sum_x += sample.data.acc_g.x;
        ga_sum_y += sample.data.acc_g.y;
        ga_sum_z += sample.data.acc_g.z;
        ga_sample_count++;

        uint32_t elapsed = now - ga_collect_start_ms;
        if (elapsed >= ACC_ZERO_COLLECT_MS){
            ga_collecting = false;
            if (ga_sample_count >= ACC_ZERO_MIN_SAMPLES){
                float avg_x = ga_sum_x / (float)ga_sample_count;
                float avg_y = ga_sum_y / (float)ga_sample_count;
                float avg_z = ga_sum_z / (float)ga_sample_count;
                /* Ax/Ay → 0；Az 保留方向只去重力分量 (sign * 1g) */
                float sign_z = (avg_z >= 0.0f) ? 1.0f : -1.0f;
                JY61P_I2C_SetAccelOffset(avg_x, avg_y, avg_z - sign_z);
            }
        }
    }

    /* UI 节流 */
    if ((now - ga_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    ga_last_ui = now;

    /* 采集中：显示进度 */
    if (ga_collecting){
        uint32_t elapsed = now - ga_collect_start_ms;
        uint32_t remaining = (elapsed >= ACC_ZERO_COLLECT_MS)
            ? 0U : (ACC_ZERO_COLLECT_MS - elapsed);
        char l1[20]; char l2[20]; char l3[20]; char l4[20]; char l5[20];
        (void)PutStr(l1, "ZEROING...");
        (void)PutStr(l2, "Stay STILL at");
        (void)PutStr(l3, "work orientation");
        uint8_t n = PutStr(l4, "n ");
        AppFmt_I32(&l4[n], (int32_t)ga_sample_count);
        n += PutStr(&l4[n], " wait ");
        AppFmt_Fixed(&l4[n], (float)remaining * 0.001f, 1);
        n += PutStr(&l4[n], "s");
        (void)PutStr(l5, "DO NOT MOVE!");
        Ui_RenderLines("Acc Calib", l1, l2, l3, l4, l5, NULL);
        return APP_TASK_RUNNING;
    }

    /* 正常显示：已校准的加速度值 + 当前偏移量 */
    char l1[20]; char l2[20]; char l3[20]; char l4[20]; char l5[20];
    uint8_t n;

    if (has_data){
        n = PutStr(l1, "Ax "); AppFmt_Fixed(&l1[n], sample.data.acc_g.x, 3);
        n = PutStr(l2, "Ay "); AppFmt_Fixed(&l2[n], sample.data.acc_g.y, 3);
        n = PutStr(l3, "Az "); AppFmt_Fixed(&l3[n], sample.data.acc_g.z, 3);
        float mod = sqrtf(sample.data.acc_g.x * sample.data.acc_g.x +
                          sample.data.acc_g.y * sample.data.acc_g.y +
                          sample.data.acc_g.z * sample.data.acc_g.z);
        n = PutStr(l4, "mod "); AppFmt_Fixed(&l4[n], mod, 3);
    } else{
        (void)PutStr(l1, "Ax --");
        (void)PutStr(l2, "Ay --");
        (void)PutStr(l3, "Az --");
        (void)PutStr(l4, "mod --");
    }
    float ox, oy, oz;
    JY61P_I2C_GetAccelOffset(&ox, &oy, &oz);
    n = PutStr(l5, "off ");
    AppFmt_Fixed(&l5[n], ox, 3);
    n += PutStr(&l5[n], "/");
    AppFmt_Fixed(&l5[n], oy, 3);
    l5[n] = '\0';

    Ui_RenderLines("Acc Calib", l1, l2, l3, l4, l5,
                   "ENTER:zero  BACK");

    /* ENTER 短按启动软件零偏采集 */
    if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
        ga_collecting = true;
        ga_collect_start_ms = BSP_Time_GetMs();
        ga_sum_x = ga_sum_y = ga_sum_z = 0.0f;
        ga_sample_count = 0U;
    }

    return APP_TASK_RUNNING;
}

/* ============================ TB6612（主动） ============================ */
/*
 * 安全策略：默认停机；每次短按发一个有界低占空比脉冲(20%/300ms 后自动 Brake)，
 * 验证各通道+方向。UP=左轮 DOWN=右轮 ENTER=双轮；BACK 由框架处理并退出。
 * 屏显编码器计数变化，顺带验证电机↔编码器接线。全程提示抬起车轮。
 */
#define TB_PULSE_DUTY 20.0f
#define TB_PULSE_MS   300U

static uint32_t tb_pulse_end;
static bool     tb_active;

static void ChkTb_Render(const char *action){
    char cntL[16];
    char cntR[16];
    uint8_t n = PutStr(cntL, "encL ");
    AppFmt_I32(&cntL[n], HallEncoder_GetCount(HALL_ENCODER_LEFT));
    n = PutStr(cntR, "encR ");
    AppFmt_I32(&cntR[n], HallEncoder_GetCount(HALL_ENCODER_RIGHT));
    Ui_RenderLines("Chk TB6612", "!! WHEELS UP !!", action, cntL,
                   cntR, "UP/DN/EN pulse", "BACK: exit");
}

static void ChkTb_Enter(void){
    (void)Chassis_Brake();
    tb_active = false;
    tb_pulse_end = 0U;
    ChkTb_Render("idle");
}

static APP_TASK_STATUS ChkTb_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();

    /* 结束到期的脉冲。 */
    if (tb_active && ((int32_t)(now - tb_pulse_end) >= 0)){
        (void)Chassis_Brake();
        tb_active = false;
        ChkTb_Render("idle");
    }

    if (!tb_active){
        const char *action = NULL;
        if (Key_GetEvent(KEY_ID_UP) == KEY_EVENT_SHORT_PRESS){
            (void)Chassis_SetDuty(TB_PULSE_DUTY, 0.0f);
            action = "L pulse";
        } else if (Key_GetEvent(KEY_ID_DOWN) == KEY_EVENT_SHORT_PRESS){
            (void)Chassis_SetDuty(0.0f, TB_PULSE_DUTY);
            action = "R pulse";
        } else if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
            (void)Chassis_SetDuty(TB_PULSE_DUTY, TB_PULSE_DUTY);
            action = "L+R pulse";
        }

        if (action != NULL){
            tb_active = true;
            tb_pulse_end = now + TB_PULSE_MS;
            ChkTb_Render(action);
        }
    }

    return APP_TASK_RUNNING;
}

static void ChkTb_Exit(void){
    (void)Chassis_Brake();
}

/* ======================= 摆杆步进电机（主动） ======================= */
/*
 * 七个模式，ENTER 长按循环切换，BACK 退出整页。**进入本页落在 JOG**——点动是最常用
 * 的操作；LEVEL 用于本次上电的动力学水平点捕获，其余五个是驱动标定模式，
 * 每个只覆盖 docs/step-motor-calibration.md 里的一到两项，
 * 并把该填回 step_motor.h 的那个数直接算好显示出来，免得在现场心算。
 *
 *  1/7 JOG    点动对位。UP/DOWN **单击**各走一步，**长按**调步长（四档 1/5/10/20 计数）；
 *             屏上给编码器计数、步长（计数）与步长对应的轴角。
 *             用来精确找位置，例如实测 STEP_MOTOR_ENC_LEVEL_COUNTS（水管水平位）。
 *  2/7 LEVEL  与 JOG 相同方式点动水管；ENTER 短按把当前位置经 H3 连杆查表换算为
 *             level_bias_deg，长按跳过/离开标定并进入 RUN。只改 RAM，不清编码器、不写 Flash。
 *  3/7 RUN    持续正/反转（目标设到远处，被限位夹在边界上），ENTER 暂停。
 *             看 cnt 的增减方向、err 是否收敛、g 行的守护状态。
 *             → DIR 极性 STEP_MOTOR_BEAM_POSITIVE_DIR_HIGH（按 UP 看摆杆倒向）
 *             → 编码器方向 STEP_MOTOR_ENCODER_INVERT（按 UP 时 cnt 应增大）
 *  4/7 TURN   走 N 整圈后自动停。走完看轴上标记实际转过几圈：
 *             → 细分数 STEP_MOTOR_MICROSTEP = 当前值 / 实际圈数
 *             ⚠ 装机后行程通常不足一圈，目标会被限位夹住（l4 显示 CLAMPED），
 *               这项标定得先拆下摆杆，或临时把 ENC_LIMIT_ENABLED 设 0 重编译。
 *  5/7 HAND   进入即断电，轴可手转；手转 N 圈后 cpr 栏就是实测每转计数。
 *             → STEP_MOTOR_ENCODER_COUNTS_PER_REV
 *             进本模式轴应能拧动、回 RUN/TURN 应拧不动，据此定
 *             → STEP_MOTOR_BEAM_ENABLE_HIGH
 *  6/7 SWEEP  在软限位内**往复**跑，转速逐档上扫并记录 |err| 峰值。
 *             不丢步时 err 稳定在跟踪误差附近，峰值明显跳高即已丢步，退一档：
 *             → STEP_MOTOR_MAX_STEP_FREQ_HZ 取 f 栏的值
 *             ⚠ 必须装上摆杆带真实负载测，空载能跑的频率挂上摆杆通常要打对折。
 *  7/7 SPAN   进入即断电，**用手**把摆杆推到两端机械极限，ENTER 交替打 A/B 点。
 *             A/B 显示原始计数，直接抄进
 *             → STEP_MOTOR_ENC_HARD_MIN/MAX_COUNTS
 *             d 栏给出两点间的电机轴角，配合量角器读到的摆杆角 θ 得减速比 k = d/θ。
 *
 * 标定有依赖顺序：MICROSTEP 错了 SWEEP 全错，COUNTS_PER_REV 错了 SPAN 全错。
 * 按 TURN → HAND → RUN → SWEEP → SPAN 的次序做，理由见标定手册。
 *
 * ⚠ 本页**不清零编码器**：零点是上电参考位，限位那组边界全都相对它，清零一次限位
 *   就护在错的地方了。需要"从零起测"的模式改为记基准值显示差值（rel 行）。
 *   限位因此全程有效——要越过软限位量机械行程，用 HAND/SPAN 的失能手推。
 */

/* SWEEP 的转速档，单位 deg/s。从远低于额定处起步，逐档确认再往上。 */
static const float sm_sweep_speed[] = {
    15.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f, 180.0f, 210.0f, 240.0f,
    270.0f, 300.0f, 330.0f, 360.0f, 390.0f, 420.0f, 450.0f, 480.0f,
    540.0f, 600.0f, 660.0f, 720.0f, 840.0f, 960.0f
};
#define SM_SWEEP_STEP_COUNT ((uint8_t)(sizeof(sm_sweep_speed) / sizeof(sm_sweep_speed[0])))

/*
 * 驱动只接受位置指令，本页的"持续转"因此改成"把目标设到很远处"——
 * 伺服会一路跑到那里，途中就是匀速（速度上限饱和），效果与旧的速度式一致，
 * 区别只是这条路径同样受位置限幅约束，测标定参数时不可能把摆杆开出行程。
 */
#define SM_FAR_COUNTS 1000000

/* RUN 模式转速上限：够慢，肉眼能跟上轴上的标记。 */
#define SM_RUN_SPEED_DEG_S 30.0f
/* TURN 模式转速上限：慢一些，减小加减速带来的丢步，让圈数判读干净。 */
#define SM_TURN_SPEED_DEG_S 45.0f
/* JOG 模式转速上限：一步只有几度，慢一点看得清，也不至于冲过目标。 */
#define SM_JOG_SPEED_DEG_S 20.0f

/*
 * JOG 的四档步长，单位编码器计数：1 / 5 / 10 / 20。
 * 换成电机轴角是 0.18° / 0.9° / 1.8° / 3.6°（×0.18°/计数）。
 *
 * 最细一档不是「一个微步」：一个微步是 STEP_ANGLE_DEG / MICROSTEP = 0.05625°（×32），
 * 换算过来只有 0.3125 个编码器计数，而伺服的误差是**整数计数**——比一个计数小的位移
 * 根本表达不出来。**闭环下的「最小可指令位移」是一个编码器计数（0.18°），由反馈分辨率
 * 决定，不是由细分数决定**；提高细分只让这一个计数走得更平顺（步进精细度），
 * 不会让它变小。要真正走进一个计数以内，只能开环按微步数发脉冲，那是另一种接口。
 *
 * ⚠ 最细一档还有一条硬约束：**必须 > 2 × 到位容差**，否则换向后的第一下是死键。
 *
 *   伺服是从带外接近、一进带就停，所以停下来时残余误差的量值≈容差，且**符号与刚才
 *   的行进方向相同**（目标在前、实测在后）。正向点完停在 err = +容差；这时按反向，
 *   新误差 = +容差 − 步长，量值只有 `步长 − 容差`。要它还在容差带外就必须
 *   `步长 − 容差 > 容差`，即 **步长 > 2 × 容差**。步长只取「容差 + 1」的话换向后 |err|
 *   恰好等于容差，伺服判为已到位——**换向第一下必然是死键**，确定性发生，不是偶发。
 *
 *   容差现取 0，残余误差恒为 0，换向不再被抵掉一份，所以最细档 1 个计数就够。
 *   容差以后若抬回非 0，最细档必须同步加大——下面的编译期断言会拦住。
 *
 * 四档的取值是**绝对手感**，与容差无关：1 用于最后对位，5/10 是日常调整量，20 是粗调。
 * 注意软限位内可用行程 SOFT_MAX - SOFT_MIN ≈ 410 计数，最粗档 20 计数横跨全行程要按
 * 二十下——JOG 是精调工具，大范围移动用 RUN 模式。
 *
 * ⚠ 单次点动落点有 ±容差 的不确定度（换向后的第一下实际只走 `步长 − 容差`），
 *   但**位置不会累积误差**：点动加的是目标而不是实测位置，目标每次精确加一个步长，
 *   落点偏差不进下一步。
 */
#define SM_JOG_STEP_MIN 1

static const int32_t sm_jog_step[] = { SM_JOG_STEP_MIN, 5, 10, 20 };
#define SM_JOG_STEP_COUNT ((uint8_t)(sizeof(sm_jog_step) / sizeof(sm_jog_step[0])))

/* 最细档跨不过换向阈值就是死键，宁可编译不过也不要上板才发现。 */
#if (SM_JOG_STEP_MIN <= (2 * STEP_MOTOR_POSITION_TOLERANCE_COUNTS))
#error "JOG 最细档必须 > 2 × STEP_MOTOR_POSITION_TOLERANCE_COUNTS，否则换向第一下是死键"
#endif

#define SM_TURN_MAX 10U
#define SM_HAND_MAX 20U

/* 遥测周期：20ms 以便看清起停瞬态与编码器跟随情况。 */
#define SM_TELEMETRY_PERIOD_MS 20U

typedef enum {
    SM_MODE_JOG = 0,     /* 进页默认落在这里：点动对位比标定常用得多。 */
    SM_MODE_LEVEL,       /* 捕获当前位置为本次上电的 H3 动力学水平点。 */
    SM_MODE_RUN,
    SM_MODE_TURN,
    SM_MODE_HAND,
    SM_MODE_SWEEP,
    SM_MODE_SPAN,
    SM_MODE_MAX
} SM_MODE;

static const char *const sm_mode_tag[SM_MODE_MAX] = {
    "SM 1/7 JOG",   "SM 2/7 LEVEL", "SM 3/7 RUN",  "SM 4/7 TURN",
    "SM 5/7 HAND",  "SM 6/7 SWEEP", "SM 7/7 SPAN"
};

static SM_MODE  sm_mode;
static uint32_t sm_last_ui;
static uint32_t sm_last_telemetry;
static bool     sm_running;
static uint8_t  sm_jog_idx;          /* JOG：当前步长档索引。 */
static uint8_t  sm_turn_target;      /* TURN：目标圈数。 */
static uint8_t  sm_hand_turns;       /* HAND：操作者已手转的圈数。 */
static uint8_t  sm_sweep_idx;        /* SWEEP：当前转速档索引。 */
static int32_t  sm_err_peak;         /* SWEEP：本档内 |err| 的峰值。 */
static int32_t  sm_span_a;           /* SPAN：A 点编码器计数。 */
static int32_t  sm_span_b;           /* SPAN：B 点编码器计数。 */
static bool     sm_span_next_is_b;   /* SPAN：下次打点记 B 而非 A。 */
static bool     sm_span_has_a;
static bool     sm_span_has_b;
static bool     sm_level_saved;      /* 本次上电已有设备页捕获的动力学水平点。 */

/*
 * 各模式的计数基准。
 *
 * 本页**不清零编码器**——编码器零点是上电参考位，限位那组边界全都相对它，
 * 清零一次限位就护在错的地方了。需要"从零起测"的模式改为记下进入时的计数，
 * 显示差值，坐标系和限位保护因此全程不受影响。
 */
static int32_t  sm_cnt_base;

/* SWEEP 往复：当前是否朝正方向。 */
static bool     sm_sweep_forward;

/* 相对本模式基准的计数。 */
static int32_t SmRelCount(void){
    return StepMotor_GetEncoderCount() - sm_cnt_base;
}

static void SmSetBase(void){
    sm_cnt_base = StepMotor_GetEncoderCount();
}

/*
 * 位置误差的绝对值，单位：编码器计数。
 *
 * 闭环下这就是丢步的直接指标：不丢步时它稳定在跟踪误差附近（约 speed/SERVO_KP
 * 换算的计数），丢步时电机没走到、编码器落后，它会超出该值且不收敛。
 * 换算与显示都用驱动的接口，不再自己复制一份公式。
 */
static int32_t SmAbsErr(void){
    int32_t e = StepMotor_GetPositionErrorCount();
    return (e < 0) ? -e : e;
}

/* "前缀 + 定点数" 一行，省去每处重复的 PutStr/AppFmt 两步。 */
static void SmLineF(char *buf, const char *prefix, float value, uint8_t decimals){
    uint8_t n = PutStr(buf, prefix);
    AppFmt_Fixed(&buf[n], value, decimals);
}

static void SmLineI(char *buf, const char *prefix, int32_t value){
    uint8_t n = PutStr(buf, prefix);
    AppFmt_I32(&buf[n], value);
}

/* 纯文本行。PutStr 按约定不写结尾 '\0'，这里补上。 */
static void SmLineS(char *buf, const char *text){
    uint8_t n = PutStr(buf, text);
    buf[n] = '\0';
}

/*
 * [SM] 遥测。字段含义：
 *   m    当前标定模式序号
 *   t    ms 时间戳
 *   cnt  编码器累计计数（限位坐标系下的绝对位置）
 *   tgt  目标计数
 *   err  位置误差 = tgt - cnt，**丢步的直接指标**
 *   deg  cnt 换算的电机轴角
 *   cmd  伺服本拍下发的速度 deg/s（只读，不是指令）
 *   f    驱动实际应用的步进频率 Hz（0=未出脉冲，落进死区或已停）
 *   en   驱动器使能
 *   g    守护状态（见 STEP_MOTOR_GUARD_STATE）
 */
static void ChkStepMotor_Telemetry(uint32_t now){
    DebugUart_Printf(
        "[SM] m=%u t=%lu cnt=%ld tgt=%ld err=%ld deg=%.2f cmd=%.2f f=%lu en=%u g=%u\r\n",
        (unsigned)sm_mode,
        (unsigned long)now,
        (long)StepMotor_GetEncoderCount(),
        (long)StepMotor_GetTargetCount(),
        (long)StepMotor_GetPositionErrorCount(),
        (double)StepMotor_CountsToDeg(StepMotor_GetEncoderCount()),
        (double)StepMotor_GetSpeed(),
        (unsigned long)StepMotor_GetStepFrequencyHz(),
        (unsigned)(StepMotor_IsEnabled() ? 1U : 0U),
        (unsigned)StepMotor_GetGuardState());
}

/* 守护状态的短名，够窄能塞进 OLED 一行。 */
static const char *SmGuardText(void){
    switch (StepMotor_GetGuardState()){
    case STEP_MOTOR_GUARD_OK:          return "g ok";
    case STEP_MOTOR_GUARD_STALLED:     return "g OUT drv off";
    case STEP_MOTOR_GUARD_RECOVER_NEG: return "g RECOVER -";
    case STEP_MOTOR_GUARD_RECOVER_POS: return "g RECOVER +";
    case STEP_MOTOR_GUARD_FAULT:       return "g FAULT!";
    case STEP_MOTOR_GUARD_DISABLED:
    default:                           return "g off";
    }
}

static void ChkStepMotor_Render(const char *action){
    char l1[18];
    char l2[18];
    char l3[18];
    char l4[18];
    const char *hint = "";
    int32_t cnt = StepMotor_GetEncoderCount();

    switch (sm_mode){
    case SM_MODE_JOG: {
        int32_t step = sm_jog_step[sm_jog_idx];
        int32_t tgt  = StepMotor_GetTargetCount();

        SmLineI(l1, "cnt ", cnt);
        SmLineI(l2, "step ", step);
        /* 这一步实际转多少度——「最小移动角度」看的就是这一行。 */
        SmLineF(l3, "sdeg ", StepMotor_CountsToDeg(step), 2U);
        /*
         * 目标已经顶在软限位上时优先报限位：JOG 点不动最常见的原因就是这个，
         * 光在动作栏闪一下不够（200ms 后就被 idle 覆盖了）。
         * 限位关掉时这两个边界不生效，此时别误报——条件里的宏是编译期常量。
         */
        if ((STEP_MOTOR_ENC_LIMIT_ENABLED != 0U) &&
            (tgt >= STEP_MOTOR_ENC_SOFT_MAX_COUNTS)){
            SmLineS(l4, "LIMIT max");
        } else if ((STEP_MOTOR_ENC_LIMIT_ENABLED != 0U) &&
                   (tgt <= STEP_MOTOR_ENC_SOFT_MIN_COUNTS)){
            SmLineS(l4, "LIMIT min");
        } else {
            SmLineS(l4, SmGuardText());
        }
        hint = "UP/DN 1step LNG size";
        break;
    }

    case SM_MODE_LEVEL:
        SmLineI(l1, "cnt ", cnt);
        SmLineI(l2, "step ", sm_jog_step[sm_jog_idx]);
        SmLineF(l3, "bias ", AppBallLevel_GetBiasDeg(), 2U);
        if (sm_level_saved){
            SmLineS(l4, "LEVEL SAVED");
        } else {
            SmLineS(l4, SmGuardText());
        }
        hint = "UP/DN jog ENT:set LNG:next";
        break;

    case SM_MODE_TURN: {
        int32_t rel = SmRelCount();
        SmLineI(l1, "N ", (int32_t)sm_turn_target);
        SmLineI(l2, "rel ", rel);
        /* 每转计数的交叉验证值：MICROSTEP 已经对了，这个数就该等于 CPR。 */
        SmLineI(l3, "c/N ", (sm_turn_target > 0U) ? (rel / (int32_t)sm_turn_target) : 0);
        /* 目标被限幅过就说明行程不够，走不满 N 圈——不提示的话会误判成丢步。 */
        SmLineS(l4, (StepMotor_GetTargetCount() !=
                     sm_cnt_base + (int32_t)((float)sm_turn_target *
                                             STEP_MOTOR_ENCODER_COUNTS_PER_REV))
                        ? "CLAMPED by limit" : "target ok");
        hint = "UP/DN N  ENT go";
        break;
    }

    case SM_MODE_HAND: {
        int32_t rel = SmRelCount();
        SmLineS(l1, StepMotor_IsEnabled()
                        ? "!! still ON !!" : "drv OFF turnable");
        SmLineI(l2, "rel ", rel);
        SmLineI(l3, "turns ", (int32_t)sm_hand_turns);
        /* 这一行就是 STEP_MOTOR_ENCODER_COUNTS_PER_REV 的实测值。 */
        SmLineI(l4, "cpr ", (sm_hand_turns > 0U)
                                ? ((rel < 0 ? -rel : rel) / (int32_t)sm_hand_turns) : 0);
        hint = "UP/DN turns ENT 0";
        break;
    }

    case SM_MODE_SWEEP:
        SmLineF(l1, "spd ", sm_sweep_speed[sm_sweep_idx], 0U);
        /* 用驱动的换算，不自己复制公式——这个 f 就是要抄回 MAX_STEP_FREQ_HZ 的数。 */
        SmLineI(l2, "f ", (int32_t)StepMotor_SpeedToStepFreq(sm_sweep_speed[sm_sweep_idx]));
        SmLineI(l3, "err ", SmAbsErr());
        /* 峰值比瞬时值可靠：丢步是累积的，人眼盯瞬时数容易漏掉。 */
        SmLineI(l4, "peak ", sm_err_peak);
        hint = "UP/DN spd ENT run";
        break;

    case SM_MODE_SPAN:
        /*
         * A/B 显示**原始计数**而非角度：这两个数要直接抄进 step_motor.h 的
         * STEP_MOTOR_ENC_HARD_MIN/MAX_COUNTS，中间做一道角度换算只会引入
         * ENCODER_COUNTS_PER_REV 的刻度误差。角度留给 d 行——减速比 k（#9）要的是它。
         */
        if (sm_span_has_a){ SmLineI(l1, "A ", sm_span_a); }
        else              { SmLineS(l1, "A --"); }
        if (sm_span_has_b){ SmLineI(l2, "B ", sm_span_b); }
        else              { SmLineS(l2, "B --"); }
        if (sm_span_has_a && sm_span_has_b){
            SmLineF(l3, "d ", StepMotor_CountsToDeg(sm_span_b - sm_span_a), 1U);
        } else {
            SmLineS(l3, "d --");
        }
        /* 实时计数：打点前就知道会记下什么数，也便于盯着接近极限的过程。 */
        SmLineI(l4, "cnt ", cnt);
        hint = "PUSH by hand  ENT mark";
        break;

    case SM_MODE_RUN:
    default:
        SmLineI(l1, "cnt ", cnt);
        SmLineF(l2, "deg ", StepMotor_CountsToDeg(cnt), 1U);
        /* 按 UP 时 cnt 应增大；err 不收敛即丢步。方向与刻度都靠这两行判。 */
        SmLineI(l3, "err ", StepMotor_GetPositionErrorCount());
        /* 守护状态：摆杆被自动纠正时，这是屏上唯一能看见的线索。 */
        SmLineS(l4, SmGuardText());
        hint = (STEP_MOTOR_ENC_LIMIT_ENABLED != 0U) ? "UP/DN run  lim on"
                                                    : "UP/DN run  lim OFF";
        break;
    }

    Ui_RenderLines(sm_mode_tag[sm_mode], action, l1, l2, l3, l4, hint);
}

/*
 * 切模式先停机。
 *
 * HAND 与 SPAN 要**失能驱动器**：这两个模式靠手推摆杆取数，而限位是全程有效的，
 * 电机开不出软限位；失能之后伺服不出力、守护也无从纠正，编码器却照常计数——
 * 这正是量机械行程该用的办法。其余模式恢复通电。
 */
static void ChkStepMotor_SetMode(SM_MODE mode){
    (void)StepMotor_Stop();
    sm_running = false;

    sm_mode = mode;
    bool hand_driven = (mode == SM_MODE_HAND) || (mode == SM_MODE_SPAN);
    (void)StepMotor_SetEnabled(!hand_driven);

    /* 从零起测的模式改记基准值，不清零编码器——清零会让限位边界失去意义。 */
    if (mode == SM_MODE_HAND || mode == SM_MODE_TURN || mode == SM_MODE_SWEEP){
        SmSetBase();
    }
    if (mode == SM_MODE_SWEEP){
        sm_err_peak = 0;
        sm_sweep_forward = true;
    }
}

static void ChkStepMotor_Enter(void){
    /*
     * 不调 StepMotor_Init()：外设已在 App_Init 里初始化过，而 Init 会把编码器零点
     * 复位——零点正是限位赖以判定的东西，进个自检页就抹掉说不过去。
     *
     * 限位全程有效，本页不碰它。要越过软限位量机械行程，用 HAND / SPAN 的
     * 失能手推，而不是临时关限位——后者会留下忘了开回来的隐患。
     */
    /* 上电抬升可能还没跑完，本页要自己下位置指令，先把它放弃掉，免得两边抢。 */
    StepMotor_AbortStartup();

    sm_cnt_base       = StepMotor_GetEncoderCount();
    sm_sweep_forward  = true;
    sm_jog_idx        = 0U;    /* 从最细一档起，误触也只走一两度。 */
    sm_turn_target    = 1U;
    sm_hand_turns     = 1U;
    sm_sweep_idx      = 0U;
    sm_err_peak       = 0;
    sm_span_a         = 0;
    sm_span_b         = 0;
    sm_span_has_a     = false;
    sm_span_has_b     = false;
    sm_span_next_is_b = false;
    sm_level_saved    = AppBallLevel_IsRuntimeCalibrated();
    sm_last_ui        = 0U;
    sm_last_telemetry = 0U;

    /* 把固件里的换算常量原样打一行，便于与驱动器拨码、SysConfig 分频当场对账。 */
    DebugUart_Printf("[SM] --- enter, clk=%lu min_f=%u max_f=%u ustep=%d cpr=%d ---\r\n",
                     (unsigned long)STEP_MOTOR_STEP_TIMER_CLK_HZ,
                     (unsigned)STEP_MOTOR_MIN_STEP_FREQ_HZ,
                     (unsigned)STEP_MOTOR_MAX_STEP_FREQ_HZ,
                     (int)STEP_MOTOR_MICROSTEP,
                     (int)STEP_MOTOR_ENCODER_COUNTS_PER_REV);
    ChkStepMotor_SetMode(SM_MODE_JOG);
    ChkStepMotor_Render("idle");
}

/* JOG / LEVEL 共用的点动键：UP/DOWN 短按走位置，长按只调整步长。 */
static const char *ChkStepMotor_HandleJogKeys(KEY_EVENT ev_up, KEY_EVENT ev_dn){
    if (ev_up == KEY_EVENT_LONG_PRESS){
        if (sm_jog_idx + 1U < SM_JOG_STEP_COUNT){
            sm_jog_idx++;
            return "step+";
        }
        return NULL;
    }
    if (ev_dn == KEY_EVENT_LONG_PRESS){
        if (sm_jog_idx > 0U){
            sm_jog_idx--;
            return "step-";
        }
        return NULL;
    }

    int32_t n = 0;
    if      (ev_up == KEY_EVENT_SHORT_PRESS) { n =  1; }
    else if (ev_up == KEY_EVENT_DOUBLE_CLICK){ n =  2; }
    else if (ev_dn == KEY_EVENT_SHORT_PRESS) { n = -1; }
    else if (ev_dn == KEY_EVENT_DOUBLE_CLICK){ n = -2; }

    if (n == 0){ return NULL; }

    /* 以目标为基准累加，连点时不会吃掉上一步尚未完成的行程。 */
    int32_t want = StepMotor_GetTargetCount() + (n * sm_jog_step[sm_jog_idx]);
    (void)StepMotor_SetSpeedLimit(SM_JOG_SPEED_DEG_S);
    (void)StepMotor_MoveToCount(want);
    sm_running = true;
    if (StepMotor_GetTargetCount() != want){ return "CLAMPED"; }
    return (n > 0) ? "jog +" : "jog -";
}

/* 各模式的 UP/DOWN/ENTER 短按语义；返回要显示的动作名，NULL 表示本拍无操作。 */
static const char *ChkStepMotor_HandleKeys(KEY_EVENT ev_up,
                                           KEY_EVENT ev_dn,
                                           KEY_EVENT ev_en){
    switch (sm_mode){
    case SM_MODE_JOG: {
        const char *jog_action = ChkStepMotor_HandleJogKeys(ev_up, ev_dn);
        if (jog_action != NULL){ return jog_action; }

        if (ev_en == KEY_EVENT_SHORT_PRESS){
            /* 连点攒下的目标一次撤掉：就地停住，目标钉回当前实测位置。 */
            (void)StepMotor_Stop();
            sm_running = false;
            return "STOP";
        }
        break;
    }

    case SM_MODE_LEVEL: {
        const char *jog_action = ChkStepMotor_HandleJogKeys(ev_up, ev_dn);
        if (jog_action != NULL){ return jog_action; }

        if (ev_en == KEY_EVENT_SHORT_PRESS){
            if (sm_running || !StepMotor_IsAtTarget()){
                return "WAIT TARGET";
            }
            if (!StepMotor_IsEnabled() ||
                (StepMotor_GetGuardState() != STEP_MOTOR_GUARD_OK)){
                return "GUARD ERR";
            }

            int32_t count = StepMotor_GetEncoderCount();
            if (!AppBallLevel_SetFromEncoderCount(count)){
                return "LEVEL RANGE ERR";
            }
            sm_level_saved = true;
            DebugUart_Printf("[SM] level saved cnt=%ld bias=%.3fdeg\r\n",
                             (long)count,
                             (double)AppBallLevel_GetBiasDeg());
            return "LEVEL SAVED";
        }
        break;
    }

    case SM_MODE_TURN:
        if (ev_up == KEY_EVENT_SHORT_PRESS && sm_turn_target < SM_TURN_MAX){
            sm_turn_target++;
            return "N+";
        }
        if (ev_dn == KEY_EVENT_SHORT_PRESS && sm_turn_target > 1U){
            sm_turn_target--;
            return "N-";
        }
        if (ev_en == KEY_EVENT_SHORT_PRESS){
            if (sm_running){
                (void)StepMotor_Stop();
                sm_running = false;
                return "ABORT";
            }
            SmSetBase();
            /*
             * 走满 N 圈 = 目标设成 N 圈对应的计数，到位自停。
             *
             * ⚠ 目标同样过限幅：摆杆装机后行程通常远小于一圈，这个目标会被夹到
             *   软限位上，走不满 N 圈——TURN 量的是"轴实际转了几圈"，装机后本就
             *   做不了。真要重标细分数得先拆下摆杆，或把 ENC_LIMIT_ENABLED 设 0
             *   重编译。页面 lim 行会显示目标是否被夹过。
             */
            (void)StepMotor_SetSpeedLimit(SM_TURN_SPEED_DEG_S);
            (void)StepMotor_MoveToCount(
                sm_cnt_base +
                (int32_t)((float)sm_turn_target * STEP_MOTOR_ENCODER_COUNTS_PER_REV));
            sm_running = true;
            return "GO";
        }
        break;

    case SM_MODE_HAND:
        /* 圈数由操作者自己数，软件只负责除法——手转不产生任何可信的圈数信号。 */
        if (ev_up == KEY_EVENT_SHORT_PRESS && sm_hand_turns < SM_HAND_MAX){
            sm_hand_turns++;
            return "turns+";
        }
        if (ev_dn == KEY_EVENT_SHORT_PRESS && sm_hand_turns > 1U){
            sm_hand_turns--;
            return "turns-";
        }
        if (ev_en == KEY_EVENT_SHORT_PRESS){
            /* "清零"只是重设基准值，不动编码器本身——限位的坐标系必须保住。 */
            SmSetBase();
            sm_hand_turns = 1U;
            return "zero";
        }
        break;

    case SM_MODE_SWEEP:
        if (ev_up == KEY_EVENT_SHORT_PRESS && sm_sweep_idx + 1U < SM_SWEEP_STEP_COUNT){
            sm_sweep_idx++;
            sm_err_peak = 0;
            /* 扫描的是**速度上限**：目标始终在远处，伺服一路饱和在上限上跑。 */
            (void)StepMotor_SetSpeedLimit(sm_sweep_speed[sm_sweep_idx]);
            return "spd+";
        }
        if (ev_dn == KEY_EVENT_SHORT_PRESS && sm_sweep_idx > 0U){
            sm_sweep_idx--;
            sm_err_peak = 0;
            (void)StepMotor_SetSpeedLimit(sm_sweep_speed[sm_sweep_idx]);
            return "spd-";
        }
        if (ev_en == KEY_EVENT_SHORT_PRESS){
            if (sm_running){
                (void)StepMotor_Stop();
                sm_running = false;
                return "STOP";
            }
            /* 每档从零起测，否则上一档的丢步会记进本档峰值。 */
            SmSetBase();
            sm_err_peak = 0;
            sm_sweep_forward = true;
            (void)StepMotor_SetSpeedLimit(sm_sweep_speed[sm_sweep_idx]);
            (void)StepMotor_MoveToCount(SM_FAR_COUNTS);   /* 夹到软限位上界 */
            sm_running = true;
            return "RUN";
        }
        break;

    case SM_MODE_SPAN:
        /*
         * 本模式驱动器是失能的：**用手**把摆杆推到机械极限再打点。
         * 限位全程有效，电机开不出软限位，量不到真实极限；失能后编码器照常计数，
         * 手推反而是唯一量得准的办法。UP/DOWN 在这里没有作用。
         */
        if (ev_en == KEY_EVENT_SHORT_PRESS){
            int32_t cnt = StepMotor_GetEncoderCount();
            bool marked_b = sm_span_next_is_b;
            if (marked_b){
                sm_span_b = cnt;
                sm_span_has_b = true;
            } else {
                sm_span_a = cnt;
                sm_span_has_a = true;
            }
            sm_span_next_is_b = !marked_b;
            return marked_b ? "mark B" : "mark A";
        }
        break;

    case SM_MODE_RUN:
    default:
        if (ev_up == KEY_EVENT_SHORT_PRESS){
            /* "持续正转" = 目标设到远处；限位启用时会被夹到边界，正是期望行为。 */
            (void)StepMotor_SetSpeedLimit(SM_RUN_SPEED_DEG_S);
            (void)StepMotor_MoveToCount(SM_FAR_COUNTS);
            sm_running = true;
            return "RUN FWD";
        }
        if (ev_dn == KEY_EVENT_SHORT_PRESS){
            (void)StepMotor_SetSpeedLimit(SM_RUN_SPEED_DEG_S);
            (void)StepMotor_MoveToCount(-SM_FAR_COUNTS);
            sm_running = true;
            return "RUN REV";
        }
        if (ev_en == KEY_EVENT_SHORT_PRESS){
            /*
             * 暂停：只停脉冲，保持通电，est/cnt 一律留着。停下来正是要比对这一段跑完的
             * est 与 cnt，当场清零等于把要看的数据抹掉；再按 UP/DOWN 可接着累积。
             * 清零挪到退出整页时做。
             */
            (void)StepMotor_Stop();
            sm_running = false;
            return "PAUSE";
        }
        break;
    }
    return NULL;
}

static APP_TASK_STATUS ChkStepMotor_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();

    /* 编码器采样与开环积分都在 StepMotor_GuardTick()（调度器 10ms）里做，这里不重复。 */

    /* 三个键每拍都取，避免切模式后残留事件在新模式里意外触发。 */
    KEY_EVENT ev_up = Key_GetEvent(KEY_ID_UP);
    KEY_EVENT ev_dn = Key_GetEvent(KEY_ID_DOWN);
    KEY_EVENT ev_en = Key_GetEvent(KEY_ID_ENTER);

    const char *action;

    if (ev_en == KEY_EVENT_LONG_PRESS){
        /* LEVEL 长按只跳出标定，不捕获当前位置；其余模式仍按顺序循环。 */
        SM_MODE next_mode = (sm_mode == SM_MODE_LEVEL)
            ? SM_MODE_RUN
            : (SM_MODE)((sm_mode + 1U) % (uint8_t)SM_MODE_MAX);
        ChkStepMotor_SetMode(next_mode);
        /* 用模式名当动作名：遥测里能看出切到了哪一模式，屏上也确认了这次长按被识别。 */
        action = sm_mode_tag[sm_mode];
    } else {
        action = ChkStepMotor_HandleKeys(ev_up, ev_dn, ev_en);
    }

    /*
     * JOG：走到位就落回 idle。点动只有几度，不清这个标志遥测会一直按运动期的
     * 20ms 高频打，串口刷得看不清；这里不报 DONE，一步一条太吵。
     */
    if (((sm_mode == SM_MODE_JOG) || (sm_mode == SM_MODE_LEVEL)) && sm_running){
        if (StepMotor_IsAtTarget()){
            sm_running = false;
        }
    }

    /* TURN：走到目标自动停。目标在 GO 时就设死了，这里只负责判到位并报 DONE。 */
    if ((sm_mode == SM_MODE_TURN) && sm_running){
        if (StepMotor_IsAtTarget()){
            sm_running = false;
            action = "DONE";
        }
    }

    if ((sm_mode == SM_MODE_SWEEP) && sm_running){
        /*
         * SWEEP 往复：到一端就翻头。
         *
         * 摆杆装机后行程只有几百个计数，朝一个方向跑不到一秒就抵住软限位停了，
         * 根本攒不出可判读的数据。往复既能持续运动，也更接近摆杆的真实工况
         * （来回摆，起停频繁，正是最容易丢步的地方）。
         */
        if (StepMotor_IsAtTarget()){
            sm_sweep_forward = !sm_sweep_forward;
            (void)StepMotor_MoveToCount(sm_sweep_forward ? SM_FAR_COUNTS : -SM_FAR_COUNTS);
        }

        /*
         * 跟踪本档 |err| 峰值。不丢步时 err 稳定在跟踪误差附近
         * （约 speed/SERVO_KP 换算的计数）；丢步时电机没走到、编码器落后，
         * err 会明显超出该值。盯瞬时值容易在两次刷屏之间漏掉，故记峰值。
         */
        int32_t err = SmAbsErr();
        if (err > sm_err_peak){
            sm_err_peak = err;
        }
    }

    if (action != NULL){
        DebugUart_Printf("[SM] --- %s ---\r\n", action);
        ChkStepMotor_Render(action);
        sm_last_ui = now;
    }

    /* 停止时也节流刷新，便于 HAND 模式手动转轴时观察计数。 */
    if ((now - sm_last_ui) >= CHK_UI_PERIOD_MS){
        sm_last_ui = now;
        ChkStepMotor_Render(sm_running ? "running" : "idle");
    }

    /* 运动期间高频输出遥测；停止时低频输出，便于手动转轴时观察编码器。 */
    uint32_t telemetry_period = sm_running ? SM_TELEMETRY_PERIOD_MS : CHK_UI_PERIOD_MS;
    if ((now - sm_last_telemetry) >= telemetry_period){
        sm_last_telemetry = now;
        ChkStepMotor_Telemetry(now);
    }

    return APP_TASK_RUNNING;
}

static void ChkStepMotor_Exit(void){
    /* 与上电策略一致：停脉冲后失能，摆杆的保持力矩由后续任务显式申请。 */
    (void)StepMotor_Stop();
    (void)StepMotor_SetEnabled(false);

    /*
     * 编码器计数与限位全程没被动过——本页从头到尾只记基准值、不清零，
     * 所以退出时无需恢复任何东西，保护一直都在。
     */
    DebugUart_Printf("[SM] --- exit, cnt=%ld limit kept ---\r\n",
                     (long)StepMotor_GetEncoderCount());

    sm_running = false;
}

/* ============================ 临时水管变节奏往复 ============================ */

/*
 * 以进页时的姿态为中心，三档依次等待 3/2/1 s 后运动。
 * 35/70/105 计数在名义水平位附近约对应水管 1/2/3°；连杆非线性
 * 使它们只是上板观察档位，不是精密角度指令。底层软限位始终生效。
 */
#define PIPE_SWING_STAGE_COUNT 3U
#define PIPE_SWING_SPEED_DEG_S 90.0f
#define PIPE_SWING_UI_PERIOD_MS 100U

static const uint32_t pipe_swing_interval_ms[PIPE_SWING_STAGE_COUNT] = {
    3000U, 2000U, 1000U
};
static const int32_t pipe_swing_offset_counts[PIPE_SWING_STAGE_COUNT] = {
    35, 70, 105
};

static int32_t pipe_swing_center;
static int32_t pipe_swing_target;
static uint32_t pipe_swing_last_move;
static uint32_t pipe_swing_last_ui;
static uint8_t pipe_swing_stage;
static bool pipe_swing_positive;

static int32_t ChkPipeSwing_ClampTarget(int32_t target){
    if (target < STEP_MOTOR_ENC_SOFT_MIN_COUNTS){
        return STEP_MOTOR_ENC_SOFT_MIN_COUNTS;
    }
    if (target > STEP_MOTOR_ENC_SOFT_MAX_COUNTS){
        return STEP_MOTOR_ENC_SOFT_MAX_COUNTS;
    }
    return target;
}

static void ChkPipeSwing_Render(uint32_t now){
    char l1[20];
    char l2[20];
    char l3[20];
    char l4[20];
    char l5[20];
    uint8_t n;

    n = PutStr(l1, "next ");
    AppFmt_I32(&l1[n], (int32_t)pipe_swing_interval_ms[pipe_swing_stage] / 1000);
    while (l1[n] != '\0'){ n++; }
    n += PutStr(&l1[n], "s ");
    n += PutStr(&l1[n], pipe_swing_positive ? "+" : "-");
    AppFmt_I32(&l1[n], (int32_t)pipe_swing_stage + 1);
    while (l1[n] != '\0'){ n++; }
    n += PutStr(&l1[n], "deg");
    l1[n] = '\0';

    n = PutStr(l2, "wait ");
    uint32_t elapsed = now - pipe_swing_last_move;
    uint32_t interval = pipe_swing_interval_ms[pipe_swing_stage];
    uint32_t remain_ms = (elapsed < interval) ? (interval - elapsed) : 0U;
    AppFmt_I32(&l2[n], (int32_t)((remain_ms + 999U) / 1000U));
    while (l2[n] != '\0'){ n++; }
    n += PutStr(&l2[n], "s");
    l2[n] = '\0';

    n = PutStr(l3, "cnt ");
    AppFmt_I32(&l3[n], StepMotor_GetEncoderCount());
    while (l3[n] != '\0'){ n++; }
    n += PutStr(&l3[n], " >");
    AppFmt_I32(&l3[n], pipe_swing_target);

    n = PutStr(l4, "base ");
    AppFmt_I32(&l4[n], pipe_swing_center);

    n = PutStr(l5, "err ");
    AppFmt_I32(&l5[n], StepMotor_GetPositionErrorCount());

    Ui_RenderLines("Pipe Swing TEMP", l1, l2, l3, l4, l5, "BACK: hold exit");
}

static void ChkPipeSwing_Enter(void){
    (void)Chassis_Brake();
    StepMotor_AbortStartup();
    (void)StepMotor_Stop();
    (void)StepMotor_SetSpeedLimit(PIPE_SWING_SPEED_DEG_S);

    pipe_swing_center = StepMotor_GetEncoderCount();
    pipe_swing_target = pipe_swing_center;
    pipe_swing_stage = 0U;
    pipe_swing_positive = true;
    pipe_swing_last_move = BSP_Time_GetMs();
    pipe_swing_last_ui = pipe_swing_last_move;

    DebugUart_Printf("[PIPE] enter center=%ld, cycle=3s/1deg 2s/2deg 1s/3deg\r\n",
                     (long)pipe_swing_center);
    ChkPipeSwing_Render(pipe_swing_last_move);
}

static APP_TASK_STATUS ChkPipeSwing_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();
    uint32_t interval = pipe_swing_interval_ms[pipe_swing_stage];

    if ((now - pipe_swing_last_move) >= interval){
        int32_t offset = pipe_swing_offset_counts[pipe_swing_stage];
        int32_t requested = pipe_swing_center + (pipe_swing_positive ? offset : -offset);
        pipe_swing_target = ChkPipeSwing_ClampTarget(requested);
        BSP_STATUS status = StepMotor_MoveToCount(pipe_swing_target);
        if (status != BSP_STATUS_OK){
            (void)StepMotor_Stop();
            DebugUart_Printf("[PIPE] command failed status=%d\r\n", (int)status);
            return APP_TASK_FAULT;
        }

        DebugUart_Printf("[PIPE] stage=%u wait=%lums dir=%c offset=%ld target=%ld%s\r\n",
                         (unsigned)(pipe_swing_stage + 1U),
                         (unsigned long)interval,
                         pipe_swing_positive ? '+' : '-',
                         (long)offset,
                         (long)pipe_swing_target,
                         (pipe_swing_target != requested) ? " CLAMPED" : "");

        pipe_swing_positive = !pipe_swing_positive;
        pipe_swing_stage = (uint8_t)((pipe_swing_stage + 1U) % PIPE_SWING_STAGE_COUNT);
        pipe_swing_last_move = now;
        pipe_swing_last_ui = 0U;
    }

    if (((now - pipe_swing_last_ui) >= PIPE_SWING_UI_PERIOD_MS) && !Ui_IsFlushBusy()){
        pipe_swing_last_ui = now;
        ChkPipeSwing_Render(now);
    }
    return APP_TASK_RUNNING;
}

static void ChkPipeSwing_Exit(void){
    (void)StepMotor_Stop();
    pipe_swing_target = StepMotor_GetEncoderCount();
    DebugUart_Printf("[PIPE] exit hold-count=%ld\r\n", (long)pipe_swing_target);
}

/* ============================ 编码器 ============================ */

static uint32_t enc_last_ui;

static void ChkEnc_Enter(void){
    HallEncoder_Reset();
    enc_last_ui = 0U;
}

static APP_TASK_STATUS ChkEnc_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();
    if ((now - enc_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    enc_last_ui = now;

    char l1[20];
    char l2[20];
    char l3[20];
    char l4[20];
    char l5[20];
    uint8_t n;

    n = PutStr(l1, "L cnt ");
    AppFmt_I32(&l1[n], HallEncoder_GetCount(HALL_ENCODER_LEFT));
    n = PutStr(l2, "L spd ");
    AppFmt_Fixed(&l2[n], HallEncoder_GetSpeed(HALL_ENCODER_LEFT), 2);
    n = PutStr(l3, "R cnt ");
    AppFmt_I32(&l3[n], HallEncoder_GetCount(HALL_ENCODER_RIGHT));
    n = PutStr(l4, "R spd ");
    AppFmt_Fixed(&l4[n], HallEncoder_GetSpeed(HALL_ENCODER_RIGHT), 2);

    n = PutStr(l5, "dir L");
    n += PutStr(&l5[n], (HallEncoder_GetDir(HALL_ENCODER_LEFT) == HALL_ENCODER_DIR_FORWARD)
                            ? "F" : "R");
    n += PutStr(&l5[n], " R");
    n += PutStr(&l5[n], (HallEncoder_GetDir(HALL_ENCODER_RIGHT) == HALL_ENCODER_DIR_FORWARD)
                            ? "F" : "R");
    l5[n] = '\0';

    Ui_RenderLines("Chk Encoder", l1, l2, l3, l4, l5, "BACK: exit");
    return APP_TASK_RUNNING;
}

/* ============================ 速度闭环 ============================ */

#define SPD_STEP 0.05f   /* 每次按键调整的目标速度步进, m/s */
/*
 * 目标速度上下限, m/s。实测(2026-07-29 抬轮空载)满占空比对应约 1.21 m/s, 目标超过
 * 1.45 m/s 即持续输出饱和; 旧值 3.0 是可达上限的 2.5 倍, 按几下就把速度环推进深度饱和。
 * 取 1.0 留出余量。
 */
#define SPD_MAX  1.0f

static float spd_target;
static uint32_t spd_last_ui;

static void ChkSpeedPid_Enter(void){
    HallEncoder_Reset();
    spd_target = 0.0f;
    Chassis_SetSpeed(0.0f);   /* 进入速度闭环, 目标 0。 */
    spd_last_ui = 0U;
}

static APP_TASK_STATUS ChkSpeedPid_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();

    /* 按键调目标: UP +step, DOWN -step, ENTER 归零。 */
    if (Key_GetEvent(KEY_ID_UP) == KEY_EVENT_SHORT_PRESS){
        spd_target += SPD_STEP;
        if (spd_target > SPD_MAX){ spd_target = SPD_MAX; }
    } else if (Key_GetEvent(KEY_ID_DOWN) == KEY_EVENT_SHORT_PRESS){
        spd_target -= SPD_STEP;
        if (spd_target < -SPD_MAX){ spd_target = -SPD_MAX; }
    } else if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
        spd_target = 0.0f;
    }
    /* 每拍设目标; 速度环由 App_ControlTick 在本 tick 之后驱动出力。 */
    Chassis_SetSpeed(spd_target);

    /*
     * 遥测: 每控制拍(20ms/50Hz)输出一行, 供上位机
     * tools/visualizers/speed_pid_viz.py 绘图整定。
     * 字段: t 设备 ms; tl/tr 目标轮速; l/r 实测轮速(m/s); dl/dr 应用占空比(%,为上一拍值);
     *       il/ir 速度环积分累加值(诊断积分饱和/退绕, 输出中的积分项为 KI*本值)。
     * t 取自本拍 on_tick 入口, 相邻行的 t 差即实际控制周期, 可据此查调度抖动。
     * 非阻塞(环形缓冲+TX 中断), 对控制环零阻塞。
     */
    CHASSIS_DUTY duty = Chassis_GetDuty();
    DebugUart_Printf("[SPD] t=%lu tl=%.3f tr=%.3f l=%.3f r=%.3f dl=%.1f dr=%.1f"
                     " il=%.3f ir=%.3f\r\n",
        (unsigned long)now,
        (double)Chassis_GetWheelSpeedTarget(HALL_ENCODER_LEFT),
        (double)Chassis_GetWheelSpeedTarget(HALL_ENCODER_RIGHT),
        (double)HallEncoder_GetSpeed(HALL_ENCODER_LEFT),
        (double)HallEncoder_GetSpeed(HALL_ENCODER_RIGHT),
        (double)duty.left_percent, (double)duty.right_percent,
        (double)Chassis_GetWheelSpeedIntegral(HALL_ENCODER_LEFT),
        (double)Chassis_GetWheelSpeedIntegral(HALL_ENCODER_RIGHT));

    if ((now - spd_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    spd_last_ui = now;

    char l1[20];
    char l2[20];
    char l3[20];
    uint8_t n;

    n = PutStr(l1, "tgt ");
    AppFmt_Fixed(&l1[n], spd_target, 2);
    n = PutStr(l2, "L spd ");
    AppFmt_Fixed(&l2[n], HallEncoder_GetSpeed(HALL_ENCODER_LEFT), 2);
    n = PutStr(l3, "R spd ");
    AppFmt_Fixed(&l3[n], HallEncoder_GetSpeed(HALL_ENCODER_RIGHT), 2);

    Ui_RenderLines("Chk Speed PID", "!! WHEELS UP !!", l1, l2, l3,
                   "UP/DN/EN adj", "BACK: exit");
    return APP_TASK_RUNNING;
}

/* ======================= 占空比-速度 扫描(开环诊断)======================= */

#define DSW_STEP 2.0f    /* 每次按键调整的占空比步进, % */
#define DSW_MAX  80.0f   /* 占空比上下限, %(安全起见不到满) */

static float dsw_duty;
static uint32_t dsw_last_ui;

static void ChkDutySweep_Enter(void){
    HallEncoder_Reset();
    dsw_duty = 0.0f;
    (void)Chassis_SetDuty(0.0f, 0.0f);   /* 开环, 双轮同占空比, 不走速度环。 */
    dsw_last_ui = 0U;
}

static APP_TASK_STATUS ChkDutySweep_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();

    /* 按键调占空比: UP +step, DOWN -step, ENTER 归零。 */
    if (Key_GetEvent(KEY_ID_UP) == KEY_EVENT_SHORT_PRESS){
        dsw_duty += DSW_STEP;
        if (dsw_duty > DSW_MAX){ dsw_duty = DSW_MAX; }
    } else if (Key_GetEvent(KEY_ID_DOWN) == KEY_EVENT_SHORT_PRESS){
        dsw_duty -= DSW_STEP;
        if (dsw_duty < -DSW_MAX){ dsw_duty = -DSW_MAX; }
    } else if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
        dsw_duty = 0.0f;
    }
    (void)Chassis_SetDuty(dsw_duty, dsw_duty);   /* 开环固定占空比。 */

    /*
     * 遥测: 复用 [SPD] 行格式(tl/tr=0 无目标, l/r=实测轮速, dl/dr=应用占空比,
     * il/ir=0 开环不跑速度环), 直接用 tools/visualizers/speed_pid_viz.py 看
     * "占空比阶梯 vs 实测速度", 定位死区与噪声拐点。
     */
    DebugUart_Printf("[SPD] t=%lu tl=0.000 tr=0.000 l=%.3f r=%.3f dl=%.1f dr=%.1f"
                     " il=0.000 ir=0.000\r\n",
        (unsigned long)now,
        (double)HallEncoder_GetSpeed(HALL_ENCODER_LEFT),
        (double)HallEncoder_GetSpeed(HALL_ENCODER_RIGHT),
        (double)dsw_duty, (double)dsw_duty);

    if ((now - dsw_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    dsw_last_ui = now;

    char l1[20];
    char l2[20];
    char l3[20];
    uint8_t n;

    n = PutStr(l1, "duty ");
    AppFmt_Fixed(&l1[n], dsw_duty, 1);
    n = PutStr(l2, "L spd ");
    AppFmt_Fixed(&l2[n], HallEncoder_GetSpeed(HALL_ENCODER_LEFT), 2);
    n = PutStr(l3, "R spd ");
    AppFmt_Fixed(&l3[n], HallEncoder_GetSpeed(HALL_ENCODER_RIGHT), 2);

    Ui_RenderLines("Chk Duty Sweep", "!! WHEELS UP !!", l1, l2, l3,
                   "UP/DN/EN adj", "BACK: exit");
    return APP_TASK_RUNNING;
}

/* ============================ 描述符 ============================ */

/* ======================= Raspberry Pi UART2 通信诊断 ======================= */

static uint32_t rpi_last_ui;
static uint8_t rpi_page;

static void ChkRpiUart_Enter(void){
    RpiUart_ResetStats();
    rpi_last_ui = 0U;
    rpi_page = 0U;
}

static APP_TASK_STATUS ChkRpiUart_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();

    if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
        rpi_page ^= 1U;
        rpi_last_ui = 0U;
    }
    if ((now - rpi_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    rpi_last_ui = now;

    RPI_UART_STATS stats;
    RPI_UART_MEASUREMENT measurement;
    RPI_UART_PREDICTION prediction;
    bool have_measurement = RpiUart_GetLatest(&measurement);
    bool live = RpiUart_Predict(0.0f, &prediction);
    RpiUart_GetStats(&stats);

    char l1[20];
    char l2[20];
    char l3[20];
    char l4[20];
    char l5[20];
    uint8_t n;

    if (rpi_page == 0U){
        (void)PutStr(l1, live ? (prediction.hold_output ? "LINK HOLD" :
                                (prediction.degraded ? "LINK DEGRADED" : "LINK OK"))
                             : (have_measurement ? "LINK TIMEOUT" : "WAIT FRAME"));
        l1[live ? (prediction.hold_output ? 9U : (prediction.degraded ? 13U : 7U))
                : (have_measurement ? 12U : 10U)] = '\0';

        n = PutStr(l2, "seq ");
        AppFmt_I32(&l2[n], have_measurement ? (int32_t)measurement.seq : -1);

        n = PutStr(l3, "x ");
        AppFmt_Fixed(&l3[n], have_measurement ? ((float)measurement.x_01mm * 0.1f) : 0.0f, 1);
        while (l3[n] != '\0'){ n++; }
        n += PutStr(&l3[n], " v ");
        AppFmt_I32(&l3[n], have_measurement ? (int32_t)measurement.velocity_mm_s : 0);

        n = PutStr(l4, "age ");
        AppFmt_Fixed(&l4[n], live ? prediction.age_ms : 0.0f, 1);
        while (l4[n] != '\0'){ n++; }
        n += PutStr(&l4[n], "ms");
        l4[n] = '\0';

        n = PutStr(l5, "Q");
        AppFmt_I32(&l5[n], live ? (int32_t)prediction.quality : 0);
        while (l5[n] != '\0'){ n++; }
        n += PutStr(&l5[n], live && prediction.velocity_trusted ? " V+" : " V-");
        n += PutStr(&l5[n], live && prediction.moving ? " M" : " -");
        l5[n] = '\0';
    } else {
        n = PutStr(l1, "rx ");
        AppFmt_I32(&l1[n], (int32_t)stats.rx_frames);

        n = PutStr(l2, "crc ");
        AppFmt_I32(&l2[n], (int32_t)stats.crc_fail);
        while (l2[n] != '\0'){ n++; }
        n += PutStr(&l2[n], " ver ");
        AppFmt_I32(&l2[n], (int32_t)stats.ver_fail);

        n = PutStr(l3, "gap ");
        AppFmt_I32(&l3[n], (int32_t)stats.seq_gap);
        while (l3[n] != '\0'){ n++; }
        n += PutStr(&l3[n], " inv ");
        AppFmt_I32(&l3[n], (int32_t)stats.invalid);

        n = PutStr(l4, "maxage ");
        AppFmt_I32(&l4[n], (int32_t)stats.max_age_ms);
        while (l4[n] != '\0'){ n++; }
        n += PutStr(&l4[n], "ms");
        l4[n] = '\0';

        n = PutStr(l5, "fps ");
        AppFmt_Fixed(&l5[n], (float)stats.frame_rate_x10 * 0.1f, 1);
    }

    Ui_RenderLines("Chk Rpi UART2", l1, l2, l3, l4, l5,
                   rpi_page == 0U ? "ENTER: stats" : "ENTER: live");
    return APP_TASK_RUNNING;
}

const APP_TASK_DESC APP_CHK_GYRO_JY61P = {
    "Gyro JY61P", ChkGyroJy_Enter, ChkGyroJy_Tick, NULL
};
const APP_TASK_DESC APP_CHK_ACCEL_CALIB = {
    "Acc Calib", ChkAccCal_Enter, ChkAccCal_Tick, NULL
};
const APP_TASK_DESC APP_CHK_GRAY_I2C = {
    "Gray I2C", ChkGrayI2c_Enter, ChkGrayI2c_Tick, ChkGrayI2c_Exit
};
const APP_TASK_DESC APP_CHK_TB6612 = {
    "TB6612", ChkTb_Enter, ChkTb_Tick, ChkTb_Exit
};
const APP_TASK_DESC APP_CHK_STEP_MOTOR = {
    "Step Motor", ChkStepMotor_Enter, ChkStepMotor_Tick, ChkStepMotor_Exit
};
const APP_TASK_DESC APP_CHK_PIPE_SWING = {
    "Pipe Swing", ChkPipeSwing_Enter, ChkPipeSwing_Tick, ChkPipeSwing_Exit
};
const APP_TASK_DESC APP_CHK_ENCODER = {
    "Encoder", ChkEnc_Enter, ChkEnc_Tick, NULL
};
const APP_TASK_DESC APP_CHK_SPEED_PID = {
    "Speed PID", ChkSpeedPid_Enter, ChkSpeedPid_Tick, NULL
};
const APP_TASK_DESC APP_CHK_DUTY_SWEEP = {
    "Duty Sweep", ChkDutySweep_Enter, ChkDutySweep_Tick, NULL
};
const APP_TASK_DESC APP_CHK_RPI_UART = {
    "Rpi UART", ChkRpiUart_Enter, ChkRpiUart_Tick, NULL
};
