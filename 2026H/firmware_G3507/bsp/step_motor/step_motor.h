/**
 * @file  step_motor.h
 * @brief BSP 摆杆步进电机位置式控制 + QEI 反馈。
 *
 * 硬件:STEP 脉冲 = SMotor(TIMG6_CCP0, PA29),DIR = PB14,EN = PB11;
 *      位置反馈 = SMotor_QEI(TIMG8, PHA=PA26 / PHB=PA27)。
 *
 * 控制模型——**只有位置,没有速度**:
 *
 *     MoveToCount(target) ─► ClampEncoderCount 限幅 ─► target_counts
 *                                                          │
 *              Tick 每 10ms: speed = clamp(KP × 误差角, ±speed_limit)
 *                                                          │
 *                                                     出脉冲 / 到位停
 *
 * 不提供速度式接口:速度指令绕得过位置限幅(给个方向一直转就出界了),位置指令绕不过。
 * 限幅因此只需在 MoveToCount 一个地方做一次,不可能被绕开。转速由 SetSpeedLimit()
 * 约束——那是**上限**不是指令,误差大时跑到上限,接近目标自动减速。
 *
 * ⚠ StepMotor_Tick() 是驱动的唯一心跳,**不调它电机根本不会动**:位置指令只登记
 *   目标,脉冲由 Tick 里的伺服下发。它同时承担编码器采样、上电抬升、越界纠正。
 *
 * 单通道:只有摆杆一路步进电机,接口不带通道参数。
 *
 * STEP 波形:50% 占空比方波,装载值四舍五入;停止时用 ODIS 强制 CCP 为低,
 * 而非只把 CC 写 0(后者在 EDGE_ALIGN_UP 下会让零事件与比较事件重合,可能挤出毛刺)。
 */
#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 硬件映射(随 SysConfig 变化)===== */
#define STEP_MOTOR_BEAM_DIR_PORT    SMotor_IO_PORT
#define STEP_MOTOR_BEAM_DIR_PIN     SMotor_IO_DIR1_PIN
#define STEP_MOTOR_BEAM_EN_PORT     SMotor_IO_PORT
#define STEP_MOTOR_BEAM_EN_PIN      SMotor_IO_EN1_PIN
#define STEP_MOTOR_BEAM_PWM_TIMER   SMotor_INST
#define STEP_MOTOR_BEAM_PWM_CHANNEL DL_TIMER_CC_0_INDEX
#define STEP_MOTOR_QEI_TIMER        SMotor_QEI_INST

/* ===== 极性(三项均已上板实测;换驱动板或改接线后须重验)===== */
/* 使能脚高电平有效;驱动器为低有效则改 0。 */
#define STEP_MOTOR_BEAM_ENABLE_HIGH 1U
/* DIR 高电平为正方向;转向相反则改 0。 */
#define STEP_MOTOR_BEAM_POSITIVE_DIR_HIGH 1U
/*
 * 编码器计数正方向与电机正方向相反时取 1。翻转只作用于 GetEncoderCount(),
 * 由它派生的误差、限位判定随之统一;GetEncoderRaw 已删,无其它出口。
 */
#define STEP_MOTOR_ENCODER_INVERT 1U

/* ===== 电机与驱动器参数 ===== */
/* 电机固有步距角,单位 deg(铭牌 1.8deg = 200 整步/转)。 */
#define STEP_MOTOR_STEP_ANGLE_DEG 1.8f
/*
 * 驱动器细分数,**须与 D36A 的细分拨码一致**。
 *
 * 这个值定的是**步进精细度**:一个微步 = STEP_ANGLE_DEG / MICROSTEP,×32 时
 * 0.05625°(电机轴)= 0.3125 个编码器计数。它与定位精细度(到位容差)是两件独立的事——
 * 容差决定"停多准",细分决定"每一步多细"。固件这边改不了步进精细度,只能跟随拨码。
 *
 * ⚠⚠ 与拨码不一致**不会报任何错**,只会让转速整体差一个固定倍数:本值填大一倍,
 *     SpeedToStepFreq 就多发一倍脉冲,电机跑得比指令快一倍(限速、上电抬升速度、
 *     越界纠正速度全部跟着翻倍)。改这里之前先确认拨码已经拨过去。
 *
 * ⚠ 一个微步已远细于一个编码器计数(0.3125 < 1),编码器分辨不出单个微步——
 *   靠计数验细分的 TURN 模式(标定手册 #2)从此失效,细分只能以拨码为准。
 *
 * ⚠ 细分越高同转速下脉冲率越高:×32 时 960 deg/s 需 17067 Hz,已接近
 *   MAX_STEP_FREQ_HZ(18000,该值需上板复核)。该上限对应的顶速是 1012 deg/s,
 *   所以当前 MAX_SPEED_DEG_S(960)不会被静默钳位。
 */
#define STEP_MOTOR_MICROSTEP 32.0f
/* 转速硬上限,单位 deg/s。SetSpeedLimit() 不会超过它。 */
#define STEP_MOTOR_MAX_SPEED_DEG_S 960.0f

/* ===== STEP 定时器 ===== */
/*
 * STEP 定时器输入时钟 = SysConfig 中 SMotor 的 clockDivider(8) × clockPrescale(8)
 * 从 32MHz 分出 = 500000 Hz。
 *
 * ⚠ 改动 SysConfig 分频后必须同步本值,并核对生成的 SMotor_INST_CLK_FREQ。
 *   二者不一致不会报编译错,只会让转速整体偏离。
 */
#define STEP_MOTOR_STEP_TIMER_CLK_HZ 500000U
/*
 * 步进频率下限,由 16 位装载值上限决定(500000/65536 ≈ 7.6 Hz)。低于此频率不出脉冲,
 * 即速度存在死区——伺服的 SERVO_MIN_SPEED_DEG_S 就是为躲开它设的。
 */
#define STEP_MOTOR_MIN_STEP_FREQ_HZ 1U
/* 步进频率上限;再高易失步。由 SWEEP 模式实测确定。 */
#define STEP_MOTOR_MAX_STEP_FREQ_HZ 18000U
/*
 * 编码器每转的 QEI 计数。MSPM0 的 QEI 2-input 模式对 A/B 各取上升沿(2 倍频),
 * 故本值 = 编码器线数 × 2。实测 2000,即 1000 线。
 */
#define STEP_MOTOR_ENCODER_COUNTS_PER_REV 2000.0f

/* ===== 位置伺服 ===== */
/*
 * 伺服比例增益,单位 1/s(误差 deg → 速度 deg/s)。位置环唯一的调节参数:
 * 步进没有稳态误差(位置由脉冲数决定)所以不要 I,10ms 周期下微分噪声大于信号所以不要 D。
 * 取 3.0 = 误差 15deg 给 45deg/s。调大则接近目标易过冲,表现为末端来回抖。
 */
#define STEP_MOTOR_SERVO_KP 3.0f
/* 运行期可调位置增益的上限；H3 可临时提高，退出任务后恢复默认值。 */
#define STEP_MOTOR_SERVO_KP_MAX 12.0f

/*
 * 到位容差,单位:编码器计数。**伺服停脉冲与 IsAtTarget() 共用本值**,
 * 分开会出现"报到位了但还在出脉冲"的矛盾状态。
 *
 * 这是**定位精细度的直接上限**:比它小的位移伺服判为已到位,一个脉冲都不会发。
 * 与步进精细度(MICROSTEP)是两件独立的事:容差决定"停多准",细分决定"每步多细"。
 *
 * **取 0 = 只接受精确相等**,这是闭环能做到的极限,也是当前取值。
 *
 * 0 够得着的理由在细分:误差是整数计数,而 ×32 下一个微步只有 0.3125 计数——逼近过程中
 * 物理位置每次只挪不到三分之一个计数,整数误差因此**逐一经过 0**,不会从 +1 跳到 -1。
 * ×8 时代每微步 1.25 计数,一步就可能跨过整个 0 点,那时取 0 是够不着的。
 * 换句话说:**取 0 是拨到 ×32 才换来的**。
 *
 * 编码器量化(±0.5 计数)已按明确要求不作为约束考虑。它是这里唯一还没被排除的物理
 * 因素,而取 0 把余量降到零——真机上若出现"点完停不下来、持续小幅爬行",就是它,
 * 把本值抬回 1 即可(每抬 1 个计数,最小步长涨 2 个计数)。
 *
 * 5 → 2 → 1 → 0 的效果(电机轴):
 *   到位精度         ±0.9°  → ±0.36° → ±0.18° → **精确相等**
 *   最小可指令位移   1.08°  → 0.54°  → 0.36°  → **0.18°** (= 容差+1 计数)
 *   最小换向安全步长 1.98°  → 0.9°   → 0.54°  → **0.18°** (= 2×容差+1)
 *
 * 注意最后一列两行合并了:容差 0 时残余误差恒为 0,换向不再被抵掉一份容差,
 * "2×容差+1" 与 "容差+1" 同为 1 个计数。**换向的 2 倍代价随容差一起归零了。**
 *
 * ⚠ 取 0 的代价:IsAtTarget() 要求误差恰好为 0 才为真。电机若因静摩擦/卡死停在
 *   差一个计数处,SWEEP 的往复翻头和 TURN 的 DONE 判定就一直等不到——那本身是故障态,
 *   但表现会从"报错"变成"卡住"。真机上这两个模式若不翻头/不报 DONE,先怀疑本值。
 *
 * 收紧到位带本会让摆杆自重下沉一点就反复补偿(末端来回抖),所以同时有下面的回差带,
 * 把"停多准"和"漂多少才重新动"拆成两件事——回差带不跟着收,抗漂移能力不受影响。
 */
#define STEP_MOTOR_POSITION_TOLERANCE_COUNTS 0

/*
 * 重新起步阈值(回差带),单位:编码器计数。**必须 > POSITION_TOLERANCE_COUNTS**。
 *
 * 已经报到位之后,误差要涨过本值伺服才重新出脉冲。没有它的话,收紧到位带就等于
 * 收紧"容忍多少漂移":摆杆带重力负载,停脉冲后靠保持力矩顶着,下沉两三个计数就会
 * 触发补偿,补完又沉,形成极限环——就是常说的末端来回抖。
 *
 * 取 6(≈1.08°)略宽于原来的容差 5,所以**抗漂移能力不比改之前差,停得却准了 2.5 倍**。
 * 抖动只可能在这一档变松时消失,不会因为收紧到位带而出现。
 *
 * ⚠ 新目标一下发就清掉到位状态,回差只在"停住不动"时起作用,不会吃掉点动指令。
 */
#define STEP_MOTOR_SERVO_RESUME_COUNTS 6

#if (STEP_MOTOR_SERVO_RESUME_COUNTS <= STEP_MOTOR_POSITION_TOLERANCE_COUNTS)
#error "SERVO_RESUME_COUNTS 必须大于 POSITION_TOLERANCE_COUNTS，否则回差退化成单阈值"
#endif

/*
 * 伺服最低出力速度,单位 deg/s。误差已出容差带但 KP 算出的速度过小时抬到本值,
 * 否则末端会以极低频率蠕动,还可能落进 MIN_STEP_FREQ_HZ 死区而永远到不了位。
 */
#define STEP_MOTOR_SERVO_MIN_SPEED_DEG_S 5.0f

/*
 * 转速上限的上电默认值,单位 deg/s。不参与任何判断,只是"没人设过时用多少"——
 * 抬升、越界纠正、自检页各模式都会用 SetSpeedLimit() 覆盖它。
 */
#define STEP_MOTOR_SERVO_DEFAULT_SPEED_LIMIT_DEG_S 90.0f

/* ===== 摆杆行程编码器硬限位(实测后写死在这里)===== */
/*
 * 判的是 QEI 实测计数,所以丢步、手推、机械打滑都看得见——这是摆杆装机后唯一
 * 起作用的行程保护,启用后**全程有效,没有运行期开关**。要越过软限位量机械行程,
 * 走 StepMotor_SetEnabled(false) 手推摆杆:失能后伺服不出力、守护也无从纠正,
 * 编码器却照常计数。这比"临时关限位"安全,不会留下忘了开回来的隐患。
 *
 * 量纲是 StepMotor_GetEncoderCount() 的累计计数(计数增大 = 电机正方向)。
 *
 * ⚠⚠ 零点是**上电时刻编码器所在位置**(Init 把计数清零)——固件没有绝对位置传感器。
 *     因此整组常数成立的前提是一条约定:
 *
 *         >>> 上电时摆杆必须停在同一个可重复的机械参考位 <<<
 *
 *     最省事的参考位是**断电自重靠住的那一端**:上电前驱动器本来就没通电,摆杆
 *     自然落到那里,不需要人去摆,重复性还比人眼对水平好。于是那一端的计数就是 0,
 *     另一端是整个行程,ENC_LEVEL_COUNTS 则是从参考位到水平位的距离。
 *
 *     约定破了,整组边界跟着平移,限位就护在错的地方。也正因如此,驱动**不提供
 *     运行期清零编码器的接口**——清零等于平移坐标系。
 *
 * 坐标轴(当前实测值):
 *
 *         0        20          180                  430      450
 *         │         │           │                    │        │
 *     HARD_MIN  SOFT_MIN      水平位              SOFT_MAX  HARD_MAX
 *    上电参考位   ├──── 合法工作区间 ────────────────┤   另一端机械端
 *
 * 上电后摆杆自重靠在 HARD_MIN 一端(计数 0),抬升是**正方向**运动到 180。
 */

/*
 * 限位总开关。启用后全程有效。
 * ⚠ 下面几个计数按实测填好之前必须保持 0——占位值在真实机械上多半立刻判越界,
 *   Tick 上电第一拍就会驱动摆杆往回赶。
 */
#define STEP_MOTOR_ENC_LIMIT_ENABLED 1U

/*
 * 两端机械极限的实测计数。**不直接生效**,只作记录,以及给上电抬升目标夹一道
 * "别顶上死点"的保险(那是物理事实,与限位开关无关)。
 * 量法:Device Check -> Step Motor -> SPAN 模式(自动失能),用手把摆杆推到两端,
 * ENTER 交替打 A/B 点,读到的原始计数即是。
 */
#define STEP_MOTOR_ENC_HARD_MIN_COUNTS 0
#define STEP_MOTOR_ENC_HARD_MAX_COUNTS 450

/*
 * 由机械极限向内收的安全余量,单位:编码器计数。要覆盖"判越界到真正停住"多走的
 * 那一段:Tick 周期 × 转速(10ms × 1333counts/s ≈ 13)+ 负载惯量过冲 + 机械公差。
 * 取太小会撞死点,取太大会白白吃掉行程(当前 40/450 ≈ 9%)。
 */
#define STEP_MOTOR_ENC_LIMIT_MARGIN_COUNTS 20

/*
 * **实际生效的软限位**,全驱动的位置限幅只认这两个值。
 * 默认由「机械极限向内收一个余量」算出;要两侧余量不对称就把右边改成裸常数。
 */
#define STEP_MOTOR_ENC_SOFT_MIN_COUNTS \
    (STEP_MOTOR_ENC_HARD_MIN_COUNTS + STEP_MOTOR_ENC_LIMIT_MARGIN_COUNTS)
#define STEP_MOTOR_ENC_SOFT_MAX_COUNTS \
    (STEP_MOTOR_ENC_HARD_MAX_COUNTS - STEP_MOTOR_ENC_LIMIT_MARGIN_COUNTS)

/*
 * 水管名义水平位的计数,即「从上电参考位到水平位」的估计距离。不是限位。
 *
 * **取 180**,与两处保持一致——三者必须同步,否则上电抬升会停在一个
 * 控制律并不认为是水平的姿态上:
 *   1. app/app_ball_config.h 的 APP_BALL_LINKAGE_MODEL_LEVEL_COUNTS = 180;
 *   2. app_ball_scurve_task.c 的 H3S_LINKAGE_TABLE 中 { 180, +0.000000f }。
 *
 * ⚠ 这只是**几何名义水平**,不是"球不动"的那个角。二者之差(实测约 +0.25°,
 *   见 docs/ball-scurve.md 的辨识一节)由 H3S_LEVEL_BIAS_DEG 承担。
 *   本值只决定上电抬升停在哪,H3 不把它当作 safe return。
 */
#define STEP_MOTOR_ENC_LEVEL_COUNTS 180

/* ===== 上电自动抬升 ===== */
/*
 * 上电后摆杆自重靠在一端、水管是歪的,控制器接管前得先抬到工作位。这一段由
 * StepMotor_Tick() 里的状态机非阻塞跑完:
 *
 *   上电 → Init 清零编码器(此处 = 参考位 = 0)
 *        → 等 DELAY_MS → 设 LIFT_SPEED、目标 LIFT_TARGET
 *        → |误差| ≤ TOLERANCE 即到位停住(按 HOLD 决定是否保持通电)
 *        → 超过 TIMEOUT 未到位 → 停机失能,不再重试
 *
 * 抬升期间独占目标:StepMotor_MoveToCount() 返回 BSP_STATUS_BUSY 并被忽略,
 * 上层可据此得知抬升尚未结束,或用 StepMotor_AbortStartup() 主动放弃。
 * StepMotor_Stop() 始终有效。
 */

/* 关掉则上电后一直失能,摆杆保持自重靠住的姿态。 */
#define STEP_MOTOR_STARTUP_LIFT_ENABLED 1U

/*
 * 抬升目标。默认就是水平位。下发前先按 HARD_MIN/MAX 夹一道(防止填错顶死点),
 * 再过软限位。
 */
#define STEP_MOTOR_STARTUP_LIFT_TARGET_COUNTS STEP_MOTOR_ENC_LEVEL_COUNTS

/*
 * 抬升转速上限,单位 deg/s。取中低速:抬升是带重力负载起步,快了容易丢步,
 * 而丢步会让"到位"判据失真。只走一次,不差这两秒。
 */
#define STEP_MOTOR_STARTUP_LIFT_SPEED_DEG_S 45.0f

/*
 * 抬升到位容差,单位:编码器计数。比伺服的 POSITION_TOLERANCE_COUNTS 宽一倍——
 * 抬升只需大致到位,判太紧会因惯量过冲而误判超时。
 */
#define STEP_MOTOR_STARTUP_LIFT_TOLERANCE_COUNTS 10

/*
 * 上电到开走之间的静默,单位 ms。给 OLED/串口/按键留就绪时间,
 * 也让操作者上电后有个照面——机器不该在上电瞬间就自己动起来。
 */
#define STEP_MOTOR_STARTUP_LIFT_DELAY_MS 800U

/*
 * 抬升超时,单位 ms,从真正开走起算(不含 DELAY)。超时 = 卡住 / 丢步 / 方向反了 /
 * 目标填错,此时停机失能。参考值取走完全程所需时间的 2~3 倍。
 */
#define STEP_MOTOR_STARTUP_LIFT_TIMEOUT_MS 8000U

/*
 * 到位后是否保持使能。默认 1——摆杆停在水平位靠的就是保持力矩,一断电立刻掉回去,
 * 抬了等于没抬。设 0 只在机械自锁时才合理。
 */
#define STEP_MOTOR_STARTUP_LIFT_HOLD 1U

/* ===== 越界纠正 ===== */
/*
 * 纠正回退的转速上限,单位 deg/s。取小值:越界纠正是异常处置,慢慢挪回来比甩回来
 * 安全,也不容易在边界上过冲成反向越界。
 */
#define STEP_MOTOR_GUARD_RECOVER_SPEED_DEG_S 60.0f

/*
 * 纠正的**滞回量**,单位:编码器计数。越界后目标设在「限位再向内 HYST 计数」,
 * 而不是边界本身。
 *
 * 直接设在边界上的话,摆杆会停在越界判据的临界点,任何抖动(机械回弹、编码器噪声、
 * 伺服容差带内的微动)都会让它反复跨过边界,守护于是在"越界→纠正→回界内→又越界"
 * 之间抽搐。留一段死区就不会连续触发。
 *
 * ⚠ 必须 < 半个合法区间,否则收敛目标落在区间外,纠正永不收敛,最终必然超时进 FAULT。
 */
#define STEP_MOTOR_GUARD_RECOVER_HYST_COUNTS 20

/*
 * 纠正超时,单位 ms,从发起纠正那一拍起算。超时未回到合法区间即判定不可恢复:
 * 停脉冲 + **断电** + 进 GUARD_FAULT 终态。
 *
 * 它防的主要是**极性搞反**:若 ENCODER_INVERT 或 POSITIVE_DIR_HIGH 反了,守护算出的
 * "朝内"实际朝外,摆杆会被加速推向死点并一直顶着,没有超时就会堵转到烧毁。
 * 其余触发场景:编码器断线(计数不动,误差永不收敛)、机械卡死。
 *
 * FAULT 是终态且断电,是因为不查明原因就自动重试只会让摆杆再往死点顶一次。
 * 恢复手段:重新上电(驱动不提供运行期清除接口)。
 */
#define STEP_MOTOR_GUARD_RECOVER_TIMEOUT_MS 3000U

/*
 * StepMotor_Tick() 的调用周期,单位 ms。上限受两处约束,取更严的一个:
 *   1) QEI 16 位环绕:相邻采样计数变化须 < 32767。满速 960deg/s ≈ 5333 counts/s,
 *      10ms 才 53 计数,余量三个数量级——不是瓶颈;
 *   2) 越界响应距离:判越界前摆杆最多多走「周期 × 转速」≈ 53 计数,已大于
 *      ENC_LIMIT_MARGIN_COUNTS(当前 20),高速验证时必须留意软限位余量或降低行程内实测速度。
 * ⚠ 第 2 条是真约束。改大本值必须同步加大 MARGIN。
 */
#define STEP_MOTOR_TICK_PERIOD_MS 10U

/**
 * @brief 限位守护状态。
 */
typedef enum {
    /** 限位未启用(STEP_MOTOR_ENC_LIMIT_ENABLED = 0),恒为此态。 */
    STEP_MOTOR_GUARD_DISABLED = 0,
    /** 限位启用且位置在范围内。 */
    STEP_MOTOR_GUARD_OK,
    /** 已越界但驱动器失能,无力矩可用——最常见的场景正是操作者主动失能手推摆杆
     *  量机械行程。只报状态,不擅自上电。 */
    STEP_MOTOR_GUARD_STALLED,
    /** 越上界,目标已设为 max-HYST,正在往负方向拉回。 */
    STEP_MOTOR_GUARD_RECOVER_NEG,
    /** 越下界,目标已设为 min+HYST,正在往正方向拉回。 */
    STEP_MOTOR_GUARD_RECOVER_POS,
    /** 纠正超时,已停机断电。终态,只能重新上电恢复。 */
    STEP_MOTOR_GUARD_FAULT
} STEP_MOTOR_GUARD_STATE;

/* ===== 生命周期 ===== */

/**
 * @brief 初始化步进输出与 QEI 计数,驱动器保持**失能**,编码器清零。
 * @note PWM/QEI/GPIO 外设本体由 SysConfig(SYSCFG_DL_init) 初始化;本函数额外补上
 *       生成代码缺失的 QEI startCounter。
 * @note 上电失能是刻意的:此刻摆杆在哪固件并不知道,通电就可能被误指令顶上死点。
 *       失能还让摆杆自重落到固定一端,给出可重复的位置参考——编码器就在这里清零,
 *       整套限位坐标系以它为原点。
 * @note 若启用了上电抬升,本函数只"上膛",实际抬升由随后的 StepMotor_Tick() 驱动
 *       (那时 SysTick 才就绪)。
 */
BSP_STATUS StepMotor_Init(void);

/**
 * @brief 驱动的唯一周期入口,须以 STEP_MOTOR_TICK_PERIOD_MS 持续调用。
 * @param now_ms 当前毫秒时间戳。
 *
 * 每拍按序做五件事,顺序不可换:
 *   1. 采样编码器(后续判断都基于最新计数;也是 QEI 16 位防环绕的硬要求)
 *   2. 上电抬升状态机 —— 只改目标
 *   3. 越界守护       —— 只改目标
 *   4. 位置伺服       —— 读最终目标,算速度,出脉冲
 *
 * @warning **不调它电机根本不会动**:位置指令只登记目标,脉冲在第 4 步下发。
 * @note 必须与应用状态无关地跑(注册在调度器而非某个任务里),这样限位保护和抬升
 *       在菜单、任务、故障态下都成立。
 */
void StepMotor_Tick(uint32_t now_ms);

/* ===== 运动指令(唯一入口)===== */

/**
 * @brief 设置目标位置,单位:编码器计数。
 * @param target_counts 目标计数;**一律先限幅**,落在软限位外的目标会被夹到边界,
 *                      不会被拒绝——上层多半是算出了个够不着的目标,走到边界停住
 *                      正是它想要的,拒绝执行反而更难处理。
 * @return 上电抬升进行中返回 BSP_STATUS_BUSY 并忽略本次指令。
 * @note 只登记目标,不阻塞;实际运动由 StepMotor_Tick() 逐拍逼近。
 *       顺带自动通电,调用方不必记得先使能。
 */
BSP_STATUS StepMotor_MoveToCount(int32_t target_counts);

/**
 * @brief 就地停住:把目标钉在当前实测位置并停脉冲。
 * @note 不改使能状态——摆杆多半正需要保持力矩撑着。
 */
BSP_STATUS StepMotor_Stop(void);

/**
 * @brief 设置伺服转速上限,单位 deg/s。会被夹到 (0, STEP_MOTOR_MAX_SPEED_DEG_S]。
 * @note 这是**上限**不是指令:误差大时伺服跑到这个速度,接近目标时自行减速。
 *       想让摆杆走慢点就调小它,而不是去给一个小的速度指令。
 */
BSP_STATUS StepMotor_SetSpeedLimit(float max_speed_deg_per_s);

/**
 * @brief 设置位置伺服比例增益，单位 1/s；不会改变位置与物理限位。
 */
BSP_STATUS StepMotor_SetServoGain(float kp_s_inv);

/** @brief 获取当前运行期位置伺服比例增益，单位 1/s。 */
float StepMotor_GetServoGain(void);

/* ===== 状态查询 ===== */

/**
 * @brief 获取编码器累计计数(已消除 16 位环绕,计数增大 = 电机正方向)。
 */
int32_t StepMotor_GetEncoderCount(void);

/**
 * @brief 获取当前目标位置,单位:编码器计数(已限幅后的值)。
 */
int32_t StepMotor_GetTargetCount(void);

/**
 * @brief 获取位置误差(目标 - 实测),单位:编码器计数。
 * @note 闭环下这就是**丢步的直接指标**:不丢步时它稳定在跟踪误差附近
 *       (约 speed/SERVO_KP 换算的计数),丢步时电机没走到、编码器落后,
 *       它会超出该值且不收敛。标定最大步进频率看的就是它的峰值。
 */
int32_t StepMotor_GetPositionErrorCount(void);

/**
 * @brief 是否已到达目标(误差在 STEP_MOTOR_POSITION_TOLERANCE_COUNTS 内)。
 */
bool StepMotor_IsAtTarget(void);

/**
 * @brief 编码器计数换算为电机轴角,单位 deg。
 */
float StepMotor_CountsToDeg(int32_t counts);

/**
 * @brief 转速换算为步进频率,单位 Hz。
 * @note 与驱动内部下发脉冲用的是同一个换算,上层要显示"这个转速对应多少 Hz"时
 *       必须用它,自己复制一份公式迟早会与驱动不同步。
 */
uint32_t StepMotor_SpeedToStepFreq(float speed_deg_per_s);

/* ===== 驱动器使能 ===== */

/**
 * @brief 使能/失能驱动器(控制 EN 脚)。
 * @note 失能后线圈断电、无保持力矩,摆杆会因自重回落。反过来,这也是**手推摆杆
 *       测量机械行程**的办法:失能后编码器照常计数,伺服不出力、守护也无从纠正。
 * @note 失能→使能的瞬间目标会被同步到当前实测位置,防止断电期间摆杆回落后
 *       一通电就被猛拽回旧目标。因此复能之后要动必须重新下位置指令。
 */
BSP_STATUS StepMotor_SetEnabled(bool enable);

/**
 * @brief 查询驱动器当前是否使能。
 */
bool StepMotor_IsEnabled(void);

/* ===== 异常可见性 ===== */

/**
 * @brief 获取限位守护状态。
 * @note 摆杆被自动纠正时,这是外部唯一能察觉的线索。
 *       GUARD_FAULT 为终态,只能重新上电恢复。
 */
STEP_MOTOR_GUARD_STATE StepMotor_GetGuardState(void);

/**
 * @brief 放弃上电抬升,把控制权立刻交还上层。
 * @note 就地停住但不改使能状态:抬到一半放弃时,摆杆多半正需要保持力矩撑着。
 */
void StepMotor_AbortStartup(void);

/* ===== 诊断(标定用)===== */

/**
 * @brief 获取伺服本拍实际下发的速度,单位 deg/s。
 * @note **只读诊断量**,不是指令——速度由伺服按位置误差算出,调用方无从设定。
 */
float StepMotor_GetSpeed(void);

/**
 * @brief 获取最近一次实际应用的步进频率,单位 Hz;0 表示未出脉冲。
 * @note 这是限幅后的真实值,与指令换算出的理论值可能不同(死区/上限),
 *       诊断转速不对时应先看它。
 */
uint32_t StepMotor_GetStepFrequencyHz(void);

/** @brief TIMG6 当前 LOAD/CC/CTR 寄存器，只用于诊断 PWM 装填时序。 */
uint32_t StepMotor_GetPwmLoadValue(void);
uint32_t StepMotor_GetPwmCompareValue(void);
uint32_t StepMotor_GetPwmCounterValue(void);

/** @brief 最近/启动以来最大 StepMotor_Tick 间隔，单位 ms。 */
uint32_t StepMotor_GetTickIntervalMs(void);
uint32_t StepMotor_GetMaxTickIntervalMs(void);

#ifdef __cplusplus
}
#endif

#endif /* STEP_MOTOR_H */
