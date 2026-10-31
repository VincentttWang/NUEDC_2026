/**
 * @file  app_init.c
 * @brief 上电初始化时序：SysConfig → BSP → 中间件 → 框架 → 注册调度任务。
 */
#include "app_init.h"
#include "app_mode.h"
#include "app_scheduler.h"
#include "app_track_tune.h"

#include "ti_msp_dl_config.h"
#include "bsp_time.h"
#include "debug_uart.h"
#include "rpi_uart.h"
#include "ui.h"
#include "key.h"
#include "chassis.h"
#include "step_motor.h"
#include "system_fault.h"

/* 控制/UI 两个调度任务的周期，单位 ms。 */
/*
 * 控制周期 10ms(100Hz)：对视觉 60fps 做**过采样**，而不是去对齐它。
 *
 * 对齐（控制环也跑 ~60Hz）看似省事，实际是错的：MCU 与相机是两套独立时钟，
 * 40kHz 定时器基频（4MHz 无因子 3）根本除不出精确 60Hz，最接近的 59.97Hz 与
 * 相机之间会产生 0.03Hz 拍频——测量龄期缓慢扫过整个帧周期并周期性丢帧/重帧，
 * 而这个拍频正落在滚球闭环 wn=2.4rad/s(0.38Hz) 的带宽内，控制器会去追它。
 * 过采样没有这个问题：每帧必在 1 拍内被消费，龄期上界 = 控制周期。
 *
 * 帧龄：Rpi_UART 每 2ms 收干净 + 最多 10ms 等到下一控制拍 ≈ 12ms 上界
 *       （20ms 控制周期时是 22ms）。
 *
 * ⚠ 改本值必须同步：app_mode.c 的 APP_CONTROL_DT，以及所有按拍生效的一阶
 *   低通系数（chassis 的 SPEED_FEEDBACK_ALPHA、line_follow 的 ERROR_LPF_ALPHA）。
 *   编码器测速窗**不跟随**，理由见 hall_encoder.h。
 */
#define APP_CONTROL_PERIOD_MS 10U
#define APP_UI_PERIOD_MS      50U
#define APP_RPI_POLL_PERIOD_MS 2U

/*
 * 步进电机的调度周期，取自驱动给出的值，与控制周期同为 10ms。
 * 该值由驱动侧独立约束决定（QEI 16 位防环绕 + 越界守护的响应距离，
 * 见 step_motor.h），不随控制周期变化。
 */
#define APP_STEP_TICK_PERIOD_MS STEP_MOTOR_TICK_PERIOD_MS

/*
 * 步进电机的周期入口：位置伺服 + 上电抬升 + 限位守护 + 编码器采样。
 * 与应用状态无关地跑——限位保护在菜单里、任务里、故障态下都得成立，
 * 而且**不调它电机根本不会动**（位置指令只登记目标，脉冲由这里的伺服下发）。
 */
static void App_StepMotorTick(void){
    StepMotor_Tick(BSP_Time_GetMs());
}

/* UART2 ISR 只搬运 RX/TX 字节；协议解析和调参逻辑留在主循环上下文。 */
static void App_RpiUartTick(void){
    RpiUart_Poll();
    AppTrackTune_Poll(App_Mode_Get() == APP_MODE_MENU);
}

void App_Init(void){
    SYSCFG_DL_init();
    BSP_Time_Init();
    DebugUart_Init();   /* 非阻塞 debug 串口(Debug_Ex/UART1): Speed PID 等遥测输出用。 */
    RpiUart_Init();     /* 树莓派视觉专线 Rpi_UART/UART2：PA24 RX，115200 8N1。 */
    AppTrackTune_Init(); /* H2/H5/H6 现场参数仅驻留 RAM，上电恢复编译期默认值。 */

    /*
     * Ui/Chassis 先于任何可能触发 SystemFault 的步骤初始化，
     * 满足 SystemFault_Halt 需 Ui/Chassis 已就绪的前置契约。
     */
    Ui_Init();
    if (Chassis_Init() != BSP_STATUS_OK){
        SystemFault_Handler(SYSTEM_FAULT_HARDWARE, "Chassis init");  /* 终态，不返回 */
    }
    Key_Init();

    /*
     * 摆杆步进：上电即失能（见 StepMotor_Init），行程未标定前不给摆杆任何力矩。
     * 失败不算致命——底盘部分照常可用，摆杆故障会在 Device Check 页上暴露出来。
     */
    (void)StepMotor_Init();

    /* 框架：状态机 + 调度任务表。 */
    App_Mode_Init();
    Scheduler_Init();
    (void)Scheduler_AddTask(App_ControlTick, APP_CONTROL_PERIOD_MS);
    (void)Scheduler_AddTask(App_UiTick, APP_UI_PERIOD_MS);
    (void)Scheduler_AddTask(App_StepMotorTick, APP_STEP_TICK_PERIOD_MS);
    (void)Scheduler_AddTask(App_RpiUartTick, APP_RPI_POLL_PERIOD_MS);

    /* 全部就绪后放开 SysTick 时基/按键扫描。 */
    Scheduler_EnableTick();
}
