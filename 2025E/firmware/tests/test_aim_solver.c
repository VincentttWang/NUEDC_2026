/*
 * test_aim_solver.c — S1 几何瞄准解算器主机单元测试。
 *
 * 覆盖 2D 化后的 AimSolver: 位姿→yaw/pitch/range、mount 偏移旋转、
 * 画圆点解算、里程相位换算、角度归一化边界。期望值为手算 (见 aim-tracking-plan.md)。
 * 纯 core 依赖 (aim_solver + kinematics), 不碰硬件。
 */
#include "aim_solver/aim_solver.h"

#include <math.h>
#include <stdio.h>

static int g_checks;
static int g_fails;

static void ExpectNear(const char *tag, float actual, float expected, float tol){
    g_checks++;
    if (fabsf(actual - expected) > tol){
        g_fails++;
        printf("  [FAIL] %-22s actual=%.5f expected=%.5f (tol=%.4f)\n",
               tag, (double)actual, (double)expected, (double)tol);
    }
}

#define TOL_ANG 0.02f
#define TOL_POS 0.001f

static AIM_SOLVER_CONFIG DefaultCfg(void){
    return AimSolver_DefaultConfig();
}

/* T1 车正对靶: pose(0,0,90) → yaw0, pitch11.31, horiz0.5, range0.5099 */
static void T1_FacingTarget(void){
    AimSolver_Init(NULL);
    KINEMATICS_POSE pose = {0.0f, 0.0f, 90.0f};
    AIM_SOLVER_RESULT r = AimSolver_SolveCenter(pose);
    ExpectNear("T1.yaw", r.yaw_cmd_deg, 0.0f, TOL_ANG);
    ExpectNear("T1.pitch", r.pitch_cmd_deg, 11.3099f, TOL_ANG);
    ExpectNear("T1.horiz", r.horizontal_m, 0.5f, TOL_POS);
    ExpectNear("T1.range", r.range_m, 0.509902f, TOL_POS);
}

/* T2 横向偏: pose(-0.3,0,90) → yaw-30.96, pitch9.73 */
static void T2_LateralOffset(void){
    AimSolver_Init(NULL);
    KINEMATICS_POSE pose = {-0.3f, 0.0f, 90.0f};
    AIM_SOLVER_RESULT r = AimSolver_SolveCenter(pose);
    ExpectNear("T2.yaw", r.yaw_cmd_deg, -30.9638f, TOL_ANG);
    ExpectNear("T2.pitch", r.pitch_cmd_deg, 9.7327f, TOL_ANG);
    ExpectNear("T2.horiz", r.horizontal_m, 0.583095f, TOL_POS);
    ExpectNear("T2.range", r.range_m, 0.591608f, TOL_POS);
}

/* T3 车头朝+X: pose(0,0,0) → yaw90 (需左转90°), pitch11.31 */
static void T3_HeadingAlongX(void){
    AimSolver_Init(NULL);
    KINEMATICS_POSE pose = {0.0f, 0.0f, 0.0f};
    AIM_SOLVER_RESULT r = AimSolver_SolveCenter(pose);
    ExpectNear("T3.yaw", r.yaw_cmd_deg, 90.0f, TOL_ANG);
    ExpectNear("T3.pitch", r.pitch_cmd_deg, 11.3099f, TOL_ANG);
}

/* T4 mount 前移0.1: pose(0,0,90) → range缩短, yaw0, pitch14.04, horiz0.4 */
static void T4_MountForward(void){
    AIM_SOLVER_CONFIG cfg = DefaultCfg();
    cfg.mount_x_m = 0.1f;
    AimSolver_Init(&cfg);
    KINEMATICS_POSE pose = {0.0f, 0.0f, 90.0f};
    AIM_SOLVER_RESULT r = AimSolver_SolveCenter(pose);
    ExpectNear("T4.yaw", r.yaw_cmd_deg, 0.0f, TOL_ANG);
    ExpectNear("T4.pitch", r.pitch_cmd_deg, 14.0362f, TOL_ANG);
    ExpectNear("T4.horiz", r.horizontal_m, 0.4f, TOL_POS);
}

/* T5 画圆 φ=0: pose(0,0,90) → yaw-6.84, pitch11.23 */
static void T5_CirclePhase0(void){
    AimSolver_Init(NULL);
    KINEMATICS_POSE pose = {0.0f, 0.0f, 90.0f};
    AIM_SOLVER_RESULT r = AimSolver_SolveCircle(pose, 0.0f);
    ExpectNear("T5.yaw", r.yaw_cmd_deg, -6.8362f, TOL_ANG);
    ExpectNear("T5.pitch", r.pitch_cmd_deg, 11.2312f, TOL_ANG);
}

/* T6 画圆 φ=90: pose(0,0,90) → yaw0, pitch17.74 (向上摆) */
static void T6_CirclePhase90(void){
    AimSolver_Init(NULL);
    KINEMATICS_POSE pose = {0.0f, 0.0f, 90.0f};
    AIM_SOLVER_RESULT r = AimSolver_SolveCircle(pose, 90.0f);
    ExpectNear("T6.yaw", r.yaw_cmd_deg, 0.0f, TOL_ANG);
    ExpectNear("T6.pitch", r.pitch_cmd_deg, 17.7447f, TOL_ANG);
}

/* T7 CirclePoint 直接检查: φ0→(0.06,0.5)Δz0.10; φ90→(0,0.5)Δz0.16 */
static void T7_CirclePoint(void){
    AimSolver_Init(NULL);
    AIM_POINT2F xy;
    float dz;
    AimSolver_CirclePoint(0.0f, &xy, &dz);
    ExpectNear("T7.p0.x", xy.x_m, 0.06f, TOL_POS);
    ExpectNear("T7.p0.y", xy.y_m, 0.5f, TOL_POS);
    ExpectNear("T7.p0.dz", dz, 0.10f, TOL_POS);
    AimSolver_CirclePoint(90.0f, &xy, &dz);
    ExpectNear("T7.p90.x", xy.x_m, 0.0f, TOL_POS);
    ExpectNear("T7.p90.y", xy.y_m, 0.5f, TOL_POS);
    ExpectNear("T7.p90.dz", dz, 0.16f, TOL_POS);
}

/* T8 里程相位换算: (0,0)=0, (0.25,0)=90, (0.25,10)=100 */
static void T8_ProgressToPhase(void){
    ExpectNear("T8.a", AimSolver_ProgressToPhaseDeg(0.0f, 0.0f), 0.0f, TOL_ANG);
    ExpectNear("T8.b", AimSolver_ProgressToPhaseDeg(0.25f, 0.0f), 90.0f, TOL_ANG);
    ExpectNear("T8.c", AimSolver_ProgressToPhaseDeg(0.25f, 10.0f), 100.0f, TOL_ANG);
}

/* T9 yaw 归一化环绕: pose(0,0,-170) → norm(90-(-170))=norm(260)=-100 */
static void T9_YawWrap(void){
    AimSolver_Init(NULL);
    KINEMATICS_POSE pose = {0.0f, 0.0f, -170.0f};
    AIM_SOLVER_RESULT r = AimSolver_SolveCenter(pose);
    ExpectNear("T9.yaw", r.yaw_cmd_deg, -100.0f, TOL_ANG);
}

int main(void){
    T1_FacingTarget();
    T2_LateralOffset();
    T3_HeadingAlongX();
    T4_MountForward();
    T5_CirclePhase0();
    T6_CirclePhase90();
    T7_CirclePoint();
    T8_ProgressToPhase();
    T9_YawWrap();

    printf("[aim_solver] PASS %d/%d\n", g_checks - g_fails, g_checks);
    return (g_fails == 0) ? 0 : 1;
}
