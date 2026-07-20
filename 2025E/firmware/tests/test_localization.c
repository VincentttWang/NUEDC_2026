/*
 * test_localization.c — S2 位姿估计器主机单元测试。
 *
 * 覆盖航位推算、单轮车心重构、角点 snap/重锚、圈进度/圈数、组合入口。
 * 纯 core 依赖 (localization + kinematics), 不碰硬件。
 */
#include "localization/localization.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

static int g_checks;
static int g_fails;

static void ExpectNear(const char *tag, float actual, float expected, float tol){
    g_checks++;
    if (fabsf(actual - expected) > tol){
        g_fails++;
        printf("  [FAIL] %-22s actual=%.6f expected=%.6f (tol=%.5f)\n",
               tag, (double)actual, (double)expected, (double)tol);
    }
}

static void ExpectEqU32(const char *tag, uint32_t actual, uint32_t expected){
    g_checks++;
    if (actual != expected){
        g_fails++;
        printf("  [FAIL] %-22s actual=%u expected=%u\n", tag, actual, expected);
    }
}

#define TOL 0.0001f
#define HALF_PI_TENTH 0.15707963f /* rad(90)*0.10 = (pi/2)*0.1 = 0.05*pi */

static KINEMATICS_POSE Pose(float x, float y, float h){
    KINEMATICS_POSE p = {x, y, h};
    return p;
}

/* L1 直线推进 */
static void L1_Straight(void){
    Localization_Init(Pose(0, 0, 0), 1.0f, 0.0f);
    Localization_UpdateDelta(0.4f, 0.0f);
    KINEMATICS_POSE p = Localization_GetPose();
    ExpectNear("L1.x", p.x_m, 0.4f, TOL);
    ExpectNear("L1.y", p.y_m, 0.0f, TOL);
    ExpectNear("L1.travelled", Localization_GetTravelled(), 0.4f, TOL);
    ExpectNear("L1.lastdelta", Localization_GetState().last_center_delta_m, 0.4f, TOL);
}

/* L2 折线: 先沿 +X 再沿 +Y */
static void L2_Polyline(void){
    Localization_Init(Pose(0, 0, 0), 1.0f, 0.0f);
    Localization_UpdateDelta(0.5f, 0.0f);
    Localization_UpdateDelta(0.5f, 90.0f);
    KINEMATICS_POSE p = Localization_GetPose();
    ExpectNear("L2.x", p.x_m, 0.5f, TOL);
    ExpectNear("L2.y", p.y_m, 0.5f, TOL);
    ExpectNear("L2.head", p.heading_deg, 90.0f, TOL);
    ExpectNear("L2.travelled", Localization_GetTravelled(), 1.0f, TOL);
}

/* L3 单轮→车心: 左轮(+y)/右轮(−y)/直行/dt=0 */
static void L3_WheelCorrection(void){
    ExpectNear("L3.left", Localization_CorrectWheelDelta(0.09f, 90.0f, 0.10f, 1.0f),
               0.09f + HALF_PI_TENTH, TOL);
    ExpectNear("L3.right", Localization_CorrectWheelDelta(0.09f, 90.0f, -0.10f, 1.0f),
               0.09f - HALF_PI_TENTH, TOL);
    ExpectNear("L3.straight", Localization_CorrectWheelDelta(0.09f, 0.0f, 0.10f, 1.0f),
               0.09f, TOL);
    ExpectNear("L3.dt0", Localization_CorrectWheelDelta(0.09f, 90.0f, 0.10f, 0.0f),
               0.09f, TOL);
}

/* L4 四角世界坐标 (side=1.0) */
static void L4_Corners(void){
    Localization_Init(Pose(0, 0, 0), 1.0f, 0.0f);
    CORE_POINT2F a = Localization_CornerPoint(LOCALIZATION_CORNER_A);
    CORE_POINT2F b = Localization_CornerPoint(LOCALIZATION_CORNER_B);
    CORE_POINT2F c = Localization_CornerPoint(LOCALIZATION_CORNER_C);
    CORE_POINT2F d = Localization_CornerPoint(LOCALIZATION_CORNER_D);
    ExpectNear("L4.Ax", a.x, -0.5f, TOL); ExpectNear("L4.Ay", a.y, 0.0f, TOL);
    ExpectNear("L4.Bx", b.x, 0.5f, TOL);  ExpectNear("L4.By", b.y, 0.0f, TOL);
    ExpectNear("L4.Cx", c.x, -0.5f, TOL); ExpectNear("L4.Cy", c.y, -1.0f, TOL);
    ExpectNear("L4.Dx", d.x, 0.5f, TOL);  ExpectNear("L4.Dy", d.y, -1.0f, TOL);
}

/* L5 ResetToCorner 只动 x/y, 不动 travelled */
static void L5_ResetToCorner(void){
    Localization_Init(Pose(0, 0, 0), 1.0f, 0.0f);
    Localization_UpdateDelta(0.3f, 0.0f);
    Localization_ResetToCorner(LOCALIZATION_CORNER_B);
    KINEMATICS_POSE p = Localization_GetPose();
    ExpectNear("L5.x", p.x_m, 0.5f, TOL);
    ExpectNear("L5.y", p.y_m, 0.0f, TOL);
    ExpectNear("L5.travelled", Localization_GetTravelled(), 0.3f, TOL);
}

/* L6 ResetToCornerAtTravel: 渐变锚定(弧长立即重锚, 位置逐帧收敛到角点) */
static void L6_ResetAtTravel(void){
    Localization_Init(Pose(0, 0, 0), 1.0f, 0.0f);
    Localization_ResetToCornerAtTravel(LOCALIZATION_CORNER_B, 1.0f);
    /* 弧长/进度立即重锚。 */
    ExpectNear("L6.travelled", Localization_GetTravelled(), 1.0f, TOL);
    ExpectNear("L6.progress", Localization_GetLapProgress(), 0.25f, TOL);
    /* 位置渐变: 多拍 UpdateDelta(0) 后收敛到 B.x=0.5。 */
    for (int i = 0; i < 40; i++){
        Localization_UpdateDelta(0.0f, 0.0f);
    }
    KINEMATICS_POSE p = Localization_GetPose();
    ExpectNear("L6.x", p.x_m, 0.5f, TOL);
}

/* L7 满周长环绕: progress 回 0, lap_count=1 */
static void L7_LapWrap(void){
    Localization_Init(Pose(0, 0, 0), 1.0f, 0.0f);
    Localization_UpdateDelta(4.0f, 0.0f);   /* 周长 = 4*side = 4.0 */
    ExpectNear("L7.progress", Localization_GetLapProgress(), 0.0f, TOL);
    ExpectEqU32("L7.lapcount", Localization_GetLapCount(), 1U);
}

/* L8 Init 归一化航向 + track_side<=0 用默认 */
static void L8_InitBoundary(void){
    Localization_Init(Pose(0, 0, 270.0f), -1.0f, 0.0f);
    LOCALIZATION_STATE st = Localization_GetState();
    ExpectNear("L8.head", st.pose.heading_deg, -90.0f, TOL);   /* 270 → -90 */
    ExpectNear("L8.side", st.track_side_m, 1.0f, TOL);          /* <=0 → 默认 1.0 */
    ExpectNear("L8.perim", st.perimeter_m, 4.0f, TOL);
}

/* L9 组合入口 UpdateFromWheel == 分步结果 */
static void L9_UpdateFromWheel(void){
    Localization_Init(Pose(0, 0, 0), 1.0f, 0.0f);
    Localization_UpdateFromWheel(0.09f, 90.0f, 0.10f, 0.0f, 1.0f);
    KINEMATICS_POSE p = Localization_GetPose();
    ExpectNear("L9.x", p.x_m, 0.09f + HALF_PI_TENTH, TOL);   /* 车心增量沿航向0 */
    ExpectNear("L9.y", p.y_m, 0.0f, TOL);
    ExpectNear("L9.lastdelta", Localization_GetState().last_center_delta_m,
               0.09f + HALF_PI_TENTH, TOL);
}

int main(void){
    L1_Straight();
    L2_Polyline();
    L3_WheelCorrection();
    L4_Corners();
    L5_ResetToCorner();
    L6_ResetAtTravel();
    L7_LapWrap();
    L8_InitBoundary();
    L9_UpdateFromWheel();

    printf("[localization] PASS %d/%d\n", g_checks - g_fails, g_checks);
    return (g_fails == 0) ? 0 : 1;
}
