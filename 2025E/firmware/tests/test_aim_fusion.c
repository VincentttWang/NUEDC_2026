/*
 * test_aim_fusion.c — S3 融合层主机单元测试。
 *
 * 覆盖视觉 bias(迁移后逻辑不变)、运动速率前馈(yaw/pitch)、指令装配。
 * 纯 core 依赖 (aim_fusion + aim_solver + kinematics), 不碰硬件。
 */
#include "aim_fusion/aim_fusion.h"

#include <math.h>
#include <stdio.h>

static int g_checks;
static int g_fails;

static void ExpectNear(const char *tag, float actual, float expected, float tol){
    g_checks++;
    if (fabsf(actual - expected) > tol){
        g_fails++;
        printf("  [FAIL] %-20s actual=%.5f expected=%.5f (tol=%.4f)\n",
               tag, (double)actual, (double)expected, (double)tol);
    }
}

#define TOL 0.02f

static KINEMATICS_POSE Pose(float x, float y, float h){
    KINEMATICS_POSE p = {x, y, h};
    return p;
}
static AIM_POINT2F Pt(float x, float y){
    AIM_POINT2F p = {x, y};
    return p;
}

/* F1 视觉 bias: 积分 + 限幅 + 丢失冻结 + dt<=0 忽略 */
static void F1_VisionBias(void){
    AIM_VISION_BIAS b;
    AimVisionBias_Init(&b, 0.5f, 1.0f);
    ExpectNear("F1.init.y", b.yaw_bias_deg, 0.0f, TOL);

    AimVisionBias_Update(&b, 2.0f, -1.0f, true, 1.0f);   /* +1.0 / -0.5 */
    ExpectNear("F1.step1.y", b.yaw_bias_deg, 1.0f, TOL);
    ExpectNear("F1.step1.p", b.pitch_bias_deg, -0.5f, TOL);

    AimVisionBias_Update(&b, 2.0f, 0.0f, true, 1.0f);    /* yaw 饱和到 +1.0 */
    ExpectNear("F1.clamp.y", b.yaw_bias_deg, 1.0f, TOL);

    AimVisionBias_Update(&b, 5.0f, 5.0f, false, 1.0f);   /* 丢失: 冻结 */
    ExpectNear("F1.frozen.y", b.yaw_bias_deg, 1.0f, TOL);
    ExpectNear("F1.frozen.p", b.pitch_bias_deg, -0.5f, TOL);

    AimVisionBias_Update(&b, 5.0f, 5.0f, true, 0.0f);    /* dt<=0: 忽略 */
    ExpectNear("F1.dt0.y", b.yaw_bias_deg, 1.0f, TOL);
}

/* F2 径向对靶 (Δβ=0): yaw_rate=−ω; pitch_rate 近距量级 */
static void F2_Radial(void){
    AIM_RATE_FF r = AimFusion_MotionRate(Pose(0, 0, 90.0f), Pt(0, 0.5f),
                                         0.1f, 0.3f, 30.0f);
    ExpectNear("F2.yaw", r.yaw_rate_deg_s, -30.0f, TOL);
    ExpectNear("F2.pitch", r.pitch_rate_deg_s, 6.6111f, TOL);
}

/* F3 纯横掠 (Δβ=90°, ω=0): yaw_rate=v/Rh; pitch_rate≈0 */
static void F3_Lateral(void){
    AIM_RATE_FF r = AimFusion_MotionRate(Pose(0, 0, 0.0f), Pt(0, 0.5f),
                                         0.1f, 0.3f, 0.0f);
    ExpectNear("F3.yaw", r.yaw_rate_deg_s, 34.3775f, TOL);
    ExpectNear("F3.pitch", r.pitch_rate_deg_s, 0.0f, TOL);
}

/* F4 原地自转 (v=0): yaw_rate=−ω, pitch_rate=0 */
static void F4_Spin(void){
    AIM_RATE_FF r = AimFusion_MotionRate(Pose(0.2f, -0.3f, 45.0f), Pt(0, 0.5f),
                                         0.1f, 0.0f, 45.0f);
    ExpectNear("F4.yaw", r.yaw_rate_deg_s, -45.0f, TOL);
    ExpectNear("F4.pitch", r.pitch_rate_deg_s, 0.0f, TOL);
}

/* F5 指令装配: yaw_cmd=ff+bias; rate 透传; bias=NULL 按 0 */
static void F5_Combine(void){
    AIM_SOLVER_RESULT ff = {10.0f, 5.0f, 1.0f, 1.0f};
    AIM_VISION_BIAS b = {1.0f, -0.5f, 0.5f, 3.0f};
    AIM_RATE_FF rate = {20.0f, 3.0f};

    AIM_FUSION_OUTPUT o = AimFusion_Combine(ff, &b, rate);
    ExpectNear("F5.yaw", o.yaw_cmd_deg, 11.0f, TOL);
    ExpectNear("F5.pitch", o.pitch_cmd_deg, 4.5f, TOL);
    ExpectNear("F5.yrate", o.yaw_rate_ff_deg_s, 20.0f, TOL);
    ExpectNear("F5.prate", o.pitch_rate_ff_deg_s, 3.0f, TOL);

    AIM_FUSION_OUTPUT o2 = AimFusion_Combine(ff, NULL, rate);
    ExpectNear("F5.null.yaw", o2.yaw_cmd_deg, 10.0f, TOL);
    ExpectNear("F5.null.pitch", o2.pitch_cmd_deg, 5.0f, TOL);
}

/* F6 远距 pitch_rate 变小: Rh=1.0 时约 1.70°/s (对照 F2 的 6.61) */
static void F6_PitchRateShrinks(void){
    AIM_RATE_FF r = AimFusion_MotionRate(Pose(0, -0.5f, 90.0f), Pt(0, 0.5f),
                                         0.1f, 0.3f, 0.0f);
    ExpectNear("F6.yaw", r.yaw_rate_deg_s, 0.0f, TOL);
    ExpectNear("F6.pitch", r.pitch_rate_deg_s, 1.7018f, TOL);
}

int main(void){
    F1_VisionBias();
    F2_Radial();
    F3_Lateral();
    F4_Spin();
    F5_Combine();
    F6_PitchRateShrinks();

    printf("[aim_fusion] PASS %d/%d\n", g_checks - g_fails, g_checks);
    return (g_fails == 0) ? 0 : 1;
}
