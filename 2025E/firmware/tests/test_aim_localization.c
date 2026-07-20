#include "aim_solver/aim_solver.h"
#include "localization/localization.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static int Near(float actual, float expected, float tolerance){
    return fabsf(actual - expected) <= tolerance;
}

static void TestStraightAndCornerAnchor(void){
    KINEMATICS_POSE start = {-0.5f, 0.0f, 0.0f};
    Localization_Init(start, 1.0f, 0.0f);
    Localization_UpdateDelta(0.4f, 0.0f);

    KINEMATICS_POSE pose = Localization_GetPose();
    assert(Near(pose.x_m, -0.1f, 0.0001f));
    assert(Near(pose.y_m, 0.0f, 0.0001f));

    Localization_ResetToCornerAtTravel(LOCALIZATION_CORNER_B, 1.0f);
    /* 渐变锚定: 弧长/进度立即重锚, 位置逐帧收敛到角点。 */
    assert(Near(Localization_GetTravelled(), 1.0f, 0.0001f));
    assert(Near(Localization_GetLapProgress(), 0.25f, 0.0001f));
    for (int i = 0; i < 40; i++){
        Localization_UpdateDelta(0.0f, 0.0f);
    }
    pose = Localization_GetPose();
    assert(Near(pose.x_m, 0.5f, 0.0001f));
    assert(Near(pose.y_m, 0.0f, 0.0001f));
}

static void TestSingleWheelCorrection(void){
    float corrected = Localization_CorrectWheelDelta(0.09f, 90.0f, 0.10f, 1.0f);
    assert(Near(corrected, 0.09f + 0.05f * KINEMATICS_PI, 0.0001f));
    assert(Near(Localization_CorrectWheelDelta(0.09f, 90.0f, 0.10f, 0.0f),
                0.09f, 0.0001f));
}

static void TestAimAndCirclePhase(void){
    KINEMATICS_POSE pose = {0.0f, 0.0f, 0.0f};
    AimSolver_Init(NULL);
    AIM_SOLVER_RESULT result = AimSolver_SolveCenter(pose);

    assert(Near(result.yaw_cmd_deg, 90.0f, 0.001f));
    assert(result.pitch_cmd_deg > 0.0f);
    assert(Near(AimSolver_ProgressToPhaseDeg(0.25f, 10.0f), 100.0f, 0.001f));
}

int main(void){
    TestStraightAndCornerAnchor();
    TestSingleWheelCorrection();
    TestAimAndCirclePhase();
    puts("aim/localization tests passed");
    return 0;
}
