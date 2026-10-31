#include "ball_scurve.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void AssertNear(float actual, float expected, float tolerance);

static BALL_SCURVE_CONFIG Config(void){
    BALL_SCURVE_CONFIG config = {
        .rolling_acceleration_gain_mm_s2 = 7004.75f,
        .max_acceleration_mm_s2 = 200.0f,
        .max_velocity_mm_s = 90.0f,
        .min_duration_s = 0.40f,
        .max_duration_s = 3.00f,
        .kp_deg_per_mm = 0.04711f,
        .kd_deg_per_mm_s = 0.03533f,
        .feedback_limit_deg = 5.0f,
        .gain_schedule_enabled = 1.0f,
        .brake_kp_deg_per_mm = 0.03000f,
        .brake_kd_deg_per_mm_s = 0.03533f,
        .hold_kp_deg_per_mm = 0.04711f,
        .hold_kd_deg_per_mm_s = 0.02000f,
        .brake_delay_s = 0.22f,
        .brake_acceleration_mm_s2 = 120.0f,
        .brake_blend_start_ratio = 0.60f,
        .brake_blend_full_ratio = 1.00f,
        .brake_blend_tau_s = 0.0f,
        .hold_enter_error_mm = 15.0f,
        .hold_enter_speed_mm_s = 20.0f,
        .hold_enter_dwell_s = 0.0f,
        .hold_exit_error_mm = 30.0f,
        .hold_exit_speed_mm_s = 35.0f,
        .hold_blend_tau_s = 0.0f,
        .velocity_full_weight_age_ms = 60.0f,
        .velocity_floor_weight_age_ms = 120.0f,
        .velocity_untrusted_weight = 0.30f,
        .stationary_velocity_weight = 1.0f,
        .angle_min_deg = -5.2f,
        .angle_max_deg = 5.1f,
        .angle_rate_limit_deg_s = 1000.0f,
        .settled_position_mm = 3.0f,
        .settled_speed_mm_s = 5.0f,
        .settled_time_s = 0.5f,
    };
    return config;
}

static BALL_SCURVE_INPUT Input(float x_mm, float velocity_mm_s){
    BALL_SCURVE_INPUT input = {
        .x_mm = x_mm,
        .velocity_mm_s = velocity_mm_s,
        .vehicle_acceleration_mm_s2 = 0.0f,
        .actual_angle_deg = 0.0f,
        .velocity_trusted = true,
        .moving = false,
        .measurement_age_ms = 20.0f,
        .dt_s = 0.02f,
    };
    return input;
}

static void TestVehicleAccelerationFeedforward(void){
    BALL_SCURVE_CONFIG config = Config();
    BALL_SCURVE_CONTROLLER controller;
    BALL_SCURVE_OUTPUT output;
    BALL_SCURVE_INPUT input = Input(0.0f, 0.0f);
    const float gravity_mm_s2 = 9806.65f;
    const float rad_to_deg = 57.29577951308232f;

    BallScurve_Init(&controller);
    input.vehicle_acceleration_mm_s2 = 120.0f;
    assert(BallScurve_Update(&controller, &config, &input, &output));
    AssertNear(output.angle_deg,
               atanf(input.vehicle_acceleration_mm_s2 / gravity_mm_s2) * rad_to_deg,
               1e-5f);

    BallScurve_Init(&controller);
    input.vehicle_acceleration_mm_s2 = -90.0f;
    assert(BallScurve_Update(&controller, &config, &input, &output));
    AssertNear(output.angle_deg,
               atanf(input.vehicle_acceleration_mm_s2 / gravity_mm_s2) * rad_to_deg,
               1e-5f);
}

static void TestSmallErrorIntegralAccumulatesAndClearsOnMotion(void){
    BALL_SCURVE_CONFIG config = Config();
    config.hold_integral_ki_deg_per_mm_s = 0.10f;
    config.hold_integral_min_error_mm = 2.0f;
    config.hold_integral_max_error_mm = 15.0f;
    config.hold_integral_max_speed_mm_s = 3.0f;
    config.hold_integral_release_speed_mm_s = 5.0f;
    config.hold_integral_release_rate_deg_s = 8.0f;
    config.hold_integral_motion_comp_deg_per_mm = 0.40f;

    BALL_SCURVE_CONTROLLER controller;
    BALL_SCURVE_OUTPUT output;
    BallScurve_Init(&controller);
    controller.target_mm = 10.0f;
    BALL_SCURVE_INPUT input = Input(5.0f, 0.0f);

    /* 超过旧版 0.80° 上限，确认积分器自身不再钳位。 */
    for (unsigned i = 0U; i < 100U; i++){
        assert(BallScurve_Update(&controller, &config, &input, &output));
    }
    assert(output.hold_integral_on);
    AssertNear(output.hold_integral_deg, 1.00f, 1e-5f);
    assert(!output.breakout_on);

    input.moving = true;
    input.velocity_mm_s = 10.0f;
    assert(BallScurve_Update(&controller, &config, &input, &output));
    assert(!output.hold_integral_on);
    /* 8 deg/s 基础清零 + 0.4*10=4 deg/s 运动补偿。 */
    AssertNear(output.hold_integral_deg, 0.76f, 1e-5f);
    for (unsigned i = 0U; i < 6U; i++){
        assert(BallScurve_Update(&controller, &config, &input, &output));
    }
    AssertNear(output.hold_integral_deg, 0.0f, 1e-6f);

    input.moving = false;
    input.x_mm = 8.5f; /* 误差 <= 2 mm：不再积分。 */
    assert(BallScurve_Update(&controller, &config, &input, &output));
    assert(!output.hold_integral_on);
    AssertNear(output.hold_integral_deg, 0.0f, 1e-6f);
}

static void TestLargeErrorUsesBreakoutInsteadOfIntegral(void){
    BALL_SCURVE_CONFIG config = Config();
    config.hold_integral_ki_deg_per_mm_s = 0.10f;
    config.hold_integral_min_error_mm = 2.0f;
    config.hold_integral_max_error_mm = 15.0f;
    config.hold_integral_max_speed_mm_s = 3.0f;
    config.hold_integral_release_speed_mm_s = 5.0f;
    config.hold_integral_release_rate_deg_s = 8.0f;
    config.hold_integral_motion_comp_deg_per_mm = 0.40f;
    config.breakout_max_angle_deg = 1.4f;
    config.breakout_ramp_rate_deg_s = 1.0f;
    config.breakout_release_rate_deg_s = 8.0f;
    config.breakout_min_error_mm = 2.0f;
    config.breakout_max_speed_mm_s = 5.0f;
    config.breakout_dwell_s = 0.0f;
    config.breakout_release_speed_mm_s = 6.0f;
    config.breakout_release_dwell_s = 0.1f;

    BALL_SCURVE_CONTROLLER controller;
    BALL_SCURVE_OUTPUT output;
    BallScurve_Init(&controller);
    controller.target_mm = 30.0f;
    BALL_SCURVE_INPUT input = Input(0.0f, 0.0f);
    assert(BallScurve_Update(&controller, &config, &input, &output));
    assert(output.breakout_on);
    assert(output.breakout_deg > 0.0f);
    assert(!output.hold_integral_on);
    AssertNear(output.hold_integral_deg, 0.0f, 1e-6f);
}

static void TestAccelerationPreviewLeadsNominalFeedforward(void){
    BALL_SCURVE_CONFIG config = Config();
    config.acceleration_preview_s = 0.12f;
    BALL_SCURVE_CONTROLLER controller;
    BALL_SCURVE_OUTPUT output;
    BallScurve_Init(&controller);
    assert(BallScurve_PlanTo(&controller, &config, 0.0f, 0.0f, 50.0f));
    BALL_SCURVE_INPUT input = Input(0.0f, 0.0f);

    assert(BallScurve_Update(&controller, &config, &input, &output));
    assert(fabsf(output.acceleration_preview_mm_s2 - output.a_ref_mm_s2) > 1.0f);
    assert(fabsf(output.actuator_lead_deg) > 0.001f);
}

static void TestReplanRequiresDwellAndCooldown(void){
    BALL_SCURVE_CONFIG config = Config();
    config.replan_error_mm = 1.0f;
    config.replan_velocity_error_mm_s = 0.0f;
    config.replan_dwell_s = 0.04f;
    config.replan_cooldown_s = 0.04f;
    BALL_SCURVE_CONTROLLER controller;
    BALL_SCURVE_OUTPUT output;
    BallScurve_Init(&controller);
    assert(BallScurve_PlanTo(&controller, &config, 0.0f, 0.0f, 50.0f));
    BALL_SCURVE_INPUT input = Input(-10.0f, 0.0f);

    assert(BallScurve_Update(&controller, &config, &input, &output));
    assert(!output.replanned);
    assert(BallScurve_Update(&controller, &config, &input, &output));
    assert(output.replanned);
    AssertNear(controller.elapsed_s, 0.0f, 1e-6f);

    assert(BallScurve_Update(&controller, &config, &input, &output));
    assert(!output.replanned); /* 新剖面仍在冷却期。 */
}

static void TestCaptureBlendsToFixedTarget(void){
    BALL_SCURVE_CONFIG config = Config();
    config.capture_enter_error_mm = 20.0f;
    config.capture_full_error_mm = 5.0f;
    config.capture_blend_tau_s = 0.0f;
    BALL_SCURVE_CONTROLLER controller;
    BALL_SCURVE_OUTPUT output;
    BallScurve_Init(&controller);
    assert(BallScurve_PlanTo(&controller, &config, 0.0f, 0.0f, 50.0f));
    BALL_SCURVE_INPUT input = Input(48.0f, 0.0f);

    assert(BallScurve_Update(&controller, &config, &input, &output));
    AssertNear(output.capture_blend, 1.0f, 1e-6f);
    AssertNear(output.x_ref_mm, 50.0f, 1e-6f);
    AssertNear(output.v_ref_mm_s, 0.0f, 1e-6f);
    AssertNear(output.a_ref_mm_s2, 0.0f, 1e-6f);
}

static void AssertNear(float actual, float expected, float tolerance){
    assert(fabsf(actual - expected) <= tolerance);
}

static void TestDisabledMatchesFixedPd(void){
    BALL_SCURVE_CONFIG config = Config();
    config.gain_schedule_enabled = 0.0f;
    BALL_SCURVE_CONTROLLER controller;
    BALL_SCURVE_OUTPUT output;
    BallScurve_Init(&controller);
    BALL_SCURVE_INPUT input = Input(10.0f, 5.0f);
    input.velocity_trusted = false;
    input.measurement_age_ms = 150.0f;

    assert(BallScurve_Update(&controller, &config, &input, &output));
    AssertNear(output.effective_kp_deg_per_mm, config.kp_deg_per_mm, 1e-6f);
    AssertNear(output.effective_kd_deg_per_mm_s, config.kd_deg_per_mm_s, 1e-6f);
    AssertNear(output.velocity_weight, 1.0f, 1e-6f);
    AssertNear(output.feedback_deg,
               config.kp_deg_per_mm * -10.0f +
               config.kd_deg_per_mm_s * -5.0f, 1e-5f);
    assert(output.gain_mode == BALL_SCURVE_GAIN_MOVE);
}

static void TestBrakeSchedule(void){
    BALL_SCURVE_CONFIG config = Config();
    BALL_SCURVE_CONTROLLER controller;
    BALL_SCURVE_OUTPUT output;
    BallScurve_Init(&controller);
    assert(BallScurve_PlanTo(&controller, &config, 0.0f, 60.0f, 100.0f));
    BALL_SCURVE_INPUT input = Input(80.0f, 60.0f);

    assert(BallScurve_Update(&controller, &config, &input, &output));
    assert(output.stopping_distance_mm > 20.0f);
    AssertNear(output.brake_blend, 1.0f, 1e-6f);
    AssertNear(output.effective_kp_deg_per_mm, config.brake_kp_deg_per_mm, 1e-6f);
    assert(output.gain_mode == BALL_SCURVE_GAIN_BRAKE);
}

static void TestHoldHysteresisAndVelocityWeight(void){
    BALL_SCURVE_CONFIG config = Config();
    BALL_SCURVE_CONTROLLER controller;
    BALL_SCURVE_OUTPUT output;
    BallScurve_Init(&controller);
    BALL_SCURVE_INPUT input = Input(10.0f, 0.0f);

    assert(BallScurve_Update(&controller, &config, &input, &output));
    AssertNear(output.hold_blend, 1.0f, 1e-6f);
    AssertNear(output.effective_kd_deg_per_mm_s, config.hold_kd_deg_per_mm_s, 1e-6f);
    assert(output.gain_mode == BALL_SCURVE_GAIN_HOLD);

    input.x_mm = 20.0f;
    input.velocity_mm_s = 0.0f;
    input.measurement_age_ms = 90.0f;
    assert(BallScurve_Update(&controller, &config, &input, &output));
    assert(output.gain_mode == BALL_SCURVE_GAIN_HOLD);
    AssertNear(output.velocity_weight, 0.65f, 1e-5f);

    input.x_mm = 31.0f;
    assert(BallScurve_Update(&controller, &config, &input, &output));
    AssertNear(output.hold_blend, 0.0f, 1e-6f);
    assert(output.gain_mode == BALL_SCURVE_GAIN_MOVE);

    BallScurve_Init(&controller);
    input = Input(10.0f, 0.0f);
    input.velocity_trusted = false;
    assert(BallScurve_Update(&controller, &config, &input, &output));
    assert(output.gain_mode == BALL_SCURVE_GAIN_MOVE);

    config.stationary_velocity_weight = 0.10f;
    BallScurve_Init(&controller);
    input = Input(10.0f, 0.0f);
    assert(BallScurve_Update(&controller, &config, &input, &output));
    AssertNear(output.velocity_weight, 0.10f, 1e-6f);
}

static void TestNewSegmentClearsBreakout(void){
    BALL_SCURVE_CONFIG config = Config();
    config.breakout_max_angle_deg = 1.4f;
    config.breakout_ramp_rate_deg_s = 1.0f;
    config.breakout_release_rate_deg_s = 8.0f;
    config.breakout_min_error_mm = 2.0f;
    config.breakout_max_speed_mm_s = 5.0f;
    config.breakout_dwell_s = 0.0f;
    config.breakout_release_speed_mm_s = 6.0f;
    config.breakout_release_dwell_s = 0.1f;

    BALL_SCURVE_CONTROLLER controller;
    BALL_SCURVE_OUTPUT output;
    BallScurve_Init(&controller);
    controller.target_mm = 50.0f;
    BALL_SCURVE_INPUT input = Input(20.0f, 0.0f);

    assert(BallScurve_Update(&controller, &config, &input, &output));
    assert(output.breakout_on);
    assert(output.breakout_deg > 0.0f);

    assert(BallScurve_PlanTo(&controller, &config, input.x_mm,
                             input.velocity_mm_s, -50.0f));
    AssertNear(controller.breakout_angle_deg, 0.0f, 1e-6f);
    AssertNear(controller.breakout_stuck_elapsed_s, 0.0f, 1e-6f);
    AssertNear(controller.breakout_release_elapsed_s, 0.0f, 1e-6f);
    assert(!controller.breakout_on);
}

int main(void){
    TestVehicleAccelerationFeedforward();
    TestAccelerationPreviewLeadsNominalFeedforward();
    TestReplanRequiresDwellAndCooldown();
    TestCaptureBlendsToFixedTarget();
    TestSmallErrorIntegralAccumulatesAndClearsOnMotion();
    TestLargeErrorUsesBreakoutInsteadOfIntegral();
    TestDisabledMatchesFixedPd();
    TestBrakeSchedule();
    TestHoldHysteresisAndVelocityWeight();
    TestNewSegmentClearsBreakout();
    puts("ball_scurve gain schedule tests passed");
    return 0;
}
