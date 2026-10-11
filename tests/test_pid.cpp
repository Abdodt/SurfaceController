#include <gtest/gtest.h>
#include "surface_controller/control/PidController.hpp"

using namespace surface_controller::control;

class PidControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        PidGains gains;
        gains.kp = 1.0f;
        gains.ki = 2.0f;
        gains.kd = 0.1f;
        gains.integral_min = -0.5f;
        gains.integral_max = 0.5f;
        gains.output_min = -1.0f;
        gains.output_max = 1.0f;
        gains.derivative_filter_hz = 20.0f;

        pid_.configure(gains, 50.0f);
    }

    PidController pid_;
};

TEST_F(PidControllerTest, ProportionalActionOnStepError) {
    // Zero initial state, error = 0.5 - 0.0 = 0.5
    // On first step, derivative is suppressed to prevent kick
    float out = pid_.update(0.5f, 0.0f, 0.02f);
    
    // P term = 1.0 * 0.5 = 0.5
    // First update does not integrate yet
    EXPECT_FLOAT_EQ(pid_.getP(), 0.5f);
    EXPECT_NEAR(out, 0.5f, 1e-4);
}

TEST_F(PidControllerTest, IntegralAccumulationAndAntiWindup) {
    // Apply constant error of 0.2 over multiple steps
    // dt = 0.02s, ki = 2.0 -> integral increases by 2.0 * 0.2 * 0.02 = 0.008 per step
    for (int i = 0; i < 100; ++i) {
        pid_.update(0.2f, 0.0f, 0.02f);
    }

    // Integral should clamp at integral_max (0.5)
    EXPECT_FLOAT_EQ(pid_.getI(), 0.5f);
    EXPECT_FALSE(pid_.getI() > 0.5f);

    // Negative integration test
    pid_.reset();
    for (int i = 0; i < 100; ++i) {
        pid_.update(-0.3f, 0.0f, 0.02f);
    }
    EXPECT_FLOAT_EQ(pid_.getI(), -0.5f);
}

TEST_F(PidControllerTest, DerivativeOnMeasurementPreventsSetpointKick) {
    // When setpoint jumps from 0 to 10 but measurement remains 0,
    // derivative-on-measurement must NOT produce an infinite or large spike!
    float out1 = pid_.update(0.0f, 0.0f, 0.02f);
    float out2 = pid_.update(10.0f, 0.0f, 0.02f); // Setpoint step jump

    // Derivative term depends on measurement delta (0 - 0 = 0), not setpoint jump!
    EXPECT_NEAR(pid_.getD(), 0.0f, 1e-3);
    EXPECT_NEAR(out2, 1.0f, 1e-3); // Clamped at output_max
}

TEST_F(PidControllerTest, DerivativeDampingOnMeasurementRate) {
    // Setpoint constant, measurement increases by 1.0 per 0.02s (rate = 50.0 rad/s)
    pid_.update(0.0f, 0.0f, 0.02f);
    pid_.update(0.0f, 1.0f, 0.02f);

    // Derivative on measurement opposes motion: D term should be negative
    EXPECT_LT(pid_.getD(), 0.0f);
}

TEST_F(PidControllerTest, OutputSaturationLimits) {
    // Huge error
    float out_high = pid_.update(100.0f, 0.0f, 0.02f);
    EXPECT_FLOAT_EQ(out_high, 1.0f);
    EXPECT_TRUE(pid_.isSaturated());

    float out_low = pid_.update(-100.0f, 0.0f, 0.02f);
    EXPECT_FLOAT_EQ(out_low, -1.0f);
    EXPECT_TRUE(pid_.isSaturated());
}

TEST_F(PidControllerTest, ResetClearsAllState) {
    pid_.update(0.5f, 0.0f, 0.02f);
    pid_.update(0.5f, 0.0f, 0.02f);
    EXPECT_NE(pid_.getOutput(), 0.0f);

    pid_.reset();
    EXPECT_FLOAT_EQ(pid_.getP(), 0.0f);
    EXPECT_FLOAT_EQ(pid_.getI(), 0.0f);
    EXPECT_FLOAT_EQ(pid_.getD(), 0.0f);
    EXPECT_FLOAT_EQ(pid_.getOutput(), 0.0f);
    EXPECT_FALSE(pid_.isSaturated());
}
