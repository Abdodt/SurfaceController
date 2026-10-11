#include <gtest/gtest.h>
#include "surface_controller/control/FlightSurfaceController.hpp"

using namespace surface_controller;
using namespace surface_controller::control;

TEST(FlightControllerTest, MultiAxisStabilization) {
    FlightSurfaceController controller;

    FlightSetpoints sp;
    sp.pitch_rate_rad_s = 0.5f;
    sp.roll_rate_rad_s = -0.4f;
    sp.yaw_rate_rad_s = 0.2f;

    ImuSample imu;
    imu.gyro_rad_s = {0.0f, 0.0f, 0.0f}; // Stationary initial state
    imu.accel_m_s2 = {0.0f, 0.0f, 9.80665f};
    imu.is_valid = true;

    SurfaceDeflections defl = controller.update(sp, imu, 0.02f);

    // With positive pitch setpoint and zero measurement, elevator should deflect positive
    EXPECT_GT(defl.elevator, 0.0f);

    // With negative roll setpoint, left aileron should deflect negative
    EXPECT_LT(defl.aileron_left, 0.0f);
    EXPECT_GT(defl.aileron_right, 0.0f);

    // With positive yaw setpoint, rudder should deflect positive
    EXPECT_GT(defl.rudder, 0.0f);
}

TEST(FlightControllerTest, TimingJitterTracking) {
    FlightSurfaceController controller;

    // Nominal period is 20,000 us (50 Hz)
    // 1st cycle: 20,050 us -> +50 us jitter
    controller.recordCycleTiming(20050, 120);

    // 2nd cycle: 19,920 us -> -80 us jitter
    controller.recordCycleTiming(19920, 110);

    // 3rd cycle: 20,400 us -> +400 us jitter
    controller.recordCycleTiming(20400, 140);

    const auto& metrics = controller.getTimingMetrics();
    EXPECT_EQ(metrics.cycle_count, 3);
    EXPECT_EQ(metrics.min_period_us, 19920);
    EXPECT_EQ(metrics.max_period_us, 20400);
    EXPECT_EQ(metrics.max_jitter_us, 400);
    EXPECT_EQ(metrics.max_execution_time_us, 140);

    // Verify jitter requirement: < 1000 us (1 ms)
    EXPECT_LT(metrics.max_jitter_us, 1000);
}
