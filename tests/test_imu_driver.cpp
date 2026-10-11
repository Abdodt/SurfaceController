#include <gtest/gtest.h>
#include "surface_controller/drivers/ImuDriver.hpp"
#include "surface_controller/drivers/SimulatedImuDriver.hpp"
#include "surface_controller/hal/SimulatedHal.hpp"

using namespace surface_controller;
using namespace surface_controller::drivers;
using namespace surface_controller::hal;

TEST(ImuDriverTest, SpiImuInitAndReadSample) {
    MockSpiBus spi;
    SpiImuDriver driver(spi);

    EXPECT_EQ(driver.init(), Status::OK);
    EXPECT_EQ(driver.selfTest(), Status::OK);

    // Prepare simulated raw buffer:
    // accel_x = 0, accel_y = 0, accel_z = 2048 (+1g = 9.80665 m/s^2)
    // temp = 0
    // gyro_x = 0, gyro_y = 0, gyro_z = 0
    uint8_t raw_data[14] = {
        0x00, 0x00, // AX = 0
        0x00, 0x00, // AY = 0
        0x08, 0x00, // AZ = 2048 (+1g)
        0x00, 0x00, // Temp
        0x00, 0x00, // GX = 0
        0x00, 0x00, // GY = 0
        0x00, 0x00  // GZ = 0
    };
    spi.setRegisters(SpiImuDriver::REG_ACCEL_XOUT_H, raw_data, 14);

    auto result = driver.readSample();
    ASSERT_TRUE(result.is_ok());
    const auto& sample = result.value();

    EXPECT_TRUE(sample.is_valid);
    EXPECT_NEAR(sample.accel_m_s2.z, 9.80665f, 0.05f);
    EXPECT_NEAR(sample.gyro_rad_s.x, 0.0f, 1e-4);
}

TEST(ImuDriverTest, SimulatedImuCalibrationRemovesBias) {
    SimulatedImuDriver sim_imu;
    sim_imu.init();
    EXPECT_FALSE(sim_imu.isCalibrated());

    sim_imu.startCalibration(50);
    for (int i = 0; i < 50; ++i) {
        auto sample_res = sim_imu.readSample();
        ASSERT_TRUE(sample_res.is_ok());
        sim_imu.processCalibrationSample(sample_res.value());
    }

    EXPECT_TRUE(sim_imu.isCalibrated());

    // After calibration, sample rates while stationary should be near 0
    auto post_cal_sample = sim_imu.readSample();
    ASSERT_TRUE(post_cal_sample.is_ok());
    EXPECT_NEAR(post_cal_sample.value().gyro_rad_s.x, 0.0f, 1e-4);
    EXPECT_NEAR(post_cal_sample.value().gyro_rad_s.y, 0.0f, 1e-4);
    EXPECT_NEAR(post_cal_sample.value().gyro_rad_s.z, 0.0f, 1e-4);
}
