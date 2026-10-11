#ifndef SURFACE_CONTROLLER_DRIVERS_SIMULATED_IMU_DRIVER_HPP
#define SURFACE_CONTROLLER_DRIVERS_SIMULATED_IMU_DRIVER_HPP

#include "surface_controller/drivers/ImuDriver.hpp"
#include <cmath>

namespace surface_controller {
namespace drivers {

/**
 * @brief High-fidelity simulated IMU for SITL simulation and unit testing.
 * Supports configurable flight dynamics, realistic noise, and fault injection.
 */
class SimulatedImuDriver : public IImuDriver {
public:
    enum class FaultMode {
        NONE,
        COMM_TIMEOUT,
        STUCK_DATA,
        SATURATION,
        HIGH_NOISE
    };

    SimulatedImuDriver() = default;
    ~SimulatedImuDriver() override = default;

    Status init() override {
        is_initialized_ = true;
        return Status::OK;
    }

    Status selfTest() override {
        return (fault_mode_ == FaultMode::COMM_TIMEOUT) ? Status::HARDWARE_FAULT : Status::OK;
    }

    Result<ImuSample> readSample() override;

    Status startCalibration(uint32_t total_samples) override {
        cal_target_ = total_samples;
        cal_count_ = 0;
        cal_gyro_acc_.reset();
        cal_accel_acc_.reset();
        in_calibration_ = true;
        is_calibrated_ = false;
        return Status::OK;
    }

    bool processCalibrationSample(const ImuSample& sample) override {
        if (!in_calibration_) return is_calibrated_;

        cal_gyro_acc_ += sample.gyro_rad_s;
        cal_accel_acc_ += sample.accel_m_s2;
        ++cal_count_;

        if (cal_count_ >= cal_target_) {
            gyro_bias_ = cal_gyro_acc_ / static_cast<float>(cal_count_);
            // Accelerometer stationary level calibration: expected Z is +9.80665 m/s^2
            math::Vector3f avg_accel = cal_accel_acc_ / static_cast<float>(cal_count_);
            accel_bias_.x = avg_accel.x;
            accel_bias_.y = avg_accel.y;
            accel_bias_.z = avg_accel.z - 9.80665f;

            in_calibration_ = false;
            is_calibrated_ = true;
        }
        return is_calibrated_;
    }

    bool isCalibrated() const noexcept override { return is_calibrated_; }

    void getCalibrationOffsets(math::Vector3f& gyro_bias, math::Vector3f& accel_bias) const noexcept override {
        gyro_bias = gyro_bias_;
        accel_bias = accel_bias_;
    }

    void setCalibrationOffsets(const math::Vector3f& gyro_bias, const math::Vector3f& accel_bias) noexcept override {
        gyro_bias_ = gyro_bias;
        accel_bias_ = accel_bias;
        is_calibrated_ = true;
    }

    // Dynamic flight simulation controls
    void setSimulatedRates(float roll_rate, float pitch_rate, float yaw_rate) noexcept {
        true_rates_ = {roll_rate, pitch_rate, yaw_rate};
    }

    void setSimulatedAcceleration(float ax, float ay, float az) noexcept {
        true_accel_ = {ax, ay, az};
    }

    void setFaultMode(FaultMode mode) noexcept {
        fault_mode_ = mode;
    }

    void setTimeStepUs(uint32_t dt_us) noexcept {
        dt_us_ = dt_us;
    }

private:
    bool is_initialized_{false};
    bool is_calibrated_{false};
    bool in_calibration_{false};
    uint32_t cal_target_{100};
    uint32_t cal_count_{0};

    math::Vector3f cal_gyro_acc_{0.0f, 0.0f, 0.0f};
    math::Vector3f cal_accel_acc_{0.0f, 0.0f, 0.0f};

    math::Vector3f gyro_bias_{0.015f, -0.02f, 0.005f}; // Realistic factory uncalibrated bias
    math::Vector3f accel_bias_{0.05f, -0.03f, 0.12f};

    math::Vector3f true_rates_{0.0f, 0.0f, 0.0f};
    math::Vector3f true_accel_{0.0f, 0.0f, 9.80665f};

    FaultMode fault_mode_{FaultMode::NONE};
    TimestampUs current_time_us_{0};
    uint32_t dt_us_{10000}; // 100 Hz = 10,000 us default
    uint32_t seq_id_{0};
    ImuSample stuck_sample_{};
};

} // namespace drivers
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_DRIVERS_SIMULATED_IMU_DRIVER_HPP
