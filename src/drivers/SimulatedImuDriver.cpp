#include "surface_controller/drivers/SimulatedImuDriver.hpp"

namespace surface_controller {
namespace drivers {

Result<ImuSample> SimulatedImuDriver::readSample() {
    if (!is_initialized_) {
        return Result<ImuSample>(Status::NOT_INITIALIZED);
    }

    if (fault_mode_ == FaultMode::COMM_TIMEOUT) {
        return Result<ImuSample>(Status::TIMEOUT);
    }

    current_time_us_ += dt_us_;
    ++seq_id_;

    if (fault_mode_ == FaultMode::STUCK_DATA && seq_id_ > 10) {
        stuck_sample_.timestamp_us = current_time_us_;
        stuck_sample_.sequence_id = seq_id_;
        return Result<ImuSample>(stuck_sample_);
    }

    ImuSample sample;
    sample.timestamp_us = current_time_us_;
    sample.sequence_id = seq_id_;
    sample.temperature_c = 35.0f;
    sample.is_valid = true;

    // Apply true rates + uncalibrated bias
    sample.gyro_rad_s.x = true_rates_.x + gyro_bias_.x;
    sample.gyro_rad_s.y = true_rates_.y + gyro_bias_.y;
    sample.gyro_rad_s.z = true_rates_.z + gyro_bias_.z;

    // Apply true acceleration + uncalibrated bias
    sample.accel_m_s2.x = true_accel_.x + accel_bias_.x;
    sample.accel_m_s2.y = true_accel_.y + accel_bias_.y;
    sample.accel_m_s2.z = true_accel_.z + accel_bias_.z;

    if (fault_mode_ == FaultMode::SATURATION) {
        sample.gyro_rad_s.x = 50.0f; // Excessive saturation
        sample.accel_m_s2.z = 100.0f;
    } else if (fault_mode_ == FaultMode::HIGH_NOISE) {
        // Pseudo noise without dynamic allocation
        float noise = (static_cast<float>(seq_id_ % 10) - 5.0f) * 0.1f;
        sample.gyro_rad_s.x += noise;
        sample.accel_m_s2.x += noise * 5.0f;
    }

    // If calibrated, subtract offsets
    if (is_calibrated_) {
        sample.gyro_rad_s.x -= gyro_bias_.x;
        sample.gyro_rad_s.y -= gyro_bias_.y;
        sample.gyro_rad_s.z -= gyro_bias_.z;

        sample.accel_m_s2.x -= accel_bias_.x;
        sample.accel_m_s2.y -= accel_bias_.y;
        sample.accel_m_s2.z -= accel_bias_.z;
    }

    stuck_sample_ = sample;
    return Result<ImuSample>(sample);
}

} // namespace drivers
} // namespace surface_controller
