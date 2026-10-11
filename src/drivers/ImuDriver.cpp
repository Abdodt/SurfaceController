#include "surface_controller/drivers/ImuDriver.hpp"
#include <cstring>

namespace surface_controller {

const char* faultCodeToString(FaultCode fault) {
    if (fault == FaultCode::NONE) return "NONE";
    if (hasFault(fault, FaultCode::SENSOR_INIT_FAILED)) return "SENSOR_INIT_FAILED";
    if (hasFault(fault, FaultCode::SENSOR_COMM_ERROR)) return "SENSOR_COMM_ERROR";
    if (hasFault(fault, FaultCode::SENSOR_TIMEOUT)) return "SENSOR_TIMEOUT";
    if (hasFault(fault, FaultCode::SENSOR_OUT_OF_BOUNDS)) return "SENSOR_OUT_OF_BOUNDS";
    if (hasFault(fault, FaultCode::SENSOR_STUCK_DATA)) return "SENSOR_STUCK_DATA";
    if (hasFault(fault, FaultCode::SENSOR_CALIBRATION_FAILED)) return "SENSOR_CALIBRATION_FAILED";
    if (hasFault(fault, FaultCode::CONTROL_TIMING_DEADLINE)) return "CONTROL_TIMING_DEADLINE";
    if (hasFault(fault, FaultCode::CONTROL_DIVERGENCE)) return "CONTROL_DIVERGENCE";
    if (hasFault(fault, FaultCode::ACTUATOR_SATURATION)) return "ACTUATOR_SATURATION";
    if (hasFault(fault, FaultCode::WATCHDOG_HEARTBEAT_TIMEOUT)) return "WATCHDOG_HEARTBEAT_TIMEOUT";
    if (hasFault(fault, FaultCode::WATCHDOG_KICK_SUPPRESSED)) return "WATCHDOG_KICK_SUPPRESSED";
    if (hasFault(fault, FaultCode::ILLEGAL_STATE_TRANSITION)) return "ILLEGAL_STATE_TRANSITION";
    if (hasFault(fault, FaultCode::UNAUTHORIZED_HEAP_ALLOC)) return "UNAUTHORIZED_HEAP_ALLOC";
    if (hasFault(fault, FaultCode::EMERGENCY_DISARM_REQUEST)) return "EMERGENCY_DISARM_REQUEST";
    return "UNKNOWN_FAULT";
}

namespace drivers {

SpiImuDriver::SpiImuDriver(hal::ISpiBus& spi_bus)
    : spi_bus_(spi_bus) {}

Status SpiImuDriver::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t tx_buf[2] = {static_cast<uint8_t>(reg & 0x7F), value}; // Write bit = 0
    spi_bus_.select();
    Status s = spi_bus_.write(tx_buf, 2);
    spi_bus_.deselect();
    return s;
}

Status SpiImuDriver::readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
    uint8_t tx_reg = static_cast<uint8_t>(reg | 0x80); // Read bit = 1
    spi_bus_.select();
    Status s = spi_bus_.write(&tx_reg, 1);
    if (s == Status::OK) {
        s = spi_bus_.read(buffer, length);
    }
    spi_bus_.deselect();
    return s;
}

Status SpiImuDriver::init() {
    // 1. Reset device
    Status s = writeRegister(REG_PWR_MGMT_1, 0x80); // Reset
    if (s != Status::OK) {
        fault_flags_ = fault_flags_ | FaultCode::SENSOR_COMM_ERROR;
        return s;
    }

    // 2. Verify WHO_AM_I identifier
    uint8_t who_am_i = 0;
    s = readRegisters(REG_WHO_AM_I, &who_am_i, 1);
    if (s != Status::OK || (who_am_i != EXPECTED_WHO_AM_I && who_am_i != 0x70 && who_am_i != 0x98)) {
        fault_flags_ = fault_flags_ | FaultCode::SENSOR_INIT_FAILED;
        return Status::HARDWARE_FAULT;
    }

    // 3. Wake up device, set clock source to PLL
    s = writeRegister(REG_PWR_MGMT_1, 0x01);
    if (s != Status::OK) return s;

    // 4. Configure Digital Low Pass Filter (DLPF) to 42 Hz
    s = writeRegister(REG_CONFIG, 0x03);
    if (s != Status::OK) return s;

    // 5. Configure Gyro Full Scale to +-2000 deg/s
    s = writeRegister(REG_GYRO_CONFIG, 0x18);
    if (s != Status::OK) return s;

    // 6. Configure Accelerometer Full Scale to +-16g
    s = writeRegister(REG_ACCEL_CONFIG, 0x18);
    if (s != Status::OK) return s;

    fault_flags_ = FaultCode::NONE;
    return Status::OK;
}

Status SpiImuDriver::selfTest() {
    uint8_t who_am_i = 0;
    Status s = readRegisters(REG_WHO_AM_I, &who_am_i, 1);
    if (s != Status::OK) return Status::HARDWARE_FAULT;
    return (who_am_i == EXPECTED_WHO_AM_I || who_am_i == 0x70 || who_am_i == 0x98)
           ? Status::OK : Status::HARDWARE_FAULT;
}

Result<ImuSample> SpiImuDriver::readSample() {
    uint8_t raw_buf[14]{};
    Status s = readRegisters(REG_ACCEL_XOUT_H, raw_buf, 14);
    if (s != Status::OK) {
        fault_flags_ = fault_flags_ | FaultCode::SENSOR_COMM_ERROR;
        return Result<ImuSample>(Status::HARDWARE_FAULT);
    }

    // Parse Big-Endian 16-bit 2's complement registers
    int16_t raw_ax = static_cast<int16_t>((raw_buf[0] << 8) | raw_buf[1]);
    int16_t raw_ay = static_cast<int16_t>((raw_buf[2] << 8) | raw_buf[3]);
    int16_t raw_az = static_cast<int16_t>((raw_buf[4] << 8) | raw_buf[5]);

    int16_t raw_temp = static_cast<int16_t>((raw_buf[6] << 8) | raw_buf[7]);

    int16_t raw_gx = static_cast<int16_t>((raw_buf[8] << 8) | raw_buf[9]);
    int16_t raw_gy = static_cast<int16_t>((raw_buf[10] << 8) | raw_buf[11]);
    int16_t raw_gz = static_cast<int16_t>((raw_buf[12] << 8) | raw_buf[13]);

    // Check for stuck sensor data
    if (raw_ax == last_raw_accel_x_) {
        ++stuck_count_;
        if (stuck_count_ > 100) { // 100 consecutive identical raw readings at 100 Hz = 1 sec
            fault_flags_ = fault_flags_ | FaultCode::SENSOR_STUCK_DATA;
        }
    } else {
        stuck_count_ = 0;
        last_raw_accel_x_ = raw_ax;
    }

    ImuSample sample;
    sample.sequence_id = ++sequence_id_;

    // Scale Accelerometer (m/s^2)
    sample.accel_m_s2.x = (static_cast<float>(raw_ax) * ACCEL_SCALE_G * GRAVITY_MSS) - accel_bias_.x;
    sample.accel_m_s2.y = (static_cast<float>(raw_ay) * ACCEL_SCALE_G * GRAVITY_MSS) - accel_bias_.y;
    sample.accel_m_s2.z = (static_cast<float>(raw_az) * ACCEL_SCALE_G * GRAVITY_MSS) - accel_bias_.z;

    // Scale Gyroscope (rad/s)
    sample.gyro_rad_s.x = math::degToRad(static_cast<float>(raw_gx) * GYRO_SCALE_DPS) - gyro_bias_.x;
    sample.gyro_rad_s.y = math::degToRad(static_cast<float>(raw_gy) * GYRO_SCALE_DPS) - gyro_bias_.y;
    sample.gyro_rad_s.z = math::degToRad(static_cast<float>(raw_gz) * GYRO_SCALE_DPS) - gyro_bias_.z;

    // Scale Temperature (deg C)
    sample.temperature_c = (static_cast<float>(raw_temp) / 340.0f) + 36.53f;

    // Data sanity checks
    float accel_mag = sample.accel_m_s2.norm();
    if (accel_mag > 60.0f || std::isnan(accel_mag)) {
        fault_flags_ = fault_flags_ | FaultCode::SENSOR_OUT_OF_BOUNDS;
        sample.is_valid = false;
        return Result<ImuSample>(Status::HARDWARE_FAULT);
    }

    sample.is_valid = true;
    return Result<ImuSample>(sample);
}

Status SpiImuDriver::startCalibration(uint32_t total_samples) {
    cal_samples_target_ = total_samples;
    cal_samples_count_ = 0;
    cal_gyro_acc_.reset();
    cal_accel_acc_.reset();
    in_calibration_ = true;
    is_calibrated_ = false;
    return Status::OK;
}

bool SpiImuDriver::processCalibrationSample(const ImuSample& sample) {
    if (!in_calibration_) return is_calibrated_;

    cal_gyro_acc_ += sample.gyro_rad_s;
    cal_accel_acc_ += sample.accel_m_s2;
    ++cal_samples_count_;

    if (cal_samples_count_ >= cal_samples_target_) {
        gyro_bias_ = cal_gyro_acc_ / static_cast<float>(cal_samples_count_);
        math::Vector3f avg_accel = cal_accel_acc_ / static_cast<float>(cal_samples_count_);
        accel_bias_.x = avg_accel.x;
        accel_bias_.y = avg_accel.y;
        accel_bias_.z = avg_accel.z - GRAVITY_MSS; // Accelerometer points +1g down when level

        in_calibration_ = false;
        is_calibrated_ = true;
    }
    return is_calibrated_;
}

void SpiImuDriver::getCalibrationOffsets(math::Vector3f& gyro_bias, math::Vector3f& accel_bias) const noexcept {
    gyro_bias = gyro_bias_;
    accel_bias = accel_bias_;
}

void SpiImuDriver::setCalibrationOffsets(const math::Vector3f& gyro_bias, const math::Vector3f& accel_bias) noexcept {
    gyro_bias_ = gyro_bias;
    accel_bias_ = accel_bias;
    is_calibrated_ = true;
}

} // namespace drivers
} // namespace surface_controller
