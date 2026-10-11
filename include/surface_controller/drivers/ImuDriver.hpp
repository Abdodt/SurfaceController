#ifndef SURFACE_CONTROLLER_DRIVERS_IMU_DRIVER_HPP
#define SURFACE_CONTROLLER_DRIVERS_IMU_DRIVER_HPP

#include <cstdint>
#include <cstddef>
#include "surface_controller/common/Types.hpp"
#include "surface_controller/common/Result.hpp"
#include "surface_controller/common/FaultCodes.hpp"
#include "surface_controller/hal/ISpiBus.hpp"
#include "surface_controller/math/Vector3.hpp"
#include "surface_controller/math/Filter.hpp"

namespace surface_controller {
namespace drivers {

/**
 * @brief Abstract interface for 6-DOF Inertial Measurement Units.
 */
class IImuDriver {
public:
    virtual ~IImuDriver() = default;

    virtual Status init() = 0;
    virtual Status selfTest() = 0;
    virtual Result<ImuSample> readSample() = 0;
    virtual Status startCalibration(uint32_t total_samples) = 0;
    virtual bool processCalibrationSample(const ImuSample& sample) = 0;
    virtual bool isCalibrated() const noexcept = 0;
    virtual void getCalibrationOffsets(math::Vector3f& gyro_bias, math::Vector3f& accel_bias) const noexcept = 0;
    virtual void setCalibrationOffsets(const math::Vector3f& gyro_bias, const math::Vector3f& accel_bias) noexcept = 0;
};

/**
 * @brief Register-level hardware IMU driver over SPI (e.g., MPU-6000 / ICM-42688 compatible).
 */
class SpiImuDriver : public IImuDriver {
public:
    // Standard register definitions
    static constexpr uint8_t REG_WHO_AM_I     = 0x75;
    static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t REG_CONFIG       = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

    static constexpr uint8_t EXPECTED_WHO_AM_I = 0x68;

    // Sensitivities: Accel +-16g (2048 LSB/g), Gyro +-2000 deg/s (16.4 LSB/(deg/s))
    static constexpr float ACCEL_SCALE_G = 1.0f / 2048.0f;
    static constexpr float GRAVITY_MSS   = 9.80665f;
    static constexpr float GYRO_SCALE_DPS = 1.0f / 16.4f;

    explicit SpiImuDriver(hal::ISpiBus& spi_bus);
    ~SpiImuDriver() override = default;

    Status init() override;
    Status selfTest() override;
    Result<ImuSample> readSample() override;
    Status startCalibration(uint32_t total_samples) override;
    bool processCalibrationSample(const ImuSample& sample) override;
    bool isCalibrated() const noexcept override { return is_calibrated_; }
    void getCalibrationOffsets(math::Vector3f& gyro_bias, math::Vector3f& accel_bias) const noexcept override;
    void setCalibrationOffsets(const math::Vector3f& gyro_bias, const math::Vector3f& accel_bias) noexcept override;

    FaultCode getFaultFlags() const noexcept { return fault_flags_; }

private:
    Status writeRegister(uint8_t reg, uint8_t value);
    Status readRegisters(uint8_t reg, uint8_t* buffer, size_t length);

    hal::ISpiBus& spi_bus_;
    math::Vector3f gyro_bias_{0.0f, 0.0f, 0.0f};
    math::Vector3f accel_bias_{0.0f, 0.0f, 0.0f};

    // Calibration accumulator
    math::Vector3f cal_gyro_acc_{0.0f, 0.0f, 0.0f};
    math::Vector3f cal_accel_acc_{0.0f, 0.0f, 0.0f};
    uint32_t cal_samples_target_{100};
    uint32_t cal_samples_count_{0};
    bool is_calibrated_{false};
    bool in_calibration_{false};

    // Data sanity tracking
    uint32_t sequence_id_{0};
    uint16_t stuck_count_{0};
    int16_t last_raw_accel_x_{0};
    FaultCode fault_flags_{FaultCode::NONE};
};

} // namespace drivers
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_DRIVERS_IMU_DRIVER_HPP
