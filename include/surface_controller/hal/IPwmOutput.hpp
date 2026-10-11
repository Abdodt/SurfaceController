#ifndef SURFACE_CONTROLLER_HAL_IPWM_OUTPUT_HPP
#define SURFACE_CONTROLLER_HAL_IPWM_OUTPUT_HPP

#include <cstdint>
#include "surface_controller/common/Types.hpp"
#include "surface_controller/common/Result.hpp"

namespace surface_controller {
namespace hal {

/**
 * @brief Abstract interface for Hardware PWM Timers driving flight surface actuators.
 */
class IPwmOutput {
public:
    virtual ~IPwmOutput() = default;

    /**
     * @brief Initialize PWM hardware for specified channel count and frequency (e.g. 50 Hz / 400 Hz).
     */
    virtual Status init(uint32_t frequency_hz) = 0;

    /**
     * @brief Set pulse width in microseconds for a channel.
     * @param channel Channel index (0 to N-1).
     * @param pulse_us Pulse width in microseconds (typically 1000 to 2000 us).
     */
    virtual Status setPulseWidthUs(uint8_t channel, PwmPulseUs pulse_us) = 0;

    /**
     * @brief Read the last commanded pulse width.
     */
    virtual PwmPulseUs getPulseWidthUs(uint8_t channel) const = 0;

    /**
     * @brief Arm or disarm PWM outputs (safety cut-off).
     */
    virtual void setArmed(bool armed) = 0;

    /**
     * @brief Return number of hardware channels available.
     */
    virtual uint8_t channelCount() const = 0;
};

} // namespace hal
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_HAL_IPWM_OUTPUT_HPP
