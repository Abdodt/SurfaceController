#ifndef SURFACE_CONTROLLER_DRIVERS_PWM_ACTUATOR_DRIVER_HPP
#define SURFACE_CONTROLLER_DRIVERS_PWM_ACTUATOR_DRIVER_HPP

#include <cstdint>
#include "surface_controller/common/Types.hpp"
#include "surface_controller/common/Result.hpp"
#include "surface_controller/hal/IPwmOutput.hpp"

namespace surface_controller {
namespace drivers {

/**
 * @brief Actuator driver translating aerodynamic deflections into PWM servo pulses.
 * Enforces mechanical travel limits, slew-rate clamping, and safe fail-safe positions.
 */
class PwmActuatorDriver {
public:
    enum ChannelMap : uint8_t {
        CHANNEL_ELEVATOR      = 0,
        CHANNEL_AILERON_LEFT  = 1,
        CHANNEL_AILERON_RIGHT = 2,
        CHANNEL_RUDDER        = 3,
        TOTAL_CHANNELS        = 4
    };

    explicit PwmActuatorDriver(hal::IPwmOutput& pwm_hal);

    Status init(uint32_t pwm_frequency_hz = 50);

    /**
     * @brief Command surface deflections to actuators.
     * @param deflections Normalized commands [-1.0f, +1.0f]
     * @param dt_s Time step for slew-rate limiting
     */
    Status writeDeflections(const SurfaceDeflections& deflections, float dt_s);

    /**
     * @brief Force all actuators immediately to aerodynamic neutral / fail-safe.
     */
    void commandFailSafe();

    /**
     * @brief Set actuator arming state.
     */
    void setArmed(bool armed);
    bool isArmed() const noexcept { return is_armed_; }

    /**
     * @brief Get last commanded pulse width on a channel.
     */
    PwmPulseUs getCommandedPulseUs(uint8_t channel) const noexcept;

    /**
     * @brief Configure max deflection slew rate (units/second, e.g. 5.0f = full travel in 0.4s).
     */
    void setMaxSlewRate(float max_rate_per_sec) noexcept { max_slew_rate_ = max_rate_per_sec; }

private:
    PwmPulseUs deflectionToPulse(float deflection) const noexcept;

    hal::IPwmOutput& pwm_hal_;
    bool is_armed_{false};
    float max_slew_rate_{10.0f}; // Default 10.0 units/sec

    float current_deflections_[TOTAL_CHANNELS]{0.0f, 0.0f, 0.0f, 0.0f};
    PwmPulseUs commanded_pulses_[TOTAL_CHANNELS]{
        PWM_PULSE_NEUTRAL_US,
        PWM_PULSE_NEUTRAL_US,
        PWM_PULSE_NEUTRAL_US,
        PWM_PULSE_NEUTRAL_US
    };
};

} // namespace drivers
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_DRIVERS_PWM_ACTUATOR_DRIVER_HPP
