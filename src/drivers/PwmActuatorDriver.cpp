#include "surface_controller/drivers/PwmActuatorDriver.hpp"
#include "surface_controller/math/MathUtils.hpp"

namespace surface_controller {
namespace drivers {

PwmActuatorDriver::PwmActuatorDriver(hal::IPwmOutput& pwm_hal)
    : pwm_hal_(pwm_hal) {}

Status PwmActuatorDriver::init(uint32_t pwm_frequency_hz) {
    Status s = pwm_hal_.init(pwm_frequency_hz);
    if (s != Status::OK) {
        return s;
    }

    // Set all channels to safe neutral
    commandFailSafe();
    setArmed(false);
    return Status::OK;
}

void PwmActuatorDriver::setArmed(bool armed) {
    is_armed_ = armed;
    pwm_hal_.setArmed(armed);
    if (!armed) {
        commandFailSafe();
    }
}

PwmPulseUs PwmActuatorDriver::deflectionToPulse(float deflection) const noexcept {
    // Map [-1.0f, +1.0f] to [1000 us, 2000 us] with 1500 us as neutral
    const float clamped = math::clamp(deflection, -1.0f, 1.0f);
    const float pulse = 1500.0f + (clamped * 500.0f);
    return static_cast<PwmPulseUs>(math::clamp(pulse, 1000.0f, 2000.0f));
}

Status PwmActuatorDriver::writeDeflections(const SurfaceDeflections& deflections, float dt_s) {
    if (!is_armed_) {
        commandFailSafe();
        return Status::OK;
    }

    const float targets[TOTAL_CHANNELS] = {
        deflections.elevator,
        deflections.aileron_left,
        deflections.aileron_right,
        deflections.rudder
    };

    for (uint8_t i = 0; i < TOTAL_CHANNELS; ++i) {
        // Apply mechanical slew-rate limiting
        current_deflections_[i] = math::slewRateLimit(
            current_deflections_[i],
            targets[i],
            max_slew_rate_,
            dt_s
        );

        commanded_pulses_[i] = deflectionToPulse(current_deflections_[i]);
        pwm_hal_.setPulseWidthUs(i, commanded_pulses_[i]);
    }

    return Status::OK;
}

void PwmActuatorDriver::commandFailSafe() {
    for (uint8_t i = 0; i < TOTAL_CHANNELS; ++i) {
        current_deflections_[i] = 0.0f;
        commanded_pulses_[i] = PWM_PULSE_NEUTRAL_US;
        pwm_hal_.setPulseWidthUs(i, PWM_PULSE_NEUTRAL_US);
    }
}

PwmPulseUs PwmActuatorDriver::getCommandedPulseUs(uint8_t channel) const noexcept {
    if (channel < TOTAL_CHANNELS) {
        return commanded_pulses_[channel];
    }
    return PWM_PULSE_NEUTRAL_US;
}

} // namespace drivers
} // namespace surface_controller
