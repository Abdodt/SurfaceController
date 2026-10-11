#include "surface_controller/control/PidController.hpp"

namespace surface_controller {
namespace control {

PidController::PidController(const PidGains& gains) {
    configure(gains);
}

void PidController::configure(const PidGains& gains, float sample_rate_hz) {
    gains_ = gains;
    d_filter_.configure(gains_.derivative_filter_hz, sample_rate_hz);
    reset();
}

float PidController::update(float setpoint, float measurement, float dt_s) {
    if (dt_s <= 0.0f) {
        return output_;
    }

    const float error = setpoint - measurement;
    last_error_ = error;

    // 1. Proportional term
    p_term_ = gains_.kp * error;

    // 2. Integral term with trapezoidal integration & anti-windup clamping
    if (!first_update_) {
        integral_state_ += gains_.ki * error * dt_s;
    }
    integral_state_ = math::clamp(integral_state_, gains_.integral_min, gains_.integral_max);
    i_term_ = integral_state_;

    // 3. Derivative term computed on MEASUREMENT (derivative kick protection)
    // d(meas)/dt is negative for standard feedback
    if (first_update_) {
        d_term_ = 0.0f;
        d_filter_.reset(0.0f);
        first_update_ = false;
    } else {
        const float d_measurement = (measurement - last_measurement_) / dt_s;
        const float raw_d_term = -gains_.kd * d_measurement;
        d_term_ = d_filter_.update(raw_d_term);
    }
    last_measurement_ = measurement;

    // 4. Sum components
    const float raw_output = p_term_ + i_term_ + d_term_;

    // 5. Saturate to physical actuator limits
    output_ = math::clamp(raw_output, gains_.output_min, gains_.output_max);
    is_saturated_ = (output_ != raw_output);

    return output_;
}

void PidController::reset() {
    integral_state_ = 0.0f;
    last_measurement_ = 0.0f;
    last_error_ = 0.0f;
    p_term_ = 0.0f;
    i_term_ = 0.0f;
    d_term_ = 0.0f;
    output_ = 0.0f;
    first_update_ = true;
    is_saturated_ = false;
    d_filter_.reset(0.0f);
}

} // namespace control
} // namespace surface_controller
