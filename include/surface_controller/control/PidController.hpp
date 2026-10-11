#ifndef SURFACE_CONTROLLER_CONTROL_PID_CONTROLLER_HPP
#define SURFACE_CONTROLLER_CONTROL_PID_CONTROLLER_HPP

#include <cmath>
#include "surface_controller/math/MathUtils.hpp"
#include "surface_controller/math/Filter.hpp"

namespace surface_controller {
namespace control {

/**
 * @brief PID configuration tuning parameters.
 */
struct PidGains {
    float kp{0.0f};                 ///< Proportional gain
    float ki{0.0f};                 ///< Integral gain
    float kd{0.0f};                 ///< Derivative gain
    float integral_min{-1.0f};      ///< Anti-windup lower bound
    float integral_max{1.0f};       ///< Anti-windup upper bound
    float output_min{-1.0f};        ///< Output saturation lower bound
    float output_max{1.0f};         ///< Output saturation upper bound
    float derivative_filter_hz{20.0f}; ///< Low-pass cutoff for derivative filter
};

/**
 * @brief High-precision, deterministic digital PID controller.
 * Implements:
 *  - Integral anti-windup clamping
 *  - Derivative on measurement (prevents setpoint step kicks)
 *  - First-order derivative low-pass filter
 *  - Output saturation limiting
 *  - Zero dynamic memory allocation
 */
class PidController {
public:
    constexpr PidController() = default;
    explicit PidController(const PidGains& gains);

    void configure(const PidGains& gains, float sample_rate_hz = 50.0f);

    /**
     * @brief Compute control output for given setpoint and measurement.
     * @param setpoint Desired target value
     * @param measurement Measured plant state
     * @param dt_s Actual elapsed time step in seconds
     * @return Saturated control effort
     */
    float update(float setpoint, float measurement, float dt_s);

    /**
     * @brief Reset internal controller states (integral and derivative filters).
     */
    void reset();

    // Inspection getters for telemetry and unit tests
    float getP() const noexcept { return p_term_; }
    float getI() const noexcept { return i_term_; }
    float getD() const noexcept { return d_term_; }
    float getOutput() const noexcept { return output_; }
    float getLastError() const noexcept { return last_error_; }
    bool isSaturated() const noexcept { return is_saturated_; }
    const PidGains& getGains() const noexcept { return gains_; }

private:
    PidGains gains_{};
    math::LowPassFilter d_filter_{};

    float integral_state_{0.0f};
    float last_measurement_{0.0f};
    float last_error_{0.0f};

    float p_term_{0.0f};
    float i_term_{0.0f};
    float d_term_{0.0f};
    float output_{0.0f};

    bool first_update_{true};
    bool is_saturated_{false};
};

} // namespace control
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_CONTROL_PID_CONTROLLER_HPP
