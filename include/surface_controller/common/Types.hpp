#ifndef SURFACE_CONTROLLER_COMMON_TYPES_HPP
#define SURFACE_CONTROLLER_COMMON_TYPES_HPP

#include <cstdint>
#include <cstddef>
#include "surface_controller/math/Vector3.hpp"

namespace surface_controller {

/**
 * @brief Timestamp representation in microseconds.
 */
using TimestampUs = uint64_t;

/**
 * @brief Pulse width representation for PWM servos in microseconds.
 * Nominal range: 1000 us (min) to 2000 us (max), with 1500 us as neutral.
 */
using PwmPulseUs = uint16_t;

constexpr PwmPulseUs PWM_PULSE_MIN_US = 1000;
constexpr PwmPulseUs PWM_PULSE_NEUTRAL_US = 1500;
constexpr PwmPulseUs PWM_PULSE_MAX_US = 2000;

/**
 * @brief Calibrated and validated IMU measurement sample.
 */
struct ImuSample {
    math::Vector3f accel_m_s2{0.0f, 0.0f, 0.0f};  ///< Linear acceleration in m/s^2 (X: forward, Y: right, Z: down)
    math::Vector3f gyro_rad_s{0.0f, 0.0f, 0.0f};  ///< Angular rate in rad/s (X: roll rate, Y: pitch rate, Z: yaw rate)
    float temperature_c{0.0f};                   ///< IMU die temperature in deg C
    TimestampUs timestamp_us{0};                 ///< Microsecond acquisition timestamp
    uint32_t sequence_id{0};                     ///< Monotonic sample counter
    bool is_valid{false};                         ///< Data validity flag
};

/**
 * @brief Desired angular rates or attitude setpoints for closed-loop flight stabilization.
 */
struct FlightSetpoints {
    float roll_rate_rad_s{0.0f};   ///< Desired roll rate (P-axis)
    float pitch_rate_rad_s{0.0f};  ///< Desired pitch rate (Q-axis)
    float yaw_rate_rad_s{0.0f};    ///< Desired yaw rate (R-axis)
    TimestampUs timestamp_us{0};
};

/**
 * @brief Aerodynamic flight surface deflections normalized to [-1.0f, +1.0f].
 */
struct SurfaceDeflections {
    float elevator{0.0f};       ///< Pitch control: [-1.0 (nose down), +1.0 (nose up)]
    float aileron_left{0.0f};   ///< Roll control: [-1.0 to +1.0]
    float aileron_right{0.0f};  ///< Roll control: differential to left
    float rudder{0.0f};         ///< Yaw control: [-1.0 (left), +1.0 (right)]
    bool is_fail_safe{false};
};

/**
 * @brief Actuator PWM pulse width commands sent to physical servo hardware.
 */
struct ActuatorCommands {
    PwmPulseUs elevator_us{PWM_PULSE_NEUTRAL_US};
    PwmPulseUs aileron_left_us{PWM_PULSE_NEUTRAL_US};
    PwmPulseUs aileron_right_us{PWM_PULSE_NEUTRAL_US};
    PwmPulseUs rudder_us{PWM_PULSE_NEUTRAL_US};
    TimestampUs timestamp_us{0};
    bool armed{false};
};

/**
 * @brief Real-time control loop timing jitter and execution metrics.
 */
struct TimingMetrics {
    uint32_t cycle_count{0};
    uint32_t period_us{0};
    uint32_t min_period_us{0xFFFFFFFFU};
    uint32_t max_period_us{0};
    int32_t jitter_us{0};
    int32_t max_jitter_us{0};
    uint32_t execution_time_us{0};
    uint32_t max_execution_time_us{0};
};

} // namespace surface_controller

#endif // SURFACE_CONTROLLER_COMMON_TYPES_HPP
