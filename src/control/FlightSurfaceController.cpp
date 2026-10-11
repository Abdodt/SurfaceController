#include "surface_controller/control/FlightSurfaceController.hpp"
#include <cstdlib>

namespace surface_controller {
namespace control {

FlightSurfaceController::FlightSurfaceController() {
    // Default aerospace baseline gains
    PidGains pitch_gains;
    pitch_gains.kp = 0.8f;
    pitch_gains.ki = 0.15f;
    pitch_gains.kd = 0.04f;
    pitch_gains.integral_min = -0.3f;
    pitch_gains.integral_max = 0.3f;
    pitch_gains.output_min = -1.0f;
    pitch_gains.output_max = 1.0f;
    pitch_gains.derivative_filter_hz = 20.0f;

    PidGains roll_gains;
    roll_gains.kp = 0.6f;
    roll_gains.ki = 0.10f;
    roll_gains.kd = 0.03f;
    roll_gains.integral_min = -0.25f;
    roll_gains.integral_max = 0.25f;
    roll_gains.output_min = -1.0f;
    roll_gains.output_max = 1.0f;
    roll_gains.derivative_filter_hz = 20.0f;

    PidGains yaw_gains;
    yaw_gains.kp = 1.0f;
    yaw_gains.ki = 0.05f;
    yaw_gains.kd = 0.01f;
    yaw_gains.integral_min = -0.2f;
    yaw_gains.integral_max = 0.2f;
    yaw_gains.output_min = -1.0f;
    yaw_gains.output_max = 1.0f;
    yaw_gains.derivative_filter_hz = 15.0f;

    configurePIDs(pitch_gains, roll_gains, yaw_gains);
}

void FlightSurfaceController::configurePIDs(const PidGains& pitch_gains,
                                            const PidGains& roll_gains,
                                            const PidGains& yaw_gains) {
    pitch_pid_.configure(pitch_gains, NOMINAL_RATE_HZ);
    roll_pid_.configure(roll_gains, NOMINAL_RATE_HZ);
    yaw_pid_.configure(yaw_gains, NOMINAL_RATE_HZ);
}

SurfaceDeflections FlightSurfaceController::update(const FlightSetpoints& setpoints,
                                                  const ImuSample& imu,
                                                  float actual_dt_s) {
    // Pitch stabilization (Y-axis gyro: Q rate)
    const float pitch_effort = pitch_pid_.update(
        setpoints.pitch_rate_rad_s,
        imu.gyro_rad_s.y,
        actual_dt_s
    );

    // Roll stabilization (X-axis gyro: P rate)
    const float roll_effort = roll_pid_.update(
        setpoints.roll_rate_rad_s,
        imu.gyro_rad_s.x,
        actual_dt_s
    );

    // Yaw stabilization (Z-axis gyro: R rate)
    const float yaw_effort = yaw_pid_.update(
        setpoints.yaw_rate_rad_s,
        imu.gyro_rad_s.z,
        actual_dt_s
    );

    // Mix 3-axis efforts into aerodynamic surface deflections
    return Mixer::mix(pitch_effort, roll_effort, yaw_effort);
}

void FlightSurfaceController::reset() {
    pitch_pid_.reset();
    roll_pid_.reset();
    yaw_pid_.reset();
}

void FlightSurfaceController::recordCycleTiming(uint32_t period_us, uint32_t execution_us) {
    timing_metrics_.cycle_count++;
    timing_metrics_.period_us = period_us;
    timing_metrics_.execution_time_us = execution_us;

    // Ignore cycle 1 (pre-scheduler initialization interval before first periodic delayUntil)
    if (timing_metrics_.cycle_count <= 1) {
        return;
    }

    if (period_us < timing_metrics_.min_period_us) {
        timing_metrics_.min_period_us = period_us;
    }
    if (period_us > timing_metrics_.max_period_us) {
        timing_metrics_.max_period_us = period_us;
    }
    if (execution_us > timing_metrics_.max_execution_time_us) {
        timing_metrics_.max_execution_time_us = execution_us;
    }

    // Jitter is deviation from nominal 20,000 us (50 Hz)
    int32_t current_jitter = static_cast<int32_t>(period_us) - static_cast<int32_t>(NOMINAL_PERIOD_US);
    timing_metrics_.jitter_us = current_jitter;

    int32_t abs_jitter = std::abs(current_jitter);
    if (abs_jitter > timing_metrics_.max_jitter_us) {
        timing_metrics_.max_jitter_us = abs_jitter;
    }
}

} // namespace control
} // namespace surface_controller
