#ifndef SURFACE_CONTROLLER_CONTROL_FLIGHT_SURFACE_CONTROLLER_HPP
#define SURFACE_CONTROLLER_CONTROL_FLIGHT_SURFACE_CONTROLLER_HPP

#include "surface_controller/control/PidController.hpp"
#include "surface_controller/control/Mixer.hpp"
#include "surface_controller/common/Types.hpp"

namespace surface_controller {
namespace control {

/**
 * @brief Coordinates closed-loop 3-axis flight surface stabilization.
 * Operates at 50 Hz, computing rate stabilization for Pitch, Roll, and Yaw.
 */
class FlightSurfaceController {
public:
    static constexpr float NOMINAL_RATE_HZ = 50.0f;
    static constexpr float NOMINAL_PERIOD_S = 1.0f / NOMINAL_RATE_HZ;
    static constexpr uint32_t NOMINAL_PERIOD_US = 20000; // 20 ms

    FlightSurfaceController();

    void configurePIDs(const PidGains& pitch_gains,
                       const PidGains& roll_gains,
                       const PidGains& yaw_gains);

    /**
     * @brief Execute one cycle of the flight surface control loop.
     * @param setpoints Desired angular rate setpoints (rad/s)
     * @param imu Calibrated IMU measurement
     * @param actual_dt_s Measured sample interval
     * @return Commanded surface deflections
     */
    SurfaceDeflections update(const FlightSetpoints& setpoints,
                              const ImuSample& imu,
                              float actual_dt_s = NOMINAL_PERIOD_S);

    /**
     * @brief Reset internal PID integrators and filters.
     */
    void reset();

    /**
     * @brief Update timing metrics to evaluate deterministic execution jitter.
     */
    void recordCycleTiming(uint32_t period_us, uint32_t execution_us);

    const TimingMetrics& getTimingMetrics() const noexcept { return timing_metrics_; }
    const PidController& getPitchPid() const noexcept { return pitch_pid_; }
    const PidController& getRollPid() const noexcept { return roll_pid_; }
    const PidController& getYawPid() const noexcept { return yaw_pid_; }

private:
    PidController pitch_pid_;
    PidController roll_pid_;
    PidController yaw_pid_;

    TimingMetrics timing_metrics_{};
};

} // namespace control
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_CONTROL_FLIGHT_SURFACE_CONTROLLER_HPP
