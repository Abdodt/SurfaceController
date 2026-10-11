#ifndef SURFACE_CONTROLLER_FSM_FLIGHT_MODE_HPP
#define SURFACE_CONTROLLER_FSM_FLIGHT_MODE_HPP

#include <cstdint>

namespace surface_controller {
namespace fsm {

/**
 * @brief Safety critical flight controller operational modes.
 */
enum class FlightMode : uint8_t {
    INIT = 0,       ///< Boot state, hardware initialization & self-test
    CALIBRATE,      ///< Stationary sensor bias accumulation & gyro zeroing
    ENGAGED,        ///< Active 50 Hz closed-loop flight surface stabilization
    FAIL_SAFE       ///< Latched emergency state: neutral trim, loop disarmed
};

/**
 * @brief State machine input stimulus events.
 */
enum class FlightEvent : uint8_t {
    BOOT_SUCCESS = 0,
    CALIBRATION_TRIGGER,
    CALIBRATION_COMPLETE,
    CALIBRATION_FAILED,
    ENGAGE_REQUEST,
    DISARM_REQUEST,
    SENSOR_FAULT_DETECTED,
    WATCHDOG_FAULT_DETECTED,
    TIMING_DEADLINE_MISSED,
    MANUAL_ESTOP
};

inline const char* flightModeToString(FlightMode mode) noexcept {
    switch (mode) {
        case FlightMode::INIT: return "INIT";
        case FlightMode::CALIBRATE: return "CALIBRATE";
        case FlightMode::ENGAGED: return "ENGAGED";
        case FlightMode::FAIL_SAFE: return "FAIL_SAFE";
        default: return "UNKNOWN";
    }
}

inline const char* flightEventToString(FlightEvent event) noexcept {
    switch (event) {
        case FlightEvent::BOOT_SUCCESS: return "BOOT_SUCCESS";
        case FlightEvent::CALIBRATION_TRIGGER: return "CALIBRATION_TRIGGER";
        case FlightEvent::CALIBRATION_COMPLETE: return "CALIBRATION_COMPLETE";
        case FlightEvent::CALIBRATION_FAILED: return "CALIBRATION_FAILED";
        case FlightEvent::ENGAGE_REQUEST: return "ENGAGE_REQUEST";
        case FlightEvent::DISARM_REQUEST: return "DISARM_REQUEST";
        case FlightEvent::SENSOR_FAULT_DETECTED: return "SENSOR_FAULT_DETECTED";
        case FlightEvent::WATCHDOG_FAULT_DETECTED: return "WATCHDOG_FAULT_DETECTED";
        case FlightEvent::TIMING_DEADLINE_MISSED: return "TIMING_DEADLINE_MISSED";
        case FlightEvent::MANUAL_ESTOP: return "MANUAL_ESTOP";
        default: return "UNKNOWN";
    }
}

} // namespace fsm
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_FSM_FLIGHT_MODE_HPP
