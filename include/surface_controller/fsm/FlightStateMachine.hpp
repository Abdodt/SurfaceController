#ifndef SURFACE_CONTROLLER_FSM_FLIGHT_STATE_MACHINE_HPP
#define SURFACE_CONTROLLER_FSM_FLIGHT_STATE_MACHINE_HPP

#include "surface_controller/fsm/FlightMode.hpp"
#include "surface_controller/common/FaultCodes.hpp"
#include "surface_controller/common/Result.hpp"

namespace surface_controller {
namespace fsm {

/**
 * @brief Safety-critical, deterministic state machine controlling flight modes.
 * Enforces strict transitions, preventing engagement without valid calibration
 * and latching fail-safe upon any safety violation.
 */
class FlightStateMachine {
public:
    FlightStateMachine();

    /**
     * @brief Process an event stimulus and perform state transition if valid.
     * @return Status::OK if transition accepted, Status::INVALID_ARGUMENT or Status::ERROR otherwise.
     */
    Status processEvent(FlightEvent event);

    /**
     * @brief Trigger emergency fail-safe with a specific fault code.
     */
    void triggerFailSafe(FaultCode fault);

    /**
     * @brief Get current active flight mode.
     */
    FlightMode getCurrentMode() const noexcept { return current_mode_; }

    /**
     * @brief Check whether active PID surface stabilization is permitted.
     */
    bool isControlActive() const noexcept { return current_mode_ == FlightMode::ENGAGED; }

    /**
     * @brief Check whether system is in fail-safe state.
     */
    bool isFailSafe() const noexcept { return current_mode_ == FlightMode::FAIL_SAFE; }

    /**
     * @brief Query if sensors are marked as calibrated.
     */
    bool isCalibrated() const noexcept { return is_calibrated_; }

    /**
     * @brief Inform the FSM of sensor calibration status.
     */
    void setCalibrationComplete(bool successful);

    /**
     * @brief Query active fault code.
     */
    FaultCode getLastFault() const noexcept { return latched_fault_; }

    /**
     * @brief Reset state machine to INIT (allowed only in controlled maintenance).
     */
    void reset();

    /**
     * @brief Get count of state transitions since boot.
     */
    uint32_t getTransitionCount() const noexcept { return transition_count_; }

private:
    void transitionTo(FlightMode new_mode);

    FlightMode current_mode_{FlightMode::INIT};
    FlightMode previous_mode_{FlightMode::INIT};
    bool is_calibrated_{false};
    FaultCode latched_fault_{FaultCode::NONE};
    uint32_t transition_count_{0};
};

} // namespace fsm
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_FSM_FLIGHT_STATE_MACHINE_HPP
