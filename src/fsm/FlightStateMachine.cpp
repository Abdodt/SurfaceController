#include "surface_controller/fsm/FlightStateMachine.hpp"

namespace surface_controller {
namespace fsm {

FlightStateMachine::FlightStateMachine()
    : current_mode_(FlightMode::INIT),
      previous_mode_(FlightMode::INIT) {}

Status FlightStateMachine::processEvent(FlightEvent event) {
    // Critical faults from any state immediately force FAIL_SAFE
    switch (event) {
        case FlightEvent::SENSOR_FAULT_DETECTED:
            triggerFailSafe(FaultCode::SENSOR_COMM_ERROR);
            return Status::OK;
        case FlightEvent::WATCHDOG_FAULT_DETECTED:
            triggerFailSafe(FaultCode::WATCHDOG_HEARTBEAT_TIMEOUT);
            return Status::OK;
        case FlightEvent::TIMING_DEADLINE_MISSED:
            triggerFailSafe(FaultCode::CONTROL_TIMING_DEADLINE);
            return Status::OK;
        case FlightEvent::MANUAL_ESTOP:
            triggerFailSafe(FaultCode::EMERGENCY_DISARM_REQUEST);
            return Status::OK;
        default:
            break;
    }

    switch (current_mode_) {
        case FlightMode::INIT:
            if (event == FlightEvent::CALIBRATION_TRIGGER) {
                transitionTo(FlightMode::CALIBRATE);
                return Status::OK;
            } else if (event == FlightEvent::ENGAGE_REQUEST) {
                // Safety invariant: Cannot engage without successful calibration
                if (is_calibrated_) {
                    transitionTo(FlightMode::ENGAGED);
                    return Status::OK;
                } else {
                    latched_fault_ = FaultCode::ILLEGAL_STATE_TRANSITION;
                    return Status::INVALID_ARGUMENT;
                }
            }
            break;

        case FlightMode::CALIBRATE:
            if (event == FlightEvent::CALIBRATION_COMPLETE) {
                is_calibrated_ = true;
                transitionTo(FlightMode::INIT);
                return Status::OK;
            } else if (event == FlightEvent::CALIBRATION_FAILED) {
                is_calibrated_ = false;
                triggerFailSafe(FaultCode::SENSOR_CALIBRATION_FAILED);
                return Status::OK;
            }
            break;

        case FlightMode::ENGAGED:
            if (event == FlightEvent::DISARM_REQUEST) {
                transitionTo(FlightMode::INIT);
                return Status::OK;
            }
            break;

        case FlightMode::FAIL_SAFE:
            // Fail-safe is latched during flight operations; ignore routine events
            return Status::ERROR;
    }

    return Status::INVALID_ARGUMENT;
}

void FlightStateMachine::triggerFailSafe(FaultCode fault) {
    latched_fault_ = latched_fault_ | fault;
    transitionTo(FlightMode::FAIL_SAFE);
}

void FlightStateMachine::setCalibrationComplete(bool successful) {
    if (successful) {
        processEvent(FlightEvent::CALIBRATION_COMPLETE);
    } else {
        processEvent(FlightEvent::CALIBRATION_FAILED);
    }
}

void FlightStateMachine::reset() {
    current_mode_ = FlightMode::INIT;
    previous_mode_ = FlightMode::INIT;
    is_calibrated_ = false;
    latched_fault_ = FaultCode::NONE;
}

void FlightStateMachine::transitionTo(FlightMode new_mode) {
    if (current_mode_ != new_mode) {
        previous_mode_ = current_mode_;
        current_mode_ = new_mode;
        ++transition_count_;
    }
}

} // namespace fsm
} // namespace surface_controller
