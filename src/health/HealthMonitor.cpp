#include "surface_controller/health/HealthMonitor.hpp"

namespace surface_controller {
namespace health {

HealthMonitor::HealthMonitor(hal::IWatchdogTimer& watchdog_timer,
                             fsm::FlightStateMachine& state_machine,
                             const hal::ITimer& timer)
    : watchdog_timer_(watchdog_timer),
      state_machine_(state_machine),
      timer_(timer) {}

Status HealthMonitor::init() {
    // Register tasks and their maximum allowable silence intervals (deadlines)
    // 100 Hz sensor task: nominal 10 ms -> max allowed 25 ms
    tracker_.registerTask(TaskId::SENSOR, "SensorTask", 25000);

    // 50 Hz control task: nominal 20 ms -> max allowed 50 ms
    tracker_.registerTask(TaskId::CONTROL, "ControlTask", 50000);

    // 10 Hz telemetry task: nominal 100 ms -> max allowed 250 ms
    tracker_.registerTask(TaskId::TELEMETRY, "TelemetryTask", 250000);

    // Initialize hardware watchdog with 250 ms reset timeout
    return watchdog_timer_.start(WATCHDOG_HARDWARE_TIMEOUT_MS);
}

void HealthMonitor::recordTaskHeartbeat(TaskId id) {
    tracker_.checkIn(id, timer_.getTimeUs());
}

void HealthMonitor::step() {
    TimestampUs now = timer_.getTimeUs();
    FaultCode fault = FaultCode::NONE;

    bool healthy = tracker_.verifyHeartbeats(now, fault);

    if (healthy && !state_machine_.isFailSafe()) {
        // All tasks checked in within strict deadlines -> refresh watchdog
        watchdog_timer_.kick();
        successful_kicks_++;
    } else {
        // Starvation, missed deadline, or existing fail-safe -> suppress watchdog kick
        suppressed_kicks_++;
        last_fault_ = fault;
        state_machine_.triggerFailSafe(fault | FaultCode::WATCHDOG_KICK_SUPPRESSED);
    }
}

} // namespace health
} // namespace surface_controller
