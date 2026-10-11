#ifndef SURFACE_CONTROLLER_HEALTH_HEALTH_MONITOR_HPP
#define SURFACE_CONTROLLER_HEALTH_HEALTH_MONITOR_HPP

#include "surface_controller/health/HeartbeatTracker.hpp"
#include "surface_controller/hal/IWatchdogTimer.hpp"
#include "surface_controller/fsm/FlightStateMachine.hpp"
#include "surface_controller/hal/ITimer.hpp"

namespace surface_controller {
namespace health {

/**
 * @brief Independent Health Monitor managing the hardware watchdog timer.
 * Verifies task liveness, prevents watchdog petting on missed deadlines,
 * and forces system into FAIL_SAFE mode on safety breaches.
 */
class HealthMonitor {
public:
    static constexpr uint32_t MONITOR_RATE_HZ = 20; // 20 Hz (50 ms period)
    static constexpr uint32_t WATCHDOG_HARDWARE_TIMEOUT_MS = 250; // 250 ms HW reset window

    HealthMonitor(hal::IWatchdogTimer& watchdog_timer,
                  fsm::FlightStateMachine& state_machine,
                  const hal::ITimer& timer);

    Status init();

    /**
     * @brief Called by the watchdog task at 20 Hz.
     * Checks heartbeats, pets hardware watchdog if healthy, triggers FAIL_SAFE if faulted.
     */
    void step();

    /**
     * @brief Direct task check-in proxy.
     */
    void recordTaskHeartbeat(TaskId id);

    HeartbeatTracker& getHeartbeatTracker() noexcept { return tracker_; }
    const HeartbeatTracker& getHeartbeatTracker() const noexcept { return tracker_; }

    uint32_t getSuccessfulKicks() const noexcept { return successful_kicks_; }
    uint32_t getSuppressedKicks() const noexcept { return suppressed_kicks_; }
    FaultCode getLastFault() const noexcept { return last_fault_; }

private:
    hal::IWatchdogTimer& watchdog_timer_;
    fsm::FlightStateMachine& state_machine_;
    const hal::ITimer& timer_;
    HeartbeatTracker tracker_;

    uint32_t successful_kicks_{0};
    uint32_t suppressed_kicks_{0};
    FaultCode last_fault_{FaultCode::NONE};
};

} // namespace health
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_HEALTH_HEALTH_MONITOR_HPP
