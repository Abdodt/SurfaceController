#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include "FreeRTOS.h"
#include "task.h"

#include "surface_controller/hal/SimulatedHal.hpp"
#include "surface_controller/drivers/SimulatedImuDriver.hpp"
#include "surface_controller/drivers/PwmActuatorDriver.hpp"
#include "surface_controller/control/FlightSurfaceController.hpp"
#include "surface_controller/fsm/FlightStateMachine.hpp"
#include "surface_controller/health/HealthMonitor.hpp"
#include "surface_controller/system/SystemTasks.hpp"
#include "surface_controller/os/MemoryTracker.hpp"

using namespace surface_controller;

// Statically allocated system singletons (Strictly Zero Dynamic Memory)
static hal::HostTimer s_timer;
static hal::SimulatedPwm s_pwm;
static hal::SimulatedWatchdog s_watchdog;
static drivers::SimulatedImuDriver s_imu;
static drivers::PwmActuatorDriver s_actuator(s_pwm);
static control::FlightSurfaceController s_controller;
static fsm::FlightStateMachine s_fsm;
static health::HealthMonitor s_health(s_watchdog, s_fsm, s_timer);

static system::SystemContext s_context{
    s_imu,
    s_actuator,
    s_controller,
    s_fsm,
    s_health,
    s_timer,
    {}, // StaticQueue
    {}, // Setpoints
    {}, // StaticMutex
    500, // Run for 500 cycles (10 seconds) in simulation mode, then gracefully report summary
    false // stop_requested
};

static system::FlightControllerSystem s_flight_system(s_context);

// FreeRTOS static allocation callbacks for Idle and Timer daemon tasks
extern "C" {

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize) {
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                    StackType_t** ppxTimerTaskStackBuffer,
                                    uint32_t* pulTimerTaskStackSize) {
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vAssertCalled(const char* pcFile, unsigned long ulLine) {
    std::printf("[FreeRTOS ASSERT] %s:%lu\n", pcFile, ulLine);
    while (1) {
        // Halt
    }
}

} // extern "C"

// Supervisory test task to orchestrate state machine progression during simulation
class MasterSupervisorTask : public os::StaticTask<system::SYSTEM_TASK_STACK_WORDS> {
public:
    void run() override {
        std::printf("[SUPERVISOR] System booted in INIT state. Requesting IMU calibration...\n");
        s_context.state_machine.processEvent(fsm::FlightEvent::CALIBRATION_TRIGGER);

        // Allow 1.5 seconds for 100 Hz calibration samples to accumulate
        delayMs(1500);

        if (s_context.state_machine.isCalibrated()) {
            std::printf("[SUPERVISOR] IMU Calibration SUCCESS! Engaging closed-loop flight control...\n");
            s_context.state_machine.processEvent(fsm::FlightEvent::ENGAGE_REQUEST);
        } else {
            std::printf("[SUPERVISOR] IMU Calibration FAILED!\n");
        }

        // Apply dynamic setpoints during engagement to simulate bank and pitch maneuvers
        delayMs(2000);
        {
            os::LockGuard lock(s_context.setpoint_mutex);
            s_context.setpoints.pitch_rate_rad_s = 0.35f; // Nose up commanded rate
            s_context.setpoints.roll_rate_rad_s = -0.50f; // Left bank commanded rate
            std::printf("[SUPERVISOR] Commanded flight maneuver: Pitch Rate = +0.35 rad/s, Roll Rate = -0.50 rad/s\n");
        }

        delayMs(3000);
        {
            os::LockGuard lock(s_context.setpoint_mutex);
            s_context.setpoints.pitch_rate_rad_s = 0.0f;
            s_context.setpoints.roll_rate_rad_s = 0.0f;
            std::printf("[SUPERVISOR] Commanded wings level trim: 0.0 rad/s\n");
        }

        delayMs(3500);

        // Simulation complete, print final aerospace timing report
        std::printf("\n=======================================================\n");
        std::printf("     FLIGHT SURFACE CONTROLLER TIMING REPORT\n");
        std::printf("=======================================================\n");
        const auto& metrics = s_context.controller.getTimingMetrics();
        std::printf("Total Control Cycles Executed : %u\n", metrics.cycle_count);
        std::printf("Nominal Loop Period           : 20000 us (50.0 Hz)\n");
        std::printf("Min Observed Period           : %u us\n", metrics.min_period_us);
        std::printf("Max Observed Period           : %u us\n", metrics.max_period_us);
        std::printf("Worst-Case Period Jitter      : %d us (Requirement: < 1000 us)\n", metrics.max_jitter_us);
        std::printf("Jitter Requirement Satisfied  : %s\n", (metrics.max_jitter_us < 1000) ? "YES [PASS]" : "NO [FAIL]");
        std::printf("Worst-Case Execution Time     : %u us\n", metrics.max_execution_time_us);
        std::printf("Watchdog Kicks Verified       : %u\n", s_context.health_monitor.getSuccessfulKicks());
        std::printf("Post-Boot Heap Violations     : %u (Zero-Allocation Verified)\n", os::MemoryTracker::getViolationCount());
        std::printf("=======================================================\n\n");

        s_context.stop_requested = true;
        delayMs(100);
        ::_exit(0);
    }
};

static MasterSupervisorTask s_supervisor_task;

int main() {
    std::printf("===========================================================\n");
    std::printf(" Object-Oriented Real-Time Flight Surface Controller (C++17)\n");
    std::printf(" Deterministic FreeRTOS Rate-Monotonic Architecture\n");
    std::printf("===========================================================\n");

    // 1. Initialize hardware, sensors, actuators, and health monitor
    if (s_flight_system.init() != Status::OK) {
        std::fprintf(stderr, "[FATAL] System hardware initialization failed!\n");
        return 1;
    }

    // 2. Start all statically-allocated FreeRTOS tasks
    if (s_flight_system.startTasks() != Status::OK) {
        std::fprintf(stderr, "[FATAL] Failed to create static tasks!\n");
        return 1;
    }

    // Start supervisor task
    s_supervisor_task.start("Supervisor", 1);

    // 3. Aerospace Safety Enactment: Permanently SEAL the heap!
    // No new, malloc, or dynamic memory allocations permitted after this point.
    os::MemoryTracker::sealBootAllocations();
    std::printf("[BOOT] Post-boot zero-allocation seal ENFORCED. Starting FreeRTOS scheduler...\n\n");

    // 4. Start the FreeRTOS Scheduler
    vTaskStartScheduler();

    // Scheduler should not exit
    return 0;
}
