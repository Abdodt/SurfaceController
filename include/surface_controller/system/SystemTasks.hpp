#ifndef SURFACE_CONTROLLER_SYSTEM_SYSTEM_TASKS_HPP
#define SURFACE_CONTROLLER_SYSTEM_SYSTEM_TASKS_HPP

#include <cstdint>
#include "surface_controller/os/Task.hpp"
#include "surface_controller/os/Queue.hpp"
#include "surface_controller/os/Mutex.hpp"
#include "surface_controller/drivers/ImuDriver.hpp"
#include "surface_controller/drivers/PwmActuatorDriver.hpp"
#include "surface_controller/control/FlightSurfaceController.hpp"
#include "surface_controller/fsm/FlightStateMachine.hpp"
#include "surface_controller/health/HealthMonitor.hpp"
#include "surface_controller/hal/ITimer.hpp"

namespace surface_controller {
namespace system {

/**
 * @brief Rate-Monotonic Priority assignments.
 * In accordance with Rate-Monotonic Scheduling (RMS) theory, tasks with shorter
 * periods receive strictly higher priorities.
 */
struct TaskPriorities {
    static constexpr UBaseType_t SENSOR    = 4;  ///< 100 Hz (10 ms period) - Highest
    static constexpr UBaseType_t CONTROL   = 3;  ///< 50 Hz  (20 ms period)
    static constexpr UBaseType_t WATCHDOG  = 2;  ///< 20 Hz  (50 ms period)
    static constexpr UBaseType_t TELEMETRY = 1;  ///< 10 Hz  (100 ms period) - Lowest
};

/**
 * @brief Statically allocated context binding all hardware drivers and state.
 */
struct SystemContext {
    drivers::IImuDriver& imu_driver;
    drivers::PwmActuatorDriver& actuator_driver;
    control::FlightSurfaceController& controller;
    fsm::FlightStateMachine& state_machine;
    health::HealthMonitor& health_monitor;
    const hal::ITimer& timer;

    // Zero-allocation static queue transferring IMU data from 100 Hz sensor task to 50 Hz control task
    os::StaticQueue<ImuSample, 4> imu_queue;

    // Flight setpoints protected by static mutex
    FlightSetpoints setpoints{};
    os::StaticMutex setpoint_mutex;

    // Simulation run duration control (0 = run forever)
    uint32_t max_cycles{0};
    bool stop_requested{false};
};

#ifdef PTHREAD_STACK_MIN
constexpr uint32_t SYSTEM_TASK_STACK_WORDS = 4096;
#else
constexpr uint32_t SYSTEM_TASK_STACK_WORDS = 1024;
#endif

/**
 * @brief 100 Hz Sensor Ingestion Task (Period: 10 ms, Priority: 4).
 */
class SensorTask : public os::StaticTask<SYSTEM_TASK_STACK_WORDS> {
public:
    explicit SensorTask(SystemContext& context);
    void run() override;

private:
    SystemContext& ctx_;
    bool cal_started_{false};
};

/**
 * @brief 50 Hz Control Loop Task (Period: 20 ms, Priority: 3).
 */
class ControlTask : public os::StaticTask<SYSTEM_TASK_STACK_WORDS> {
public:
    explicit ControlTask(SystemContext& context);
    void run() override;

private:
    SystemContext& ctx_;
};

/**
 * @brief 20 Hz Health Monitor / Watchdog Task (Period: 50 ms, Priority: 2).
 */
class WatchdogTask : public os::StaticTask<SYSTEM_TASK_STACK_WORDS> {
public:
    explicit WatchdogTask(SystemContext& context);
    void run() override;

private:
    SystemContext& ctx_;
};

/**
 * @brief 10 Hz Telemetry & Diagnostic Logger Task (Period: 100 ms, Priority: 1).
 */
class TelemetryTask : public os::StaticTask<SYSTEM_TASK_STACK_WORDS> {
public:
    explicit TelemetryTask(SystemContext& context);
    void run() override;

private:
    SystemContext& ctx_;
};

/**
 * @brief Complete flight system orchestrator holding all static task instances.
 */
class FlightControllerSystem {
public:
    explicit FlightControllerSystem(SystemContext& context);

    Status init();
    Status startTasks();

private:
    SystemContext& ctx_;
    SensorTask sensor_task_;
    ControlTask control_task_;
    WatchdogTask watchdog_task_;
    TelemetryTask telemetry_task_;
};

} // namespace system
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_SYSTEM_SYSTEM_TASKS_HPP
