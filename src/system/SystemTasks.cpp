#include "surface_controller/system/SystemTasks.hpp"
#include <cstdio>

namespace surface_controller {
namespace system {

SensorTask::SensorTask(SystemContext& context)
    : ctx_(context) {}

void SensorTask::run() {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(10); // 100 Hz = 10 ms

    while (!ctx_.stop_requested) {
        // Deterministic delay for Rate-Monotonic execution
        delayUntil(last_wake_time, period_ticks);

        // 1. Ingest IMU sample
        auto sample_res = ctx_.imu_driver.readSample();
        if (sample_res.is_ok()) {
            ImuSample sample = sample_res.value();

            // Handle calibration mode
            if (ctx_.state_machine.getCurrentMode() == fsm::FlightMode::CALIBRATE) {
                if (!cal_started_) {
                    ctx_.imu_driver.startCalibration(100);
                    cal_started_ = true;
                }
                bool done = ctx_.imu_driver.processCalibrationSample(sample);
                if (done) {
                    ctx_.state_machine.setCalibrationComplete(true);
                    cal_started_ = false;
                }
            }

            // Post to control queue (overwrite if full to ensure latest data)
            if (ctx_.imu_queue.isFull()) {
                ImuSample dummy;
                ctx_.imu_queue.receive(dummy, 0);
            }
            ctx_.imu_queue.send(sample, 0);
        } else {
            // Sensor acquisition failure
            ctx_.state_machine.triggerFailSafe(FaultCode::SENSOR_COMM_ERROR);
        }

        // 2. Health monitor heartbeat check-in
        ctx_.health_monitor.recordTaskHeartbeat(health::TaskId::SENSOR);
    }
}

ControlTask::ControlTask(SystemContext& context)
    : ctx_(context) {}

void ControlTask::run() {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(20); // 50 Hz = 20 ms
    TimestampUs last_loop_time_us = ctx_.timer.getTimeUs();

    while (!ctx_.stop_requested) {
        delayUntil(last_wake_time, period_ticks);

        TimestampUs start_time_us = ctx_.timer.getTimeUs();
        uint32_t period_us = static_cast<uint32_t>(start_time_us - last_loop_time_us);
        last_loop_time_us = start_time_us;

        // 1. Fetch latest sensor data (non-blocking)
        ImuSample imu_sample;
        bool has_sample = ctx_.imu_queue.receive(imu_sample, 0);

        // 2. Fetch thread-safe setpoints
        FlightSetpoints current_setpoints;
        {
            os::LockGuard lock(ctx_.setpoint_mutex);
            current_setpoints = ctx_.setpoints;
        }

        // 3. Mode-dependent flight surface control
        const float dt_s = 0.02f;
        auto mode = ctx_.state_machine.getCurrentMode();

        if (mode == fsm::FlightMode::ENGAGED && has_sample && imu_sample.is_valid) {
            ctx_.actuator_driver.setArmed(true);
            SurfaceDeflections deflections = ctx_.controller.update(
                current_setpoints,
                imu_sample,
                dt_s
            );
            ctx_.actuator_driver.writeDeflections(deflections, dt_s);
        } else if (mode == fsm::FlightMode::FAIL_SAFE) {
            ctx_.actuator_driver.commandFailSafe();
            ctx_.actuator_driver.setArmed(false);
            ctx_.controller.reset();
        } else {
            // INIT or CALIBRATE
            ctx_.actuator_driver.commandFailSafe();
            ctx_.actuator_driver.setArmed(false);
        }

        // 4. Timing jitter recording
        TimestampUs end_time_us = ctx_.timer.getTimeUs();
        uint32_t exec_time_us = static_cast<uint32_t>(end_time_us - start_time_us);
        ctx_.controller.recordCycleTiming(period_us, exec_time_us);

        // 5. Health monitor check-in
        ctx_.health_monitor.recordTaskHeartbeat(health::TaskId::CONTROL);

        if (ctx_.max_cycles > 0 && ctx_.controller.getTimingMetrics().cycle_count >= ctx_.max_cycles) {
            ctx_.stop_requested = true;
        }
    }
}

WatchdogTask::WatchdogTask(SystemContext& context)
    : ctx_(context) {}

void WatchdogTask::run() {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(50); // 20 Hz = 50 ms

    while (!ctx_.stop_requested) {
        delayUntil(last_wake_time, period_ticks);

        // Step the health monitor to evaluate all task heartbeats and kick watchdog
        ctx_.health_monitor.step();
    }
}

TelemetryTask::TelemetryTask(SystemContext& context)
    : ctx_(context) {}

void TelemetryTask::run() {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(100); // 10 Hz = 100 ms
    uint32_t log_counter = 0;

    while (!ctx_.stop_requested) {
        delayUntil(last_wake_time, period_ticks);

        if (++log_counter % 5 == 0) {
            const auto& metrics = ctx_.controller.getTimingMetrics();
            auto mode = ctx_.state_machine.getCurrentMode();

            std::printf("[TELEM] Mode=%s | Cyc=%u | Per=%u us | Jitter=%d us (Max=%d us) | Elev=%u us | AilL=%u us | Rud=%u us | WD_Kicks=%u\n",
                        fsm::flightModeToString(mode),
                        metrics.cycle_count,
                        metrics.period_us,
                        metrics.jitter_us,
                        metrics.max_jitter_us,
                        ctx_.actuator_driver.getCommandedPulseUs(drivers::PwmActuatorDriver::CHANNEL_ELEVATOR),
                        ctx_.actuator_driver.getCommandedPulseUs(drivers::PwmActuatorDriver::CHANNEL_AILERON_LEFT),
                        ctx_.actuator_driver.getCommandedPulseUs(drivers::PwmActuatorDriver::CHANNEL_RUDDER),
                        ctx_.health_monitor.getSuccessfulKicks());
        }

        ctx_.health_monitor.recordTaskHeartbeat(health::TaskId::TELEMETRY);
    }
}

FlightControllerSystem::FlightControllerSystem(SystemContext& context)
    : ctx_(context),
      sensor_task_(context),
      control_task_(context),
      watchdog_task_(context),
      telemetry_task_(context) {}

Status FlightControllerSystem::init() {
    Status s = ctx_.actuator_driver.init(50);
    if (s != Status::OK) return s;

    s = ctx_.imu_driver.init();
    if (s != Status::OK) return s;

    s = ctx_.health_monitor.init();
    if (s != Status::OK) return s;

    return Status::OK;
}

Status FlightControllerSystem::startTasks() {
    // Start tasks in Rate-Monotonic order with static allocations
    Status s = sensor_task_.start("Sensor100Hz", TaskPriorities::SENSOR);
    if (s != Status::OK) return s;

    s = control_task_.start("Control50Hz", TaskPriorities::CONTROL);
    if (s != Status::OK) return s;

    s = watchdog_task_.start("Watchdog20Hz", TaskPriorities::WATCHDOG);
    if (s != Status::OK) return s;

    s = telemetry_task_.start("Telem10Hz", TaskPriorities::TELEMETRY);
    if (s != Status::OK) return s;

    return Status::OK;
}

} // namespace system
} // namespace surface_controller
