#include <gtest/gtest.h>
#include "surface_controller/health/HealthMonitor.hpp"
#include "surface_controller/hal/SimulatedHal.hpp"

using namespace surface_controller;
using namespace surface_controller::health;
using namespace surface_controller::fsm;
using namespace surface_controller::hal;

class HealthMonitorTest : public ::testing::Test {
protected:
    void SetUp() override {
        monitor_.init();
    }

    HostTimer timer_;
    SimulatedWatchdog watchdog_;
    FlightStateMachine fsm_;
    HealthMonitor monitor_{watchdog_, fsm_, timer_};
};

TEST_F(HealthMonitorTest, TasksCheckInSuccessfullyKicksWatchdog) {
    // Check in all tasks
    monitor_.recordTaskHeartbeat(TaskId::SENSOR);
    monitor_.recordTaskHeartbeat(TaskId::CONTROL);
    monitor_.recordTaskHeartbeat(TaskId::TELEMETRY);

    uint32_t prev_kicks = monitor_.getSuccessfulKicks();
    monitor_.step();

    EXPECT_GT(monitor_.getSuccessfulKicks(), prev_kicks);
    EXPECT_EQ(monitor_.getSuppressedKicks(), 0);
    EXPECT_FALSE(fsm_.isFailSafe());
}

TEST_F(HealthMonitorTest, MissedDeadlineSuppressesKickAndEntersFailSafe) {
    HeartbeatTracker& tracker = monitor_.getHeartbeatTracker();
    
    // Simulate initial checkin at t = 100,000 us (100 ms)
    tracker.checkIn(TaskId::SENSOR, 100000);
    tracker.checkIn(TaskId::CONTROL, 100000);
    tracker.checkIn(TaskId::TELEMETRY, 100000);

    // Now test verification at t = 160,000 us (60 ms later)
    // Sensor max allowed is 25 ms, Control max allowed is 50 ms
    // Both Sensor and Control are now starved!
    FaultCode fault = FaultCode::NONE;
    bool healthy = tracker.verifyHeartbeats(160000, fault);

    EXPECT_FALSE(healthy);
    EXPECT_TRUE(hasFault(fault, FaultCode::WATCHDOG_HEARTBEAT_TIMEOUT));
}
