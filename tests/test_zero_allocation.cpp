#include <gtest/gtest.h>
#include "surface_controller/os/MemoryTracker.hpp"
#include "surface_controller/control/FlightSurfaceController.hpp"
#include "surface_controller/fsm/FlightStateMachine.hpp"
#include "surface_controller/drivers/SimulatedImuDriver.hpp"
#include "surface_controller/drivers/PwmActuatorDriver.hpp"
#include "surface_controller/hal/SimulatedHal.hpp"

using namespace surface_controller;

TEST(ZeroAllocationTest, CriticalFlightLoopsExecuteWithZeroHeapAllocations) {
    os::MemoryTracker::resetForTest();

    // Instantiate stack-allocated objects
    control::FlightSurfaceController controller;
    fsm::FlightStateMachine fsm;
    drivers::SimulatedImuDriver imu;
    imu.init();

    hal::SimulatedPwm pwm;
    drivers::PwmActuatorDriver actuator(pwm);
    actuator.init(50);
    actuator.setArmed(true);

    FlightSetpoints setpoints;
    setpoints.pitch_rate_rad_s = 0.2f;
    setpoints.roll_rate_rad_s = -0.1f;

    // Enter strictly enforced zero-heap scope
    {
        os::ZeroHeapScope scope;

        // 1. Perform 100 cycles of IMU ingestion & calibration
        imu.startCalibration(50);
        for (int i = 0; i < 50; ++i) {
            auto res = imu.readSample();
            if (res.is_ok()) {
                imu.processCalibrationSample(res.value());
            }
        }

        // 2. Perform state machine transitions
        fsm.processEvent(fsm::FlightEvent::CALIBRATION_TRIGGER);
        fsm.processEvent(fsm::FlightEvent::CALIBRATION_COMPLETE);
        fsm.processEvent(fsm::FlightEvent::ENGAGE_REQUEST);
        EXPECT_EQ(fsm.getCurrentMode(), fsm::FlightMode::ENGAGED);

        // 3. Perform 200 cycles of 50 Hz closed-loop control calculation
        for (int i = 0; i < 200; ++i) {
            auto imu_res = imu.readSample();
            if (imu_res.is_ok()) {
                SurfaceDeflections defl = controller.update(setpoints, imu_res.value(), 0.02f);
                actuator.writeDeflections(defl, 0.02f);
                controller.recordCycleTiming(20000, 150);
            }
        }

        // 4. Trigger fail-safe transition
        fsm.processEvent(fsm::FlightEvent::SENSOR_FAULT_DETECTED);
        actuator.commandFailSafe();
        EXPECT_TRUE(fsm.isFailSafe());

        // Verify zero heap allocation occurred in this scope!
        EXPECT_FALSE(scope.violated());
        EXPECT_EQ(os::MemoryTracker::getViolationCount(), 0);
    }
}
