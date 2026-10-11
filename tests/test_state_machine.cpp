#include <gtest/gtest.h>
#include "surface_controller/fsm/FlightStateMachine.hpp"

using namespace surface_controller;
using namespace surface_controller::fsm;

class FlightStateMachineTest : public ::testing::Test {
protected:
    FlightStateMachine fsm_;
};

TEST_F(FlightStateMachineTest, StartsInInitMode) {
    EXPECT_EQ(fsm_.getCurrentMode(), FlightMode::INIT);
    EXPECT_FALSE(fsm_.isControlActive());
    EXPECT_FALSE(fsm_.isFailSafe());
    EXPECT_FALSE(fsm_.isCalibrated());
}

TEST_F(FlightStateMachineTest, CannotEngageWithoutCalibration) {
    // Attempting to engage before calibration must be rejected
    Status s = fsm_.processEvent(FlightEvent::ENGAGE_REQUEST);
    EXPECT_NE(s, Status::OK);
    EXPECT_EQ(fsm_.getCurrentMode(), FlightMode::INIT);
    EXPECT_FALSE(fsm_.isControlActive());
    EXPECT_TRUE(hasFault(fsm_.getLastFault(), FaultCode::ILLEGAL_STATE_TRANSITION));
}

TEST_F(FlightStateMachineTest, ValidCalibrationAndEngagementSequence) {
    // 1. Trigger calibration
    Status s = fsm_.processEvent(FlightEvent::CALIBRATION_TRIGGER);
    EXPECT_EQ(s, Status::OK);
    EXPECT_EQ(fsm_.getCurrentMode(), FlightMode::CALIBRATE);

    // 2. Complete calibration
    s = fsm_.processEvent(FlightEvent::CALIBRATION_COMPLETE);
    EXPECT_EQ(s, Status::OK);
    EXPECT_EQ(fsm_.getCurrentMode(), FlightMode::INIT);
    EXPECT_TRUE(fsm_.isCalibrated());

    // 3. Now engage flight controller
    s = fsm_.processEvent(FlightEvent::ENGAGE_REQUEST);
    EXPECT_EQ(s, Status::OK);
    EXPECT_EQ(fsm_.getCurrentMode(), FlightMode::ENGAGED);
    EXPECT_TRUE(fsm_.isControlActive());

    // 4. Disarm back to INIT
    s = fsm_.processEvent(FlightEvent::DISARM_REQUEST);
    EXPECT_EQ(s, Status::OK);
    EXPECT_EQ(fsm_.getCurrentMode(), FlightMode::INIT);
    EXPECT_FALSE(fsm_.isControlActive());
}

TEST_F(FlightStateMachineTest, CalibrationFailureTriggersFailSafe) {
    fsm_.processEvent(FlightEvent::CALIBRATION_TRIGGER);
    fsm_.processEvent(FlightEvent::CALIBRATION_FAILED);

    EXPECT_EQ(fsm_.getCurrentMode(), FlightMode::FAIL_SAFE);
    EXPECT_TRUE(fsm_.isFailSafe());
    EXPECT_TRUE(hasFault(fsm_.getLastFault(), FaultCode::SENSOR_CALIBRATION_FAILED));
}

TEST_F(FlightStateMachineTest, CriticalFaultsFromEngagedTriggerFailSafe) {
    // Bring system to ENGAGED
    fsm_.processEvent(FlightEvent::CALIBRATION_TRIGGER);
    fsm_.processEvent(FlightEvent::CALIBRATION_COMPLETE);
    fsm_.processEvent(FlightEvent::ENGAGE_REQUEST);
    EXPECT_EQ(fsm_.getCurrentMode(), FlightMode::ENGAGED);

    // Sensor fault
    fsm_.processEvent(FlightEvent::SENSOR_FAULT_DETECTED);
    EXPECT_EQ(fsm_.getCurrentMode(), FlightMode::FAIL_SAFE);
    EXPECT_TRUE(fsm_.isFailSafe());
    EXPECT_FALSE(fsm_.isControlActive());
}

TEST_F(FlightStateMachineTest, FailSafeIsLatched) {
    fsm_.triggerFailSafe(FaultCode::WATCHDOG_HEARTBEAT_TIMEOUT);
    EXPECT_TRUE(fsm_.isFailSafe());

    // Routine operational requests must be rejected when latched in FAIL_SAFE
    EXPECT_NE(fsm_.processEvent(FlightEvent::ENGAGE_REQUEST), Status::OK);
    EXPECT_NE(fsm_.processEvent(FlightEvent::DISARM_REQUEST), Status::OK);
    EXPECT_NE(fsm_.processEvent(FlightEvent::CALIBRATION_TRIGGER), Status::OK);
    EXPECT_EQ(fsm_.getCurrentMode(), FlightMode::FAIL_SAFE);

    // Explicit ground maintenance reset restores system
    fsm_.reset();
    EXPECT_EQ(fsm_.getCurrentMode(), FlightMode::INIT);
    EXPECT_FALSE(fsm_.isFailSafe());
}
