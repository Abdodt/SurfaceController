#include <gtest/gtest.h>
#include "surface_controller/control/Mixer.hpp"
#include "surface_controller/drivers/PwmActuatorDriver.hpp"
#include "surface_controller/hal/SimulatedHal.hpp"

using namespace surface_controller;
using namespace surface_controller::control;
using namespace surface_controller::drivers;
using namespace surface_controller::hal;

TEST(MixerTest, PitchRollYawDeflections) {
    // Pure pitch nose up (+0.5f)
    SurfaceDeflections pitch_only = Mixer::mix(0.5f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(pitch_only.elevator, 0.5f);
    EXPECT_FLOAT_EQ(pitch_only.aileron_left, 0.0f);
    EXPECT_FLOAT_EQ(pitch_only.aileron_right, 0.0f);
    EXPECT_FLOAT_EQ(pitch_only.rudder, 0.0f);

    // Pure roll bank right (+0.7f)
    // Left aileron down (+0.7), Right aileron up (-0.7)
    SurfaceDeflections roll_only = Mixer::mix(0.0f, 0.7f, 0.0f);
    EXPECT_FLOAT_EQ(roll_only.elevator, 0.0f);
    EXPECT_FLOAT_EQ(roll_only.aileron_left, 0.7f);
    EXPECT_FLOAT_EQ(roll_only.aileron_right, -0.7f);
    EXPECT_FLOAT_EQ(roll_only.rudder, 0.0f);

    // Pure yaw rudder right (+0.4f)
    SurfaceDeflections yaw_only = Mixer::mix(0.0f, 0.0f, 0.4f);
    EXPECT_FLOAT_EQ(yaw_only.rudder, 0.4f);
}

TEST(MixerTest, SaturationClamping) {
    SurfaceDeflections clamped = Mixer::mix(2.5f, -3.0f, 1.8f);
    EXPECT_FLOAT_EQ(clamped.elevator, 1.0f);
    EXPECT_FLOAT_EQ(clamped.aileron_left, -1.0f);
    EXPECT_FLOAT_EQ(clamped.aileron_right, 1.0f);
    EXPECT_FLOAT_EQ(clamped.rudder, 1.0f);
}

TEST(PwmActuatorDriverTest, DisarmedForcesNeutral) {
    SimulatedPwm pwm;
    PwmActuatorDriver driver(pwm);
    driver.init(50);
    driver.setArmed(false);

    SurfaceDeflections cmd;
    cmd.elevator = 0.8f;
    cmd.aileron_left = 0.5f;
    cmd.aileron_right = -0.5f;
    cmd.rudder = 0.2f;

    driver.writeDeflections(cmd, 0.02f);

    // While disarmed, all channels must remain strictly at neutral 1500 us
    EXPECT_EQ(pwm.getPulseWidthUs(PwmActuatorDriver::CHANNEL_ELEVATOR), PWM_PULSE_NEUTRAL_US);
    EXPECT_EQ(pwm.getPulseWidthUs(PwmActuatorDriver::CHANNEL_AILERON_LEFT), PWM_PULSE_NEUTRAL_US);
    EXPECT_EQ(pwm.getPulseWidthUs(PwmActuatorDriver::CHANNEL_AILERON_RIGHT), PWM_PULSE_NEUTRAL_US);
    EXPECT_EQ(pwm.getPulseWidthUs(PwmActuatorDriver::CHANNEL_RUDDER), PWM_PULSE_NEUTRAL_US);
}

TEST(PwmActuatorDriverTest, SlewRateLimiting) {
    SimulatedPwm pwm;
    PwmActuatorDriver driver(pwm);
    driver.init(50);
    driver.setArmed(true);
    driver.setMaxSlewRate(2.0f); // Max 2.0 deflection units per second

    SurfaceDeflections step;
    step.elevator = 1.0f; // Step jump from 0.0 to 1.0

    // Step at dt = 0.1s -> max change = 2.0 * 0.1 = 0.2
    driver.writeDeflections(step, 0.1f);
    // Deflection should now be 0.2 -> pulse = 1500 + (0.2 * 500) = 1600 us
    EXPECT_NEAR(driver.getCommandedPulseUs(PwmActuatorDriver::CHANNEL_ELEVATOR), 1600, 5);

    // Next step at dt = 0.1s -> deflection becomes 0.4 -> pulse = 1700 us
    driver.writeDeflections(step, 0.1f);
    EXPECT_NEAR(driver.getCommandedPulseUs(PwmActuatorDriver::CHANNEL_ELEVATOR), 1700, 5);
}

TEST(PwmActuatorDriverTest, CommandFailSafeImmediateNeutral) {
    SimulatedPwm pwm;
    PwmActuatorDriver driver(pwm);
    driver.init(50);
    driver.setArmed(true);
    driver.setMaxSlewRate(100.0f); // High slew rate

    SurfaceDeflections cmd;
    cmd.elevator = 1.0f;
    driver.writeDeflections(cmd, 0.1f);
    EXPECT_EQ(pwm.getPulseWidthUs(PwmActuatorDriver::CHANNEL_ELEVATOR), PWM_PULSE_MAX_US);

    // Emergency fail-safe must immediately bypass slew rate and force 1500 us
    driver.commandFailSafe();
    EXPECT_EQ(pwm.getPulseWidthUs(PwmActuatorDriver::CHANNEL_ELEVATOR), PWM_PULSE_NEUTRAL_US);
}
