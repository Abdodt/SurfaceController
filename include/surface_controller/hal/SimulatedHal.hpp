#ifndef SURFACE_CONTROLLER_HAL_SIMULATED_HAL_HPP
#define SURFACE_CONTROLLER_HAL_SIMULATED_HAL_HPP

#include <chrono>
#include <thread>
#include <cstring>
#include <array>
#include "surface_controller/hal/ITimer.hpp"
#include "surface_controller/hal/IPwmOutput.hpp"
#include "surface_controller/hal/IWatchdogTimer.hpp"
#include "surface_controller/hal/ISpiBus.hpp"
#include "surface_controller/hal/II2cBus.hpp"
#include "surface_controller/hal/IGpio.hpp"

namespace surface_controller {
namespace hal {

/**
 * @brief Host platform microsecond timer using std::chrono::steady_clock.
 */
class HostTimer : public ITimer {
public:
    HostTimer() : start_time_(std::chrono::steady_clock::now()) {}

    TimestampUs getTimeUs() const override {
        auto now = std::chrono::steady_clock::now();
        return static_cast<TimestampUs>(
            std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_).count()
        );
    }

    void delayUs(uint32_t us) override {
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }

private:
    std::chrono::steady_clock::time_point start_time_;
};

/**
 * @brief Simulated 4-channel PWM servo interface.
 */
class SimulatedPwm : public IPwmOutput {
public:
    Status init(uint32_t frequency_hz) override {
        frequency_hz_ = frequency_hz;
        pulses_.fill(PWM_PULSE_NEUTRAL_US);
        armed_ = false;
        return Status::OK;
    }

    Status setPulseWidthUs(uint8_t channel, PwmPulseUs pulse_us) override {
        if (channel < pulses_.size()) {
            pulses_[channel] = pulse_us;
            return Status::OK;
        }
        return Status::INVALID_ARGUMENT;
    }

    PwmPulseUs getPulseWidthUs(uint8_t channel) const override {
        if (channel < pulses_.size()) {
            return pulses_[channel];
        }
        return PWM_PULSE_NEUTRAL_US;
    }

    void setArmed(bool armed) override {
        armed_ = armed;
    }

    uint8_t channelCount() const override {
        return static_cast<uint8_t>(pulses_.size());
    }

private:
    uint32_t frequency_hz_{50};
    std::array<PwmPulseUs, 4> pulses_{{PWM_PULSE_NEUTRAL_US, PWM_PULSE_NEUTRAL_US, PWM_PULSE_NEUTRAL_US, PWM_PULSE_NEUTRAL_US}};
    bool armed_{false};
};

/**
 * @brief Simulated hardware watchdog timer.
 */
class SimulatedWatchdog : public IWatchdogTimer {
public:
    Status start(uint32_t timeout_ms) override {
        timeout_ms_ = timeout_ms;
        started_ = true;
        kicked_ = true;
        return Status::OK;
    }

    void kick() override {
        kicked_ = true;
        kick_count_++;
    }

    bool wasResetByWatchdog() const override {
        return false;
    }

    uint32_t getKickCount() const noexcept { return kick_count_; }
    bool isKicked() const noexcept { return kicked_; }
    void clearKickedFlag() noexcept { kicked_ = false; }

private:
    uint32_t timeout_ms_{250};
    bool started_{false};
    bool kicked_{false};
    uint32_t kick_count_{0};
};

/**
 * @brief Loopback / register-based simulated SPI bus.
 */
class MockSpiBus : public ISpiBus {
public:
    MockSpiBus() {
        // Pre-populate WHO_AM_I register (0x75)
        registers_[0x75] = 0x68;
    }

    Status transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length) override {
        if (!selected_ || !tx_data || !rx_data) return Status::ERROR;
        for (size_t i = 0; i < length; ++i) {
            rx_data[i] = registers_[tx_data[i] & 0x7F];
        }
        return Status::OK;
    }

    Status write(const uint8_t* tx_data, size_t length) override {
        if (!selected_ || !tx_data) return Status::ERROR;
        if (length == 1) {
            current_reg_ = tx_data[0] & 0x7F;
        } else if (length > 1) {
            current_reg_ = tx_data[0] & 0x7F;
            for (size_t i = 1; i < length; ++i) {
                registers_[current_reg_ + (i - 1)] = tx_data[i];
            }
        }
        return Status::OK;
    }

    Status read(uint8_t* rx_data, size_t length) override {
        if (!selected_ || !rx_data) return Status::ERROR;
        for (size_t i = 0; i < length; ++i) {
            rx_data[i] = registers_[current_reg_ + i];
        }
        return Status::OK;
    }

    void select() override { selected_ = true; }
    void deselect() override { selected_ = false; }

    void setRegister(uint8_t reg, uint8_t val) { registers_[reg] = val; }
    uint8_t getRegister(uint8_t reg) const { return registers_[reg]; }

    void setRegisters(uint8_t start_reg, const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; ++i) {
            registers_[start_reg + i] = data[i];
        }
    }

private:
    bool selected_{false};
    uint8_t current_reg_{0};
    uint8_t registers_[256]{};
};

/**
 * @brief Simulated GPIO pin.
 */
class SimulatedGpio : public IGpio {
public:
    void write(bool state) override { state_ = state; }
    bool read() const override { return state_; }
    void toggle() override { state_ = !state_; }

private:
    bool state_{false};
};

} // namespace hal
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_HAL_SIMULATED_HAL_HPP
