#ifndef SURFACE_CONTROLLER_MATH_FILTER_HPP
#define SURFACE_CONTROLLER_MATH_FILTER_HPP

#include <cmath>
#include <algorithm>
#include "surface_controller/math/MathUtils.hpp"

namespace surface_controller {
namespace math {

/**
 * @brief Discrete 1st-order low-pass filter for sensor measurements and derivative damping.
 */
class LowPassFilter {
public:
    constexpr LowPassFilter() = default;

    /**
     * @brief Initialize filter with cutoff frequency (Hz) and sample rate (Hz).
     */
    void configure(float cutoff_freq_hz, float sample_rate_hz) noexcept {
        if (cutoff_freq_hz <= 0.0f || sample_rate_hz <= 0.0f) {
            alpha_ = 1.0f;
            return;
        }
        const float dt = 1.0f / sample_rate_hz;
        const float rc = 1.0f / (2.0f * PI * cutoff_freq_hz);
        alpha_ = dt / (rc + dt);
        alpha_ = clamp(alpha_, 0.0f, 1.0f);
    }

    float update(float input) noexcept {
        if (!initialized_) {
            output_ = input;
            initialized_ = true;
            return output_;
        }
        output_ += alpha_ * (input - output_);
        return output_;
    }

    void reset(float initial_value = 0.0f) noexcept {
        output_ = initial_value;
        initialized_ = false;
    }

    float value() const noexcept { return output_; }
    bool is_initialized() const noexcept { return initialized_; }

private:
    float alpha_{1.0f};
    float output_{0.0f};
    bool initialized_{false};
};

/**
 * @brief 3-sample median filter for rejecting transient spikes / EMI noise without phase lag.
 */
class MedianFilter3 {
public:
    constexpr MedianFilter3() = default;

    float update(float sample) noexcept {
        buffer_[head_] = sample;
        head_ = (head_ + 1) % 3;
        if (count_ < 3) {
            ++count_;
        }

        if (count_ < 3) {
            return sample;
        }

        // Sort 3 elements without dynamic memory
        float a = buffer_[0];
        float b = buffer_[1];
        float c = buffer_[2];

        if ((a <= b && b <= c) || (c <= b && b <= a)) {
            return b;
        }
        if ((b <= a && a <= c) || (c <= a && a <= b)) {
            return a;
        }
        return c;
    }

    void reset() noexcept {
        buffer_[0] = 0.0f;
        buffer_[1] = 0.0f;
        buffer_[2] = 0.0f;
        head_ = 0;
        count_ = 0;
    }

private:
    float buffer_[3]{0.0f, 0.0f, 0.0f};
    uint8_t head_{0};
    uint8_t count_{0};
};

} // namespace math
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_MATH_FILTER_HPP
