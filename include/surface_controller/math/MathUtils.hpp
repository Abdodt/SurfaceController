#ifndef SURFACE_CONTROLLER_MATH_MATH_UTILS_HPP
#define SURFACE_CONTROLLER_MATH_MATH_UTILS_HPP

#include <cmath>
#include <cstdint>
#include <algorithm>

namespace surface_controller {
namespace math {

constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;

constexpr float degToRad(float deg) noexcept {
    return deg * DEG_TO_RAD;
}

constexpr float radToDeg(float rad) noexcept {
    return rad * RAD_TO_DEG;
}

template <typename T>
constexpr T clamp(T val, T min_val, T max_val) noexcept {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

template <typename T>
constexpr T deadband(T val, T band) noexcept {
    if (val > band) {
        return val - band;
    } else if (val < -band) {
        return val + band;
    }
    return static_cast<T>(0);
}

inline float slewRateLimit(float current, float target, float max_rate, float dt_s) noexcept {
    const float max_delta = max_rate * dt_s;
    const float delta = target - current;
    return current + clamp(delta, -max_delta, max_delta);
}

inline float mapRange(float val, float in_min, float in_max, float out_min, float out_max) noexcept {
    const float clamped = clamp(val, in_min, in_max);
    return out_min + (clamped - in_min) * (out_max - out_min) / (in_max - in_min);
}

template <typename T>
constexpr T absDiff(T a, T b) noexcept {
    return (a >= b) ? (a - b) : (b - a);
}

} // namespace math
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_MATH_MATH_UTILS_HPP
