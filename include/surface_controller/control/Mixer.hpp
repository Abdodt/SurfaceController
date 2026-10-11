#ifndef SURFACE_CONTROLLER_CONTROL_MIXER_HPP
#define SURFACE_CONTROLLER_CONTROL_MIXER_HPP

#include "surface_controller/common/Types.hpp"
#include "surface_controller/math/MathUtils.hpp"

namespace surface_controller {
namespace control {

/**
 * @brief Aerodynamic flight surface mixer.
 * Converts 3-axis stabilization control efforts into individual surface deflections:
 *  - Elevator = Pitch effort
 *  - Left Aileron = +Roll effort
 *  - Right Aileron = -Roll effort
 *  - Rudder = Yaw effort
 */
class Mixer {
public:
    constexpr Mixer() = default;

    /**
     * @brief Mix 3-axis controller efforts into normalized surface deflections [-1.0f, +1.0f].
     */
    static SurfaceDeflections mix(float pitch_effort, float roll_effort, float yaw_effort) noexcept {
        SurfaceDeflections deflections;
        deflections.elevator = math::clamp(pitch_effort, -1.0f, 1.0f);
        deflections.aileron_left = math::clamp(roll_effort, -1.0f, 1.0f);
        deflections.aileron_right = math::clamp(-roll_effort, -1.0f, 1.0f);
        deflections.rudder = math::clamp(yaw_effort, -1.0f, 1.0f);
        deflections.is_fail_safe = false;
        return deflections;
    }

    /**
     * @brief Create neutral/glide trim deflections for fail-safe state.
     */
    static SurfaceDeflections getFailSafeDeflections() noexcept {
        SurfaceDeflections deflections;
        deflections.elevator = 0.0f;       // Neutral elevator
        deflections.aileron_left = 0.0f;   // Neutral ailerons
        deflections.aileron_right = 0.0f;
        deflections.rudder = 0.0f;         // Neutral rudder
        deflections.is_fail_safe = true;
        return deflections;
    }
};

} // namespace control
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_CONTROL_MIXER_HPP
