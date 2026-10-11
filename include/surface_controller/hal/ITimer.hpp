#ifndef SURFACE_CONTROLLER_HAL_ITIMER_HPP
#define SURFACE_CONTROLLER_HAL_ITIMER_HPP

#include <cstdint>
#include "surface_controller/common/Types.hpp"

namespace surface_controller {
namespace hal {

/**
 * @brief High-precision timer interface for sub-millisecond timestamps and jitter profiling.
 */
class ITimer {
public:
    virtual ~ITimer() = default;

    /**
     * @brief Get monotonic time in microseconds.
     */
    virtual TimestampUs getTimeUs() const = 0;

    /**
     * @brief Busy-wait delay in microseconds (used sparingly during hardware init).
     */
    virtual void delayUs(uint32_t us) = 0;
};

} // namespace hal
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_HAL_ITIMER_HPP
