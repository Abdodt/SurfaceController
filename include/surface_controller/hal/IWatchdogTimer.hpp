#ifndef SURFACE_CONTROLLER_HAL_IWATCHDOG_TIMER_HPP
#define SURFACE_CONTROLLER_HAL_IWATCHDOG_TIMER_HPP

#include <cstdint>
#include "surface_controller/common/Result.hpp"

namespace surface_controller {
namespace hal {

/**
 * @brief Hardware watchdog timer interface.
 * If not refreshed within the timeout period, triggers an MCU hardware reset.
 */
class IWatchdogTimer {
public:
    virtual ~IWatchdogTimer() = default;

    /**
     * @brief Start the hardware watchdog with a given timeout window in milliseconds.
     */
    virtual Status start(uint32_t timeout_ms) = 0;

    /**
     * @brief Kick / refresh / pet the watchdog timer to prevent system reset.
     */
    virtual void kick() = 0;

    /**
     * @brief Check if previous reboot was caused by a watchdog reset.
     */
    virtual bool wasResetByWatchdog() const = 0;
};

} // namespace hal
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_HAL_IWATCHDOG_TIMER_HPP
