#ifndef SURFACE_CONTROLLER_HAL_IGPIO_HPP
#define SURFACE_CONTROLLER_HAL_IGPIO_HPP

#include <cstdint>

namespace surface_controller {
namespace hal {

/**
 * @brief Abstract interface for General Purpose Input/Output pins.
 */
class IGpio {
public:
    virtual ~IGpio() = default;

    virtual void write(bool state) = 0;
    virtual bool read() const = 0;
    virtual void toggle() = 0;
};

} // namespace hal
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_HAL_IGPIO_HPP
