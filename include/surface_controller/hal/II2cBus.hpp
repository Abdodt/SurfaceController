#ifndef SURFACE_CONTROLLER_HAL_II2C_BUS_HPP
#define SURFACE_CONTROLLER_HAL_II2C_BUS_HPP

#include <cstdint>
#include <cstddef>
#include "surface_controller/common/Result.hpp"

namespace surface_controller {
namespace hal {

/**
 * @brief Abstract interface for Hardware I2C Bus.
 */
class II2cBus {
public:
    virtual ~II2cBus() = default;

    /**
     * @brief Write registers over I2C to a slave address.
     */
    virtual Status writeRegister(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, size_t length) = 0;

    /**
     * @brief Read registers over I2C from a slave address.
     */
    virtual Status readRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t length) = 0;
};

} // namespace hal
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_HAL_II2C_BUS_HPP
