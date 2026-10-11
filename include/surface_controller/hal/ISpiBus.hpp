#ifndef SURFACE_CONTROLLER_HAL_ISPI_BUS_HPP
#define SURFACE_CONTROLLER_HAL_ISPI_BUS_HPP

#include <cstdint>
#include <cstddef>
#include "surface_controller/common/Result.hpp"

namespace surface_controller {
namespace hal {

/**
 * @brief Abstract interface for Hardware SPI Bus.
 */
class ISpiBus {
public:
    virtual ~ISpiBus() = default;

    /**
     * @brief Perform full-duplex SPI transfer.
     */
    virtual Status transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length) = 0;

    /**
     * @brief Write data over SPI.
     */
    virtual Status write(const uint8_t* tx_data, size_t length) = 0;

    /**
     * @brief Read data over SPI.
     */
    virtual Status read(uint8_t* rx_data, size_t length) = 0;

    /**
     * @brief Assert chip select (Active Low).
     */
    virtual void select() = 0;

    /**
     * @brief De-assert chip select.
     */
    virtual void deselect() = 0;
};

} // namespace hal
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_HAL_ISPI_BUS_HPP
