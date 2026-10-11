#ifndef SURFACE_CONTROLLER_COMMON_FAULT_CODES_HPP
#define SURFACE_CONTROLLER_COMMON_FAULT_CODES_HPP

#include <cstdint>

namespace surface_controller {

/**
 * @brief System fault codes categorized by severity for flight safety.
 */
enum class FaultCode : uint32_t {
    NONE                        = 0x00000000,
    SENSOR_INIT_FAILED          = 0x00000001,
    SENSOR_COMM_ERROR           = 0x00000002,
    SENSOR_TIMEOUT              = 0x00000004,
    SENSOR_OUT_OF_BOUNDS        = 0x00000008,
    SENSOR_STUCK_DATA           = 0x00000010,
    SENSOR_CALIBRATION_FAILED   = 0x00000020,
    
    CONTROL_TIMING_DEADLINE     = 0x00000100,
    CONTROL_DIVERGENCE          = 0x00000200,
    ACTUATOR_SATURATION         = 0x00000400,
    
    WATCHDOG_HEARTBEAT_TIMEOUT  = 0x00001000,
    WATCHDOG_KICK_SUPPRESSED    = 0x00002000,
    
    ILLEGAL_STATE_TRANSITION    = 0x00010000,
    UNAUTHORIZED_HEAP_ALLOC     = 0x00020000,
    EMERGENCY_DISARM_REQUEST    = 0x00040000
};

inline constexpr FaultCode operator|(FaultCode a, FaultCode b) {
    return static_cast<FaultCode>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline constexpr FaultCode operator&(FaultCode a, FaultCode b) {
    return static_cast<FaultCode>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

inline constexpr bool hasFault(FaultCode flags, FaultCode test) {
    return (static_cast<uint32_t>(flags) & static_cast<uint32_t>(test)) != 0;
}

const char* faultCodeToString(FaultCode fault);

} // namespace surface_controller

#endif // SURFACE_CONTROLLER_COMMON_FAULT_CODES_HPP
