#ifndef SURFACE_CONTROLLER_HEALTH_HEARTBEAT_TRACKER_HPP
#define SURFACE_CONTROLLER_HEALTH_HEARTBEAT_TRACKER_HPP

#include <cstdint>
#include <cstddef>
#include "surface_controller/common/Types.hpp"
#include "surface_controller/common/FaultCodes.hpp"

namespace surface_controller {
namespace health {

/**
 * @brief Identifiers for critical system tasks.
 */
enum class TaskId : uint8_t {
    SENSOR = 0,     ///< 100 Hz sensor ingestion task
    CONTROL = 1,    ///< 50 Hz PID surface control task
    TELEMETRY = 2,  ///< 10 Hz telemetry & logging task
    COUNT = 3
};

struct TaskHeartbeatRecord {
    const char* name{nullptr};
    TimestampUs last_checkin_us{0};
    uint32_t max_allowed_interval_us{0};
    uint32_t checkin_count{0};
    uint32_t missed_deadlines{0};
    bool is_registered{false};
};

/**
 * @brief Thread-safe (lockless / atomic) heartbeat tracker for real-time tasks.
 * Validates that each task satisfies its Rate-Monotonic timing bounds.
 */
class HeartbeatTracker {
public:
    HeartbeatTracker();

    /**
     * @brief Register a task with its maximum allowable silence window.
     */
    void registerTask(TaskId id, const char* name, uint32_t max_allowed_interval_us);

    /**
     * @brief Called by a task upon completing its periodic cycle.
     */
    void checkIn(TaskId id, TimestampUs current_time_us);

    /**
     * @brief Evaluates all registered task heartbeats against their deadlines.
     * @param current_time_us Current monotonic timestamp
     * @param fault_out Populated with specific fault if deadline violated
     * @return true if all tasks healthy, false if any task missed deadline
     */
    bool verifyHeartbeats(TimestampUs current_time_us, FaultCode& fault_out);

    const TaskHeartbeatRecord& getRecord(TaskId id) const noexcept;

    void reset();

private:
    TaskHeartbeatRecord records_[static_cast<size_t>(TaskId::COUNT)];
};

} // namespace health
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_HEALTH_HEARTBEAT_TRACKER_HPP
