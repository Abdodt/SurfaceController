#include "surface_controller/health/HeartbeatTracker.hpp"

namespace surface_controller {
namespace health {

HeartbeatTracker::HeartbeatTracker() {
    reset();
}

void HeartbeatTracker::registerTask(TaskId id, const char* name, uint32_t max_allowed_interval_us) {
    auto idx = static_cast<size_t>(id);
    if (idx < static_cast<size_t>(TaskId::COUNT)) {
        records_[idx].name = name;
        records_[idx].max_allowed_interval_us = max_allowed_interval_us;
        records_[idx].last_checkin_us = 0;
        records_[idx].checkin_count = 0;
        records_[idx].missed_deadlines = 0;
        records_[idx].is_registered = true;
    }
}

void HeartbeatTracker::checkIn(TaskId id, TimestampUs current_time_us) {
    auto idx = static_cast<size_t>(id);
    if (idx < static_cast<size_t>(TaskId::COUNT) && records_[idx].is_registered) {
        records_[idx].last_checkin_us = current_time_us;
        records_[idx].checkin_count++;
    }
}

bool HeartbeatTracker::verifyHeartbeats(TimestampUs current_time_us, FaultCode& fault_out) {
    bool all_healthy = true;
    fault_out = FaultCode::NONE;

    for (size_t i = 0; i < static_cast<size_t>(TaskId::COUNT); ++i) {
        auto& rec = records_[i];
        if (!rec.is_registered) continue;

        // Skip check if task has not checked in yet during warm-up period (first 500 ms)
        if (rec.last_checkin_us == 0) {
            if (current_time_us > 500000ULL) { // 500 ms
                rec.missed_deadlines++;
                all_healthy = false;
                fault_out = fault_out | FaultCode::WATCHDOG_HEARTBEAT_TIMEOUT;
            }
            continue;
        }

        if (current_time_us > rec.last_checkin_us) {
            uint64_t elapsed_us = current_time_us - rec.last_checkin_us;
            if (elapsed_us > rec.max_allowed_interval_us) {
                rec.missed_deadlines++;
                all_healthy = false;
                fault_out = fault_out | FaultCode::WATCHDOG_HEARTBEAT_TIMEOUT;
            }
        }
    }

    return all_healthy;
}

const TaskHeartbeatRecord& HeartbeatTracker::getRecord(TaskId id) const noexcept {
    auto idx = static_cast<size_t>(id);
    return records_[idx];
}

void HeartbeatTracker::reset() {
    for (size_t i = 0; i < static_cast<size_t>(TaskId::COUNT); ++i) {
        records_[i] = TaskHeartbeatRecord{};
    }
}

} // namespace health
} // namespace surface_controller
