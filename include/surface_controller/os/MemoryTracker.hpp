#ifndef SURFACE_CONTROLLER_OS_MEMORY_TRACKER_HPP
#define SURFACE_CONTROLLER_OS_MEMORY_TRACKER_HPP

#include <cstdint>
#include <cstddef>
#include <atomic>

namespace surface_controller {
namespace os {

/**
 * @brief Memory subsystem monitor and post-boot zero-heap enforcer.
 * Conforms to DO-178C / MISRA-C++ guidelines prohibiting dynamic memory
 * allocation during deterministic flight operations.
 */
class MemoryTracker {
public:
    /**
     * @brief Seal heap allocation. Called once at the end of system boot/task initialization.
     * After this call, any dynamic allocation is treated as a critical aerospace safety violation.
     */
    static void sealBootAllocations() noexcept;

    /**
     * @brief Check whether boot memory allocations have been permanently sealed.
     */
    static bool isBootSealed() noexcept;

    /**
     * @brief Record an allocation event (called by memory hooks/operators).
     * Returns true if allocation is permitted (during boot), false if post-boot violation.
     */
    static bool recordAllocation(size_t bytes) noexcept;

    /**
     * @brief Record a deallocation event.
     */
    static void recordDeallocation(size_t bytes) noexcept;

    /**
     * @brief Get count of post-boot allocation violations.
     */
    static uint32_t getViolationCount() noexcept;

    /**
     * @brief Get cumulative bytes allocated during boot.
     */
    static size_t getBootBytesAllocated() noexcept;

    /**
     * @brief Reset tracker state (used exclusively in unit tests).
     */
    static void resetForTest() noexcept;

private:
    static std::atomic<bool> boot_sealed_;
    static std::atomic<uint32_t> violation_count_;
    static std::atomic<size_t> boot_bytes_allocated_;
    static std::atomic<size_t> total_allocations_;
    static std::atomic<size_t> total_deallocations_;
};

/**
 * @brief RAII Scope Guard that strictly prohibits heap allocations while active.
 * Throws failure or trips violation counter if any allocation occurs within scope.
 */
class ZeroHeapScope {
public:
    ZeroHeapScope() noexcept;
    ~ZeroHeapScope() noexcept;

    bool violated() const noexcept;

private:
    uint32_t initial_violations_{0};
    bool was_sealed_{false};
};

} // namespace os
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_OS_MEMORY_TRACKER_HPP
