#include "surface_controller/os/MemoryTracker.hpp"
#include <cstdlib>
#include <new>
#include <cstdio>

namespace surface_controller {
namespace os {

std::atomic<bool> MemoryTracker::boot_sealed_{false};
std::atomic<uint32_t> MemoryTracker::violation_count_{0};
std::atomic<size_t> MemoryTracker::boot_bytes_allocated_{0};
std::atomic<size_t> MemoryTracker::total_allocations_{0};
std::atomic<size_t> MemoryTracker::total_deallocations_{0};

void MemoryTracker::sealBootAllocations() noexcept {
    boot_sealed_.store(true, std::memory_order_seq_cst);
}

bool MemoryTracker::isBootSealed() noexcept {
    return boot_sealed_.load(std::memory_order_seq_cst);
}

bool MemoryTracker::recordAllocation(size_t bytes) noexcept {
    total_allocations_.fetch_add(1, std::memory_order_relaxed);
    if (boot_sealed_.load(std::memory_order_seq_cst)) {
        violation_count_.fetch_add(1, std::memory_order_seq_cst);
        return false;
    }
    boot_bytes_allocated_.fetch_add(bytes, std::memory_order_relaxed);
    return true;
}

void MemoryTracker::recordDeallocation(size_t /*bytes*/) noexcept {
    total_deallocations_.fetch_add(1, std::memory_order_relaxed);
}

uint32_t MemoryTracker::getViolationCount() noexcept {
    return violation_count_.load(std::memory_order_seq_cst);
}

size_t MemoryTracker::getBootBytesAllocated() noexcept {
    return boot_bytes_allocated_.load(std::memory_order_relaxed);
}

void MemoryTracker::resetForTest() noexcept {
    boot_sealed_.store(false, std::memory_order_seq_cst);
    violation_count_.store(0, std::memory_order_seq_cst);
    boot_bytes_allocated_.store(0, std::memory_order_relaxed);
    total_allocations_.store(0, std::memory_order_relaxed);
    total_deallocations_.store(0, std::memory_order_relaxed);
}

ZeroHeapScope::ZeroHeapScope() noexcept {
    was_sealed_ = MemoryTracker::isBootSealed();
    initial_violations_ = MemoryTracker::getViolationCount();
    MemoryTracker::sealBootAllocations();
}

ZeroHeapScope::~ZeroHeapScope() noexcept {
    if (!was_sealed_) {
        // Restore previous unsealed state if it wasn't sealed before
        // But leave violations counter intact
    }
}

bool ZeroHeapScope::violated() const noexcept {
    return MemoryTracker::getViolationCount() > initial_violations_;
}

} // namespace os
} // namespace surface_controller

#if defined(SURFACE_CONTROLLER_ENABLE_HEAP_INTERCEPTION)

// Global operator new/delete overrides to intercept any unauthorized runtime allocations
void* operator new(std::size_t count) {
    if (!surface_controller::os::MemoryTracker::recordAllocation(count)) {
        #ifndef NDEBUG
        std::fprintf(stderr, "[FATAL] Unauthorized dynamic memory allocation (%zu bytes) post-boot!\n", count);
        #endif
    }
    return std::malloc(count);
}

void* operator new[](std::size_t count) {
    if (!surface_controller::os::MemoryTracker::recordAllocation(count)) {
        #ifndef NDEBUG
        std::fprintf(stderr, "[FATAL] Unauthorized dynamic memory array allocation (%zu bytes) post-boot!\n", count);
        #endif
    }
    return std::malloc(count);
}

void operator delete(void* ptr) noexcept {
    if (ptr) {
        surface_controller::os::MemoryTracker::recordDeallocation(0);
        std::free(ptr);
    }
}

void operator delete[](void* ptr) noexcept {
    if (ptr) {
        surface_controller::os::MemoryTracker::recordDeallocation(0);
        std::free(ptr);
    }
}

void operator delete(void* ptr, std::size_t /*sz*/) noexcept {
    if (ptr) {
        surface_controller::os::MemoryTracker::recordDeallocation(0);
        std::free(ptr);
    }
}

void operator delete[](void* ptr, std::size_t /*sz*/) noexcept {
    if (ptr) {
        surface_controller::os::MemoryTracker::recordDeallocation(0);
        std::free(ptr);
    }
}

#endif // SURFACE_CONTROLLER_ENABLE_HEAP_INTERCEPTION
