#ifndef SURFACE_CONTROLLER_OS_MUTEX_HPP
#define SURFACE_CONTROLLER_OS_MUTEX_HPP

#include "FreeRTOS.h"
#include "semphr.h"

namespace surface_controller {
namespace os {

/**
 * @brief Zero-allocation FreeRTOS Mutex wrapper using static allocation.
 * Provides priority inheritance by default.
 */
class StaticMutex {
public:
    StaticMutex() {
        handle_ = xSemaphoreCreateMutexStatic(&mutex_buffer_);
    }

    ~StaticMutex() {
        if (handle_ != nullptr) {
            vSemaphoreDelete(handle_);
            handle_ = nullptr;
        }
    }

    StaticMutex(const StaticMutex&) = delete;
    StaticMutex& operator=(const StaticMutex&) = delete;
    StaticMutex(StaticMutex&&) = delete;
    StaticMutex& operator=(StaticMutex&&) = delete;

    bool lock(TickType_t wait_ticks = portMAX_DELAY) noexcept {
        if (handle_ == nullptr) return false;
        return xSemaphoreTake(handle_, wait_ticks) == pdTRUE;
    }

    bool tryLock() noexcept {
        return lock(0);
    }

    bool unlock() noexcept {
        if (handle_ == nullptr) return false;
        return xSemaphoreGive(handle_) == pdTRUE;
    }

    SemaphoreHandle_t getHandle() const noexcept { return handle_; }

private:
    StaticSemaphore_t mutex_buffer_{};
    SemaphoreHandle_t handle_{nullptr};
};

/**
 * @brief RAII scoped lock guard for StaticMutex.
 */
class LockGuard {
public:
    explicit LockGuard(StaticMutex& mutex, TickType_t wait_ticks = portMAX_DELAY)
        : mutex_(mutex), locked_(mutex_.lock(wait_ticks)) {}

    ~LockGuard() {
        if (locked_) {
            mutex_.unlock();
        }
    }

    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;

    bool isLocked() const noexcept { return locked_; }

private:
    StaticMutex& mutex_;
    bool locked_{false};
};

} // namespace os
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_OS_MUTEX_HPP
