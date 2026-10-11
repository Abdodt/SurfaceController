#ifndef SURFACE_CONTROLLER_OS_SEMAPHORE_HPP
#define SURFACE_CONTROLLER_OS_SEMAPHORE_HPP

#include <cstdint>
#include "FreeRTOS.h"
#include "semphr.h"

namespace surface_controller {
namespace os {

/**
 * @brief Zero-allocation FreeRTOS Binary Semaphore wrapper.
 */
class StaticBinarySemaphore {
public:
    StaticBinarySemaphore() {
        handle_ = xSemaphoreCreateBinaryStatic(&sem_buffer_);
    }

    ~StaticBinarySemaphore() {
        if (handle_ != nullptr) {
            vSemaphoreDelete(handle_);
            handle_ = nullptr;
        }
    }

    StaticBinarySemaphore(const StaticBinarySemaphore&) = delete;
    StaticBinarySemaphore& operator=(const StaticBinarySemaphore&) = delete;

    bool give() noexcept {
        if (handle_ == nullptr) return false;
        return xSemaphoreGive(handle_) == pdTRUE;
    }

    bool giveFromISR(BaseType_t* higher_priority_task_woken = nullptr) noexcept {
        if (handle_ == nullptr) return false;
        return xSemaphoreGiveFromISR(handle_, higher_priority_task_woken) == pdTRUE;
    }

    bool take(TickType_t wait_ticks = portMAX_DELAY) noexcept {
        if (handle_ == nullptr) return false;
        return xSemaphoreTake(handle_, wait_ticks) == pdTRUE;
    }

    bool takeFromISR(BaseType_t* higher_priority_task_woken = nullptr) noexcept {
        if (handle_ == nullptr) return false;
        return xSemaphoreTakeFromISR(handle_, higher_priority_task_woken) == pdTRUE;
    }

    SemaphoreHandle_t getHandle() const noexcept { return handle_; }

private:
    StaticSemaphore_t sem_buffer_{};
    SemaphoreHandle_t handle_{nullptr};
};

/**
 * @brief Zero-allocation FreeRTOS Counting Semaphore wrapper.
 */
template <UBaseType_t MaxCount, UBaseType_t InitialCount = 0>
class StaticCountingSemaphore {
public:
    StaticCountingSemaphore() {
        handle_ = xSemaphoreCreateCountingStatic(MaxCount, InitialCount, &sem_buffer_);
    }

    ~StaticCountingSemaphore() {
        if (handle_ != nullptr) {
            vSemaphoreDelete(handle_);
            handle_ = nullptr;
        }
    }

    StaticCountingSemaphore(const StaticCountingSemaphore&) = delete;
    StaticCountingSemaphore& operator=(const StaticCountingSemaphore&) = delete;

    bool give() noexcept {
        if (handle_ == nullptr) return false;
        return xSemaphoreGive(handle_) == pdTRUE;
    }

    bool take(TickType_t wait_ticks = portMAX_DELAY) noexcept {
        if (handle_ == nullptr) return false;
        return xSemaphoreTake(handle_, wait_ticks) == pdTRUE;
    }

    SemaphoreHandle_t getHandle() const noexcept { return handle_; }

private:
    StaticSemaphore_t sem_buffer_{};
    SemaphoreHandle_t handle_{nullptr};
};

} // namespace os
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_OS_SEMAPHORE_HPP
