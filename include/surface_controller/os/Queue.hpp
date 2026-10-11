#ifndef SURFACE_CONTROLLER_OS_QUEUE_HPP
#define SURFACE_CONTROLLER_OS_QUEUE_HPP

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include "FreeRTOS.h"
#include "queue.h"

namespace surface_controller {
namespace os {

/**
 * @brief Type-safe statically-allocated FreeRTOS queue wrapper.
 * Strictly zero heap allocation.
 */
template <typename T, size_t Capacity>
class StaticQueue {
    static_assert(Capacity > 0, "Capacity must be greater than zero");
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable for FreeRTOS queue semantics");

public:
    static constexpr size_t QUEUE_CAPACITY = Capacity;
    static constexpr size_t ITEM_SIZE = sizeof(T);

    StaticQueue() {
        handle_ = xQueueCreateStatic(
            static_cast<UBaseType_t>(Capacity),
            static_cast<UBaseType_t>(sizeof(T)),
            storage_area_,
            &queue_buffer_
        );
    }

    ~StaticQueue() {
        if (handle_ != nullptr) {
            vQueueDelete(handle_);
            handle_ = nullptr;
        }
    }

    StaticQueue(const StaticQueue&) = delete;
    StaticQueue& operator=(const StaticQueue&) = delete;
    StaticQueue(StaticQueue&&) = delete;
    StaticQueue& operator=(StaticQueue&&) = delete;

    /**
     * @brief Post an item to the back of the queue.
     */
    bool send(const T& item, TickType_t wait_ticks = 0) noexcept {
        if (handle_ == nullptr) return false;
        return xQueueSend(handle_, &item, wait_ticks) == pdPASS;
    }

    /**
     * @brief Post an item from an Interrupt Service Routine.
     */
    bool sendFromISR(const T& item, BaseType_t* higher_priority_task_woken = nullptr) noexcept {
        if (handle_ == nullptr) return false;
        return xQueueSendFromISR(handle_, &item, higher_priority_task_woken) == pdPASS;
    }

    /**
     * @brief Overwrite item in queue (for single-item latest telemetry queues).
     */
    bool overwrite(const T& item) noexcept {
        static_assert(Capacity == 1, "Overwrite only valid for capacity 1 queues");
        if (handle_ == nullptr) return false;
        return xQueueOverwrite(handle_, &item) == pdPASS;
    }

    /**
     * @brief Receive an item from the queue.
     */
    bool receive(T& item, TickType_t wait_ticks = portMAX_DELAY) noexcept {
        if (handle_ == nullptr) return false;
        return xQueueReceive(handle_, &item, wait_ticks) == pdPASS;
    }

    /**
     * @brief Receive an item from an ISR.
     */
    bool receiveFromISR(T& item, BaseType_t* higher_priority_task_woken = nullptr) noexcept {
        if (handle_ == nullptr) return false;
        return xQueueReceiveFromISR(handle_, &item, higher_priority_task_woken) == pdPASS;
    }

    /**
     * @brief Peek at the item at the front without removing it.
     */
    bool peek(T& item, TickType_t wait_ticks = 0) const noexcept {
        if (handle_ == nullptr) return false;
        return xQueuePeek(handle_, &item, wait_ticks) == pdPASS;
    }

    /**
     * @brief Return number of items currently in the queue.
     */
    size_t size() const noexcept {
        return (handle_ != nullptr) ? static_cast<size_t>(uxQueueMessagesWaiting(handle_)) : 0;
    }

    /**
     * @brief Return available capacity remaining in the queue.
     */
    size_t spaceAvailable() const noexcept {
        return (handle_ != nullptr) ? static_cast<size_t>(uxQueueSpacesAvailable(handle_)) : 0;
    }

    bool isEmpty() const noexcept { return size() == 0; }
    bool isFull() const noexcept { return spaceAvailable() == 0; }

    void reset() noexcept {
        if (handle_ != nullptr) {
            xQueueReset(handle_);
        }
    }

    QueueHandle_t getHandle() const noexcept { return handle_; }

private:
    StaticQueue_t queue_buffer_{};
    uint8_t storage_area_[Capacity * sizeof(T)]{};
    QueueHandle_t handle_{nullptr};
};

} // namespace os
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_OS_QUEUE_HPP
