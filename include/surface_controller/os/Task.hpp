#ifndef SURFACE_CONTROLLER_OS_TASK_HPP
#define SURFACE_CONTROLLER_OS_TASK_HPP

#include <cstdint>
#include <cstddef>
#include "FreeRTOS.h"
#include "task.h"
#include "surface_controller/common/Result.hpp"

namespace surface_controller {
namespace os {

/**
 * @brief Base class for FreeRTOS tasks with static stack and TCB allocation.
 * Guarantees zero heap allocation at runtime.
 */
template <uint32_t StackDepthWords>
class StaticTask {
public:
    static constexpr uint32_t STACK_DEPTH = StackDepthWords;

    StaticTask() = default;
    virtual ~StaticTask() {
        if (handle_ != nullptr) {
            vTaskDelete(handle_);
            handle_ = nullptr;
        }
    }

    // Prohibit copying and moving to preserve static task address stability
    StaticTask(const StaticTask&) = delete;
    StaticTask& operator=(const StaticTask&) = delete;
    StaticTask(StaticTask&&) = delete;
    StaticTask& operator=(StaticTask&&) = delete;

    /**
     * @brief Pure virtual entry point for the task.
     */
    virtual void run() = 0;

    /**
     * @brief Create and start the static task under FreeRTOS.
     */
    Status start(const char* name, UBaseType_t priority) {
        if (handle_ != nullptr) {
            return Status::BUSY;
        }

        handle_ = xTaskCreateStatic(
            &StaticTask::taskEntry,
            name,
            StackDepthWords,
            this,
            priority,
            stack_buffer_,
            &tcb_buffer_
        );

        if (handle_ == nullptr) {
            return Status::ERROR;
        }

        return Status::OK;
    }

    TaskHandle_t getHandle() const noexcept { return handle_; }

    UBaseType_t getPriority() const noexcept {
        return (handle_ != nullptr) ? uxTaskPriorityGet(handle_) : 0;
    }

    void setPriority(UBaseType_t priority) noexcept {
        if (handle_ != nullptr) {
            vTaskPrioritySet(handle_, priority);
        }
    }

    void suspend() noexcept {
        if (handle_ != nullptr) {
            vTaskSuspend(handle_);
        }
    }

    void resume() noexcept {
        if (handle_ != nullptr) {
            vTaskResume(handle_);
        }
    }

    UBaseType_t getStackHighWaterMark() const noexcept {
        return (handle_ != nullptr) ? uxTaskGetStackHighWaterMark(handle_) : 0;
    }

protected:
    /**
     * @brief Deterministic periodic delay using vTaskDelayUntil (rate-monotonic scheduler).
     */
    static void delayUntil(TickType_t& previous_wake_time, TickType_t increment_ticks) noexcept {
        vTaskDelayUntil(&previous_wake_time, increment_ticks);
    }

    /**
     * @brief Simple delay in ticks.
     */
    static void delay(TickType_t ticks) noexcept {
        vTaskDelay(ticks);
    }

    /**
     * @brief Simple delay in milliseconds.
     */
    static void delayMs(uint32_t ms) noexcept {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

private:
    static void taskEntry(void* parameter) {
        auto* self = static_cast<StaticTask*>(parameter);
        if (self != nullptr) {
            self->run();
        }
        // Task should not return. If it does, safely delete
        vTaskDelete(nullptr);
    }

    StaticTask_t tcb_buffer_{};
    StackType_t stack_buffer_[StackDepthWords]{};
    TaskHandle_t handle_{nullptr};
};

} // namespace os
} // namespace surface_controller

#endif // SURFACE_CONTROLLER_OS_TASK_HPP
