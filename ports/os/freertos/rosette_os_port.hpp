/**
 * @file ports/os/freertos/rosette_os_port.hpp
 * @brief FreeRTOS OS layer of the platform contract.
 *
 * platform.hpp includes this header by its fixed name when the build selects
 * ROSETTE_OS=freertos (port.cmake beside it); see platform.hpp for the
 * contract each type satisfies. Both types create their kernel
 * object once, at construction (which may precede the scheduler start):
 * statically when the board's FreeRTOSConfig.h sets
 * configSUPPORT_STATIC_ALLOCATION, otherwise from the kernel heap. Nothing
 * allocates afterwards. On SMP ports (RP2350 dual-core) the kernel's
 * semaphores already serialize across cores.
 */

#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

namespace rosette::platform {

/**
 * @brief Mutex over a statically allocated FreeRTOS mutex (priority inheritance).
 *
 * Must not be used from interrupt handlers.
 */
class Mutex {
public:
#if configSUPPORT_STATIC_ALLOCATION
    Mutex() noexcept : handle_(xSemaphoreCreateMutexStatic(&storage_)) {}
#else
    Mutex() noexcept : handle_(xSemaphoreCreateMutex()) { configASSERT(handle_); }
#endif
    ~Mutex() { vSemaphoreDelete(handle_); }
    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;

    /// @brief Blocks until the mutex is held by the calling task.
    void lock() noexcept { xSemaphoreTake(handle_, portMAX_DELAY); }
    /// @brief Releases a mutex the calling task holds.
    void unlock() noexcept { xSemaphoreGive(handle_); }
    /// @brief Takes the mutex if free; returns whether it was taken.
    bool try_lock() noexcept { return xSemaphoreTake(handle_, 0) == pdTRUE; }

private:
#if configSUPPORT_STATIC_ALLOCATION
    StaticSemaphore_t storage_{};
#endif
    SemaphoreHandle_t handle_;
};

/**
 * @brief Auto-reset event over a statically allocated binary semaphore.
 */
class Event {
public:
#if configSUPPORT_STATIC_ALLOCATION
    Event() noexcept : handle_(xSemaphoreCreateBinaryStatic(&storage_)) {}
#else
    Event() noexcept : handle_(xSemaphoreCreateBinary()) { configASSERT(handle_); }
#endif
    ~Event() { vSemaphoreDelete(handle_); }
    Event(const Event&) = delete;
    Event& operator=(const Event&) = delete;

    /// @brief Wakes one current or future wait; task context only.
    void signal() noexcept { xSemaphoreGive(handle_); }

    /// @brief Wakes one current or future wait from an interrupt handler,
    /// yielding to the woken task on return when it has higher priority.
    void signal_from_isr() noexcept {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(handle_, &woken);
        portYIELD_FROM_ISR(woken);
    }

    /**
     * @brief Waits for a signal, consuming it.
     *
     * @param[in] timeout_ms Longest wait; 0 polls, kWaitForever blocks.
     * @return `true` when signaled, `false` on timeout.
     */
    bool wait_for(uint32_t timeout_ms) noexcept {
        const TickType_t ticks =
            (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
        return xSemaphoreTake(handle_, ticks) == pdTRUE;
    }

    /// @brief Clears a pending signal without waiting; returns whether one was pending.
    bool consume() noexcept { return xSemaphoreTake(handle_, 0) == pdTRUE; }

private:
#if configSUPPORT_STATIC_ALLOCATION
    StaticSemaphore_t storage_{};
#endif
    SemaphoreHandle_t handle_;
};

/**
 * @brief Blocks the calling task for at least `ms` milliseconds.
 */
inline void sleep_ms(uint32_t ms) noexcept {
    const TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay((ticks == 0 && ms > 0) ? 1 : ticks);  // never round a real delay to a no-op
}

}  // namespace rosette::platform
