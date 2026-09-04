/**
 * @file net/wiznet/executor.hpp
 * @brief Execution policies for the WIZnet socket provider.
 *
 * `WiznetSocketProvider<Chip, Bus, Executor>` is identical whether it runs in a
 * bare superloop or under FreeRTOS — the register maps, ring buffers, UDP
 * framing and socket-claim table do not change. The one axis that does is
 * *how you wait for a receive event*:
 *
 *  - @ref PollingExecutor — cooperative. `recv_from()` polls the chip's RX
 *    register; between polls it parks the CPU in `WFE`
 *    (`rosette_platform_wait_for_event`), woken by the chip's INT line if
 *    `board_init` wired that GPIO IRQ, or by a short backstop timeout
 *    otherwise. No interrupt task; the superloop owns the CPU.
 *
 *  - @ref FreeRtosExecutor — event-driven. A dedicated task drains the chip's
 *    interrupt registers and unblocks the per-socket task parked in
 *    `recv_from()` via a direct-to-task notification.
 *
 * A new RTOS is a new policy struct, not a new copy of the provider. Each
 * policy is a class template on the `WiznetDevice` type so it can hold a typed
 * reference without type erasure.
 *
 * Required surface (both policies):
 *   - `explicit Executor(Device&) noexcept`
 *   - `static uint32_t now_ms() noexcept`
 *   - `bool start() noexcept` / `void stop() noexcept` — dispatch lifecycle
 *   - `struct CriticalSection` — RAII guard around the socket-claim table
 *   - `struct RxScope` — RAII guard bracketing a `recv_from()` wait
 *   - `bool wait_rx(uint32_t remaining_ms) noexcept` — block up to the budget,
 *     then let the caller re-poll (the caller owns the deadline check)
 */

#pragma once

#include <cstdint>

#include "rosette/platform/host_runtime.hpp"

#ifndef ROSETTE_NO_FREERTOS
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}
#endif

namespace rosette::net::wiznet {

/**
 * @brief Cooperative, single-threaded execution policy for a bare superloop.
 *
 * No interrupt task is spawned. `recv_from()` polls the chip directly; between
 * polls this policy parks the CPU in `WFE` via
 * `rosette_platform_wait_for_event()`. When `board_init` has wired the chip's
 * INT line to a GPIO IRQ, an arriving datagram wakes the wait in microseconds;
 * otherwise the backstop timeout bounds the latency. Critical sections are
 * no-ops because the superloop is the only context touching the driver.
 */
template<typename Device>
struct PollingExecutor {
    explicit PollingExecutor(Device& /*device*/) noexcept {}

    static uint32_t now_ms() noexcept { return rosette_platform_get_ms(); }

    bool start() noexcept { return true; }
    void stop() noexcept {}

    /// No cross-context contention in a superloop.
    struct CriticalSection {
        CriticalSection() noexcept = default;
        CriticalSection(const CriticalSection&) = delete;
        CriticalSection& operator=(const CriticalSection&) = delete;
    };

    /// Nothing to arm: `recv_from()` polls the chip register itself.
    struct RxScope {
        RxScope(Device& /*device*/, uint8_t /*socket*/) noexcept {}
        RxScope(const RxScope&) = delete;
        RxScope& operator=(const RxScope&) = delete;
    };

    /**
     * @brief Parks in `WFE` until the chip interrupts or the backstop elapses.
     *
     * With the INT GPIO IRQ wired, a receive wakes this almost immediately. The
     * cap only bounds the rare missed-edge case (a burst datagram landing while
     * INT is already asserted low); the caller's `recv_from()` re-poll then
     * picks it up on the next backstop tick, so no datagram is lost.
     */
    bool wait_rx(uint32_t remaining_ms) noexcept
    {
        rosette_platform_wait_for_event(
            remaining_ms < kBackstopMs ? remaining_ms : kBackstopMs);
        return true;
    }

    static constexpr uint32_t kBackstopMs = 20u;
};

#ifndef ROSETTE_NO_FREERTOS

/**
 * @brief Event-driven execution policy for FreeRTOS.
 *
 * `start()` spawns the `wiz_irq` task, which parks on a notification from the
 * chip ISR (`WiznetDevice::handle_interrupt_from_isr`), drains the socket
 * interrupt registers (`service_socket_interrupts`) and notifies whichever task
 * is waiting in `recv_from()` for that socket.
 */
template<typename Device>
class FreeRtosExecutor {
public:
    explicit FreeRtosExecutor(Device& device) noexcept
        : device_(device) {}

    FreeRtosExecutor(const FreeRtosExecutor&) = delete;
    FreeRtosExecutor& operator=(const FreeRtosExecutor&) = delete;

    ~FreeRtosExecutor() { stop(); }

    static uint32_t now_ms() noexcept
    {
        return static_cast<uint32_t>(xTaskGetTickCount()) *
               static_cast<uint32_t>(portTICK_PERIOD_MS);
    }

    bool start() noexcept
    {
        if (task_ != nullptr) {
            return true;
        }
        const BaseType_t rc = xTaskCreate(&trampoline,
                                          "wiz_irq",
                                          512u,
                                          this,
                                          configMAX_PRIORITIES - 2u,
                                          &task_);
        return rc == pdPASS;
    }

    void stop() noexcept
    {
        TaskHandle_t task = nullptr;
        taskENTER_CRITICAL();
        task = task_;
        task_ = nullptr;
        taskEXIT_CRITICAL();
        if (task != nullptr) {
            vTaskDelete(task);
        }
    }

    /// Guards the provider's socket-claim table against concurrent `open()`.
    struct CriticalSection {
        CriticalSection() noexcept { taskENTER_CRITICAL(); }
        ~CriticalSection() { taskEXIT_CRITICAL(); }
        CriticalSection(const CriticalSection&) = delete;
        CriticalSection& operator=(const CriticalSection&) = delete;
    };

    /// Registers the calling task as the wake target for @p socket's RX events.
    struct RxScope {
        RxScope(Device& device, uint8_t socket) noexcept
            : device_(device), socket_(socket)
        {
            device_.register_socket_rx_task(socket_);
        }
        ~RxScope() { device_.unregister_socket_rx_task(socket_); }
        RxScope(const RxScope&) = delete;
        RxScope& operator=(const RxScope&) = delete;

    private:
        Device& device_;
        uint8_t socket_;
    };

    /**
     * @brief Blocks until the dispatch task signals RX data or the budget lapses.
     *
     * The return value is advisory; the caller re-polls and owns the deadline.
     */
    bool wait_rx(uint32_t remaining_ms) noexcept
    {
        return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(remaining_ms)) != 0u;
    }

private:
    static void trampoline(void* arg) noexcept
    {
        static_cast<FreeRtosExecutor*>(arg)->run();
    }

    void run() noexcept
    {
        device_.register_interrupt_dispatch_task();
        for (;;) {
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            device_.service_socket_interrupts();
        }
    }

    Device& device_;
    TaskHandle_t task_ = nullptr;
};

template<typename Device>
using DefaultExecutor = FreeRtosExecutor<Device>;

#else  // ROSETTE_NO_FREERTOS

template<typename Device>
using DefaultExecutor = PollingExecutor<Device>;

#endif

}  // namespace rosette::net::wiznet
