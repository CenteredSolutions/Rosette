#pragma once

#include <cstddef>
#include <cstdint>

#include "hardware/gpio.h"
#include "hardware/spi.h"

#ifndef ROSETTE_NO_FREERTOS
#  include "FreeRTOS.h"
#  include "semphr.h"
#  include "task.h"
#endif

namespace rosette::platform {

class PicoSpiBus {
public:
    using address_type = uint32_t;
#ifndef ROSETTE_NO_FREERTOS
    using lock_timeout_type = TickType_t;
    static constexpr lock_timeout_type wait_forever = portMAX_DELAY;
#else
    using lock_timeout_type = uint32_t;
    static constexpr lock_timeout_type wait_forever = 0u;
#endif

    struct Config {
        spi_inst_t* spi = nullptr;
        uint cs_pin = 0u;
        uint8_t read_fill = 0xFFu;
    };

    explicit PicoSpiBus(const Config& config) noexcept
        : spi_(config.spi)
        , cs_pin_(config.cs_pin)
        , read_fill_(config.read_fill)
    {
#ifndef ROSETTE_NO_FREERTOS
        mutex_ = xSemaphoreCreateMutex();
#endif
    }

    PicoSpiBus(const PicoSpiBus& other) noexcept
        : spi_(other.spi_)
        , cs_pin_(other.cs_pin_)
        , read_fill_(other.read_fill_)
    {
#ifndef ROSETTE_NO_FREERTOS
        mutex_ = xSemaphoreCreateMutex();
#endif
    }

    PicoSpiBus(PicoSpiBus&& other) noexcept
        : spi_(other.spi_)
        , cs_pin_(other.cs_pin_)
        , read_fill_(other.read_fill_)
    {
#ifndef ROSETTE_NO_FREERTOS
        mutex_ = xSemaphoreCreateMutex();
#endif
    }

    PicoSpiBus& operator=(const PicoSpiBus&) = delete;
    PicoSpiBus& operator=(PicoSpiBus&&) = delete;

    PicoSpiBus(spi_inst_t* spi, uint cs_pin) noexcept
        : PicoSpiBus(Config{spi, cs_pin, 0xFFu}) {}

    class LockGuard {
    public:
        explicit LockGuard(PicoSpiBus& bus, lock_timeout_type timeout = wait_forever) noexcept
            : bus_(&bus)
            , locked_(bus.lock(timeout)) {}

        LockGuard(const LockGuard&) = delete;
        LockGuard& operator=(const LockGuard&) = delete;

        LockGuard(LockGuard&& other) noexcept
            : bus_(other.bus_)
            , locked_(other.locked_)
        {
            other.bus_ = nullptr;
            other.locked_ = false;
        }

        LockGuard& operator=(LockGuard&&) = delete;

        ~LockGuard()
        {
            if (locked_ && (bus_ != nullptr)) {
                bus_->unlock();
            }
        }

        /**
         * @brief Explicit conversion operator to bool.
         *
         * Allows objects of this class to be explicitly converted to a boolean value,
         * indicating whether the object is currently locked.
         *
         * @return true if the object is locked, false otherwise.
         *
         * @note The 'explicit' keyword prevents implicit conversions to bool,
         *       requiring the user to write 'if (static_cast<bool>(obj))' or 'if (obj.operator bool())'
         *       instead of just 'if (obj)'. This helps avoid unintended conversions.
         */
        explicit operator bool() const noexcept
        {
            return locked_;
        }

    private:
        PicoSpiBus* bus_ = nullptr;
        bool locked_ = false;
    };

    LockGuard acquire(lock_timeout_type timeout = wait_forever) noexcept
    {
        return LockGuard(*this, timeout);
    }

    void write(const uint8_t* prefix,
               std::size_t prefix_length,
               const uint8_t* data,
               std::size_t length) {
        if (spi_ == nullptr || prefix == nullptr || prefix_length == 0u ||
            data == nullptr || length == 0u) {
            return;
        }

        const auto lock = acquire();
        if (!lock) {
            return;
        }

        begin_transaction();
        spi_write_blocking(spi_, prefix, prefix_length);
        spi_write_blocking(spi_, data, static_cast<size_t>(length));
        end_transaction();
    }

    void read(const uint8_t* prefix,
              std::size_t prefix_length,
              uint8_t* data,
              std::size_t length) {
        if (spi_ == nullptr || prefix == nullptr || prefix_length == 0u ||
            data == nullptr || length == 0u) {
            return;
        }

        const auto lock = acquire();
        if (!lock) {
            return;
        }

        begin_transaction();
        spi_write_blocking(spi_, prefix, prefix_length);
        spi_read_blocking(spi_, read_fill_, data, static_cast<size_t>(length));
        end_transaction();
    }

    spi_inst_t* spi() const noexcept { return spi_; }
    uint cs_pin() const noexcept { return cs_pin_; }

private:
    bool lock(lock_timeout_type timeout) noexcept
    {
#ifndef ROSETTE_NO_FREERTOS
        return (mutex_ == nullptr) ||
               (xSemaphoreTake(mutex_, timeout) == pdTRUE);
#else
        (void)timeout;
        return true;
#endif
    }

    void unlock() noexcept
    {
#ifndef ROSETTE_NO_FREERTOS
        if (mutex_ != nullptr) {
            (void)xSemaphoreGive(mutex_);
        }
#endif
    }

    void begin_transaction() {
#ifndef ROSETTE_NO_FREERTOS
        taskENTER_CRITICAL();
#endif
        gpio_put(cs_pin_, 0);
    }

    void end_transaction() {
        gpio_put(cs_pin_, 1);
#ifndef ROSETTE_NO_FREERTOS
        taskEXIT_CRITICAL();
#endif
    }

    spi_inst_t* spi_;
    uint cs_pin_;
    uint8_t read_fill_;
#ifndef ROSETTE_NO_FREERTOS
    SemaphoreHandle_t mutex_ = nullptr;
#endif
};

}  // namespace rosette::platform
