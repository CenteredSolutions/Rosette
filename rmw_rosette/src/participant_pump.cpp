/**
 * @file participant_pump.cpp
 * @brief ParticipantPump: the host thread that runs a participant, and its waiters.
 *
 * See rmw_rosette/participant_pump.hpp for the design.
 */

#include "rmw_rosette/participant_pump.hpp"

#if RMW_ROSETTE_PUMP_THREAD

#  include "rosette/rtps/participant_driver.hpp"

namespace rmw_rosette {

bool ParticipantPump::start(rosette::rtps::ParticipantDriver& driver) noexcept {
    driver_ = &driver;
    stop_requested_.store(false);
    if (!thread_.start(&ParticipantPump::run, this, "rosette_pump")) {
        driver_ = nullptr;
        return false;
    }

    return true;
}

void ParticipantPump::stop() noexcept {
    if (driver_ == nullptr) {
        return;
    }

    stop_requested_.store(true);
    driver_->wake();
    thread_.join();
    driver_ = nullptr;
}

void ParticipantPump::run(void* self) noexcept {
    auto& pump = *static_cast<ParticipantPump*>(self);
    pump.driver_->spin_until(
        [&pump]() noexcept {
            pump.wake_ready_waiters_locked();
            return pump.stop_requested_.load();
        },
        rosette::platform::kWaitForever);
}

void ParticipantPump::wake_ready_waiters_locked() noexcept {
    for (Waiter* waiter = waiters_; waiter != nullptr; waiter = waiter->next) {
        if (waiter->ready(waiter->argument)) {
            waiter->event->signal();
        }
    }
}

void ParticipantPump::unlink_locked(Waiter& waiter) noexcept {
    for (Waiter** link = &waiters_; *link != nullptr; link = &(*link)->next) {
        if (*link == &waiter) {
            *link = waiter.next;
            return;
        }
    }
}

bool ParticipantPump::wait_until(ReadyFunction ready, void* argument,
                                 uint32_t timeout_ms) noexcept {
    // One event per thread: a thread is in at most one wait at a time, and
    // the pump signals it only while it is linked.
    thread_local rosette::platform::Event event;
    rosette::platform::Mutex&             mutex = driver_->mutex();
    const uint32_t                        start_ms = rosette::platform::monotonic_ms();

    mutex.lock();
    bool held = ready(argument);
    if (!held && timeout_ms != 0u) {
        // A signal left from an earlier wait of this thread would end the
        // first sleep early; it is harmless, but drop it before sleeping.
        (void)event.consume();
        Waiter self{ready, argument, &event, waiters_};
        waiters_ = &self;
        for (;;) {
            uint32_t sleep_ms = rosette::platform::kWaitForever;
            if (timeout_ms != rosette::platform::kWaitForever) {
                const uint32_t elapsed = rosette::platform::monotonic_ms() - start_ms;
                if (elapsed >= timeout_ms) {
                    break;
                }

                sleep_ms = timeout_ms - elapsed;
            }

            mutex.unlock();
            (void)event.wait_for(sleep_ms);
            mutex.lock();
            held = ready(argument);
            if (held) {
                break;
            }
        }

        unlink_locked(self);
    }

    mutex.unlock();
    return held;
}

}  // namespace rmw_rosette

#endif  // RMW_ROSETTE_PUMP_THREAD
