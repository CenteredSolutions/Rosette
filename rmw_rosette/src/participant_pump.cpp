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
    while (!pump.stop_requested_.load()) {
        // While a thread receives in wait_until(), the pump keeps the timers
        // and answers wake() but leaves the sockets to that thread; it
        // switches back once the last one leaves (both ends call wake()).
        const bool pump_receives = pump.receiving_waiters_.load() == 0u;
        pump.driver_->spin_until(
            [&pump, pump_receives]() noexcept {
                pump.wake_ready_waiters_locked();
                return pump.stop_requested_.load()
                       || (pump.receiving_waiters_.load() == 0u) != pump_receives;
            },
            rosette::platform::kWaitForever, pump.driver_->wake_event(), pump_receives);
    }
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

    mutex.lock();
    const bool held_at_once = ready(argument);
    if (held_at_once || timeout_ms == 0u) {
        mutex.unlock();
        return held_at_once;
    }

    // A signal left from an earlier wait of this thread would end the first
    // sleep early; it is harmless, but drop it before sleeping.
    (void)event.consume();
    Waiter self{ready, argument, &event, waiters_};
    waiters_ = &self;
    mutex.unlock();

    // This thread receives for itself: the datagram that makes `ready` hold
    // wakes it directly, and it runs the pass that delivers it, so a round
    // trip pays no handoff from the pump. The first receiving thread moves
    // the pump off the sockets, so it neither wakes for that datagram nor
    // holds the lock when this thread wants it. Staying linked lets the pump
    // signal this thread for what only the pump sees: wake() from a guard
    // condition or an rmw entry point.
    if (receiving_waiters_.fetch_add(1u) == 0u) {
        driver_->wake();
    }

    const bool held = driver_->spin_until([ready, argument]() noexcept { return ready(argument); },
                                          timeout_ms, event);

    if (receiving_waiters_.fetch_sub(1u) == 1u) {
        driver_->wake();
    }

    mutex.lock();
    unlink_locked(self);
    mutex.unlock();
    return held;
}

}  // namespace rmw_rosette

#endif  // RMW_ROSETTE_PUMP_THREAD
