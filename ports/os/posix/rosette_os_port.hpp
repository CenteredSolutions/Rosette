/**
 * @file ports/os/posix/rosette_os_port.hpp
 * @brief POSIX OS layer of the platform contract (Linux, macOS).
 *
 * platform.hpp includes this header by its fixed name when the build selects
 * ROSETTE_OS=posix (port.cmake beside it); see platform.hpp for the contract
 * each type satisfies.
 *
 * Event is a non-blocking self-pipe rather than a condition variable so that
 * its read end can sit in the same poll() as the transport's sockets: one
 * wait returns on whichever comes first, a datagram or a signal
 * (PosixSocketProvider::wait_readable()). Writing to a pipe is also
 * async-signal-safe, so signal handlers may signal an Event.
 *
 * Thread is a POSIX-only addition to the contract: a host library that must
 * run protocol work while the program's own threads do not (rmw_rosette's
 * participant pump) starts one. Boards run that work from their executor.
 */

#pragma once

#include <cerrno>
#include <cstdint>

#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

namespace rosette::platform {

/**
 * @brief Non-recursive mutex over pthread_mutex_t.
 */
class Mutex {
public:
    Mutex() noexcept = default;
    ~Mutex() { pthread_mutex_destroy(&m_); }
    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;

    /// @brief Blocks until the mutex is held by the caller.
    void lock() noexcept { pthread_mutex_lock(&m_); }
    /// @brief Releases a mutex the caller holds.
    void unlock() noexcept { pthread_mutex_unlock(&m_); }
    /// @brief Takes the mutex if free; returns whether it was taken.
    bool try_lock() noexcept { return pthread_mutex_trylock(&m_) == 0; }

private:
    pthread_mutex_t m_ = PTHREAD_MUTEX_INITIALIZER;
};

/**
 * @brief Auto-reset event over a non-blocking self-pipe.
 */
class Event {
public:
    Event() noexcept {
        if (pipe(fds_) == 0) {
            for (int fd : fds_) {
                fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
                fcntl(fd, F_SETFD, FD_CLOEXEC);
            }
        } else {
            fds_[0] = fds_[1] = -1;
        }
    }
    ~Event() {
        if (fds_[0] >= 0) close(fds_[0]);
        if (fds_[1] >= 0) close(fds_[1]);
    }
    Event(const Event&) = delete;
    Event& operator=(const Event&) = delete;

    /**
     * @brief Wakes one current or future wait; safe from any thread and from
     * signal handlers. Signals that arrive before a wait are not lost.
     */
    void signal() noexcept {
        const uint8_t one = 1u;
        // A full pipe already guarantees the next wait returns; EAGAIN is fine.
        (void)!write(fds_[1], &one, 1);
    }

    /// @brief Same as signal(); POSIX has no interrupt context.
    void signal_from_isr() noexcept { signal(); }

    /**
     * @brief Waits for a signal, consuming it.
     *
     * @param[in] timeout_ms Longest wait; 0 polls, kWaitForever blocks.
     * @return `true` when signaled, `false` on timeout.
     */
    bool wait_for(uint32_t timeout_ms) noexcept {
        pollfd p{fds_[0], POLLIN, 0};
        const int t = (timeout_ms == UINT32_MAX) ? -1 : static_cast<int>(timeout_ms);
        int rc;
        do {
            rc = poll(&p, 1, t);
        } while (rc < 0 && errno == EINTR);
        return rc > 0 && consume();
    }

    /**
     * @brief Clears a pending signal without waiting.
     *
     * @return `true` when a signal was pending.
     */
    bool consume() noexcept {
        uint8_t buf[64];
        bool any = false;
        while (read(fds_[0], buf, sizeof(buf)) > 0) any = true;
        return any;
    }

    /// @brief Readable file descriptor for poll()-based waits (POSIX only).
    int native_handle() const noexcept { return fds_[0]; }

private:
    int fds_[2] = {-1, -1};
};

/**
 * @brief A joinable thread that never takes an asynchronous signal.
 *
 * A library thread must not take the signals the program installs handlers
 * for: rclcpp and rclpy handle SIGINT on the program's threads, and a
 * handler running on a library thread that holds a library lock can
 * deadlock. start() therefore blocks every signal around pthread_create(),
 * so the new thread inherits a fully blocked mask, and restores the
 * caller's mask before it returns.
 *
 * Verification:
 * - Correct means: `entry(argument)` runs once on a new thread whose signal
 *   mask blocks every signal; join() returns after `entry` returns.
 * - Covered by: rmw_rosette/test/test_participant_pump.cpp (TC-RMW-PUMP-001),
 *   which runs the participant pump on one.
 *
 * References:
 * - POSIX.1-2017 pthread_create(), pthread_sigmask(), pthread_join()
 */
class Thread {
public:
    Thread() noexcept = default;
    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;

    /**
     * @brief Runs `entry(argument)` on a new thread.
     *
     * @param[in] entry    The thread's body; it returns when the thread is to exit.
     * @param[in] argument Passed to `entry`; must stay valid until join() returns.
     * @param[in] name     Debugger and `top` name, at most 15 characters; static storage.
     * @return true when the thread runs; false when the OS refused to create it.
     */
    bool start(void (*entry)(void*), void* argument, const char* name) noexcept {
        entry_ = entry;
        argument_ = argument;
        sigset_t all;
        sigset_t previous;
        sigfillset(&all);
        pthread_sigmask(SIG_SETMASK, &all, &previous);
        const int created = pthread_create(&thread_, nullptr, &Thread::trampoline, this);
        pthread_sigmask(SIG_SETMASK, &previous, nullptr);
        if (created != 0) {
            return false;
        }

#if defined(__linux__)
        (void)pthread_setname_np(thread_, name);
#else
        (void)name;
#endif
        return true;
    }

    /// @brief Waits for a started thread's `entry` to return.
    void join() noexcept { (void)pthread_join(thread_, nullptr); }

private:
    static void* trampoline(void* self) noexcept {
        auto& thread = *static_cast<Thread*>(self);
        thread.entry_(thread.argument_);
        return nullptr;
    }

    pthread_t thread_{};
    void (*entry_)(void*) = nullptr;
    void* argument_ = nullptr;
};

/**
 * @brief Suspends the calling thread for at least `ms` milliseconds.
 */
inline void sleep_ms(uint32_t ms) noexcept {
    timespec ts{static_cast<time_t>(ms / 1000u), static_cast<long>((ms % 1000u) * 1000000L)};
    while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {
    }
}

}  // namespace rosette::platform
