/**
 * @file rosette/platform/os_posix.hpp
 * @brief POSIX OS layer of the platform contract (Linux, macOS).
 *
 * Included through platform.hpp when ROSETTE_OS_POSIX is defined; see that
 * header for the contract each type satisfies.
 *
 * Event is a non-blocking self-pipe rather than a condition variable so that
 * its read end can sit in the same poll() as the transport's sockets: one
 * wait returns on whichever comes first, a datagram or a signal
 * (PosixSocketProvider::wait_readable()). Writing to a pipe is also
 * async-signal-safe, so signal handlers may signal an Event.
 */

#pragma once

#include <cerrno>
#include <cstdint>

#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
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
 * @brief Suspends the calling thread for at least `ms` milliseconds.
 */
inline void sleep_ms(uint32_t ms) noexcept {
    timespec ts{static_cast<time_t>(ms / 1000u), static_cast<long>((ms % 1000u) * 1000000L)};
    while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {
    }
}

}  // namespace rosette::platform
