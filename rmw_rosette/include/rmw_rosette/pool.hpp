// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_rosette/pool.hpp
 * @brief Fixed-capacity, no-heap object pool for rmw entity handles.
 *
 * rmw's convention (see rmw_create_node/rmw_create_publisher/...) is to
 * allocate handles as needed and free them on rmw_destroy_*(). On a host
 * that's normally just `new`/`delete`. On an MCU with a bounded, small
 * number of nodes/publishers/subscriptions known at firmware-image build
 * time, heap allocation buys nothing but fragmentation risk — so every
 * rmw_rosette entity lives in one of these instead: a fixed array plus a
 * parallel "in use" flag, linear-scanned on acquire/release. This mirrors
 * the pattern rosette::rtps::RTPSParticipant itself already uses for its writer/
 * reader storage tables (see participant.hpp's writer_storage_/
 * writer_alive_).
 */

#pragma once

#include <cstddef>
#include <new>

#include "rosette/platform/platform.hpp"

namespace rmw_rosette {

template <typename T, std::size_t N>
class Pool {
public:
    /**
     * @brief Claims and value-initializes one slot from the pool's fixed
     * storage array.
     *
     * `rmw_create_*()` entry points across rmw_rosette call this in place
     * of `new` to hand back the backing storage for one entity handle.
     *
     * Scans `used_` linearly for the first free slot, marks it used, and
     * resets its storage to `T{}` before returning a pointer to it. Slot
     * order is not preserved across acquire/release cycles.
     *
     * @return Pointer to a freshly value-initialized `T`, or `nullptr`
     *   when all `N` slots are in use.
     */
    T* acquire() noexcept {
        rosette::platform::LockGuard guard(mutex_);
        for (std::size_t i = 0; i < N; ++i) {
            if (!used_[i]) {
                used_[i] = true;
                reset_in_place(storage_[i]);
                return &storage_[i];
            }
        }
        return nullptr;
    }

    /**
     * @brief Returns a previously acquired slot to the pool.
     *
     * Called from the matching `rmw_destroy_*()` entry point once an
     * entity's resources (writers, readers, etc.) have already been torn
     * down.
     *
     * Scans `storage_` linearly for the slot at `obj`'s address, clears
     * it back to `T{}`, and marks it free.
     *
     * @param[in] obj Pointer previously returned by `acquire()` on this
     *   same pool, not already released.
     */
    void release(T* obj) noexcept {
        rosette::platform::LockGuard guard(mutex_);
        for (std::size_t i = 0; i < N; ++i) {
            if (&storage_[i] == obj) {
                reset_in_place(storage_[i]);
                used_[i] = false;
                return;
            }
        }
    }

    /// @return The fixed slot count `N` this pool was instantiated with.
    static constexpr std::size_t capacity() noexcept { return N; }

private:
    /// Returns a slot to its value-initialized state without building a
    /// temporary: some entities (the context, with its participant and
    /// receive buffer) are far larger than a stack frame should hold.
    static void reset_in_place(T& slot) noexcept {
        slot.~T();
        new (&slot) T{};
    }

    T storage_[N] = {};
    bool used_[N] = {};
    /// Serializes acquire()/release() across threads and cores; entity
    /// creation and destruction may happen on any thread, even after the
    /// owning context's participant is gone.
    rosette::platform::Mutex mutex_;
};

}  // namespace rmw_rosette
