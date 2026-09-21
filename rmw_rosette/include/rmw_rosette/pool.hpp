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
 * the pattern rtps::RTPSParticipant itself already uses for its writer/
 * reader storage tables (see participant.hpp's writer_storage_/
 * writer_alive_).
 */

#pragma once

#include <cstddef>

namespace rmw_rosette {

template <typename T, std::size_t N>
class Pool {
public:
    /// Returns a pointer to a freshly value-initialized T, or nullptr if
    /// every slot is in use.
    T* acquire() noexcept {
        for (std::size_t i = 0; i < N; ++i) {
            if (!used_[i]) {
                used_[i] = true;
                storage_[i] = T{};
                return &storage_[i];
            }
        }
        return nullptr;
    }

    /// Returns `obj` to the pool. `obj` must be a pointer this pool
    /// returned from acquire() and not already released.
    void release(T* obj) noexcept {
        for (std::size_t i = 0; i < N; ++i) {
            if (&storage_[i] == obj) {
                storage_[i] = T{};
                used_[i] = false;
                return;
            }
        }
    }

    static constexpr std::size_t capacity() noexcept { return N; }

private:
    T storage_[N] = {};
    bool used_[N] = {};
};

}  // namespace rmw_rosette
