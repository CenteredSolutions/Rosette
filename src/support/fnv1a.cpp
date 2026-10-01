/**
 * @file src/support/fnv1a.cpp
 * @brief 64-bit FNV-1a; see rosette/support/fnv1a.hpp.
 */

#include "rosette/support/fnv1a.hpp"

namespace rosette::support {

uint64_t fnv1a64(const uint8_t* data, std::size_t length, uint64_t basis) noexcept {
    uint64_t h = basis;
    for (std::size_t i = 0u; i < length; ++i) {
        h ^= data[i];
        h *= kFnv1a64Prime;
    }

    return h;
}

}  // namespace rosette::support
