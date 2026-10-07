#pragma once

/**
 * @file support/fnv1a.hpp
 * @brief 32- and 64-bit FNV-1a, small deterministic hashes for short identifying inputs.
 *
 * Rosette uses FNV-1a where it needs a well-spread, repeatable function of a
 * few bytes and no adversary is choosing the input: the self-assigned address
 * candidates of `net/address_candidate.hpp`, the host chip layer's per-process
 * identity, and the device field of an RTPS GUID prefix. It needs no tables
 * and compiles to a byte loop around one multiply. The 32-bit variant suits a
 * 32-bit field and is a single-instruction multiply on a Cortex-M; the 64-bit
 * variant is for wider outputs.
 *
 * The definitions live in src/support/fnv1a.cpp so that a caller with a
 * fixed-size input cannot have the loop unrolled into one 64-bit multiply
 * (a runtime-library call on the Cortex-M0+) per input byte at each call site.
 *
 * Each step multiplies by an odd prime, which is invertible modulo 2^n, so
 * two inputs of equal length that differ only in their first byte always hash
 * differently. Callers that derive a sequence from a counter place the counter
 * first so the bytes after it spread it through every output bit.
 */

#include <cstddef>
#include <cstdint>

namespace rosette::support {

/// FNV-1a 32-bit offset basis.
inline constexpr uint32_t kFnv1a32OffsetBasis = 0x811C9DC5u;

/// FNV-1a 32-bit prime.
inline constexpr uint32_t kFnv1a32Prime = 0x01000193u;

/// FNV-1a 64-bit offset basis.
inline constexpr uint64_t kFnv1a64OffsetBasis = 0xCBF29CE484222325ull;

/// FNV-1a 64-bit prime.
inline constexpr uint64_t kFnv1a64Prime = 0x00000100000001B3ull;

/**
 * @brief Computes the 32-bit FNV-1a hash of a byte string.
 *
 * For each octet: XOR it into the hash, then multiply the hash by the FNV
 * prime modulo 2^32. Passing a previous result as @p basis continues a hash
 * across several buffers.
 *
 * @param[in] data   Message octets; may be null when @p length is 0.
 * @param[in] length Message length in octets.
 * @param[in] basis  Starting value; the FNV offset basis for a fresh hash.
 * @return The 32-bit hash.
 *
 * Verification:
 * - Correct means: the published FNV-1a 32-bit test vectors hold: "" gives
 *   0x811c9dc5, "a" gives 0xe40c292c, and "foobar" gives 0xbf9cf968.
 * - Covered by: tests/host/address_candidate_test.cpp (TC-NET-FNV-002).
 *
 * References:
 * - G. Fowler, L. C. Noll, K.-P. Vo, D. Eastlake, "The FNV Non-Cryptographic
 *   Hash Algorithm", draft-eastlake-fnv, §2 (FNV-1a) and §2.1 (32-bit parameters).
 */
uint32_t fnv1a32(const uint8_t* data, std::size_t length,
                 uint32_t basis = kFnv1a32OffsetBasis) noexcept;

/**
 * @brief Computes the 64-bit FNV-1a hash of a byte string.
 *
 * For each octet: XOR it into the hash, then multiply the hash by the FNV
 * prime modulo 2^64. Passing a previous result as @p basis continues a hash
 * across several buffers.
 *
 * @param[in] data   Message octets; may be null when @p length is 0.
 * @param[in] length Message length in octets.
 * @param[in] basis  Starting value; the FNV offset basis for a fresh hash.
 * @return The 64-bit hash.
 *
 * Verification:
 * - Correct means: the published FNV-1a 64-bit test vectors hold: "" gives
 *   0xcbf29ce484222325, "a" gives 0xaf63dc4c8601ec8c, and "foobar" gives
 *   0x85944171f73967e8.
 * - Covered by: tests/host/address_candidate_test.cpp (TC-NET-FNV-001).
 *
 * References:
 * - G. Fowler, L. C. Noll, K.-P. Vo, D. Eastlake, "The FNV Non-Cryptographic
 *   Hash Algorithm", draft-eastlake-fnv, §2 (FNV-1a) and §2.1 (64-bit parameters).
 */
uint64_t fnv1a64(const uint8_t* data, std::size_t length,
                 uint64_t basis = kFnv1a64OffsetBasis) noexcept;

}  // namespace rosette::support
