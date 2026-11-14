// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file nothrow_operator_new.cpp
 * @brief `operator new(std::nothrow)` for RP2 firmware built without exceptions.
 *
 * The socket providers allocate with `new (std::nothrow)` and check for null.
 * libstdc++'s version of that operator calls the throwing `operator new`
 * inside a try block, so linking it brings in the personality routine, the
 * catch machinery and libgcc's unwinder (about 4 KB, part of it in RAM on
 * the RP2 family) for an exception no firmware here can throw. The Pico SDK
 * replaces the throwing operators with `malloc` (pico_cxx_options/new_delete.cpp)
 * and leaves this one to libstdc++; this file completes the set.
 *
 * port.cmake adds this file to pico_cxx_options beside new_delete.cpp, so
 * every executable compiles it as one of its own objects. An object of the
 * executable always takes precedence over libstdc++'s archive member, whatever
 * order the static libraries that call it are linked in.
 */

#if !PICO_CXX_ENABLE_EXCEPTIONS && !PICO_CXX_DISABLE_ALLOCATION_OVERRIDES

#  include <cstdlib>
#  include <new>

/**
 * @brief Allocates `size` octets from the C heap, or returns null.
 *
 * Replaces the standard library's nothrow single-object allocation, which
 * the firmware's socket providers use to report an exhausted heap as a null
 * socket instead of an exception.
 *
 * It is compiled under exactly the conditions under which the Pico SDK's
 * new_delete.cpp replaces the throwing `operator new` and `operator delete`
 * with `malloc` and `free`, so memory from this operator is always released
 * by the matching `free`. A build that enables C++ exceptions or supplies its
 * own allocator (PICO_CXX_DISABLE_ALLOCATION_OVERRIDES) keeps libstdc++'s
 * form, which defers to whatever throwing operator that build provides.
 *
 * @param[in] size Octets requested; 0 returns a unique pointer or null, as
 *   `malloc(0)` does.
 * @return The allocation, aligned for any fundamental type, or null when the
 *   heap cannot satisfy it. The caller releases it with `delete`.
 *
 * Verification:
 * - Correct means: a firmware image that allocates with `new (std::nothrow)`
 *   links neither `__gxx_personality_v0` nor `_Unwind_RaiseException`.
 * - Covered by: `arm-none-eabi-nm rclc_chatter.elf` on a w5100s-evb-pico2
 *   build lists neither symbol (wiznet and freertos_tcp network stacks).
 *
 * References:
 * - ISO/IEC 14882:2017 [new.delete.single] ¶5–7: a program may replace the
 *   nothrow form; the default one calls the replaceable `operator new(size)`.
 */
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
    return std::malloc(size);
}

#endif
