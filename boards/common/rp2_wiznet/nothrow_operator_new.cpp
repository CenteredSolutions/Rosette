// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file nothrow_operator_new.cpp
 * @brief `operator new(std::nothrow)` for firmware built without exceptions.
 *
 * The socket providers allocate with `new (std::nothrow)` and check for null.
 * libstdc++'s version of that operator calls the throwing `operator new`
 * inside a try block, so linking it brings in the personality routine, the
 * catch machinery and libgcc's unwinder (about 4 KB, part of it in RAM on
 * the RP2 family) for an exception no firmware here can throw. The Pico SDK
 * replaces the throwing operators with `malloc` (pico_cxx_options) and leaves
 * this one to libstdc++; this file completes the set.
 *
 * References:
 * - ISO C++ [new.delete.single]: a program may replace the nothrow form.
 */

#include <cstdlib>
#include <new>

void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
    return std::malloc(size);
}
