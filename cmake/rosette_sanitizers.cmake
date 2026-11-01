# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# Sanitizer builds of the host test suites.
#
#   cmake -S tests/spec -B build/tests-spec-san -DROSETTE_SANITIZE=ON
#
# ROSETTE_SANITIZE builds every target created after the include with
# AddressSanitizer and UndefinedBehaviorSanitizer. The checks run on the host,
# so they cover the portable logic (parsers, pools, state machines) that the
# firmware shares; they do not model the target CPU, but -fsanitize=alignment
# does flag the unaligned accesses a Cortex-M0+ would fault on.
#
# Undefined behavior is fatal (-fno-sanitize-recover) so a finding fails the
# test that hit it. ROSETTE_SANITIZE_THREAD selects ThreadSanitizer instead,
# which cannot be combined with AddressSanitizer.
#
# Include this file after project() and before any target is defined.

option(ROSETTE_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(ROSETTE_SANITIZE_THREAD "Build with ThreadSanitizer instead of ASan/UBSan" OFF)

if (ROSETTE_SANITIZE AND ROSETTE_SANITIZE_THREAD)
    message(FATAL_ERROR "ROSETTE_SANITIZE and ROSETTE_SANITIZE_THREAD are mutually exclusive")
endif()

if (ROSETTE_SANITIZE OR ROSETTE_SANITIZE_THREAD)
    if (NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(FATAL_ERROR "Sanitizers need GCC or Clang, not ${CMAKE_CXX_COMPILER_ID}")
    endif()

    if (ROSETTE_SANITIZE_THREAD)
        set(_rosette_sanitizers thread)
    else()
        set(_rosette_sanitizers address,undefined)
        add_compile_options(-fno-sanitize-recover=undefined)
    endif()

    add_compile_options(-fsanitize=${_rosette_sanitizers} -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=${_rosette_sanitizers})

    # Leak reports stay on: the host tests own everything they allocate.
    message(STATUS "Sanitizers enabled: ${_rosette_sanitizers}")
endif()
