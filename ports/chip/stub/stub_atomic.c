/**
 * @file ports/chip/stub/stub_atomic.c
 * @brief The atomic helpers a core without atomic instructions calls, for one thread.
 *
 * The portable core uses std::atomic read-modify-write operations (the RTPS
 * buffer pool's reference counts). A core with exclusive-access instructions
 * (Cortex-M3 and up) compiles them inline; on a Cortex-M0+ (ARMv6-M), which
 * has none, GCC emits calls to `__atomic_fetch_add_4()` and its relatives, and
 * the chip port must define them. The Pico SDK's pico_atomic does so for the
 * RP2040 under a hardware spin lock. A real port makes each one atomic by
 * masking interrupts, or with a spin lock across cores; the stub runs one
 * thread with no interrupts, so a plain read-modify-write is enough.
 *
 * Verification:
 * - Correct means: tests/port_stub links for Cortex-M0+ with no undefined
 *   `__atomic_*` symbol.
 *
 * References:
 * - GCC libatomic ABI: https://gcc.gnu.org/wiki/Atomic/GCCMM/LIbrary
 * - Pico SDK src/rp2_common/pico_atomic/atomic.c
 */

#include <stdbool.h>
#include <stdint.h>

#define STUB_ATOMIC_RMW(n, type, name, op)                                      \
    type __atomic_fetch_##name##_##n(volatile void* ptr, type val, int model) { \
        (void)model;                                                            \
        volatile type* p = (volatile type*)ptr;                                 \
        const type old = *p;                                                    \
        *p = (type)(old op val);                                                \
        return old;                                                             \
    }

#define STUB_ATOMIC_SIZE(n, type)                                                        \
    STUB_ATOMIC_RMW(n, type, add, +)                                                     \
    STUB_ATOMIC_RMW(n, type, sub, -)                                                     \
    STUB_ATOMIC_RMW(n, type, and, &)                                                     \
    STUB_ATOMIC_RMW(n, type, or, |)                                                      \
    STUB_ATOMIC_RMW(n, type, xor, ^)                                                     \
    type __atomic_exchange_##n(volatile void* ptr, type val, int model) {                \
        (void)model;                                                                     \
        volatile type* p = (volatile type*)ptr;                                          \
        const type old = *p;                                                             \
        *p = val;                                                                        \
        return old;                                                                      \
    }                                                                                    \
    bool __atomic_compare_exchange_##n(volatile void* ptr, void* expected, type desired, \
                                       bool weak, int success, int failure) {            \
        (void)weak;                                                                      \
        (void)success;                                                                   \
        (void)failure;                                                                   \
        volatile type* p = (volatile type*)ptr;                                          \
        type* e = (type*)expected;                                                       \
        if (*p == *e) {                                                                  \
            *p = desired;                                                                \
            return true;                                                                 \
        }                                                                                \
        *e = *p;                                                                         \
        return false;                                                                    \
    }

/* The compiler declares the 4-octet built-ins with `unsigned int`; uint32_t is
 * `unsigned long` on arm-none-eabi, which would redeclare them with another
 * type. */
_Static_assert(sizeof(unsigned int) == 4, "the 4-octet built-ins take a 32-bit unsigned int");

STUB_ATOMIC_SIZE(1, uint8_t)
STUB_ATOMIC_SIZE(2, uint16_t)
STUB_ATOMIC_SIZE(4, unsigned int)
