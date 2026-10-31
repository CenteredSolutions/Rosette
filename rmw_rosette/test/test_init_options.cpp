/**
 * @file test_init_options.cpp
 * @brief Unit tests for who owns an rmw_init_options_t's strings.
 *
 * rcl_init() puts an rcutils_strdup()'d enclave ("/" by default) into the
 * context's copy of the init options, and rcl_context_fini() finalizes that
 * copy through rmw_init_options_fini(). An rmw that copies the options
 * shallowly and finalizes them by zeroing leaks the enclave once per
 * rcl_init(): 16 B of heap per session on the RP2040, where a board rebuilds
 * its session after every link loss. These tests count every allocation
 * through the options' allocator and require copy + fini to return them all.
 *
 * Where the headers have discovery options, the static peer list is a third
 * owned allocation, and rmw_init() checks the requested discovery range
 * before it creates anything.
 */

#include "rmw_rosette/ros_features.h"

#include <gtest/gtest.h>

#include <rcutils/allocator.h>
#include <rcutils/strdup.h>
#include <rmw/error_handling.h>
#include <rmw/init.h>
#include <rmw/init_options.h>
#include <rmw/rmw.h>
#if RMW_ROSETTE_HAS(DISCOVERY_OPTIONS)
#  include <rmw/discovery_options.h>
#endif

#include <cstdlib>
#include <cstring>
#include <string>

namespace {

/// Live allocations made through `counting_allocator()`.
int g_live = 0;

/// Allocations attempted so far, and the 0-based attempt that fails (-1: none fail).
int g_attempts = 0;
int g_fail_at = -1;

/// Counts an allocation attempt and reports whether it is the one to fail.
bool should_fail() {
    return g_attempts++ == g_fail_at;
}

void* count_allocate(size_t size, void*) {
    if (should_fail()) {
        return nullptr;
    }

    ++g_live;
    return std::malloc(size);
}

void count_deallocate(void* pointer, void*) {
    if (pointer != nullptr) {
        --g_live;
    }

    std::free(pointer);
}

void* count_reallocate(void* pointer, size_t size, void*) {
    if (pointer == nullptr) {
        if (should_fail()) {
            return nullptr;
        }

        ++g_live;
    }

    return std::realloc(pointer, size);
}

void* count_zero_allocate(size_t count, size_t size, void*) {
    if (should_fail()) {
        return nullptr;
    }

    ++g_live;
    return std::calloc(count, size);
}

rcutils_allocator_t counting_allocator() {
    rcutils_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = count_allocate;
    allocator.deallocate = count_deallocate;
    allocator.reallocate = count_reallocate;
    allocator.zero_allocate = count_zero_allocate;
    return allocator;
}

/// Options as rcl_init() leaves the context's copy: an enclave, and here a security root too.
rmw_init_options_t options_with_strings() {
    rmw_init_options_t options = rmw_get_zero_initialized_init_options();
    EXPECT_EQ(rmw_init_options_init(&options, counting_allocator()), RMW_RET_OK);
    options.enclave = rcutils_strdup("/", options.allocator);
    options.security_options.security_root_path = rcutils_strdup("/keys", options.allocator);
    return options;
}

class InitOptionsTest : public ::testing::Test {
protected:
    void SetUp() override {
        g_live = 0;
        g_attempts = 0;
        g_fail_at = -1;
    }

    void TearDown() override { rmw_reset_error(); }
};

// TC-RMW-INITOPT-001: fini frees the enclave and the security root rcl put there.
TEST_F(InitOptionsTest, FiniFreesTheStringsTheOptionsOwn) {
    rmw_init_options_t options = options_with_strings();
    ASSERT_EQ(g_live, 2);

    EXPECT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
    EXPECT_EQ(g_live, 0);
    EXPECT_EQ(options.enclave, nullptr);
    EXPECT_EQ(options.implementation_identifier, nullptr);
}

// TC-RMW-INITOPT-002: a copy owns its own strings; finalizing both frees everything once.
TEST_F(InitOptionsTest, CopyDuplicatesTheStrings) {
    rmw_init_options_t source = options_with_strings();
    source.domain_id = 42;
    rmw_init_options_t copy = rmw_get_zero_initialized_init_options();
    ASSERT_EQ(rmw_init_options_copy(&source, &copy), RMW_RET_OK);
    EXPECT_EQ(g_live, 4);
    EXPECT_EQ(copy.domain_id, 42u);
    EXPECT_STREQ(copy.enclave, "/");
    EXPECT_NE(copy.enclave, source.enclave);
    EXPECT_STREQ(copy.security_options.security_root_path, "/keys");
    EXPECT_NE(copy.security_options.security_root_path, source.security_options.security_root_path);

    EXPECT_EQ(rmw_init_options_fini(&source), RMW_RET_OK);
    EXPECT_STREQ(copy.enclave, "/");  // still valid: not the source's string
    EXPECT_EQ(rmw_init_options_fini(&copy), RMW_RET_OK);
    EXPECT_EQ(g_live, 0);
}

// TC-RMW-INITOPT-003: rcl's init -> copy -> fini cycle, many times over, leaks nothing.
TEST_F(InitOptionsTest, RepeatedSessionsReturnEveryAllocation) {
    for (int session = 0; session < 100; ++session) {
        rmw_init_options_t user = rmw_get_zero_initialized_init_options();
        ASSERT_EQ(rmw_init_options_init(&user, counting_allocator()), RMW_RET_OK);
        rmw_init_options_t context_copy = rmw_get_zero_initialized_init_options();
        ASSERT_EQ(rmw_init_options_copy(&user, &context_copy), RMW_RET_OK);
        context_copy.enclave = rcutils_strdup("/", context_copy.allocator);  // as rcl_init()
        ASSERT_EQ(rmw_init_options_fini(&context_copy), RMW_RET_OK);
        ASSERT_EQ(rmw_init_options_fini(&user), RMW_RET_OK);
    }

    EXPECT_EQ(g_live, 0);
}

// TC-RMW-INITOPT-004: copying into initialized options is refused, and nothing is allocated.
TEST_F(InitOptionsTest, CopyRefusesAnInitializedDestination) {
    rmw_init_options_t source = options_with_strings();
    rmw_init_options_t destination = rmw_get_zero_initialized_init_options();
    ASSERT_EQ(rmw_init_options_init(&destination, counting_allocator()), RMW_RET_OK);
    EXPECT_EQ(rmw_init_options_copy(&source, &destination), RMW_RET_INVALID_ARGUMENT);
    EXPECT_EQ(destination.enclave, nullptr);
    EXPECT_EQ(g_live, 2);
    EXPECT_EQ(rmw_init_options_fini(&destination), RMW_RET_OK);
    EXPECT_EQ(rmw_init_options_fini(&source), RMW_RET_OK);
    EXPECT_EQ(g_live, 0);
}

// TC-RMW-INITOPT-005: finalizing zero-initialized or already-finalized options is refused, as
// the stock rmws refuse it, and frees nothing twice.
TEST_F(InitOptionsTest, FiniRefusesOptionsThatAreNotInitialized) {
    rmw_init_options_t options = rmw_get_zero_initialized_init_options();
    EXPECT_EQ(rmw_init_options_fini(&options), RMW_RET_INVALID_ARGUMENT);
    rmw_init_options_t used = options_with_strings();
    EXPECT_EQ(rmw_init_options_fini(&used), RMW_RET_OK);
    EXPECT_EQ(rmw_init_options_fini(&used), RMW_RET_INVALID_ARGUMENT);
    EXPECT_EQ(g_live, 0);
}

// TC-RMW-INITOPT-014: init and copy refuse what the rmw API calls invalid before touching
// anything: an invalid allocator, options initialized twice, an uninitialized source, and a
// source whose identifier names another rmw (compared by content, not by address).
TEST_F(InitOptionsTest, InitAndCopyRefuseInvalidArguments) {
    rmw_init_options_t options = rmw_get_zero_initialized_init_options();
    EXPECT_EQ(rmw_init_options_init(&options, rcutils_get_zero_initialized_allocator()),
              RMW_RET_INVALID_ARGUMENT);
    EXPECT_EQ(options.implementation_identifier, nullptr);
    ASSERT_EQ(rmw_init_options_init(&options, counting_allocator()), RMW_RET_OK);
    EXPECT_EQ(rmw_init_options_init(&options, counting_allocator()), RMW_RET_INVALID_ARGUMENT);

    rmw_init_options_t empty = rmw_get_zero_initialized_init_options();
    rmw_init_options_t destination = rmw_get_zero_initialized_init_options();
    EXPECT_EQ(rmw_init_options_copy(&empty, &destination), RMW_RET_INVALID_ARGUMENT);

    const std::string same_name = options.implementation_identifier;
    const char* const own_identifier = options.implementation_identifier;
    options.implementation_identifier = same_name.c_str();
    EXPECT_EQ(rmw_init_options_copy(&options, &destination), RMW_RET_OK);
    EXPECT_EQ(rmw_init_options_fini(&destination), RMW_RET_OK);
    options.implementation_identifier = "rmw_other_implementation";
    EXPECT_EQ(rmw_init_options_copy(&options, &destination), RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    EXPECT_EQ(destination.implementation_identifier, nullptr);
    rmw_reset_error();

    options.implementation_identifier = own_identifier;
    EXPECT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
    EXPECT_EQ(g_live, 0);
}

// TC-RMW-INITOPT-006: options stamped by another rmw are rejected and left exactly as found.
TEST_F(InitOptionsTest, FiniRefusesOptionsOfAnotherRmw) {
    rmw_init_options_t options = options_with_strings();
    const char* const  own_identifier = options.implementation_identifier;
    options.implementation_identifier = "rmw_other_implementation";
    char* const enclave = options.enclave;

    EXPECT_EQ(rmw_init_options_fini(&options), RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    EXPECT_EQ(g_live, 2);
    EXPECT_EQ(options.enclave, enclave);
    EXPECT_STREQ(options.implementation_identifier, "rmw_other_implementation");
    rmw_reset_error();

    options.implementation_identifier = own_identifier;
    EXPECT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
    EXPECT_EQ(g_live, 0);
}

// TC-RMW-INITOPT-007: a failed enclave duplication returns BAD_ALLOC and leaves dst untouched.
TEST_F(InitOptionsTest, CopyReportsAnEnclaveAllocationFailure) {
    rmw_init_options_t source = options_with_strings();
    rmw_init_options_t destination = rmw_get_zero_initialized_init_options();

    g_attempts = 0;
    g_fail_at = 0;
    EXPECT_EQ(rmw_init_options_copy(&source, &destination), RMW_RET_BAD_ALLOC);
    EXPECT_EQ(g_live, 2);
    EXPECT_EQ(destination.implementation_identifier, nullptr);
    EXPECT_EQ(destination.enclave, nullptr);
    EXPECT_EQ(destination.security_options.security_root_path, nullptr);
    rmw_reset_error();

    g_fail_at = -1;
    EXPECT_EQ(rmw_init_options_fini(&source), RMW_RET_OK);
    EXPECT_EQ(g_live, 0);
}

// TC-RMW-INITOPT-008: when the security root fails after the enclave was copied, the enclave is
// released.
TEST_F(InitOptionsTest, CopyReleasesTheEnclaveWhenTheSecurityRootFails) {
    rmw_init_options_t source = options_with_strings();
    rmw_init_options_t destination = rmw_get_zero_initialized_init_options();

    g_attempts = 0;
    g_fail_at = 1;
    EXPECT_EQ(rmw_init_options_copy(&source, &destination), RMW_RET_BAD_ALLOC);
    EXPECT_GE(g_attempts, 2);
    EXPECT_EQ(g_live, 2);  // only the source's strings remain
    EXPECT_EQ(destination.implementation_identifier, nullptr);
    EXPECT_EQ(destination.enclave, nullptr);
    EXPECT_EQ(destination.security_options.security_root_path, nullptr);
    rmw_reset_error();

    g_fail_at = -1;
    EXPECT_EQ(rmw_init_options_fini(&source), RMW_RET_OK);
    EXPECT_EQ(g_live, 0);
}

// TC-RMW-INITOPT-009: fresh options carry RMW_DEFAULT_DOMAIN_ID, so rcl_init() applies
// ROS_DOMAIN_ID.
TEST_F(InitOptionsTest, InitLeavesTheDomainForRclToResolve) {
    rmw_init_options_t options = rmw_get_zero_initialized_init_options();
    ASSERT_EQ(rmw_init_options_init(&options, counting_allocator()), RMW_RET_OK);
    EXPECT_EQ(options.domain_id, RMW_DEFAULT_DOMAIN_ID);
    EXPECT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
}

// TC-RMW-INITOPT-010: rmw_init() refuses a domain above 232, whose ports would not fit in 16 bits.
TEST_F(InitOptionsTest, RmwInitRefusesADomainAbove232) {
    rmw_init_options_t options = options_with_strings();
    options.domain_id = 233u;
#if RMW_ROSETTE_HAS(DISCOVERY_OPTIONS)
    // A valid range, so the refusal is the domain's.
    options.discovery_options.automatic_discovery_range = RMW_AUTOMATIC_DISCOVERY_RANGE_SUBNET;
#endif
    rmw_context_t context = rmw_get_zero_initialized_context();
    EXPECT_EQ(rmw_init(&options, &context), RMW_RET_INVALID_ARGUMENT);
    rmw_reset_error();
    EXPECT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
}

#if RMW_ROSETTE_HAS(DISCOVERY_OPTIONS)
/// Calls rmw_init() with `options`, and finalizes the context when it succeeds.
rmw_ret_t init_and_fini(const rmw_init_options_t& options) {
    rmw_context_t   context = rmw_get_zero_initialized_context();
    const rmw_ret_t ret = rmw_init(&options, &context);
    if (ret == RMW_RET_OK) {
        EXPECT_EQ(rmw_shutdown(&context), RMW_RET_OK);
        EXPECT_EQ(rmw_context_fini(&context), RMW_RET_OK);
    }

    rmw_reset_error();
    return ret;
}

// TC-RMW-INITOPT-011: rmw_init() accepts the discovery scopes the participant
// provides and refuses the rest before creating anything.
TEST_F(InitOptionsTest, InitChecksTheDiscoveryRange) {
    rmw_init_options_t options = rmw_get_zero_initialized_init_options();
    ASSERT_EQ(rmw_init_options_init(&options, rcutils_get_default_allocator()), RMW_RET_OK);
    options.enclave = rcutils_strdup("/", options.allocator);
    options.domain_id = 77;

    options.discovery_options.automatic_discovery_range = RMW_AUTOMATIC_DISCOVERY_RANGE_SUBNET;
    EXPECT_EQ(init_and_fini(options), RMW_RET_OK);
    options.discovery_options.automatic_discovery_range =
        RMW_AUTOMATIC_DISCOVERY_RANGE_SYSTEM_DEFAULT;
    EXPECT_EQ(init_and_fini(options), RMW_RET_OK);

    options.discovery_options.automatic_discovery_range = RMW_AUTOMATIC_DISCOVERY_RANGE_NOT_SET;
    EXPECT_EQ(init_and_fini(options), RMW_RET_INVALID_ARGUMENT);
    options.discovery_options.automatic_discovery_range = RMW_AUTOMATIC_DISCOVERY_RANGE_LOCALHOST;
    EXPECT_EQ(init_and_fini(options), RMW_RET_UNSUPPORTED);
    options.discovery_options.automatic_discovery_range = RMW_AUTOMATIC_DISCOVERY_RANGE_OFF;
    EXPECT_EQ(init_and_fini(options), RMW_RET_UNSUPPORTED);

    options.discovery_options.automatic_discovery_range = RMW_AUTOMATIC_DISCOVERY_RANGE_SUBNET;
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ASSERT_EQ(rmw_discovery_options_init(&options.discovery_options, 1, &allocator), RMW_RET_OK);
    std::strcpy(options.discovery_options.static_peers[0].peer_address, "192.168.0.10");
    EXPECT_EQ(init_and_fini(options), RMW_RET_UNSUPPORTED);

    EXPECT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
}

// TC-RMW-INITOPT-013: fresh options pass rmw_init() with only the enclave set, as the stock
// rmws' do; the range is this middleware's default, which rcl replaces.
TEST_F(InitOptionsTest, FreshOptionsPassRmwInit) {
    rmw_init_options_t options = rmw_get_zero_initialized_init_options();
    ASSERT_EQ(rmw_init_options_init(&options, rcutils_get_default_allocator()), RMW_RET_OK);
    EXPECT_EQ(options.discovery_options.automatic_discovery_range,
              RMW_AUTOMATIC_DISCOVERY_RANGE_SYSTEM_DEFAULT);
    options.enclave = rcutils_strdup("/", options.allocator);
    options.domain_id = 78;
    EXPECT_EQ(init_and_fini(options), RMW_RET_OK);
    EXPECT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
}

// TC-RMW-INITOPT-012: a copy owns its own static peer list; finalizing both frees everything once.
TEST_F(InitOptionsTest, CopyDuplicatesTheStaticPeers) {
    rmw_init_options_t  source = options_with_strings();
    rcutils_allocator_t allocator = counting_allocator();
    ASSERT_EQ(rmw_discovery_options_init(&source.discovery_options, 2, &allocator), RMW_RET_OK);
    std::strcpy(source.discovery_options.static_peers[1].peer_address, "10.42.0.176");
    const int source_live = g_live;

    rmw_init_options_t copy = rmw_get_zero_initialized_init_options();
    ASSERT_EQ(rmw_init_options_copy(&source, &copy), RMW_RET_OK);
    EXPECT_EQ(g_live, 2 * source_live);
    ASSERT_EQ(copy.discovery_options.static_peers_count, 2u);
    EXPECT_NE(copy.discovery_options.static_peers, source.discovery_options.static_peers);
    EXPECT_STREQ(copy.discovery_options.static_peers[1].peer_address, "10.42.0.176");

    EXPECT_EQ(rmw_init_options_fini(&source), RMW_RET_OK);
    EXPECT_STREQ(copy.discovery_options.static_peers[1].peer_address, "10.42.0.176");
    EXPECT_EQ(rmw_init_options_fini(&copy), RMW_RET_OK);
    EXPECT_EQ(g_live, 0);
}
#endif

}  // namespace
