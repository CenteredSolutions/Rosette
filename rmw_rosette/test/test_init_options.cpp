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
 */

#include <gtest/gtest.h>

#include <rcutils/allocator.h>
#include <rcutils/strdup.h>
#include <rmw/error_handling.h>
#include <rmw/init_options.h>
#include <rmw/rmw.h>

#include <cstdlib>
#include <cstring>

namespace {

/// Live allocations made through `counting_allocator()`.
int g_live = 0;

void* count_allocate(size_t size, void*) {
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
        ++g_live;
    }

    return std::realloc(pointer, size);
}

void* count_zero_allocate(size_t count, size_t size, void*) {
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
    void SetUp() override { g_live = 0; }

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

// TC-RMW-INITOPT-005: finalizing zero-initialized or already-finalized options is harmless.
TEST_F(InitOptionsTest, FiniOfEmptyOptionsIsANoOp) {
    rmw_init_options_t options = rmw_get_zero_initialized_init_options();
    EXPECT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
    rmw_init_options_t used = options_with_strings();
    EXPECT_EQ(rmw_init_options_fini(&used), RMW_RET_OK);
    EXPECT_EQ(rmw_init_options_fini(&used), RMW_RET_OK);
    EXPECT_EQ(g_live, 0);
}

}  // namespace
