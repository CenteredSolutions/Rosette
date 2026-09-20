/**
 * @file test_pub_sub_loopback.cpp
 * @brief Two rmw_rosette contexts over loopback, no ROS 2 install required.
 *
 * Intent: a host-only smoke test that exercises rmw_init -> create_node ->
 * create_publisher/subscription -> publish -> wait -> take across two
 * independent rmw_context_t instances talking over 127.0.0.1, using a
 * real rosidl-generated message type (std_msgs/msg/String is the obvious
 * first choice, matching tests/ros2/posix_chatter_smoke.cpp's existing
 * chatter pattern).
 *
 * Currently every rmw_* call this would exercise is a stub — this file
 * exists to pin down the shape of the test once they're implemented, not
 * to pass yet.
 */

#include <gtest/gtest.h>

#include <rmw/rmw.h>

TEST(RmwRosetteLoopback, DISABLED_PublishAndTake) {
    // TODO once rmw_init/create_node/create_publisher/create_subscription/
    // publish/wait/take are implemented:
    //   1. rmw_init() two contexts (or one — a participant can talk to
    //      itself over loopback multicast/unicast, same as two real
    //      processes would).
    //   2. Create a std_msgs/msg/String publisher on one, subscription on
    //      the other, same topic name.
    //   3. Drive SPDP/SEDP to completion (participant->tick() in a loop
    //      with a generous timeout — matching endpoint discovery is not
    //      instantaneous).
    //   4. rmw_publish() a message, rmw_wait() + rmw_take() on the other
    //      side, assert the string round-tripped.
    GTEST_SKIP() << "rmw_rosette entry points are not implemented yet";
}
