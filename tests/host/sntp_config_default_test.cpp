// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file tests/host/sntp_config_default_test.cpp
 * @brief SntpConfig's defaults, built once plain and once with ROSETTE_ROS_BACKEND_AGENT.
 *
 * - TC-NET-SNTP-CFG-001: no server is configured by default, on either backend.
 * - TC-NET-SNTP-CFG-002: SNTP is enabled by default, except in agent-backend
 *   builds, where the agent is the time source.
 */

#include "rosette/net/sntp_client.hpp"
#include "test_util.hpp"

int main() {
    const rosette::net::SntpConfig config{};
    check(config.server.port == 0u, "TC-NET-SNTP-CFG-001 no NTP server is built in");
#if defined(ROSETTE_ROS_BACKEND_AGENT)
    check(!config.enabled, "TC-NET-SNTP-CFG-002 agent backend: SNTP is off by default");
#else
    check(config.enabled, "TC-NET-SNTP-CFG-002 without the agent backend: SNTP is on by default");
#endif
    return test_summary();
}
