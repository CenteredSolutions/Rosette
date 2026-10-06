/**
 * @file spdp_test_support.hpp
 * @brief SPDP announcements for the SPDP reader's spec tests.
 */

#pragma once

#include "spec_test_common.hpp"

#include "rosette/rtps/spdp.hpp"

#include <string.h>
#include <initializer_list>
#include <vector>

namespace spec_test {
namespace spdp_support {

/// An SPDP announcement listing `metas` as metatraffic unicast and `users` as
/// default unicast locators, in that order.
inline bool build_announcement(const GuidPrefix& gp, std::initializer_list<Locator> metas,
                               std::initializer_list<Locator> users, std::vector<Octet>& out) {
    Octet buf[768] = {};
    uint32_t off = 4u;
    const uint16_t rep[2] = {0x0003u, 0x0000u};
    memcpy(buf, rep, 4u);
    Octet guid[16] = {};
    memcpy(guid, gp.value, 12u);
    guid[15] = 0xC1u;
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::PARTICIPANT_GUID, guid, 16u)) {
        return false;
    }

    for (const Locator& l : metas) {
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::METATRAFFIC_UNICAST_LOCATOR, &l, 24u)) {
            return false;
        }
    }

    for (const Locator& l : users) {
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::DEFAULT_UNICAST_LOCATOR, &l, 24u)) {
            return false;
        }
    }

    if (!plcdr_write_sentinel(buf, sizeof(buf), off)) {
        return false;
    }

    out.assign(buf, buf + off);
    return true;
}

}  // namespace spdp_support
}  // namespace spec_test
