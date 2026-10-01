#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "rosette/net/socket.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "FreeRTOS_Routing.h"
}

namespace rosette::net::freertos {

/**
 * FreeRTOS + TCP does not include support for UDP multicasting out of the box.
 * This class implements IGMPv2 and MLDv1 to provide that support.
 * 
 * It is designed to be used in conjunction with the FreeRtosSocketProvider and
 * should work with any Socket implementation that uses that provider. It is not
 * designed to be used directly by application code, but rather as a helper for
 * the Socket implementation to manage multicast group memberships and send the
 * appropriate IGMP/MLD messages when joining/leaving groups and when processing
 * incoming multicast packets.
 */
class MulticastService {
public:
    using OutputFunction = bool (*)(NetworkInterface_t* interface,
                                    const uint8_t* frame,
                                    std::size_t length,
                                    void* context) noexcept;

    void configure_output(OutputFunction output, void* context) noexcept;
    int join_group(const SocketMulticastMembership& membership) noexcept;
    int leave_group(const SocketMulticastMembership& membership,
                    bool send_leave = true) noexcept;
    void process_frame(const uint8_t* frame,
                       std::size_t length,
                       NetworkInterface_t* interface) noexcept;
    void periodic() noexcept;

private:
    static constexpr std::size_t kMaxMemberships = 16u;

    struct MembershipEntry {
        bool used = false;
        SocketMulticastMembership membership{};
        NetworkEndPoint_t* endpoint = nullptr;
        UBaseType_t reference_count = 0u;
        TickType_t next_unsolicited_tick = 0u;
        TickType_t next_query_tick = 0u;
        TickType_t next_refresh_tick = 0u;
        uint8_t unsolicited_reports_remaining = 0u;
    };

    static uint32_t membership_group_address(const SocketMulticastMembership& membership) noexcept;
    static uint32_t membership_interface_address(const SocketMulticastMembership& membership) noexcept;
    static bool is_unspecified_ipv6(const std::array<uint8_t, 16>& address) noexcept;
    static bool tick_due(TickType_t now, TickType_t due) noexcept;
    static bool tick_before(TickType_t lhs, TickType_t rhs) noexcept;
    static bool is_ipv4_multicast(uint32_t address) noexcept;
    static bool is_ipv6_multicast(const std::array<uint8_t, 16>& address) noexcept;
    static TickType_t response_delay_ticks(uint8_t max_response_time,
                                           uint32_t group_address) noexcept;
    static TickType_t response_delay_ticks_ipv6(uint16_t max_response_time_ms,
                                                const std::array<uint8_t, 16>& group_address) noexcept;

    NetworkEndPoint_t* find_ipv4_endpoint(uint32_t interface_address) noexcept;
    NetworkEndPoint_t* find_ipv6_endpoint(const SocketMulticastMembership& membership) noexcept;
    MembershipEntry* find_membership(const SocketMulticastMembership& membership) noexcept;
    MembershipEntry* find_free_entry() noexcept;
    bool send_igmp_message(const MembershipEntry& entry,
                           uint8_t type,
                           uint32_t destination_address) noexcept;
    bool send_mld_message(const MembershipEntry& entry,
                          uint8_t type,
                          const std::array<uint8_t, 16>& destination_address) noexcept;
    void add_allowed_mac(const MembershipEntry& entry) noexcept;
    void remove_allowed_mac(const MembershipEntry& entry) noexcept;

    std::array<MembershipEntry, kMaxMemberships> memberships_{};
    OutputFunction output_ = nullptr;
    void* output_context_ = nullptr;
};

MulticastService& freertos_multicast_service() noexcept;

}  // namespace rosette::net
