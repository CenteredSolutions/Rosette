// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "freertos_tcp_igmp.hpp"

#include <algorithm>
#include <array>
#include <cstring>

extern "C" {
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_IPv4_Private.h"
#include "FreeRTOS_IPv4_Utils.h"
#include "FreeRTOS_IPv6.h"
#include "FreeRTOS_IPv6_Private.h"
#include "FreeRTOS_IPv6_Utils.h"
#include "task.h"
}

namespace rosette::net::freertos {

namespace {

constexpr uint8_t kIgmpTypeMembershipQuery = 0x11u;
constexpr uint8_t kIgmpTypeMembershipReportV2 = 0x16u;
constexpr uint8_t kIgmpTypeLeaveGroup = 0x17u;

constexpr uint8_t kMldTypeListenerQuery = 130u;
constexpr uint8_t kMldTypeListenerReport = 131u;
constexpr uint8_t kMldTypeListenerDone = 132u;

constexpr uint8_t kIpv6NextHeaderHopByHop = ipIPv6_EXT_HEADER_HOP_BY_HOP;
constexpr uint8_t kIpv6HopLimitLinkLocal = 1u;
constexpr uint8_t kRouterAlertOptionType = 5u;
constexpr uint8_t kRouterAlertOptionLength = 2u;
constexpr uint16_t kRouterAlertValueMld = 0u;
constexpr uint8_t kPadNOptionType = 1u;
constexpr uint8_t kPadNOptionLength = 0u;

constexpr TickType_t kDefaultMaxResponseTicks = pdMS_TO_TICKS(10000u);
constexpr TickType_t kUnsolicitedRepeatTicks = pdMS_TO_TICKS(1000u);
constexpr TickType_t kMembershipRefreshTicks = pdMS_TO_TICKS(60000u);
constexpr std::size_t kMinEthernetFrameLength = 60u;
constexpr uint32_t kAllRoutersGroup = 0xE0000002u;

const std::array<uint8_t, 16> kIpv6AllRoutersLinkLocal{
    0xFFu, 0x02u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x02u,
};

#include "pack_struct_start.h"
struct IgmpHeader {
    uint8_t type;
    uint8_t max_response_time;
    uint16_t checksum;
    uint32_t group_address;
}
#include "pack_struct_end.h"
;

#include "pack_struct_start.h"
struct IPv6HopByHopHeader {
    uint8_t next_header;
    uint8_t hdr_ext_len;
    uint8_t option_type;
    uint8_t option_length;
    uint16_t option_value;
    uint8_t padn_type;
    uint8_t padn_length;
}
#include "pack_struct_end.h"
;

#include "pack_struct_start.h"
struct MldHeader {
    uint8_t type;
    uint8_t code;
    uint16_t checksum;
    uint16_t max_response_delay;
    uint16_t reserved;
    IPv6_Address_t multicast_address;
}
#include "pack_struct_end.h"
;

bool same_membership(const SocketMulticastMembership& lhs,
                     const SocketMulticastMembership& rhs) noexcept
{
    return (lhs.domain == rhs.domain) &&
           (lhs.group == rhs.group) &&
           (lhs.interface == rhs.interface) &&
           (lhs.port == rhs.port) &&
           (lhs.interface_index == rhs.interface_index);
}

}  // namespace

uint32_t MulticastService::membership_group_address(
    const SocketMulticastMembership& membership) noexcept
{
    return FreeRTOS_inet_addr_quick(membership.group[0],
                                    membership.group[1],
                                    membership.group[2],
                                    membership.group[3]);
}

uint32_t MulticastService::membership_interface_address(
    const SocketMulticastMembership& membership) noexcept
{
    return FreeRTOS_inet_addr_quick(membership.interface[0],
                                    membership.interface[1],
                                    membership.interface[2],
                                    membership.interface[3]);
}

bool MulticastService::is_unspecified_ipv6(const std::array<uint8_t, 16>& address) noexcept
{
    for (uint8_t byte : address) {
        if (byte != 0u) {
            return false;
        }
    }
    return true;
}

bool MulticastService::tick_due(TickType_t now, TickType_t due) noexcept
{
    return (due != 0u) &&
           (static_cast<TickType_t>(now - due) < (portMAX_DELAY / 2u));
}

bool MulticastService::tick_before(TickType_t lhs, TickType_t rhs) noexcept
{
    return static_cast<TickType_t>(lhs - rhs) > (portMAX_DELAY / 2u);
}

bool MulticastService::is_ipv4_multicast(uint32_t address) noexcept
{
    const uint32_t host_order = FreeRTOS_ntohl(address);
    return (host_order >= 0xE0000000u) && (host_order <= 0xEFFFFFFFu);
}

bool MulticastService::is_ipv6_multicast(const std::array<uint8_t, 16>& address) noexcept
{
    return address[0] == 0xFFu;
}

TickType_t MulticastService::response_delay_ticks(uint8_t max_response_time,
                                                  uint32_t group_address) noexcept
{
    const TickType_t window =
        (max_response_time == 0u)
            ? kDefaultMaxResponseTicks
            : std::max<TickType_t>(1u, pdMS_TO_TICKS(static_cast<uint32_t>(max_response_time) * 100u));
    const TickType_t jitter = static_cast<TickType_t>(
        (xTaskGetTickCount() ^ group_address) % static_cast<uint32_t>(window));
    return std::max<TickType_t>(1u, jitter + 1u);
}

TickType_t MulticastService::response_delay_ticks_ipv6(
    uint16_t max_response_time_ms,
    const std::array<uint8_t, 16>& group_address) noexcept
{
    const TickType_t window =
        (max_response_time_ms == 0u)
            ? kDefaultMaxResponseTicks
            : std::max<TickType_t>(1u, pdMS_TO_TICKS(max_response_time_ms));
    uint32_t hash = 0u;
    for (uint8_t byte : group_address) {
        hash = (hash * 33u) ^ byte;
    }
    const TickType_t jitter = static_cast<TickType_t>(
        (xTaskGetTickCount() ^ hash) % static_cast<uint32_t>(window));
    return std::max<TickType_t>(1u, jitter + 1u);
}

NetworkEndPoint_t* MulticastService::find_ipv4_endpoint(uint32_t interface_address) noexcept
{
    if (interface_address != 0u) {
        NetworkEndPoint_t* by_address = FreeRTOS_FindEndPointOnIP_IPv4(interface_address);
        if ((by_address != nullptr) && ENDPOINT_IS_IPv4(by_address)) {
            return by_address;
        }
    }

    for (NetworkEndPoint_t* endpoint = FreeRTOS_FirstEndPoint(nullptr);
         endpoint != nullptr;
         endpoint = FreeRTOS_NextEndPoint(nullptr, endpoint)) {
        if (ENDPOINT_IS_IPv4(endpoint)) {
            return endpoint;
        }
    }

    return nullptr;
}

NetworkEndPoint_t* MulticastService::find_ipv6_endpoint(
    const SocketMulticastMembership& membership) noexcept
{
    for (NetworkEndPoint_t* endpoint = FreeRTOS_FirstEndPoint(nullptr);
         endpoint != nullptr;
         endpoint = FreeRTOS_NextEndPoint(nullptr, endpoint)) {
        if (!ENDPOINT_IS_IPv6(endpoint)) {
            continue;
        }

        if ((membership.interface_index != 0u) &&
            (endpoint->bits.bIPv6 == 0u)) {
            continue;
        }

        if (!is_unspecified_ipv6(membership.interface)) {
            if (std::memcmp(endpoint->ipv6_settings.xIPAddress.ucBytes,
                            membership.interface.data(),
                            membership.interface.size()) != 0) {
                continue;
            }
        }

        return endpoint;
    }

    return nullptr;
}

MulticastService::MembershipEntry* MulticastService::find_membership(
    const SocketMulticastMembership& membership) noexcept
{
    for (auto& entry : memberships_) {
        if (entry.used && same_membership(entry.membership, membership)) {
            return &entry;
        }
    }

    return nullptr;
}

MulticastService::MembershipEntry* MulticastService::find_free_entry() noexcept
{
    for (auto& entry : memberships_) {
        if (!entry.used) {
            return &entry;
        }
    }

    return nullptr;
}

bool MulticastService::send_igmp_message(const MembershipEntry& entry,
                                         uint8_t type,
                                         uint32_t destination_address) noexcept
{
    if ((output_ == nullptr) || (entry.endpoint == nullptr) ||
        (entry.endpoint->pxNetworkInterface == nullptr)) {
        return false;
    }

    const uint32_t source_address = entry.endpoint->ipv4_settings.ulIPAddress;
    if (source_address == 0u) {
        return false;
    }

    std::array<uint8_t, kMinEthernetFrameLength> frame{};
    auto* packet = reinterpret_cast<IPPacket_t*>(frame.data());
    auto* igmp = reinterpret_cast<IgmpHeader*>(frame.data() + sizeof(IPPacket_t));

    vSetMultiCastIPv4MacAddress(destination_address, &packet->xEthernetHeader.xDestinationAddress);
    std::memcpy(packet->xEthernetHeader.xSourceAddress.ucBytes,
                entry.endpoint->xMACAddress.ucBytes,
                ipMAC_ADDRESS_LENGTH_BYTES);
    packet->xEthernetHeader.usFrameType = ipIPv4_FRAME_TYPE;

    packet->xIPHeader.ucVersionHeaderLength = 0x45u;
    packet->xIPHeader.ucDifferentiatedServicesCode = 0xC0u;
    packet->xIPHeader.usLength = FreeRTOS_htons(sizeof(IPHeader_t) + sizeof(IgmpHeader));
    packet->xIPHeader.usIdentification = FreeRTOS_htons(
        static_cast<uint16_t>(xTaskGetTickCount() & 0xFFFFu));
    packet->xIPHeader.usFragmentOffset = 0u;
    packet->xIPHeader.ucTimeToLive = 1u;
    packet->xIPHeader.ucProtocol = static_cast<uint8_t>(ipPROTOCOL_IGMP);
    packet->xIPHeader.usHeaderChecksum = 0u;
    packet->xIPHeader.ulSourceIPAddress = source_address;
    packet->xIPHeader.ulDestinationIPAddress = destination_address;
    packet->xIPHeader.usHeaderChecksum =
        usGenerateChecksum(0u,
                           reinterpret_cast<const uint8_t*>(&packet->xIPHeader.ucVersionHeaderLength),
                           sizeof(IPHeader_t));

    igmp->type = type;
    igmp->max_response_time = 0u;
    igmp->checksum = 0u;
    igmp->group_address = membership_group_address(entry.membership);
    igmp->checksum = usGenerateChecksum(0u,
                                        reinterpret_cast<const uint8_t*>(igmp),
                                        sizeof(IgmpHeader));

    const std::size_t wire_length = sizeof(IPPacket_t) + sizeof(IgmpHeader);
    return output_(entry.endpoint->pxNetworkInterface,
                   frame.data(),
                   std::max(wire_length, kMinEthernetFrameLength),
                   output_context_);
}

bool MulticastService::send_mld_message(const MembershipEntry& entry,
                                        uint8_t type,
                                        const std::array<uint8_t, 16>& destination_address) noexcept
{
    if ((output_ == nullptr) || (entry.endpoint == nullptr) ||
        (entry.endpoint->pxNetworkInterface == nullptr)) {
        return false;
    }

    constexpr std::size_t packet_length =
        sizeof(IPPacket_IPv6_t) + sizeof(IPv6HopByHopHeader) + sizeof(MldHeader);
    std::array<uint8_t, packet_length> frame{};

    auto* packet = reinterpret_cast<IPPacket_IPv6_t*>(frame.data());
    auto* hop = reinterpret_cast<IPv6HopByHopHeader*>(frame.data() + sizeof(IPPacket_IPv6_t));
    auto* mld = reinterpret_cast<MldHeader*>(frame.data() + sizeof(IPPacket_IPv6_t) +
                                             sizeof(IPv6HopByHopHeader));

    IPv6_Address_t destination{};
    std::memcpy(destination.ucBytes, destination_address.data(), destination_address.size());
    vSetMultiCastIPv6MacAddress(&destination, &packet->xEthernetHeader.xDestinationAddress);
    std::memcpy(packet->xEthernetHeader.xSourceAddress.ucBytes,
                entry.endpoint->xMACAddress.ucBytes,
                ipMAC_ADDRESS_LENGTH_BYTES);
    packet->xEthernetHeader.usFrameType = ipIPv6_FRAME_TYPE;

    std::memset(&packet->xIPHeader, 0, sizeof(packet->xIPHeader));
    packet->xIPHeader.ucVersionTrafficClass = ipTYPE_IPv6;
    packet->xIPHeader.usPayloadLength =
        FreeRTOS_htons(static_cast<uint16_t>(sizeof(IPv6HopByHopHeader) + sizeof(MldHeader)));
    packet->xIPHeader.ucNextHeader = kIpv6NextHeaderHopByHop;
    packet->xIPHeader.ucHopLimit = kIpv6HopLimitLinkLocal;
    std::memcpy(packet->xIPHeader.xSourceAddress.ucBytes,
                entry.endpoint->ipv6_settings.xIPAddress.ucBytes,
                ipSIZE_OF_IPv6_ADDRESS);
    std::memcpy(packet->xIPHeader.xDestinationAddress.ucBytes,
                destination_address.data(),
                destination_address.size());

    hop->next_header = ipPROTOCOL_ICMP_IPv6;
    hop->hdr_ext_len = 0u;
    hop->option_type = kRouterAlertOptionType;
    hop->option_length = kRouterAlertOptionLength;
    hop->option_value = FreeRTOS_htons(kRouterAlertValueMld);
    hop->padn_type = kPadNOptionType;
    hop->padn_length = kPadNOptionLength;

    std::memset(mld, 0, sizeof(*mld));
    mld->type = type;
    mld->code = 0u;
    mld->checksum = 0u;
    std::memcpy(mld->multicast_address.ucBytes,
                entry.membership.group.data(),
                entry.membership.group.size());

    (void)usGenerateProtocolChecksum(frame.data(), packet_length, pdTRUE);

    return output_(entry.endpoint->pxNetworkInterface,
                   frame.data(),
                   packet_length,
                   output_context_);
}

void MulticastService::add_allowed_mac(const MembershipEntry& entry) noexcept
{
    if ((entry.endpoint == nullptr) || (entry.endpoint->pxNetworkInterface == nullptr)) {
        return;
    }

    MACAddress_t mac{};
    if (entry.membership.domain == SocketDomain::ipv6) {
        IPv6_Address_t address{};
        std::memcpy(address.ucBytes, entry.membership.group.data(), entry.membership.group.size());
        vSetMultiCastIPv6MacAddress(&address, &mac);
    } else {
        vSetMultiCastIPv4MacAddress(membership_group_address(entry.membership), &mac);
    }

    if (entry.endpoint->pxNetworkInterface->pfAddAllowedMAC != nullptr) {
        entry.endpoint->pxNetworkInterface->pfAddAllowedMAC(
            entry.endpoint->pxNetworkInterface,
            mac.ucBytes);
    }
}

void MulticastService::remove_allowed_mac(const MembershipEntry& entry) noexcept
{
    if ((entry.endpoint == nullptr) || (entry.endpoint->pxNetworkInterface == nullptr)) {
        return;
    }

    MACAddress_t mac{};
    if (entry.membership.domain == SocketDomain::ipv6) {
        IPv6_Address_t address{};
        std::memcpy(address.ucBytes, entry.membership.group.data(), entry.membership.group.size());
        vSetMultiCastIPv6MacAddress(&address, &mac);
    } else {
        vSetMultiCastIPv4MacAddress(membership_group_address(entry.membership), &mac);
    }

    if (entry.endpoint->pxNetworkInterface->pfRemoveAllowedMAC != nullptr) {
        entry.endpoint->pxNetworkInterface->pfRemoveAllowedMAC(
            entry.endpoint->pxNetworkInterface,
            mac.ucBytes);
    }
}

void MulticastService::configure_output(OutputFunction output, void* context) noexcept
{
    taskENTER_CRITICAL();
    output_ = output;
    output_context_ = context;
    taskEXIT_CRITICAL();
}

int MulticastService::join_group(const SocketMulticastMembership& membership) noexcept
{
    NetworkEndPoint_t* endpoint = nullptr;
    if (membership.domain == SocketDomain::ipv4) {
        const uint32_t group_address = membership_group_address(membership);
        if (!is_ipv4_multicast(group_address)) {
            return -1;
        }
        endpoint = find_ipv4_endpoint(membership_interface_address(membership));
    } else if (membership.domain == SocketDomain::ipv6) {
        if (!is_ipv6_multicast(membership.group)) {
            return -1;
        }
        endpoint = find_ipv6_endpoint(membership);
    } else {
        return -1;
    }

    if (endpoint == nullptr) {
        return -1;
    }

    MembershipEntry snapshot{};
    bool added_mac = false;
    bool send_report = false;
    bool joined = false;
    const TickType_t now = xTaskGetTickCount();

    taskENTER_CRITICAL();
    MembershipEntry* entry = find_membership(membership);
    if (entry != nullptr) {
        ++entry->reference_count;
        joined = true;
    } else {
        entry = find_free_entry();
        if (entry != nullptr) {
            entry->used = true;
            entry->membership = membership;
            entry->endpoint = endpoint;
            entry->reference_count = 1u;
            entry->unsolicited_reports_remaining = 1u;
            entry->next_unsolicited_tick = now + kUnsolicitedRepeatTicks;
            entry->next_query_tick = 0u;
            entry->next_refresh_tick = now + kMembershipRefreshTicks;
            snapshot = *entry;
            added_mac = true;
            send_report = true;
            joined = true;
        }
    }
    taskEXIT_CRITICAL();

    if (!joined) {
        return -1;
    }

    if (added_mac) {
        add_allowed_mac(snapshot);
    }

    if (send_report) {
        if (membership.domain == SocketDomain::ipv6) {
            (void)send_mld_message(snapshot, kMldTypeListenerReport, snapshot.membership.group);
        } else {
            (void)send_igmp_message(snapshot,
                                    kIgmpTypeMembershipReportV2,
                                    membership_group_address(snapshot.membership));
        }
    }

    return 0;
}

int MulticastService::leave_group(const SocketMulticastMembership& membership,
                                  bool send_leave) noexcept
{
    MembershipEntry snapshot{};
    bool remove_mac = false;
    bool emit_leave = false;

    taskENTER_CRITICAL();
    MembershipEntry* entry = find_membership(membership);
    if (entry == nullptr) {
        taskEXIT_CRITICAL();
        return 0;
    }

    if (entry->reference_count > 1u) {
        --entry->reference_count;
        taskEXIT_CRITICAL();
        return 0;
    }

    snapshot = *entry;
    *entry = {};
    remove_mac = true;
    emit_leave = send_leave;
    taskEXIT_CRITICAL();

    if (emit_leave) {
        if (membership.domain == SocketDomain::ipv6) {
            (void)send_mld_message(snapshot, kMldTypeListenerDone, kIpv6AllRoutersLinkLocal);
        } else {
            (void)send_igmp_message(snapshot, kIgmpTypeLeaveGroup, FreeRTOS_htonl(kAllRoutersGroup));
        }
    }

    if (remove_mac) {
        remove_allowed_mac(snapshot);
    }

    return 0;
}

void MulticastService::process_frame(const uint8_t* frame,
                                     std::size_t length,
                                     NetworkInterface_t* interface) noexcept
{
    if ((frame == nullptr) || (interface == nullptr)) {
        return;
    }

    const auto* ethernet = reinterpret_cast<const EthernetHeader_t*>(frame);
    if ((ethernet->usFrameType == ipIPv4_FRAME_TYPE) &&
        (length >= (sizeof(IPPacket_t) + sizeof(IgmpHeader)))) {
        const auto* packet = reinterpret_cast<const IPPacket_t*>(frame);
        const std::size_t ip_header_length =
            static_cast<std::size_t>(packet->xIPHeader.ucVersionHeaderLength & 0x0Fu) * 4u;
        if ((ip_header_length >= sizeof(IPHeader_t)) &&
            (length >= (sizeof(EthernetHeader_t) + ip_header_length + sizeof(IgmpHeader))) &&
            (packet->xIPHeader.ucProtocol == static_cast<uint8_t>(ipPROTOCOL_IGMP))) {
            const auto* igmp =
                reinterpret_cast<const IgmpHeader*>(frame + sizeof(EthernetHeader_t) + ip_header_length);
            if (igmp->type == kIgmpTypeMembershipQuery) {
                const TickType_t now = xTaskGetTickCount();
                taskENTER_CRITICAL();
                for (auto& entry : memberships_) {
                    if (!entry.used || (entry.membership.domain != SocketDomain::ipv4) ||
                        (entry.endpoint == nullptr) ||
                        (entry.endpoint->pxNetworkInterface != interface)) {
                        continue;
                    }

                    if ((igmp->group_address != 0u) &&
                        (igmp->group_address != membership_group_address(entry.membership))) {
                        continue;
                    }

                    const TickType_t candidate =
                        now + response_delay_ticks(igmp->max_response_time,
                                                   membership_group_address(entry.membership));
                    if ((entry.next_query_tick == 0u) ||
                        tick_before(candidate, entry.next_query_tick)) {
                        entry.next_query_tick = candidate;
                    }
                }
                taskEXIT_CRITICAL();
            }
        }
        return;
    }

    if ((ethernet->usFrameType == ipIPv6_FRAME_TYPE) &&
        (length >= (sizeof(IPPacket_IPv6_t) + sizeof(IPv6HopByHopHeader) + sizeof(MldHeader)))) {
        const auto* packet = reinterpret_cast<const IPPacket_IPv6_t*>(frame);
        std::size_t offset = sizeof(IPPacket_IPv6_t);
        uint8_t next_header = packet->xIPHeader.ucNextHeader;

        if (next_header == kIpv6NextHeaderHopByHop) {
            const auto* hop = reinterpret_cast<const IPv6HopByHopHeader*>(frame + offset);
            offset += sizeof(IPv6HopByHopHeader);
            next_header = hop->next_header;
        }

        if ((next_header == ipPROTOCOL_ICMP_IPv6) &&
            (length >= (offset + sizeof(MldHeader)))) {
            const auto* mld = reinterpret_cast<const MldHeader*>(frame + offset);
            if (mld->type == kMldTypeListenerQuery) {
                std::array<uint8_t, 16> query_group{};
                std::memcpy(query_group.data(),
                            mld->multicast_address.ucBytes,
                            query_group.size());
                const uint16_t max_response_ms = FreeRTOS_ntohs(mld->max_response_delay);
                const TickType_t now = xTaskGetTickCount();

                taskENTER_CRITICAL();
                for (auto& entry : memberships_) {
                    if (!entry.used || (entry.membership.domain != SocketDomain::ipv6) ||
                        (entry.endpoint == nullptr) ||
                        (entry.endpoint->pxNetworkInterface != interface)) {
                        continue;
                    }

                    if (!is_unspecified_ipv6(query_group) &&
                        (entry.membership.group != query_group)) {
                        continue;
                    }

                    const TickType_t candidate =
                        now + response_delay_ticks_ipv6(max_response_ms, entry.membership.group);
                    if ((entry.next_query_tick == 0u) ||
                        tick_before(candidate, entry.next_query_tick)) {
                        entry.next_query_tick = candidate;
                    }
                }
                taskEXIT_CRITICAL();
            }
        }
    }
}

void MulticastService::periodic() noexcept
{
    const TickType_t now = xTaskGetTickCount();

    for (std::size_t index = 0; index < memberships_.size(); ++index) {
        MembershipEntry snapshot{};
        bool send_query_report = false;
        bool send_unsolicited_report = false;
        bool send_refresh_report = false;

        taskENTER_CRITICAL();
        MembershipEntry& entry = memberships_[index];
        if (entry.used) {
            snapshot = entry;

            if (tick_due(now, entry.next_query_tick)) {
                entry.next_query_tick = 0u;
                send_query_report = true;
                snapshot = entry;
            } else if ((entry.unsolicited_reports_remaining > 0u) &&
                       tick_due(now, entry.next_unsolicited_tick)) {
                --entry.unsolicited_reports_remaining;
                entry.next_unsolicited_tick =
                    (entry.unsolicited_reports_remaining > 0u)
                        ? (now + kUnsolicitedRepeatTicks)
                        : 0u;
                send_unsolicited_report = true;
                snapshot = entry;
            } else if (tick_due(now, entry.next_refresh_tick)) {
                entry.next_refresh_tick = now + kMembershipRefreshTicks;
                send_refresh_report = true;
                snapshot = entry;
            }
        }
        taskEXIT_CRITICAL();

        if (send_query_report || send_unsolicited_report || send_refresh_report) {
            bool ok = false;
            if (snapshot.membership.domain == SocketDomain::ipv6) {
                ok = send_mld_message(snapshot, kMldTypeListenerReport, snapshot.membership.group);
            } else {
                ok = send_igmp_message(snapshot,
                                       kIgmpTypeMembershipReportV2,
                                       membership_group_address(snapshot.membership));
            }

            if (!ok) {
                taskENTER_CRITICAL();
                MembershipEntry& entry = memberships_[index];
                if (entry.used) {
                    if (send_query_report) {
                        entry.next_query_tick = now + kUnsolicitedRepeatTicks;
                    } else if (send_unsolicited_report) {
                        entry.next_unsolicited_tick = now + kUnsolicitedRepeatTicks;
                        ++entry.unsolicited_reports_remaining;
                    } else if (send_refresh_report) {
                        entry.next_refresh_tick = now + kUnsolicitedRepeatTicks;
                    }
                }
                taskEXIT_CRITICAL();
            }
        }
    }
}

MulticastService& freertos_multicast_service() noexcept
{
    static MulticastService service;
    return service;
}

}  // namespace rosette::net
