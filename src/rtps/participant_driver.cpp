/**
 * @file participant_driver.cpp
 * @brief ParticipantDriver: construction and one locked protocol pass.
 */

#include "rosette/rtps/participant_driver.hpp"

namespace rosette::rtps {

ParticipantDriver::ParticipantDriver(RTPSParticipant& participant, SocketUdpTransport& transport,
                                     rosette::net::SocketProvider& provider, Octet* rx_buffer,
                                     uint32_t rx_capacity) noexcept
    : participant_(participant)
    , transport_(transport)
    , provider_(provider)
    , rx_buffer_(rx_buffer)
    , rx_capacity_(rx_capacity)
{
    provider_.set_receive_event(&wake_);
}

ParticipantDriver::~ParticipantDriver() noexcept
{
    provider_.set_receive_event(nullptr);
}

uint32_t ParticipantDriver::pump_locked() noexcept
{
    uint32_t src_addr = 0u;
    uint16_t src_port = 0u;
    uint32_t received = 0u;
    while ((received = transport_.poll_meta(rx_buffer_, rx_capacity_, src_addr, src_port)) > 0u) {
        participant_.parse_received(rx_buffer_, received);
    }
    while ((received = transport_.poll_user(rx_buffer_, rx_capacity_, src_addr, src_port)) > 0u) {
        participant_.parse_received(rx_buffer_, received);
    }
    const uint32_t now_ms = rosette::platform::monotonic_ms();
    participant_.tick(now_ms);
    return now_ms;
}

}  // namespace rosette::rtps
