/**
 * @file participant_driver.cpp
 * @brief ParticipantDriver: construction and one locked protocol pass.
 */

#include "rosette/rtps/participant_driver.hpp"

namespace rosette::rtps {

ParticipantDriver::ParticipantDriver(RTPSParticipant& participant, SocketUdpTransport& transport,
                                     rosette::net::SocketProvider& provider, Octet* rx_buffer,
                                     uint32_t rx_capacity, uint32_t pass_budget_us) noexcept
    : participant_(participant),
      transport_(transport),
      provider_(provider),
      rx_buffer_(rx_buffer),
      rx_capacity_(rx_capacity),
      pass_budget_us_(pass_budget_us) {
    provider_.set_receive_event(&wake_);
}

ParticipantDriver::~ParticipantDriver() noexcept
{
    provider_.set_receive_event(nullptr);
}

ParticipantDriver::Pass ParticipantDriver::pump_locked() noexcept {
    const uint64_t start_us = rosette::platform::monotonic_us();
    uint32_t src_addr = 0u;
    uint16_t src_port = 0u;
    uint8_t        empty_in_a_row = 0u;
    Pass           pass{};
    while (empty_in_a_row < 2u) {
        const uint32_t received =
            next_is_user_ ? transport_.poll_user(rx_buffer_, rx_capacity_, src_addr, src_port)
                          : transport_.poll_meta(rx_buffer_, rx_capacity_, src_addr, src_port);
        next_is_user_ = !next_is_user_;
        if (received == 0u) {
            ++empty_in_a_row;
            continue;
        }

        empty_in_a_row = 0u;
        participant_.parse_received(rx_buffer_, received);
        if (rosette::platform::monotonic_us() - start_us >= pass_budget_us_) {
            pass.backlog = true;
            break;
        }
    }

    pass.now_ms = rosette::platform::monotonic_ms();
    participant_.tick(pass.now_ms);
    return pass;
}

}  // namespace rosette::rtps
