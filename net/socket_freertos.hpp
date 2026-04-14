#pragma once

#include "socket.hpp"

namespace milkdds::net {

class FreeRtosSocketProvider final : public SocketProvider {
public:
    std::unique_ptr<Socket> open(
        SocketDomain domain = SocketDomain::ipv4,
        SocketType type = SocketType::datagram,
        SocketProtocol protocol = SocketProtocol::udp) noexcept override;
};

}  // namespace milkdds::wiznet
