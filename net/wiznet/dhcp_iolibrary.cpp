#include "dhcp_iolibrary.hpp"

#include "net/wiznet/w5100s.hpp"
#include "platform/bus_pico_spi.hpp"

template class milkdds::net::wiznet::IolibraryDhcpClient<
    milkdds::net::wiznet::W5100S,
    milkdds::platform::PicoSpiBus>;
