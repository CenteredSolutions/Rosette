#include "rosette/net/wiznet/dhcp_iolibrary.hpp"

#include "rosette/net/wiznet/w5100s.hpp"
#include "rosette/platform/bus_pico_spi.hpp"

template class rosette::net::wiznet::IolibraryDhcpClient<
    rosette::net::wiznet::W5100S,
    rosette::platform::PicoSpiBus>;
