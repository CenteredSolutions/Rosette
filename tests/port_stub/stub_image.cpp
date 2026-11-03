// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file tests/port_stub/stub_image.cpp
 * @brief Entry point of the stub-port image, and the WIZnet driver instantiated on the stub bus.
 *
 * The image never runs; it exists to be linked (tests/port_stub/CMakeLists.txt).
 * The compiled libraries go in whole, so they need no reference from here.
 * The WIZnet driver is header-only, so this file instantiates it for each
 * chip model on ports/chip/stub's bus: the driver then compiles and links for a chip that
 * is not an RP2040 / RP2350. The MACRAW device is instantiated too, on an OS
 * port with no RTOS, which shows the MAC provider depends on no IP stack.
 */

#include "stub_bus.hpp"

#include "ports/net/wiznet/device.hpp"
#include "ports/net/wiznet/macraw_device.hpp"
#include "ports/net/wiznet/slaac_provider.hpp"
#include "ports/net/wiznet/socket_provider.hpp"
#include "ports/net/wiznet/w5100s.hpp"
#include "ports/net/wiznet/w6x00.hpp"

namespace wiznet = rosette::net::wiznet;
using rosette::platform::StubBus;

template class wiznet::WiznetDevice<wiznet::W6100, StubBus>;
template class wiznet::WiznetDevice<wiznet::W6300, StubBus>;
template class wiznet::WiznetSocketProvider<wiznet::W6100, StubBus>;
template class wiznet::WiznetSocketProvider<wiznet::W6300, StubBus>;
template class wiznet::WiznetSlaacProvider<wiznet::W6100, StubBus>;
template class wiznet::WiznetSlaacProvider<wiznet::W6300, StubBus>;
template class wiznet::WiznetMacrawDevice<wiznet::W5100S, StubBus>;
template class wiznet::WiznetMacrawDevice<wiznet::W6100, StubBus>;
template class wiznet::WiznetMacrawDevice<wiznet::W6300, StubBus>;

/**
 * @brief Instantiates the W5100S driver as a board would use it.
 *
 * The device's IPv6 members name registers the W5100S does not have, so they
 * are instantiated only when called, and explicit instantiation of the whole
 * class does not compile. Building the socket provider instantiates its
 * virtual members, the whole socket path.
 *
 * @param[in] bus The chip's register bus.
 * @return 0 when the chip answers.
 */
int use_w5100s(StubBus& bus) {
    static wiznet::WiznetDevice<wiznet::W5100S, StubBus> device(bus);
    static wiznet::WiznetSocketProvider<wiznet::W5100S, StubBus> sockets(device);
    return device.verify() ? 0 : 1;
}

int main() {
    return 0;
}
