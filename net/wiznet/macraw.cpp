#include "wiznet/macraw.hpp"

#include "board_config.h"
#include "bus_pico_spi.hpp"
#include "wiznet/w5100s.hpp"

#include <cstring>

namespace {

using DefaultChip = milkdds::net::wiznet::W5100S;
using DefaultBus = milkdds::platform::PicoSpiBus;
using DefaultMacDevice = milkdds::net::wiznet::WiznetMacrawDevice<DefaultChip, DefaultBus>;
using DefaultInterface = milkdds::net::wiznet::MacrawNetworkInterface<DefaultChip, DefaultBus>;

DefaultInterface g_default_interface{
    DefaultMacDevice{DefaultBus{RTPS_SPI_INST, RTPS_PIN_SPI_CS}},
    std::array<uint8_t, 6> RTPS_MAC_ADDR_DEFAULT
};

NetworkInterface_t g_default_network_interface;

}  // namespace

extern "C" void wiznet_macraw_init(void)
{
    (void)g_default_interface.fill_interface_descriptor(g_default_network_interface, "wiz0");
}

extern "C" bool wiznet_macraw_link_status(void)
{
    return g_default_interface.link_status() == pdTRUE;
}

extern "C" void wiznet_macraw_interrupt_from_isr(void)
{
    g_default_interface.handle_interrupt_from_isr();
}

extern "C" void wiznet_macraw_set_mac_address(const uint8_t mac[6])
{
    if (mac == nullptr) {
        return;
    }

    std::array<uint8_t, 6> mac_array{};
    std::memcpy(mac_array.data(), mac, mac_array.size());
    g_default_interface.set_fallback_mac_address(mac_array);
}

extern "C" BaseType_t xNetworkInterfaceInitialise(void)
{
    return g_default_interface.initialise();
}

extern "C" BaseType_t xNetworkInterfaceOutput(
    NetworkBufferDescriptor_t* pxDescriptor,
    BaseType_t xReleaseAfterSend)
{
    return g_default_interface.output(pxDescriptor, xReleaseAfterSend);
}
