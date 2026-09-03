#include "wiznet/macraw.hpp"

#include "wiznet/board.hpp"

#include <cstring>

namespace {

using DefaultChip = rosette::net::wiznet::BoardChip;
using DefaultBus = rosette::net::wiznet::BoardBus;
using DefaultMacDevice = rosette::net::wiznet::WiznetMacrawDevice<DefaultChip, DefaultBus>;
using DefaultInterface = rosette::net::wiznet::MacrawNetworkInterface<DefaultChip, DefaultBus>;

DefaultInterface g_default_interface{
    DefaultMacDevice{DefaultBus{ROSETTE_SPI_INST, ROSETTE_PIN_SPI_CS}},
    std::array<uint8_t, 6> ROSETTE_MAC_ADDR_DEFAULT
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
