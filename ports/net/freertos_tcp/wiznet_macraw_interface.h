#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "NetworkInterface.h"

#ifdef __cplusplus
extern "C" {
#endif

void wiznet_macraw_init(void);
BaseType_t xNetworkInterfaceInitialise(void);
BaseType_t xNetworkInterfaceOutput(NetworkBufferDescriptor_t* pxDescriptor,
                                   BaseType_t xReleaseAfterSend);
bool wiznet_macraw_link_status(void);
void wiznet_macraw_interrupt_from_isr(void);
void wiznet_macraw_set_mac_address(const uint8_t mac[6]);

#ifdef __cplusplus
}
#endif
