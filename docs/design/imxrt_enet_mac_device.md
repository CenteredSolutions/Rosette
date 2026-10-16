# i.MX RT ENET as a MacDevice, under FreeRTOS+TCP

The MIMXRT1020-EVK carries an i.MX RT1021 (Cortex-M7, 500 MHz, 256 KiB FlexRAM) with the chip's own 10/100 ENET
MAC, a KSZ8081RNB PHY over RMII, 8 MiB of QSPI NOR, and 32 MiB of SDRAM. This board runs Rosette with the IP stack
on the microcontroller: FreeRTOS+TCP over the ENET, where the WIZnet boards run it in the network chip.

The ENET reaches FreeRTOS+TCP through `rosette::net::MacDevice` (`include/rosette/net/mac.hpp`), not through
FreeRTOS+TCP's own `NXP1060/NetworkInterface.c`. The IP-stack port (`ports/net/freertos_tcp/`) then stays free of
chip code, and the same ENET driver serves lwIP when `ports/net/lwip/` exists.

## Where the bytes go

A received RTPS datagram, and the buffers it passes through, on each design:

| Design | Receive path | Buffer pools |
|---|---|---|
| FreeRTOS+TCP `NXP1060` driver | DMA → driver ring → `memcpy` → network buffer → `memcpy` → rmw `rx_buffer` → parse | driver RX ring (10 × 1536), driver TX ring (3 × 1536), stack pool (10 × 1550): about 35 KiB |
| Copying `MacDevice` (the contract as it is) | the same two copies | the same three pools, sized by the board: 4 RX, 2 TX, 8 stack is about 21 KiB |
| Lending `MacDevice` | DMA → network buffer → parse in place | the stack pool alone; the RX ring holds lent network buffers: 8 stack buffers is about 12 KiB |

At 100 Mbit/s a 1.5 KiB frame occupies the wire for 120 µs, and copying it on a 500 MHz M7 from DTCM takes one to
two microseconds. The copies cost little time. What lending removes is the second and third pool: the RAM the
driver keeps for frames the stack already has room for. On a 256 KiB part that is the reason to do it.

Transmit is lent at the driver level only. FreeRTOS+TCP with `ipconfigZERO_COPY_TX_DRIVER` hands the network buffer
to the driver, the transmit descriptor points at it, and the driver releases it when the ENET reports the frame
sent. The RTPS layer keeps building messages into its own buffer pool and `send_to()` copies once into a network
buffer: a reliable writer must keep each sample for repair after the frame has gone, so the network buffer cannot be
the history's storage.

## Contract changes to MacDevice

The contract as written fits a SPI-attached controller in MACRAW mode. An on-chip MAC needs three more operations,
which the WIZnet MACRAW device implements trivially.

1. **Receive notification.** The receive task sleeps on a `platform::Event`. A SPI controller's board signals it
   from a GPIO edge (`NetworkInterface::attach_rx_interrupt(pin)`); the ENET signals it from its own NVIC
   interrupt. `MacDevice::set_receive_event(platform::Event*)`, the counterpart of
   `SocketProvider::set_receive_event()`, hands the device the event, and `ENET_IRQHandler` calls
   `signal_from_isr()` on it. The ENET interrupt's priority is numerically at least
   `configMAX_SYSCALL_INTERRUPT_PRIORITY`, the FreeRTOS rule for an interrupt that calls a `FromISR` API. The
   WIZnet device ignores the event, and its board keeps attaching the INT pin.
2. **Multicast filter.** The ENET drops multicast frames whose CRC-32 hash bucket is not set in `GAUR`/`GALR`, so
   SPDP on 239.255.0.1 never arrives unless someone programs it. `NetworkInterface::add_allowed_mac()` counts
   requests and tells nobody. `MacDevice::add_multicast(const MacAddress&)` and `remove_multicast()` reach the device;
   the ENET driver keeps a reference count per hash bucket (64 octets), because two groups can share a bucket and
   leaving one must not deafen the other.
3. **Link changes.** The ENET's speed and duplex (`ECR[SPEED]`, `RCR[RMII_10T]`, `TCR[FDEN]`) must match what the
   PHY negotiated. `phy_status()` reads the PHY over MDIO and reprograms the MAC when the result changes, and the
   receive task calls it on each idle wake (`kRxIdleWaitMs`) and calls `FreeRTOS_NetworkDown()` when the link drops.

## The lending contract

`rosette::net::LendingMacDevice` derives from `MacDevice` and adds buffer exchange. The consumer owns every buffer;
the device only holds them between being lent one and giving it back.

```cpp
struct LentFrame {
    uint8_t* data;       // first octet of the Ethernet header
    std::size_t length;  // octets of frame, without FCS
    void* owner;         // the consumer's handle: a NetworkBufferDescriptor_t*, or an lwIP pbuf*
};

virtual std::size_t lent_buffer_bytes() const noexcept = 0;  // capacity each lent RX buffer must have
virtual std::size_t lent_buffer_alignment() const noexcept = 0;
virtual bool lend_receive_buffer(uint8_t* buffer, void* owner) noexcept = 0;  // false: the ring is full
virtual bool take_received_frame(LentFrame& frame) noexcept = 0;            // false: nothing received
virtual bool send_lent_frame(const LentFrame& frame) noexcept = 0;         // false: the TX ring is full
virtual bool take_sent_frame(void*& owner) noexcept = 0;                    // false: nothing finished
```

Lent buffers are the IP stack's own: FreeRTOS+TCP's network buffers, whose `NetworkBufferDescriptor_t*` is the
`owner` the device hands back. The device keeps no frame storage of its own.

`ports/net/freertos_tcp/lending_interface.hpp` is the consumer: a second interface template beside
`NetworkInterface<Device>`, so each stays one short file. Its receive task replaces each frame it takes with a
fresh network buffer before posting the frame to the IP task. When the pool has none free it drops the frame and
lends the same buffer back, so the receive ring never runs empty. That is the invariant a test must prove: with the
stack pool exhausted, the ring still holds `ENET_RXBD_NUM` buffers and later frames are received once buffers free.

The copying `MacDevice` operations stay on the ENET driver too, implemented over the same rings, so the board can
build either interface and measure one against the other.


## Order of work

1. Board and chip: `boards/mimxrt1020-evk` (MIMXRT1021DAG5A, evkmimxrt1020 clocks, IS25LP064A boot block, LPUART1
   console through the on-board debug probe), FlexRAM layout, and the existing bare-metal superloop blinking and
   printing.
2. FreeRTOS on imxrt: the kernel's `ARM_CM4F` port (the RT1021's M7 is not the r0p1 revision the `ARM_CM7/r0p1` port
   works around), SysTick beside the PIT time base, ENET interrupt priority at or below
   `configMAX_SYSCALL_INTERRUPT_PRIORITY`.
3. `ports/net/freertos_tcp/board_network.cpp`: the open item in `ports_move_plan.md` that lets any
   FreeRTOS+TCP image link.
4. The ENET `MacDevice`, copying, with the three contract additions. Bench: DHCP, ping, `wiznet_udp_echo`'s
   counterpart, then `led_files` against `rmw_fastrtps_cpp` and `rmw_cyclonedds_cpp`. Record RAM and round-trip
   times.
5. The lending contract and interface. Measure the same image again; the difference is the case for keeping it.
6. Borrowed socket receive, if step 5's numbers leave the rx copy worth removing.

## Bench

The EVK's DAPLink USB port is the only connection the board needs: it flashes and debugs the RT1021 over CMSIS-DAP,
bridges LPUART1 to a virtual serial port, and powers the board with the default jumpers. The RT1021's own USB port
stays unused; this board's console is LPUART1 (`ROSETTE_IMXRT_USB_CONSOLE` off). Flashing uses pyOCD
(`pyocd flash -t mimxrt1020`) or a copy to the DAPLink's mass-storage disk.
