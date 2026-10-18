# i.MX RT ENET as a MacDevice, under FreeRTOS+TCP

The MIMXRT1020-EVK carries an i.MX RT1021 (Cortex-M7, 500 MHz, 256 KiB FlexRAM) with the chip's own 10/100 ENET
MAC, a KSZ8081RNB PHY over RMII, 8 MiB of QSPI NOR, and 32 MiB of SDRAM. This board runs Rosette with the IP stack
on the microcontroller: FreeRTOS+TCP over the ENET, where the WIZnet boards run it in the network chip.

The ENET reaches FreeRTOS+TCP through Rosette's MAC contracts, not through FreeRTOS+TCP's own
`NXP1060/NetworkInterface.c`: `rosette::net::ZeroCopyMacDevice` (`include/rosette/net/zero_copy_mac.hpp`), which moves
frames in the IP stack's own buffers, or the copying `rosette::net::MacDevice` (`include/rosette/net/mac.hpp`). The
IP-stack port (`ports/net/freertos_tcp/`) then stays free of chip code, and the same ENET driver serves lwIP when
`ports/net/lwip/` exists.

## Where the bytes go

A received RTPS datagram, and the buffers it passes through, on each design:

| Design | Receive path | Buffer pools |
|---|---|---|
| FreeRTOS+TCP `NXP1060` driver | DMA → driver ring → `memcpy` → network buffer → `memcpy` → rmw `rx_buffer` → parse | driver RX ring (10 × 1536), driver TX ring (3 × 1536), stack pool (10 × 1550): about 35 KiB |
| Copying `MacDevice` (the contract as it is) | the same two copies | the same three pools, sized by the board: 4 RX, 2 TX, 8 stack is about 21 KiB |
| `ZeroCopyMacDevice` | DMA → network buffer → parse in place | the stack pool alone; the RX ring holds posted network buffers: 10 stack buffers is 15 KiB |

At 100 Mbit/s a 1.5 KiB frame occupies the wire for 120 µs, and copying it on a 500 MHz M7 from DTCM takes one to
two microseconds. The copies cost little time. What zero copy removes is the second and third pool: the RAM the
driver keeps for frames the stack already has room for. On a 256 KiB part that is the reason to do it.

Transmit is zero-copy at the driver level only. FreeRTOS+TCP with `ipconfigZERO_COPY_TX_DRIVER` hands the network buffer
to the driver, the transmit descriptor points at it, and the interface releases it when the ENET reports the frame
sent. The RTPS layer keeps building messages into its own buffer pool and `send_to()` copies once into a network
buffer: a reliable writer must keep each sample for repair after the frame has gone, so the network buffer cannot be
the history's storage.

## The control half: MacLink

`MacLink` is what every MAC has whatever its data path: `initialise()`, the station address, `max_frame_size()`,
`phy_status()`, receive notification, and the multicast filter. `MacDevice` is `MacLink` plus whole-frame copying
(`send_frame()`, `peek_frame_size()`, `receive_frame()`), the shape a SPI controller in MACRAW mode has;
`ZeroCopyMacDevice` is `MacLink` plus buffer exchange. A zero-copy device has no frame storage to copy through, so it
does not implement the copying operations rather than implementing them badly.

Three of the control operations exist for an on-chip MAC, and the WIZnet MACRAW device implements them trivially.

1. **Receive notification.** The receive task sleeps on a `platform::Event`. A SPI controller's board signals it
   from a GPIO edge (`NetworkInterface::attach_rx_interrupt(pin)`); the ENET signals it from its own NVIC
   interrupt. `MacLink::set_receive_event(platform::Event*)`, the counterpart of
   `SocketProvider::set_receive_event()`, hands the device the event, and `ENET_IRQHandler` calls
   `signal_from_isr()` on it. The ENET interrupt's priority is numerically at least
   `configMAX_SYSCALL_INTERRUPT_PRIORITY`, the FreeRTOS rule for an interrupt that calls a `FromISR` API. The
   WIZnet device ignores the event, and its board keeps attaching the INT pin.
2. **Multicast filter.** The ENET drops multicast frames whose CRC-32 hash bucket is not set in `GAUR`/`GALR`, so
   SPDP on 239.255.0.1 never arrives unless someone programs it. FreeRTOS+TCP's `pfAddAllowedMAC` /
   `pfRemoveAllowedMAC` reach the device as `MacLink::add_multicast(const MacAddress&)` and `remove_multicast()`
   (`InterfaceBinding::add_allowed_mac()`); the ENET driver keeps a reference count per hash bucket (64 octets), because two groups can share a bucket and
   leaving one must not deafen the other.
3. **Link changes.** The ENET's speed and duplex (`ECR[SPEED]`, `RCR[RMII_10T]`, `TCR[FDEN]`) must match what the
   PHY negotiated. `phy_status()` reads the PHY over MDIO and reprograms the MAC when the result changes, and the
   receive task calls it on each idle wake (`kRxIdleWaitMs`) and calls `FreeRTOS_NetworkDown()` when the link drops.

## The zero-copy contract

`rosette::net::ZeroCopyMacDevice` adds buffer exchange to `MacLink`. The consumer owns every buffer; the device only
holds one from the call that posts it until the call that hands it back. This is the ownership model of
Linux AF_XDP (`Documentation/networking/af_xdp.rst`): `post_receive_buffer()` is its fill ring,
`take_received_frame()` its RX ring, `post_send_frame()` its TX ring, and `take_sent_frame()` its completion ring;
"post" is the RDMA verbs word (`ibv_post_recv()`, `ibv_post_send()`) for giving a buffer to a queue.

```cpp
struct ZeroCopyFrame {
    uint8_t* data;       // first octet of the Ethernet header
    std::size_t length;  // octets of frame, without FCS
    void* owner;         // the consumer's handle: a NetworkBufferDescriptor_t*, or an lwIP pbuf*
};

virtual std::size_t frame_offset() const noexcept = 0;                      // padding the DMA writes first
virtual std::size_t buffer_alignment() const noexcept = 0;                  // of data - frame_offset()
virtual std::size_t buffer_bytes() const noexcept = 0;                      // a receive buffer's writable octets
virtual bool post_receive_buffer(uint8_t* data, void* owner) noexcept = 0;  // false: ring full or misaligned
virtual bool take_received_frame(ZeroCopyFrame& frame) noexcept = 0;        // false: nothing received
virtual bool post_send_frame(const ZeroCopyFrame& frame) noexcept = 0;      // false: the TX ring is full
virtual bool take_sent_frame(void*& owner) noexcept = 0;                    // false: nothing finished
```

**Layout.** The ENET's DMA wants each buffer on a 64-octet boundary, and FreeRTOS+TCP wants the IP header, 14
octets into the frame, on a word boundary. The ENET's SHIFT16 accelerators (`RACC`, `TACC`) reconcile the two: the
DMA writes, and reads, two octets of padding first, so the frame starts two octets into the aligned region. The
reported receive length and the programmed transmit length both count those two octets; the device hides that. On
the bench, a 64-octet UDP echo leaves the board as a 106-octet frame and a 1472-octet one as 1514, each exactly
its headers plus payload.

**Where the buffers live.** FreeRTOS+TCP's BufferAllocation_2 takes buffers from a cached heap with a 10-octet
header at an arbitrary alignment, which no DMA can use. The zero-copy board selects BufferAllocation_1 instead and
supplies the storage itself: `ports/net/freertos_tcp/zero_copy_buffer_pool.hpp` lays every network buffer out in the
device's layout, in DTCM (uncached, DMA-reachable), with the stack's back-pointer and IP-version byte in the
octets before each region, where the DMA never writes. `ROSETTE_FREERTOS_TCP_ZERO_COPY` in `board.cmake` selects
BufferAllocation_1 and `ipconfigZERO_COPY_TX_DRIVER`.

**The descriptors are driven directly.** fsl_enet has zero-copy calls, but `ENET_GetRxFrame()` allocates the
replacement buffer inside the receive call, and `ENET_StartTxFrame()` reclaims transmit buffers in the interrupt.
FreeRTOS+TCP's pool cannot be used from an interrupt, and the zero-copy contract wants the consumer to choose the
replacement. So `ImxrtEnetZeroCopyDevice` (`ports/chip/imxrt/imxrt_enet_zero_copy_device.hpp`) owns the two rings,
about 12 octets per slot including the owner handle, and fsl_enet only brings the MAC up, through
`ImxrtEnetLink`, which both ENET devices share. The transmit-done interrupt wakes the receive task, which returns
sent buffers to the pool.

**The consumer** is `ports/net/freertos_tcp/zero_copy_interface.hpp`, beside the copying `interface.hpp`; what the two
share (the descriptor, the receive wake-up, the link, the multicast filter) is `interface_binding.hpp`. Its receive
task posts a fresh network buffer for each frame it takes before handing the frame to the IP task. When the pool has
none free, it drops the frame and posts the same buffer back, so the receive ring never runs empty. On the bench, a
5000-frame ping flood at 1400 octets loses nothing, and after a 5 s UDP blast that saturates the link the first echo
returns 3 ms later.

**Pool exhaustion.** Four of the ten buffers always sit in the receive ring, so a flood exhausts the rest. FreeRTOS+TCP
defaults a socket's send to wait forever for a buffer. The echo task then deadlocked: its socket's queue held the
remaining buffers, and only that task drains it. `FreeRTOSIPConfig.h` bounds sends to
`ipconfigUDP_MAX_SEND_BLOCK_TIME_TICKS` (`ipconfigSOCK_DEFAULT_SEND_BLOCK_TIME`) and a socket's queue to four
datagrams (`ipconfigUDP_MAX_RX_PACKETS`), so a send fails instead of waiting forever and one socket cannot take the
whole pool.

`ImxrtEnetMacDevice` stays beside it: `-DROSETTE_IMXRT_ENET_ZERO_COPY=OFF` builds the EVK with the copying device
(`boards/mimxrt1020-evk/enet_copying_interface.cpp`), so the two can be measured against each other.

## Measured

`udp_echo` on the MIMXRT1020-EVK, the same workload on each image (the echo probe, a 5000-frame ping flood, a 5 s
UDP blast, the probe again), RAM read from the board afterwards:

| | copying | zero copy |
|---|---|---|
| DTCM `.bss` | 16,348 B | 22,556 B |
| FreeRTOS heap at its peak | 31,360 B | 16,000 B |
| RAM at the peak | 47,708 B | 38,556 B |
| network buffers in the heap at the peak | 15,360 B | none (in DTCM, 15,424 B) |
| UDP echo round trip, median | 1.0 to 1.1 ms | 0.9 to 1.0 ms |

Zero copy saves the copying driver's six frame buffers (9 KiB). The heap peak is now the tasks and kernel objects
alone, so the board's 64 KiB heap has room to shrink.

## Order of work

1. Board and chip: `boards/mimxrt1020-evk` (MIMXRT1021DAG5A, evkmimxrt1020 clocks, IS25LP064A boot block, LPUART1
   console through the on-board debug probe), FlexRAM layout, and the existing bare-metal superloop blinking and
   printing.
2. FreeRTOS on imxrt: the kernel's `ARM_CM4F` port (the RT1021's M7 is not the r0p1 revision the `ARM_CM7/r0p1` port
   works around), SysTick beside the PIT time base, ENET interrupt priority at or below
   `configMAX_SYSCALL_INTERRUPT_PRIORITY`.
3. `ports/net/freertos_tcp/board_network.cpp`: the open item in `ports_move_plan.md` that lets any
   FreeRTOS+TCP image link.
4. The ENET `MacDevice`, copying, with the three contract additions. Bench: DHCP, ping, `udp_echo`'s
   counterpart, then `led_files` against `rmw_fastrtps_cpp` and `rmw_cyclonedds_cpp`. Record RAM and round-trip
   times.
5. The zero-copy contract and interface. Measure the same image again; the difference is the case for keeping it.
6. Borrowed socket receive, if step 5's numbers leave the rx copy worth removing.

## Bench

The EVK's DAPLink USB port is the only connection the board needs: it flashes and debugs the RT1021 over CMSIS-DAP,
bridges LPUART1 to a virtual serial port, and powers the board with the default jumpers. The RT1021's own USB port
stays unused; this board's console is LPUART1 (`ROSETTE_IMXRT_USB_CONSOLE` off). Flashing uses pyOCD
(`pyocd flash -t mimxrt1020`) or a copy to the DAPLink's mass-storage disk.
