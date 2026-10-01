# WIZnet W6100 / W6300 support

Status: W6100 and W6300 over 1-bit SPI, IPv4 and hardware IPv6. The W6100 is
hardware-verified on the W6100-EVB-Pico (RP2040): DHCP, IPv6 link-local and
SLAAC global addresses, ping and UDP echo over each (`examples/wiznet_udp_echo`,
`tools/wiznet_udp_echo_probe.py`). The W6300 builds but has not run on
hardware. 4-bit QSPI for the W6300 is scoped below and not implemented.

## Why

The register-level WIZnet driver (`include/rosette/net/wiznet/`) was built
around one part, the W5100S. The W6100 and W6300 are the current-generation
WIZnet controllers — 8 sockets, larger socket memory, IPv4/IPv6 — and the
W6300-EVB-Pico2 (RP2350) is the natural successor board. This change adds both
parts as compile-time `Chip` traits so a board selects one with
`#define ROSETTE_WIZNET_CHIP W6100` (or `W6300`), reusing the whole
`WiznetDevice` / `WiznetSocketProvider` / `WiznetLink` / `Dhcp*` stack unchanged.

Rosette does not use the WIZnet ioLibrary: the register-level driver is
header-only and is the only code that talks to the chip. The io(6)Library
sources are cited as references for chip behavior.

## SPI frame formats

Addresses in the traits are packed into 24 bits: `(offset << 8) | block`, where
`block` is the raw block-select number:

| block | value |
|---|---|
| common registers | `0` |
| socket N registers | `1 + 4N` |
| socket N TX buffer | `2 + 4N` |
| socket N RX buffer | `3 + 4N` |

`Chip::make_header<IsRead>(addr)` turns that into the on-wire frame. Both parts
send a 3-byte address/control phase then the data phase; they differ in layout
(from the WIZnet io(6)Library `WIZCHIP_READ` / `WIZCHIP_WRITE`):

**W6100** — `{ addr[15:8], addr[7:0], (BSB << 3) | RWB }`, `RWB` = `0x00` read /
`0x04` write, operation-mode bits (VDM) = 0. `BSB` here is the raw block number
shifted left 3.

**W6300** — `{ OP-code, addr[15:8], addr[7:0] }`, OP-code first.
`OP-code = QMODE[7:6] | RWB[5] | BSB[4:0]`, `RWB` = `0x00` read / `0x20` write,
`QMODE` = 0 for single-lane. `BSB` is the raw block number, unshifted.

Frame encoding is pinned by `tests/host/wiznet_header_test.cpp`.

## Driver seams added to `device.hpp`

All default to the exact prior W5100S behaviour:

* `Chip::reset(write_u8)` — the W5100S pulses `MR.RST`; the W61xx/W63xx unlock
  the chip (`CHPLCKR <- 0xCE`), pulse `SYCR0.RST`, then unlock the network
  registers (`NETLCKR <- 0x3A`) so `SHAR`/`SIPR`/`GAR`/`SUBR` are writable for
  the rest of bring-up. The chip-version read that immediately follows covers
  the post-reset settle time.
* `Chip::socket_interrupt_clear(s)` / `Chip::common_interrupt_clear()` — the
  W5100S clears `Sn_IR` / `IR` in place (write-1); the W61xx/W63xx have separate
  write-only `Sn_IRCLR` (`+0x28`) / `IRCLR` (`0x2108`) registers.
* `Chip::per_socket_buffer_blocks()` — `false` for the W5100S (one shared TX
  region + one shared RX region, `Chip::{tx,rx}_buffer_base()`), `true` for the
  W61xx/W63xx (`Chip::{tx,rx}_buffer_block_addr(s, off)`, each socket's buffer
  its own block from 0). The device caches the per-socket buffer layout in one
  `buffers_[socket]` array (`{tx,rx}_offset` + `{tx,rx}_size`), filled once by
  `init_*()` by **summing the sizes it actually assigns** (even split in socket
  mode; the whole region to socket 0 in MACRAW). `socket_{rx,tx}_capacity()`
  reads this cache instead of an `Sn_{RX,TX}BUF_SIZE` SPI read on every packet;
  only the *free*-space register (`Sn_TX_FSR`) is still polled live.

`Chip::auto_wrap()` (already present) is `true` for the W61xx/W63xx: their
buffer hardware wraps at the socket buffer size, so `ring_read` / `ring_write`
hand the raw pointer over in one transfer instead of splitting at the boundary.

## Socket memory

| part | TX / RX total | per socket (8) |
|---|---|---|
| W6100 | 16 KiB each | 2 KiB |
| W6300 | 32 KiB each | 4 KiB |

## UDP receive header

The chip stores a packet-info header in front of every received datagram, and
its layout differs by part, so `Chip::read_udp_rx_header()` decodes it:

| part | layout |
|---|---|
| W5100S | sender IPv4 (4), sender port (2), payload length (2) |
| W6100 / W6300 | info + length (2: info in bits 15..11, length in 10..0), sender address (4, or 16 when info bit 7 marks IPv6), sender port (2) |

IPRAW6 sockets use the W61xx layout without the port
(`Chip::read_ipraw_rx_header()`). `tests/host/wiznet_udp_socket_test.cpp` runs
the driver's receive and send paths for all three parts against a register-file
model of the chip (`tests/host/fake_wiznet_chip.hpp`).

## IPv6

The W6100 and W6300 run IPv6 in silicon: with a link-local address in LLAR, a
global address in GUAR (with its prefix mask in SUB6R) and a default router in
GA6R, the chip answers neighbor solicitations and echo requests itself and
routes UDP6 traffic. The driver adds:

* **UDP6 sockets** — `WiznetSocketProvider::open(SocketDomain::ipv6)` binds a
  socket in `Sn_MR` UDP6 mode (`0x0A`); sends use `SEND6` (`0xA0`) to
  `Sn_DIP6R`. The chip picks the source address from the destination's scope.
* **Address configuration** — `WiznetSlaacProvider`
  (`slaac_provider.hpp`), started by `board_network` on these parts. Duplicate
  address detection uses the chip's socket-less `SLCR.NS` command; router
  solicitation uses `SLCR.RS`.
* **Router advertisements are read by the host.** The chip latches the prefix
  option of each RA in PLR / PFR / VLTR / PLTR / PAR, but decodes it 8 octets
  past where the option starts, as if an 8-octet option always came first. A
  router that sends the prefix option first (legal: RFC 4861 §4.2 fixes no
  order) produces a /81 prefix with no autonomous flag and the prefix's own
  octets in PLTR. The provider therefore diverts RAs with `ICMP6BLKR.RA`,
  receives them on an IPRAW6 socket (`Sn_MR` `0x0B`, next header 58) that it
  holds only while soliciting, and decodes them with
  `rosette/net/ipv6_router_advert.hpp`. That also yields the router's address
  for GA6R, which the latch registers do not carry. The W6300 shares the
  W6100's IPv6 block and is assumed to latch the same way.

## Known limitations / follow-ups

* **Chip-version check** compares one byte (`CIDR` MSB: `0x61` / `0x63`),
  assuming big-endian register layout. Widen to a 16-bit `CIDR` compare if a
  board reports a spurious mismatch.
* **IGMP** — `Sn_MR_MC` (bit 5) picks v1 vs v2; the driver leaves it 0 (v2).
  IGMPv3 (source-specific multicast) is not available on these parts.
* **INT pin** — `SYCR1.IEN` resets to 1, and the socket RECV interrupt
  enabled through `SIMR` / `Sn_IMR` asserts INT; on the W6100-EVB-Pico the
  echo round trip matches the chip's own ping time, well below the firmware's
  1 s idle wait.
* **Multicast from a unicast socket** — the W6x00 has no `SEND_MAC`; whether the
  chip addresses an IPv4 multicast destination correctly on its own is not yet
  checked on hardware (RTPS SPDP depends on it). IPv6 multicast sockets are not
  supported.
* **W6300** — not run on hardware (no EVB on hand). `tests/host/wiznet_header_test.cpp`
  pins its frame encoding and the register-file tests cover its socket paths.

## Phase 2 — 4-bit QSPI for the W6300

The W6300's host interface is QSPI (single / dual / quad lane). On RP2040 /
RP2350 the hardware QSPI block is dedicated to XIP flash, so a wide driver needs
a PIO state-machine program (WIZnet's `WIZnet-PICO-C` / `wiznet-pio-spi` is the
reference).

The bus seam is already in place:

* `board.hpp` selects `BoardBus` from `ROSETTE_WIZNET_BUS_QSPI` in
  `board_config.h` — `Rp2SpiBus` (hardware SPI) otherwise, `Rp2QspiBus` when
  set.
* `ports/chip/rp2/rp2_qspi_bus.hpp` is a **single-lane** `Rp2QspiBus` today: GPIO
  bit-bang, SPI mode 0, same `acquire()` / `write(prefix,…)` / `read(prefix,…)`
  surface. The `w6300-evb-pico2` board uses it (the EVB routes the W6300 onto a
  non-hardware-SPI pin set, so even 1-bit needs bit-bang or PIO).
* A W6300 frame's OP-code + address phase is always single-lane; only the data
  phase goes wide. That boundary is exactly the `prefix` / `data` split in the
  transfer methods, so phase 2 is contained to `Rp2QspiBus::transfer_data()`
  (plus a `QMODE` bit in `W6300::make_header` and the PIO program). Neither
  `device.hpp` nor the traits' register maps change.
