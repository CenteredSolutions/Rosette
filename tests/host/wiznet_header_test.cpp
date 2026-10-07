// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file wiznet_header_test.cpp
 * @brief Host checks for the WIZnet chip traits' SPI frame encoding.
 *
 * These pin the exact bytes `WiznetDevice` puts on the wire for the W5100S,
 * W6100 and W6300 register traits — the address/control phase of every SPI
 * transaction. The W5500 vectors are hand-derived from the W5500 datasheet
 * §2.2 (SPI frame) and §3 (block select bits). The W5100S vectors are a regression guard for the
 * `device.hpp` refactor that added the per-part reset / buffer-addressing / interrupt-ack seams;
 * the W61xx/W63xx vectors are hand-derived from the WIZnet io(6)Library `WIZCHIP_READ` /
 * `WIZCHIP_WRITE` and register `#define`s.
 *
 * Build and run standalone:
 * @code
 *   c++ -std=c++17 -Wall -Wextra -DROSETTE_OS_POSIX -DROSETTE_CHIP_HOST -I include \
 *       tests/host/wiznet_header_test.cpp -o /tmp/wiznet_header_test && /tmp/wiznet_header_test
 * @endcode
 * or through CMake: `cmake -S tests/host -B build/tests-host && ctest --test-dir build/tests-host`.
 */

#include "fake_clock.hpp"

#include "ports/net/wiznet/w5100s.hpp"
#include "ports/net/wiznet/w5500.hpp"
#include "ports/net/wiznet/w6x00.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <initializer_list>
#include <string>
#include <vector>

namespace wiz = rosette::net::wiznet;

namespace {

int g_failures = 0;

void check(bool condition, const char* what)
{
    std::printf("[%s] %s\n", condition ? "PASS" : "FAIL", what);
    if (!condition) {
        ++g_failures;
    }
}

/// A `Bus` that records every transaction instead of touching hardware.
struct RecordingBus {
    struct Xfer {
        bool read;
        std::vector<uint8_t> prefix;
        std::vector<uint8_t> data;
    };

    std::vector<Xfer> log;
    std::deque<uint8_t> next_reads;  ///< bytes handed back by read(); 0 when drained

    struct Lock { ~Lock() {} };  // non-trivial dtor: mirrors the real bus guard
    Lock acquire() noexcept { return {}; }

    void write(const uint8_t* prefix, std::size_t plen,
               const uint8_t* data, std::size_t len)
    {
        log.push_back({false, {prefix, prefix + plen}, {data, data + len}});
    }

    void read(const uint8_t* prefix, std::size_t plen,
              uint8_t* data, std::size_t len)
    {
        log.push_back({true, {prefix, prefix + plen}, {}});
        for (std::size_t i = 0; i < len; ++i) {
            if (next_reads.empty()) {
                data[i] = 0u;
            } else {
                data[i] = next_reads.front();
                next_reads.pop_front();
            }
        }
    }
};

bool prefix_is(const RecordingBus::Xfer& xfer, bool read,
               std::initializer_list<uint8_t> bytes)
{
    return xfer.read == read &&
           xfer.prefix.size() == bytes.size() &&
           std::equal(bytes.begin(), bytes.end(), xfer.prefix.begin());
}

template<typename Chip>
void check_common_frames(const char* name)
{
    using Device = wiz::WiznetDevice<Chip, RecordingBus>;

    // set_mac_address -> one write to the MAC register.
    {
        RecordingBus bus;
        Device dev{bus};
        dev.set_mac_address({0x02, 0x08, 0xDC, 0x01, 0x02, 0x03});
        check(bus.log.size() == 1 && !bus.log[0].read &&
                  bus.log[0].data == std::vector<uint8_t>{0x02, 0x08, 0xDC, 0x01, 0x02, 0x03},
              (std::string(name) + ": set_mac_address issues one 6-byte write").c_str());
    }

    // configure_ipv4 -> gateway, subnet, then IP.
    {
        RecordingBus bus;
        Device dev{bus};
        dev.configure_ipv4({192, 168, 1, 50}, {255, 255, 255, 0}, {192, 168, 1, 1});
        check(bus.log.size() == 3 && !bus.log[0].read && !bus.log[1].read && !bus.log[2].read,
              (std::string(name) + ": configure_ipv4 issues three writes").c_str());
    }
}

/// The W5100S's frames { 0xF0 | 0x0F, addr_hi, addr_lo } and its shared-region buffer layout.
void check_w5100s_frames() {
    // ── W5100S: regression guard (bytes must not move) ──────────────────────
    {
        using Device = wiz::WiznetDevice<wiz::W5100S, RecordingBus>;
        RecordingBus bus;
        Device dev{bus};

        dev.hardware_reset();                       // MR.RST
        dev.set_mac_address({1, 2, 3, 4, 5, 6});    // SHAR @ 0x0009

        check(bus.log.size() == 2, "W5100S: reset + set_mac = two transactions");
        check(prefix_is(bus.log[0], /*read=*/false, {0xF0, 0x00, 0x00}),
              "W5100S reset frame = write {0xF0,0x00,0x00}");
        check(prefix_is(bus.log[1], /*read=*/false, {0xF0, 0x00, 0x09}),
              "W5100S SHAR write frame = {0xF0,0x00,0x09}");
    }

    // ── W5100S buffer layout: per-socket spans of one shared region ─────────
    {
        using Device = wiz::WiznetDevice<wiz::W5100S, RecordingBus>;

        RecordingBus sock_bus;
        sock_bus.next_reads = {0x51};  // chip-version read; command polls then read 0
        Device sock{sock_bus};
        sock.init_socket_mode({1, 2, 3, 4, 5, 6});
        // TX: 2 KiB spans from 0x4000; RX: the default 4/2/1/1 KiB from 0x6000.
        check(sock.tx_buffer_address(0) == 0x4000u && sock.tx_buffer_address(2) == 0x5000u,
              "W5100S socket-mode TX spans: socket 0 @ 0x4000, socket 2 @ 0x5000");
        check(sock.rx_buffer_address(1) == 0x7000u && sock.rx_buffer_address(3) == 0x7C00u,
              "W5100S socket-mode RX spans follow 4/2/1/1: socket 1 @ 0x7000, socket 3 @ 0x7C00");
        check(sock.socket_tx_capacity(1) == 2048u && sock.socket_rx_capacity(1) == 2048u,
              "W5100S socket-mode capacity cached at 2 KiB (no per-packet SPI read)");

        RecordingBus raw_bus;
        raw_bus.next_reads = {0x51};
        Device raw{raw_bus};
        raw.init_macraw({1, 2, 3, 4, 5, 6});
        check(raw.tx_buffer_address(0) == 0x4000u && raw.rx_buffer_address(0) == 0x6000u,
              "W5100S MACRAW: socket 0 owns the whole region (span 0)");
        check(raw.socket_tx_capacity(0) == 8192u && raw.socket_rx_capacity(0) == 8192u,
              "W5100S MACRAW: socket 0 capacity cached at the full 8 KiB");
    }
}

/// The W5500's frames { addr_hi, addr_lo, (BSB<<3)|(RWB<<2)|OM }, reset, and buffer layout.
void check_w5500_frames() {
    check_common_frames<wiz::W5500>("W5500");

    using C = wiz::W5500;
    static_assert(C::common_version() == 0x003900u, "W5500 VERSIONR packed addr");
    static_assert(C::common_link_status() == 0x002E00u, "W5500 PHYCFGR packed addr");
    static_assert(C::socket_mode(1) == 0x000005u, "W5500 Sn_MR(1) packed addr");
    static_assert(C::socket_rx_buffer_size(7) == 0x001E1Du, "W5500 Sn_RXBUF_SIZE(7) packed addr");
    static_assert(!C::has_hw_ipv6() && !C::has_socketless_arp(), "W5500: no IPv6, no SLCR");

    auto rd_version = C::make_header<true>(C::common_version());
    check(rd_version.length == 3 && rd_version.bytes[0] == 0x00 && rd_version.bytes[1] == 0x39
              && rd_version.bytes[2] == 0x00,
          "W5500 read VERSIONR frame = {0x00,0x39,0x00}");

    auto wr_sn_mr = C::make_header<false>(C::socket_mode(1));  // block 5 -> (5<<3)|4 = 0x2C
    check(wr_sn_mr.bytes[0] == 0x00 && wr_sn_mr.bytes[1] == 0x00 && wr_sn_mr.bytes[2] == 0x2C,
          "W5500 write Sn_MR(1) frame = {0x00,0x00,0x2C}");

    auto wr_txbuf = C::make_header<false>(C::tx_buffer_block_addr(2, 0x0100));  // block 10
    check(wr_txbuf.bytes[0] == 0x01 && wr_txbuf.bytes[1] == 0x00 && wr_txbuf.bytes[2] == 0x54,
          "W5500 write TX-buffer frame = {0x01,0x00,0x54}");

    auto rd_rxbuf = C::make_header<true>(C::rx_buffer_block_addr(0, 0x0040));  // block 3
    check(rd_rxbuf.bytes[0] == 0x00 && rd_rxbuf.bytes[1] == 0x40 && rd_rxbuf.bytes[2] == 0x18,
          "W5500 read RX-buffer frame = {0x00,0x40,0x18}");

    RecordingBus bus;
    wiz::WiznetDevice<C, RecordingBus> dev{bus};
    dev.hardware_reset();
    check(bus.log.size() == 1 && prefix_is(bus.log[0], false, {0x00, 0x00, 0x04})
              && bus.log[0].data == std::vector<uint8_t>{0x80},
          "W5500 reset() = MR.RST write {0x00,0x00,0x04} <- 0x80");

    RecordingBus sock_bus;
    sock_bus.next_reads = {0x04};  // VERSIONR; command polls then read 0
    wiz::WiznetDevice<C, RecordingBus> sock{sock_bus};
    check(!std::holds_alternative<wiz::WiznetDevice<C, RecordingBus>::InvalidChipVersion>(
              sock.init_socket_mode({1, 2, 3, 4, 5, 6})),
          "W5500 socket mode accepts VERSIONR 0x04");
    check(sock.socket_rx_capacity(0) == 8192u && sock.socket_rx_capacity(1) == 2048u
              && sock.socket_rx_capacity(7) == 1024u && sock.socket_tx_capacity(3) == 2048u,
          "W5500 default layout: RX 8/2/1.. KiB, TX 2 KiB per socket");
}

}  // namespace

int main() {
    check_w5100s_frames();
    check_common_frames<wiz::W5100S>("W5100S");
    check_common_frames<wiz::W6100>("W6100");
    check_common_frames<wiz::W6300>("W6300");
    check_w5500_frames();

    // ── IPv4 conflict bit + hardware-IPv6 gate (address bring-up) ──────────
    {
        static_assert(wiz::W5100S::common_ip_conflict_mask() == 0x80u, "W5100S IR_CONFLICT");
        static_assert(wiz::W6100::common_ip_conflict_mask() == 0x04u, "W6100 IR_IPCONF");
        static_assert(wiz::W6300::common_ip_conflict_mask() == 0x04u, "W6300 IR_IPCONF");
        static_assert(!wiz::W5100S::has_hw_ipv6(), "W5100S has no IPv6");
        static_assert(wiz::W6100::has_hw_ipv6() && wiz::W6300::has_hw_ipv6(), "W61xx/W63xx dual stack");
        static_assert(wiz::W6100::socket_dest_ip6(1) == 0x013005u, "W6100 Sn_DIP6R(1) packed addr");
        static_assert(wiz::W6100::common_ip6_link_local() == 0x414000u, "W6100 LLAR packed addr");
        check(true, "IPv4 conflict-bit traits pinned");
    }

    // ── W6100 frame encoding: { addr_hi, addr_lo, (BSB<<3)|RW } ─────────────
    {
        using C = wiz::W6100;
        static_assert(C::common_version() == 0x000000u, "W6100 CIDR packed addr");
        static_assert(C::common_mac_address() == 0x412000u, "W6100 SHAR packed addr");
        static_assert(C::socket_mode(1) == 0x000005u, "W6100 Sn_MR(1) packed addr");
        static_assert(C::tx_buffer_block_addr(2, 0x0100) == 0x01000Au, "W6100 TX buf blk 10");

        auto rd_cidr = C::make_header<true>(C::common_version());
        check(rd_cidr.length == 3 &&
                  rd_cidr.bytes[0] == 0x00 && rd_cidr.bytes[1] == 0x00 && rd_cidr.bytes[2] == 0x00,
              "W6100 read CIDR frame = {0x00,0x00,0x00}");

        auto wr_shar = C::make_header<false>(C::common_mac_address());
        check(wr_shar.bytes[0] == 0x41 && wr_shar.bytes[1] == 0x20 && wr_shar.bytes[2] == 0x04,
              "W6100 write SHAR frame = {0x41,0x20,0x04}");

        auto wr_sn_mr = C::make_header<false>(C::socket_mode(1));  // block 5 -> (5<<3)|4 = 0x2C
        check(wr_sn_mr.bytes[0] == 0x00 && wr_sn_mr.bytes[1] == 0x00 && wr_sn_mr.bytes[2] == 0x2C,
              "W6100 write Sn_MR(1) frame = {0x00,0x00,0x2C}");

        auto wr_txbuf = C::make_header<false>(C::tx_buffer_block_addr(2, 0x0100));  // block 10 -> (10<<3)|4 = 0x54
        check(wr_txbuf.bytes[0] == 0x01 && wr_txbuf.bytes[1] == 0x00 && wr_txbuf.bytes[2] == 0x54,
              "W6100 write TX-buffer frame = {0x01,0x00,0x54}");

        // Per-socket buffer blocks: the address is span-independent, so it is
        // valid with no init_*() call. Socket 3 RX block = 3 + 4*3 = 15.
        RecordingBus blk_bus;
        wiz::WiznetDevice<C, RecordingBus> blk{blk_bus};
        check(blk.rx_buffer_address(3) == 0x00000Fu, "W6100 socket 3 RX buffer block @ 0x0F");

        // reset(): unlock chip, SYCR0.RST, unlock net.
        RecordingBus bus;
        wiz::WiznetDevice<C, RecordingBus> dev{bus};
        dev.hardware_reset();
        check(bus.log.size() == 3 &&
                  prefix_is(bus.log[0], false, {0x41, 0xF4, 0x04}) &&   // CHPLCKR write
                  prefix_is(bus.log[1], false, {0x20, 0x04, 0x04}) &&   // SYCR0 write
                  prefix_is(bus.log[2], false, {0x41, 0xF5, 0x04}),     // NETLCKR write
              "W6100 reset() = CHPLCKR, SYCR0, NETLCKR writes");
        check(bus.log[0].data == std::vector<uint8_t>{0xCE} &&
                  bus.log[1].data == std::vector<uint8_t>{0x00} &&
                  bus.log[2].data == std::vector<uint8_t>{0x3A},
              "W6100 reset() payloads = 0xCE, 0x00, 0x3A");
    }

    // ── W6300 frame encoding: { OP-code, addr_hi, addr_lo } (op-code first) ─
    {
        using C = wiz::W6300;
        static_assert(C::socket_status(2) == 0x003009u, "W6300 Sn_SR(2) packed addr");
        static_assert(C::rx_buffer_block_addr(0, 0x0040) == 0x004003u, "W6300 RX buf blk 3");

        auto rd_sn_sr = C::make_header<true>(C::socket_status(2));  // block 9, op-code = 9
        check(rd_sn_sr.bytes[0] == 0x09 && rd_sn_sr.bytes[1] == 0x00 && rd_sn_sr.bytes[2] == 0x30,
              "W6300 read Sn_SR(2) frame = {0x09,0x00,0x30}");

        auto wr_sipr = C::make_header<false>(C::common_ip_address());  // block 0, op-code = 0x20
        check(wr_sipr.bytes[0] == 0x20 && wr_sipr.bytes[1] == 0x41 && wr_sipr.bytes[2] == 0x38,
              "W6300 write SIPR frame = {0x20,0x41,0x38}");

        auto rd_rxbuf = C::make_header<true>(C::rx_buffer_block_addr(0, 0x0040));  // block 3, op-code = 3
        check(rd_rxbuf.bytes[0] == 0x03 && rd_rxbuf.bytes[1] == 0x00 && rd_rxbuf.bytes[2] == 0x40,
              "W6300 read RX-buffer frame = {0x03,0x00,0x40}");

        RecordingBus bus;
        wiz::WiznetDevice<C, RecordingBus> dev{bus};
        dev.hardware_reset();
        check(bus.log.size() == 3 &&
                  prefix_is(bus.log[0], false, {0x20, 0x41, 0xF4}) &&
                  prefix_is(bus.log[1], false, {0x20, 0x20, 0x04}) &&
                  prefix_is(bus.log[2], false, {0x20, 0x41, 0xF5}),
              "W6300 reset() = CHPLCKR, SYCR0, NETLCKR writes (op-code first)");
    }

    std::printf("\n%s\n", g_failures == 0 ? "ALL PASS" : "FAILURES");
    return g_failures == 0 ? 0 : 1;
}
