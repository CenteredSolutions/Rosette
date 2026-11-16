# Network Configuration

What a Rosette board does on the network, what it needs from the network, and
which settings change it. Read it before you put a board on a LAN you share
with other ROS 2 machines, and when a board is on the network but not in the
ROS graph.

How the address layer is built (providers, managers, conflict handling) is in
[design/network_address_bringup.md](design/network_address_bringup.md); the
W6100 / W6300 driver and its IPv6 support are in
[design/wiznet_w61xx_w63xx.md](design/wiznet_w61xx_w63xx.md).

## What a board does at boot

Every WIZnet firmware calls `net::bring_up_board_network(hostname)`, or
`net::bring_up_board_network(hostname, config)` with its own `NetworkConfig`
(see [Choosing the addressing methods](#choosing-the-addressing-methods)),
before `rcl_init()` (`include/rosette/net/board_network.hpp`). It
blocks until the board is addressable, then `poll_board_network(now)` in the
main loop keeps it that way. With the default config:

| Step | What happens | Console |
| --- | --- | --- |
| Chip and PHY | reset the chip, verify it answers on SPI, wait for Ethernet link | `[net] PHY link up` |
| IPv4 | DHCP (RFC 2131), sending `hostname` as option 12; if no server answers within 15 s, an RFC 3927 link-local address in 169.254/16 | `[net] address 192.168.1.57` |
| IPv6 (W6100, W6300) | a link-local address fe80::/64 + modified EUI-64 of the MAC, checked by duplicate address detection; then router solicitation, and a global address from any advertised autonomous /64 prefix (RFC 4862 SLAAC) | `[net] IPv6 address fe80:0:0:0:208:dcff:fe12:3456` |
| Interrupts | route the chip's INT pin to the socket layer, so the firmware sleeps until a datagram arrives | |

Bring-up returns once an IPv4 address is held. IPv6 addresses arrive in the
background: the link-local address about a second after the link comes up,
and a global address when a router advertises a prefix. `board_address_snapshot()` lists every address the board
holds per family; its `v6.primary()` is the global address once there is one
and the link-local address before that.

After bring-up, `poll_board_network()` renews the DHCP lease at T1 by
unicast to the server that granted it, keeping the address throughout; reacts
to an address conflict the chip detects (DHCPDECLINE and a fresh DISCOVER, or
a new link-local candidate); refreshes the IPv6 prefix before its lifetime
runs out; and, when the cable is pulled and replaced, releases the addresses
and acquires them again from the start. Call it every loop iteration; it
never blocks.

**Link-local IPv4 upgrades to DHCP.** A board that booted with no DHCP server
keeps its 169.254/16 address and retries DHCP in the background every minute
(`upgrade_probe_ms`). When a server answers, the board moves to the leased
address; until then the link-local address keeps working.

### Identity

| Item | Source |
| --- | --- |
| MAC | WIZnet OUI `00:08:DC` + the last three bytes of the RP2040 / RP2350 unique ID (`board_get_mac_address()` in `boards/<board>/board_init.c`); stable across reboots, distinct per board |
| DHCP host name | the argument to `bring_up_board_network()` (`"rosette-rclc"` in `rclc_chatter`) |
| IPv6 interface ID | modified EUI-64 of the MAC (RFC 4291 Appendix A): MAC `00:08:dc:12:34:56` → `::208:dcff:fe12:3456`. If another host already holds that address, the board moves to the next of up to three further identifiers, derived from the MAC, prefix, and a conflict counter (RFC 7217); the sequence is the same on every boot |
| IPv4 link-local | a 169.254/16 address derived from the MAC and a conflict counter (RFC 3927 §2.1), the same on every boot and after every link flap |

To give a board a fixed IPv4 address, reserve one for its MAC on the DHCP
server, or list `Method::static_ipv4` in its config.

### Choosing the addressing methods

A `NetworkConfig` (`include/rosette/net/network_config.hpp`) lists the
methods per family in preference order. The first method that binds holds the
address; while a later one holds it, the earlier ones are retried every
`upgrade_probe_ms`.

```cpp
namespace net = rosette::net;
constexpr net::NetworkConfig kNetwork{
    .v4 = {net::Method::dhcp, net::Method::ipv4_link_local},  // preference order
    .v6 = {net::Method::slaac},                               // W6100 / W6300 only
    .upgrade_probe_ms = 300'000,                              // retry DHCP every 5 min
    .ready_when = net::ready::any_v4,                         // what bring-up waits for
    .ready_timeout_ms = 30'000,                               // default: wait forever
};
const net::BringUpResult result = net::bring_up_board_network("my-board", kNetwork);
```

| Field | Values | Default |
| --- | --- | --- |
| `v4` | `dhcp`, `ipv4_link_local`, `static_ipv4` (address in `static_v4`) | `dhcp`, `ipv4_link_local` |
| `v6` | `slaac`; empty for no IPv6 | empty (`board_default_network()` adds `slaac` on IPv6 parts) |
| `upgrade_probe_ms` | 0 never retries | 60000 |
| `ready_when` | `net::ready::any_v4`, `routable_v4`, `any_address`, or any `bool(const AddressSnapshot&)` function | `any_v4` |
| `ready_timeout_ms` | milliseconds | wait forever |

Bring-up returns `ready`, or `timed_out` when the timeout passed first: the
network keeps coming up in `poll_board_network()`, so the firmware can start
its loop and wait for the network there. It returns `unsupported_config`, and
starts nothing, when the board cannot run the config (SLAAC on the W5100S, a
method listed twice, `static_ipv4` without an address); the console names the
reason. The config is a plain value, so firmware can also build it at run
time, for example from settings stored in flash.

The designated initializers above are C++20; GCC and Clang accept them in
the C++17 this tree builds with.

## What ROS 2 traffic needs

A Rosette board is a DDSI-RTPS participant, the same as any Fast DDS or
Cyclone DDS process. It uses the standard RTPS port mapping (DDSI-RTPS 2.5
§9.6.1.1) over **IPv4**. The chip's IPv6 addresses serve UDP6 sockets
(`udp_echo` uses them), but `rmw_rosette` announces IPv4 locators
only, so ROS traffic flows over IPv4.

| Traffic | Address and port (domain *d*, participant *p*) | Domain 0, *p* = 0 | Domain 42, *p* = 0 |
| --- | --- | --- | --- |
| Participant discovery (SPDP), multicast | 239.255.0.1 : 7400 + 250*d* | 239.255.0.1:7400 | 239.255.0.1:17900 |
| Endpoint discovery (SEDP), unicast | board : 7410 + 250*d* + 2*p* | 7410 | 17910 |
| User data, unicast | board : 7411 + 250*d* + 2*p* | 7411 | 17911 |

So the network between a board and its ROS 2 peers must:

- **Put them on one IPv4 subnet**, or route 239.255.0.1 between subnets.
  Participant discovery is multicast and does not cross a router by default.
- **Deliver IPv4 multicast.** Some Wi-Fi access points and managed switches
  drop or rate-limit multicast. A wired switch is the reliable setup.
- **Let the peer's firewall accept UDP** on the ports above. On Ubuntu with
  `ufw` enabled, for domain 0:
  `sudo ufw allow from 192.168.1.0/24 proto udp to any port 7400:7500`.
- **Not restrict the peer to localhost.** `ROS_LOCALHOST_ONLY` must be unset
  or `0` on the ROS 2 host.

### Datagram size

A WIZnet board sends and receives UDP payloads of at most 1472 octets over
IPv4 (a 1500-byte Ethernet MTU minus the IPv4 and UDP headers;
`ROSETTE_RTPS_MAX_DATAGRAM_SIZE`), and 1452 over IPv6.
Rosette fragments anything larger itself (DATA_FRAG). Stock Fast DDS sends
datagrams up to 64 KB by default and relies on IP fragmentation, which the
chip does not reassemble: a sample from a stock peer whose datagram exceeds
1472 octets never reaches a board. Discovery and small messages are
unaffected. When a board subscribes to large messages, cap the peer's
datagram size (Fast DDS `maxMessageSize`) as
[rmw_rosette/README.md](../rmw_rosette/README.md#large-messages-and-small-links)
describes.

## ROS domain

A firmware runs on the domain in its rcl init options. `rclc_support_init()`
uses the default, domain 0, because a microcontroller has no `ROS_DOMAIN_ID`
environment variable. To choose another domain, set it in the init options:

```cpp
rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
rcl_init_options_init(&init_options, allocator);
rcl_init_options_set_domain_id(&init_options, 42);
rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator);
```

and give the ROS 2 host the same `ROS_DOMAIN_ID`. On a LAN where other
people run ROS 2, a domain other than 0 keeps your board and theirs apart and
keeps the board's peer tables (`RMW_ROSETTE_MAX_KNOWN_PARTICIPANTS` and
`RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS`) for your own nodes.

## Checking the network below ROS

`examples/udp_echo` brings the network up exactly as a ROS firmware
does and answers on plain UDP, so it separates network problems from ROS
problems:

| Port | Behaviour |
| --- | --- |
| UDP 7 (IPv4 and IPv6) | echoes each datagram back to its sender (RFC 862) |
| UDP 8 (IPv4 and IPv6) | replies with the board's addresses and echo counters; the datagram `bootsel` reboots the board into its USB bootloader |

```console
$ tools/udp_echo_probe.py 192.168.1.57
```

The probe asks the board for its addresses, then runs `ping`, `ping6`, and a
UDP echo round trip to each one. The same status reaches the USB console
every five seconds:

```text
[echo] ipv4 192.168.1.57
ipv6-link-local fe80:0:0:0:208:dcff:fe12:3456
ipv6-global 2001:db8:0:1:208:dcff:fe12:3456
echo ipv4=12/0 ipv6=12/0 arp_timeouts=0
```

To reach a link-local IPv6 address from Linux, name the host interface:
`ping6 fe80::208:dcff:fe12:3456%eth0`.

If the echo works and ROS discovery does not, the cause is above the IP
layer: domain, multicast delivery, or firewall ports (see
[GETTING_STARTED.md](../GETTING_STARTED.md#9-debugging)).
