# Network address bring-up

Scope: how a Rosette interface acquires and keeps its addresses. On the
WIZnet hardware-socket path that is IPv4 (DHCP, RFC 3927 link-local, static,
duplicate-address reaction, carrier supervision) and, on the W6100 / W6300,
IPv6 stateless autoconfiguration; on FreeRTOS+TCP it is an observer of the
stack's own addressing. What a user configures is in
[../network_configuration.md](../network_configuration.md).

`rosette/net/board_network.hpp` is the composition every WIZnet
firmware uses: it declares DHCP with a link-local fallback for IPv4 and, on
parts with hardware IPv6, SLAAC for IPv6, over the pieces below.
`udp_echo`, `rtps_chatter`, and `rclc_chatter` call it. Firmware that
needs a different address policy declares its own list the same way.

## The pieces

Four small pieces bring an interface online:

* **`rosette::net::AddressProvider`** — one way to get one address for one
  family. Three methods: `begin()`, `stop()`, `poll(now_ms)`, plus the optional
  pair `begin_probe()` / `claim()` for a provider that can acquire while
  another one holds the interface (see [Upgrading](#upgrading-from-a-fallback)).
  `poll()` is called
  every superloop iteration with the loop's timestamp; it drives acquisition,
  then keeps the address valid (DHCP renewal) and watches for a duplicate. It
  returns the provider's current `AddressLease` (`{family, source, state,
  flags, addresses, gateway_v4, gateway_v6, ntp}`), where `addresses` is an
  inline `AddressList` of every address the provider holds, each with its
  prefix length, flags, and valid / preferred lifetimes.

* **`rosette::net::AddressManager`** — runs an ordered `AddressProvider*` list
  for **one family**. The whole rule: try the providers in order, first to bind
  wins; a provider that reports `failed` or hasn't bound by `provider_timeout_ms`
  hands off to the next; when the list is exhausted (or a bound provider is later
  lost) it waits an exponential backoff and restarts from provider[0]. While a
  lower-preference provider holds the interface, it probes the providers ahead
  of it every `upgrade_probe_ms` and hands the interface to one that binds. Exposes
  `lease()` + `stable()` and nothing else — it knows nothing about snapshots,
  IPv6, the link, or the platform.

* **`rosette::net::NetworkAddress`** — the object `main()` drives. Combines a v4
  `AddressManager` and an optional v6 one into one `AddressSnapshot`,
  fires a single `on_change(const AddressSnapshot&)`, exposes `has_address()` /
  `stable()`, and — given a `NetInterface` — **supervises carrier**: a link drop
  releases the address, its return kicks a fresh acquisition. `poll(now_ms)`.

* **`rosette::net::NetInterface`** — the *one* backend hook. A provider that
  isn't the chip driver needs three things from the hardware: write the chosen
  address (`apply(const AddressLease&)` — clears when `has_address` is false),
  report duplicates (`bool conflict()`), and report carrier (`bool link_up()`).
  WIZnet supplies `wiznet::WiznetInterface<Chip,Bus>`; FreeRTOS+TCP owns its
  endpoint and uses `NullNetInterface` (always up).

Nothing in the address layer blocks: `main()` calls `poll(now)` every iteration and services the protocol stack
only while `has_address()` holds. `wait_stable()` is an opt-in for firmware
with nothing to do offline.

### One manager per family

There is no primary/secondary role and no combined bimodal manager. IPv4 (a
serial DHCP→fallback ladder) and IPv6 (SLAAC on the W6100 / W6300) are
independent `AddressManager`s over independent provider lists.
`NetworkAddress` is the only thing that knows about both.

## Providers

| provider | family | `poll(now)` does |
|---|---|---|
| `StaticAddressProvider` | v4 / v6 | writes the address via `NetInterface`, binds; a conflict → `failed` (a static address can't be renumbered) |
| `wiznet::DhcpAddressProvider` | v4 | steps a `wiznet::DhcpClient` (RFC 2131 state machine, one non-blocking step per poll, a socket only while waiting for a reply); writes the lease through `NetInterface::apply()` on `bound`, removes it on `lost` (expiry or NAK), writes nothing on a renewal; as holder a conflict → DHCPDECLINE and a fresh DISCOVER ≥10 s later. Supports `begin_probe()` |
| `Ipv4LinkLocalProvider` | v4 | RFC 3927 FSM (below); no internal clock; the terminal fallback — never reports `failed`, holds link-local and defends it |
| `freertos::FreeRtosObserver` | one per family | observe-only: maps the stack's up endpoints into a lease; never fails, never blocks |
| `wiznet::WiznetSlaacProvider` | v6 | RFC 4862 SLAAC on the W6100 / W6300: link-local address with DAD, router solicitation, global address from an autonomous prefix, refresh before lifetimes end; see [W6100 / W6300 hardware IPv6](#w6100--w6300-hardware-ipv6) |

### Declaring it

There are no `ROSETTE_NET_*` policy macros. The providers are declared in
fallback order, straight into an `AddressManager`; this is what
`boards/common/wiznet/wiznet_board_network.cpp` does:

```cpp
namespace wiz = rosette::net::wiznet;
namespace net = rosette::net;

// One NetInterface for the chip.
static wiz::WiznetInterface<wiz::BoardChip, wiz::BoardBus> iface{device};

// The IPv4 providers, tried in order — first to bind wins.
static wiz::DhcpAddressProvider dhcp{sockets, mac, iface, wiz::dhcp_config("my-hostname")};
static net::Ipv4LinkLocalProvider zeroconf{mac, iface};   // 169.254/16 fallback

static net::AddressProvider* v4_providers[] = {&dhcp, &zeroconf};
static net::AddressManager   v4{v4_providers};              // policy defaults: 15 s / 30 s / 60 s
static net::NetworkAddress   net_addr{v4, /*v6=*/nullptr, /*link=*/&iface};

net_addr.on_change([](const net::AddressSnapshot& s){
    /* s.assigned, s.v4.routable(), s.v4.duplicate() … */
});
net_addr.begin();

for (;;) {
    const uint32_t now = rosette::platform::monotonic_ms();
    net_addr.poll(now);
    if (net_addr.has_address()) {
        (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
}
```

`poll(now)` runs every iteration and the executor is serviced only while an
address is held. When carrier drops, `NetworkAddress` releases the address,
`has_address()` goes false, and the protocol stack is left idle; when carrier
returns it re-acquires from the top (a fresh DHCP DISCOVER, since `begin()` —
not a renew — is what the link-up edge calls).

`board_network` wraps this loop for firmware that does not declare its own
lists. The firmware passes a `NetworkConfig` (`rosette/net/network_config.hpp`)
naming the methods per family in preference order; `board_network` owns one
provider per method the chip can run and hands the managers the ones the
config lists, in its order. `bring_up_board_network()` then steps the loop
until the config's `ready_when` predicate accepts the snapshot or
`ready_timeout_ms` passes, because `rcl_init()` must run after the address
exists (`rmw_rosette/network_port.hpp`), and `poll_board_network()` keeps
stepping it afterwards:

```cpp
constexpr net::NetworkConfig kNetwork{
    .v4 = {net::Method::static_ipv4, net::Method::ipv4_link_local},
    .static_v4 = {{192, 168, 1, 50}, 24, {192, 168, 1, 1}},
    .ready_when = net::ready::any_v4,
    .ready_timeout_ms = 30'000,
};
if (net::bring_up_board_network("my-board", kNetwork) != net::BringUpResult::ready) { ... }
```

A config the board cannot run (SLAAC on the W5100S, a method listed twice, a
static method without an address) is refused whole by `check_config()`, with
the reason on the console, before the chip is touched: a board never runs
with a method the firmware asked for silently missing. A bring-up that times
out starts nothing over; the next `poll_board_network()` continues from where
it stopped, including retrying a chip that did not answer.

Other layouts are just a different list:

```cpp
// static only
static net::StaticAddressProvider ip{
    net::StaticAddressProvider::v4({192,168,1,50}, 24, {192,168,1,1}), iface};
static net::AddressProvider* v4_providers[] = {&ip};

// DHCP, then a fixed address if there is no server
static net::AddressProvider* v4_providers[] = {&dhcp, &ip};

// zeroconf only
static net::AddressProvider* v4_providers[] = {&zeroconf};
```

`AddressManager{list}` takes an optional `AddressManagerPolicy{provider_timeout_ms,
retry_backoff_ms, upgrade_probe_ms}` as a second argument; the defaults (15 s,
30 s, 60 s) suit a board.

## RFC 3927 IPv4 link-local — no ARP probe

There is no ARP-probe abstraction and no blocking probe call. The WIZnet chip
already does duplicate-address detection in hardware (`IR_CONFLICT` /
`IR_IPCONF`), surfaced by `NetInterface::conflict()`. `Ipv4LinkLocalProvider` is
a plain cooperative FSM stepped from `poll()`:

```
picking  → take candidate N (ipv4_link_local_candidate(mac, N), N counting
           up from 0 at begin()), write it (the chip
           emits a gratuitous ARP), start a ~3 s settle window
settling → each poll: conflict() → new candidate (or → rate_limited after
           MAX_CONFLICTS); window elapsed with no conflict → bound
bound    → conflict() → defend once (re-announce), → defending
defending→ another conflict within DEFEND_INTERVAL → relinquish, re-pick;
           interval elapsed → back to bound
rate_limited → wait RATE_LIMIT_INTERVAL, reset the count, re-pick
```

The RFC 3927 "announce" is implicit — writing `SIPR` makes the chip gratuitous-ARP.
The same `conflict()` signal covers both the pre-claim settle and steady state.

`WiznetInterface` writes a link-local address with subnet mask 0.0.0.0 and no
gateway. RFC 3927 §2.6.2 has a host with only a link-local address ARP for
every destination directly, and mask 0 makes the chip do exactly that, so a
link-local board reaches peers on the link that hold routable addresses. With
the /16 mask the chip sends those packets to the absent gateway; the ARP fails
and stalls that socket's next send for the chip's retry time (1.6 s, measured
on the W6100).

## Next-hop resolution: the ARP cache

A WIZnet socket remembers one destination MAC (`Sn_DHAR`), so a socket that
alternates between peers makes the chip ARP before nearly every SEND, and an
unanswered ARP stalls the socket's next send for the whole retry time. The
board therefore resolves next hops itself (`ports/net/wiznet/arp_cache.hpp`,
`arp_resolver.hpp`) and hands every IPv4 send its MAC:

- **Route.** Multicast goes to the group MAC, broadcast to ff:ff:ff:ff:ff:ff.
  Unicast goes to the next hop's cached MAC. The next hop is the destination
  when it is on the subnet (always, under the link-local mask 0.0.0.0), and
  the gateway otherwise.
- **Send without ARP.** The W5100S uses `SEND_MAC` per datagram. The W6100 and
  W6300 have no `SEND_MAC`, so their IPv4 sockets open with
  `Sn_MR2.DHAM_MANUAL` and every `SEND` uses `Sn_DHAR`. A multicast-joined
  W6x00 socket keeps the chip's own ARP, since its `Sn_DHAR` holds the group.
- **Miss: drop and resolve in the background.** A datagram whose next hop is
  unknown is dropped, as a lost datagram, and the chip's socket-less ARP
  command (`SLCR`) resolves the next hop off the send path. RTPS reliability
  and DHCP retransmission recover the first datagram. The gateway is resolved
  as soon as an address is configured, so the first send off the subnet (an
  SNTP request, which would otherwise wait `retry_ms` to retry) finds its MAC
  already there. Falling back to the chip's inline ARP would bring the stall
  back.
- **Negative cache.** A next hop that does not answer is retried with
  exponential backoff (1 s to 32 s). Sends to it cost nothing in between.
  Receiving a datagram from it shortens the backoff to 1 s.
- **Refresh and expiry.** An entry in use is re-resolved in the background
  60 s after its last answer (RFC 1122 §2.3.2.1) and follows a changed MAC.
  Sends keep using the MAC until 120 s after that answer; between the two, an
  unanswered refresh is retried with the same backoff, so a peer that misses
  several requests (a Wi-Fi host in power save) keeps its MAC as long as it
  answers one. A datagram received from it brings the next retry forward to
  1 s. An entry nobody sent to since its last answer leaves the table instead
  of being refreshed.

The cache holds `ROSETTE_WIZNET_ARP_CACHE_ENTRIES` next hops (CMake, default
8; 0 leaves ARP to the chip) and is emptied whenever `SIPR`/`SUBR`/`GAR`
change. On the W6x00 the socket-less engine is shared with SLAAC's NS and RS.
`WiznetDevice` records which user owns it and latches each completion for its
owner, so neither user ever reads or clears the other's result.

## Conflict / DAD reaction

Detection is off the driver's existing cadence — no new bus-lock-contending
poller:

`WiznetInterface::conflict()` calls `WiznetDevice::poll_common_interrupt()`
(one register read) and consumes the latch. The IP-conflict bit is
`Chip::common_ip_conflict_mask()`: `IR_CONFLICT` 0x80 on the W5100S, `IR_IPCONF`
0x04 on the W6x00. The same code runs on every OS.

`WiznetInterface` is shared by every WIZnet v4 provider, and the chip has one
conflict latch, so exactly one provider reads it: the one holding the
interface. A provider being probed never reads it (below).

| holder | reaction |
|---|---|
| DHCP | DHCPDECLINE, remove the address, fresh DISCOVER after the RFC 2131 ≥10 s pause |
| zeroconf | defend once, then a new `169.254.x.y` candidate |
| static | `failed` → `AddressManager` falls through to the next provider |

## Upgrading from a fallback

When DHCP was down at boot, link-local holds the interface, and RFC 3927 §1.9
asks the host to keep looking for a routable address. Every
`upgrade_probe_ms` the manager probes each provider ahead of the holder, one
at a time, with `begin_probe()`:

* The prober runs its normal acquisition, but **never writes the interface
  and never reads the conflict latch**. DHCP's DISCOVER goes out from the
  link-local source address with the BROADCAST flag set, so the server's
  broadcast OFFER and ACK reach the board. (RFC 2131 §4.1 asks for source
  0.0.0.0; the one LAN router tested so far answers either way.)
* The holder keeps being polled, so it keeps defending its address (RFC 3927
  §2.5) and its address stays on the chip for the whole probe.
* A prober that binds takes over: the manager stops the holder (which removes
  its address), then `claim()` writes the prober's lease, so the chip ends up
  with the routable address and a single interface write per step.
* A prober that fails or times out is stopped; the next round starts
  `upgrade_probe_ms` later.

DHCP renewal is the same idea inside one provider: at T1 the client unicasts
a REQUEST with `ciaddr` to its server, and at T2 broadcasts one (RFC 2131
§4.4.5); a successful renewal writes nothing to the interface, so the address
never leaves the chip.

A provider that cannot acquire without writing the interface (link-local,
static) returns `false` from `begin_probe()` and is only retried by the
backoff restart.

## Link failure and re-negotiation

Carrier is a `NetInterface` property (`link_up()` — WIZnet reads PHYSR live).
`NetworkAddress` supervises it, so the per-family managers stay link-agnostic:

* **carrier lost** → `NetworkAddress` calls `stop()` on its managers (which
  release the address and clear the chip via `NetInterface::apply`),
  `has_address()` goes false, and `main()` leaves the protocol stack idle.
* **carrier back** → `NetworkAddress` calls `begin()` on its managers. Because
  `begin()` — not a renewal — is what runs, DHCP does a fresh **DISCOVER** (the
  old lease is presumed invalid on a possibly-different network), zeroconf
  re-picks a candidate, static re-writes `SIPR`. No provider-specific re-negotiation
  code — the normal `stop()`/`begin()` lifecycle already does the right thing.

`board_network` brings the chip up once (`WiznetLink::bring_chip_up()`:
verify the part, set socket mode with the MAC) and does not wait for the PHY
separately: `NetworkAddress` starts the managers when carrier first comes up,
exactly as it does after any later carrier loss.

## FreeRTOS+TCP path

`freertos::FreeRtosObserver` only *observes* — it maps the first IPv4
endpoint that is up (and an IPv6 one, a global address first) into an
`AddressLease`, classifying
`169.254/16` → `zeroconf_v4` and `fe80::/10` → `link_local_v6`. The stack does
DHCP, AutoIP fallback (`ipconfigDHCP_FALL_BACK_AUTO_IP` = `1`), RA/SLAAC, and DAD
itself. Give the v4 observer to a `v4` manager and the v6 observer to a `v6`
manager (each list is just that one provider), then combine with `NetworkAddress`.

## W6100 / W6300 hardware IPv6

The W6100 and W6300 run IPv6 in silicon once the host programs their
addresses: a link-local address (LLAR), a global address and prefix (GUAR,
SUB6R), and a default router (GA6R). The chip then answers neighbor
solicitations and echo requests itself and carries UDP6 traffic. Choosing the
addresses is `wiznet::WiznetSlaacProvider<Chip,Bus>` (`slaac_provider.hpp`),
one `AddressProvider` for family v6:

1. **Link-local.** fe80::/64 plus the modified EUI-64 of the MAC, checked by
   duplicate address detection with the chip's socket-less `SLCR.NS` command,
   then written to LLAR. A probe that is answered moves to the next candidate
   from `ipv6_address_candidate()` (`rosette/net/address_candidate.hpp`), up
   to IDGEN_RETRIES (3) times (RFC 7217 §5–6). The provider is `bound` from
   here on.
2. **Router discovery.** Router solicitations (`SLCR.RS`), with the chip's RA
   handling diverted (`ICMP6BLKR.RA`) to an IPRAW6 socket the provider holds
   only while soliciting; `parse_router_advert()`
   (`rosette/net/ipv6_router_advert.hpp`) decodes the answers. The chip's own
   prefix latch is not used because it decodes the prefix option at the wrong
   offset ([wiznet_w61xx_w63xx.md](wiznet_w61xx_w63xx.md#ipv6)).
3. **Global.** An autonomous /64 prefix plus an interface identifier, chosen
   and checked by DAD the same way with its own per-prefix conflict counter,
   goes to GUAR; the advertising router goes to GA6R.
4. **Refresh.** The provider solicits again before the router, preferred, or
   valid lifetime ends and follows a changed prefix. An ended preferred
   lifetime marks the global address `deprecated`; an ended valid lifetime
   withdraws it. Without a router it keeps soliciting with RFC 7559 backoff.

The lease lists the link-local address and, when present, the global address;
`AddressList::primary()` picks the global one (RFC 6724 rules 2 and 3), and
`WiznetInterface::apply()` programs LLAR and GUAR from the list. When every
link-local candidate is duplicated, IPv6 stops on the interface
(RFC 4862 §5.4.5) and the provider reports `failed`; when every global
candidate for a prefix is duplicated, the link-local address stays in service.

Candidates are FNV-1a hashes (`rosette/support/fnv1a.hpp`) of the MAC, the
prefix, and the counter, with counter 0 fixed to the modified EUI-64. No key is
secret: the goal is escaping a collision, not RFC 7217's privacy, and a
first-choice address that follows from the MAC label keeps boards easy to find.

`board_network` gives the provider its own `AddressManager` and passes it to
`NetworkAddress` as the v6 manager, so IPv4 and IPv6 are acquired side by
side. The choice is made at compile time from `Chip::has_hw_ipv6()`: on the
W5100S the v6 manager is absent and `NetworkAddress` runs IPv4 alone.
The config's `v6` list decides whether SLAAC runs (`board_default_network()`
lists it on these parts). The default `ready_when` accepts the snapshot once
IPv4 is up, so a stuck IPv6 duplicate address detection cannot hold up
bring-up; a firmware that needs IPv6 before `rcl_init()` names that in its
predicate (`examples/udp_echo` waits for every family, with a
timeout).

## Verification

`tests/host/`: `ipv4ll_provider_test` (RFC 3927 FSM against a fake `NetInterface`),
`address_snapshot_test` (`AddressSnapshot` equality + link-local classifiers),
`wiznet_slaac_provider_test` (link-local and global DAD, router solicitation,
RA handling, lifetimes, against a register-file chip model),
`ipv6_router_advert_test` (RFC 4861 RA decoding, option order),
`wiznet_udp_socket_test` (IPv4 and IPv6 UDP receive and send per chip),
`address_manager_test` (serial list, fall-through, timeout, exhaustion →
backoff → restart-from-top, bound-provider loss → re-cascade),
`network_address_test` (v4+v6 → one snapshot, platform publish, `on_change`,
`has_address`, `wait_stable`, and carrier drop → release → return →
re-acquire), `wiznet_address_provider_test` (the WIZnet template stack
instantiated against a `RecordingBus`), `wiznet_header_test` (conflict-bit
traits), `dhcp_client_test` (the RFC 2131 client against a scripted socket:
wire format, retransmission, unicast renewal, rebinding, decline, expiry),
`address_upgrade_test` (DHCP probed while link-local holds, the conflict latch
staying with the holder, the upgrade, renewal without an interface write),
and `network_config_test` (config checking against a backend's methods,
providers ordered by the config, the ready predicates). All fakeable — `fake_clock.hpp` for the platform clock (`monotonic_us()`),
`fake_interface.hpp` for the `NetInterface` (address / conflict / carrier),
`mock_provider.hpp` for a scriptable `AddressProvider`. The provider FSMs take
`poll(now_ms)` so their tests use plain integers, no clock hook.

On hardware: DHCP acquisition is exercised by every `rtps_chatter` and
`rclc_chatter` boot on the W5100S-EVB-Pico2, and `udp_echo` on the
W6100-EVB-Pico acquires IPv4, IPv6 link-local, and a SLAAC global address,
with ping and UDP echo over each (`tools/udp_echo_probe.py`). On the
W6100-EVB-Pico with a test DHCP server that answered only after a delay, the
board fell back to link-local, kept echoing through failed DHCP probes, moved
to the leased address when the server answered, and renewed it at T1 by
unicast without an echo gap; a leased address another host already held was
declined. The same board ran a static-first config (`192.168.32.112`, SLAAC
beside it), refused a config listing DHCP twice, and, given a predicate that
never holds, returned `timed_out` after its 5 s budget and still acquired
DHCP and SLAAC addresses from `poll_board_network()`.
