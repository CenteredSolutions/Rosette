# Network address bring-up

Scope: how a Rosette interface acquires and keeps its addresses. On the
WIZnet hardware-socket path that is IPv4 (DHCP, RFC 3927 link-local, static,
duplicate-address reaction, carrier supervision) and, on the W6100 / W6300,
IPv6 stateless autoconfiguration; on FreeRTOS+TCP it is an observer of the
stack's own addressing. What a user configures is in
[../network_configuration.md](../network_configuration.md).

`rosette/net/wiznet/board_network.hpp` is the composition every WIZnet
firmware uses: it declares DHCP with a link-local fallback for IPv4 and, on
parts with hardware IPv6, SLAAC for IPv6, over the pieces below.
`wiznet_udp_echo`, `rtps_chatter`, and `rclc_chatter` call it. Firmware that
needs a different address policy declares its own list the same way.

## The pieces

Four small pieces bring an interface online:

* **`rosette::net::AddressProvider`** — one way to get one address for one
  family. Three methods: `begin()`, `stop()`, `poll(now_ms)`. `poll()` is called
  every superloop iteration with the loop's timestamp; it drives acquisition,
  then keeps the address valid (DHCP renewal) and watches for a duplicate. It
  returns the provider's current `AddressLease` (`{family, method, phase,
  conflict, address, prefix_len, gateway, lease_remaining_s, ntp}`).

* **`rosette::net::AddressManager`** — runs an ordered `AddressProvider*` list
  for **one family**. The whole rule: try the providers in order, first to bind
  wins; a provider that reports `failed` or hasn't bound by `provider_timeout_ms`
  hands off to the next; when the list is exhausted (or a bound provider is later
  lost) it waits an exponential backoff and restarts from provider[0]. Exposes
  `lease()` + `stable()` and nothing else — it knows nothing about snapshots,
  IPv6, the link, or the platform.

* **`rosette::net::NetworkAddress`** — the object `main()` drives. Combines a v4
  `AddressManager` and an optional v6 one into one `AddressSnapshot`, publishes
  the addresses to the platform runtime (`rosette_platform_set_ipv{4,6}_addr`),
  fires a single `on_change(const AddressSnapshot&)`, exposes `has_address()` /
  `stable()`, and — given a `NetInterface` — **supervises carrier**: a link drop
  releases the address, its return kicks a fresh acquisition. `poll(now_ms)`.

* **`rosette::net::NetInterface`** — the *one* backend hook. A provider that
  isn't the chip driver needs three things from the hardware: write the chosen
  address (`apply(const AddressLease&)` — clears when `has_address` is false),
  report duplicates (`bool conflict()`), and report carrier (`bool link_up()`).
  WIZnet supplies `wiznet::WiznetInterface<Chip,Bus>`; FreeRTOS+TCP owns its
  endpoint and uses `NullNetInterface` (always up).

Nothing in the address layer blocks except a first DHCP exchange (below):
`main()` calls `poll(now)` every iteration and services the protocol stack
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
| `wiznet::DhcpAddressProvider` | v4 | wraps `DhcpPoller`; first `poll()` runs one bounded exchange; then renew at T1; conflict → `DhcpPoller::handle_conflict()` (DHCPDECLINE + fresh DISCOVER); a lease aged past validity is dropped. Every address write is routed through `NetInterface::apply()` — `DhcpProtocol` gets a `set_apply()` callback rather than touching `configure_ipv4` itself |
| `Ipv4LinkLocalProvider` | v4 | RFC 3927 FSM (below); no internal clock; the terminal fallback — never reports `failed`, holds link-local and defends it |
| `freertos::FreeRtosObserver` | one per family | observe-only: maps `rosette_platform_get_ipv{4,6}_addr()` into a lease; never fails, never blocks |
| `wiznet::WiznetSlaacProvider` | v6 | RFC 4862 SLAAC on the W6100 / W6300: link-local address with DAD, router solicitation, global address from an autonomous prefix, refresh before lifetimes end; see [W6100 / W6300 hardware IPv6](#w6100--w6300-hardware-ipv6) |

### Declaring it

There are no `ROSETTE_NET_*` policy macros. The providers are declared in
fallback order, straight into an `AddressManager`; this is what
`src/net/wiznet/board_network.cpp` does:

```cpp
namespace wiz = rosette::net::wiznet;
namespace net = rosette::net;

// One NetInterface for the chip.
static wiz::WiznetInterface<wiz::BoardChip, wiz::BoardBus> iface{device};

// The IPv4 providers, tried in order — first to bind wins.
static wiz::DhcpAddressProvider<wiz::BoardChip, wiz::BoardBus> dhcp{
    device, sockets, iface, wiz::dhcp_config("my-hostname")};
static net::Ipv4LinkLocalProvider zeroconf{mac, iface};   // 169.254/16 fallback

static net::AddressProvider* v4_providers[] = {&dhcp, &zeroconf};
static net::AddressManager   v4{v4_providers};              // policy defaults: 15 s / 30 s
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

`board_network` makes one simplification: `bring_up_board_network()` blocks
until the first address is held, because `rcl_init()` must run after the
address exists (`rmw_rosette/network_port.hpp`), and `poll_board_network()`
then keeps it.

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
retry_backoff_ms}` as a second argument; the defaults (15 s, 30 s) suit a board.

## RFC 3927 IPv4 link-local — no ARP probe

There is no ARP-probe abstraction and no blocking probe call. The WIZnet chip
already does duplicate-address detection in hardware (`IR_CONFLICT` /
`IR_IPCONF`), surfaced by `NetInterface::conflict()`. `Ipv4LinkLocalProvider` is
a plain cooperative FSM stepped from `poll()`:

```
picking  → choose a 169.254.x.y candidate from the MAC, write it (the chip
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

## Conflict / DAD reaction

Detection is off the driver's existing cadence — no new bus-lock-contending
poller:

* **FreeRTOS:** `WiznetDevice::service_socket_interrupts()` latches the
  IP-conflict bit (`Chip::common_ip_conflict_mask()` — `IR_CONFLICT` 0x80 on the
  W5100S, `IR_IPCONF` 0x04 on the W6x00) in the same locked block it already
  uses to read the common interrupt register.
* **No-RTOS:** `WiznetInterface::conflict()` calls
  `WiznetDevice::poll_common_interrupt()` — one register read — then consumes
  the latch.

`WiznetInterface` is shared by every WIZnet v4 provider; only the active one is
polled, so exactly one provider consumes each conflict:

| active provider | reaction |
|---|---|
| DHCP | `DhcpPoller::handle_conflict()` → DHCPDECLINE, clear the lease + chip config, fresh DISCOVER after the RFC 2131 ≥10 s pause |
| zeroconf | defend once, then a new `169.254.x.y` candidate |
| static | `failed` → `AddressManager` falls through to the next provider |

**Blocking.** `DhcpAddressProvider`'s first `poll()` and a conflict-recovery
exchange run a blocking DHCP exchange bounded by
`DhcpConfig::exchange_timeout_ms` (10 s). During that window the protocol
stack is not serviced; RTPS reliability recovers anything missed. The
FreeRTOS `DhcpTask` path does not block. Everything else in the address layer
is non-blocking.

**Fallback is sticky.** Once link-local binds (DHCP was down at boot), the
manager stays on it; it retries DHCP from the top only if link-local itself
fails or carrier drops and returns.

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

`board_network` waits for the PHY once, in `bring_up_board_network()`
(`WiznetLink::bring_link_up()`: verify the chip, set socket mode, wait for
link), because nothing can be acquired before the chip answers. From then on
carrier is supervised by `NetworkAddress` in the loop.

## FreeRTOS+TCP path

`freertos::FreeRtosObserver` only *observes* — it maps
`rosette_platform_get_ipv{4,6}_addr()` into an `AddressLease`, classifying
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
   then written to LLAR. The provider is `bound` from here on.
2. **Router discovery.** Router solicitations (`SLCR.RS`), with the chip's RA
   handling diverted (`ICMP6BLKR.RA`) to an IPRAW6 socket the provider holds
   only while soliciting; `parse_router_advert()`
   (`rosette/net/ipv6_router_advert.hpp`) decodes the answers. The chip's own
   prefix latch is not used because it decodes the prefix option at the wrong
   offset ([wiznet_w61xx_w63xx.md](wiznet_w61xx_w63xx.md#ipv6)).
3. **Global.** An autonomous /64 prefix plus the interface identifier, checked
   by DAD, goes to GUAR; the advertising router goes to GA6R.
4. **Refresh.** The provider solicits again before the router or valid
   lifetime ends, follows a changed prefix, and withdraws what expires.
   Without a router it keeps soliciting with RFC 7559 backoff.

A duplicate link-local address stops IPv6 on the interface (RFC 4862 §5.4.5)
and reports `failed`; a duplicate global address leaves the link-local address
in service.

`board_network` gives the provider its own `AddressManager` and passes it to
`NetworkAddress` as the v6 manager, so IPv4 and IPv6 are acquired side by
side. The choice is made at compile time from `Chip::has_hw_ipv6()`: on the
W5100S the v6 manager is absent and `NetworkAddress` runs IPv4 alone.
`bring_up_board_network()` returns once IPv4 and the IPv6 link-local address
are held; the global address arrives later through `poll_board_network()`.

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
instantiated against a `RecordingBus`), and `wiznet_header_test` (conflict-bit
traits). All fakeable — `fake_runtime.hpp` for `block_until_stable`'s clock,
`fake_interface.hpp` for the `NetInterface` (address / conflict / carrier),
`mock_provider.hpp` for a scriptable `AddressProvider`. The provider FSMs take
`poll(now_ms)` so their tests use plain integers, no clock hook.

On hardware: DHCP acquisition is exercised by every `rtps_chatter` and
`rclc_chatter` boot on the W5100S-EVB-Pico2, and `wiznet_udp_echo` on the
W6100-EVB-Pico acquires IPv4, IPv6 link-local, and a SLAAC global address,
with ping and UDP echo over each (`tools/wiznet_udp_echo_probe.py`). Checks that need a bench setup: T1
renewal on a short lease; DHCP server unplugged → `169.254.x.y` → the board
still discovers on-link peers; conflict injection (a second host on the same
IP) → the reaction runs, backs off, and re-acquires.
