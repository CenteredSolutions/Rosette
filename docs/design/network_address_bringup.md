# Network address bring-up

Status: IPv4 (DHCP / zeroconf / static / conflict) landed for the WIZnet
hardware-socket path; FreeRTOS+TCP observer landed; WIZnet hardware IPv6 is a
documented seam, not implemented.

## Why

Every firmware `main()` used to bring the interface online by hand —
`WiznetLink::begin()` did `verify → init_socket_mode → PHY-wait → DHCP-or-static`
with one baked-in DHCP→static fallback. No RFC 3927 IPv4 link-local, no reaction
to a duplicate address.

Four small pieces replace it:

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

`rosette::App` is unchanged. Nothing in the address layer blocks: `main()` calls
`net.poll(now)` every iteration and gates `app.spin_once(now)` on
`net.has_address()`. `wait_stable()` stays as an opt-in for firmware with
nothing to do offline.

### One manager per family

There is no primary/secondary role and no combined bimodal manager. IPv4 (a
serial DHCP→fallback ladder) and IPv6 (SLAAC / link-local, when hardware IPv6
lands) are independent `AddressManager`s over independent provider lists.
`NetworkAddress` is the only thing that knows about both.

## Providers

| provider | family | `poll(now)` does |
|---|---|---|
| `StaticAddressProvider` | v4 / v6 | writes the address via `NetInterface`, binds; a conflict → `failed` (a static address can't be renumbered) |
| `wiznet::DhcpAddressProvider` | v4 | wraps `DhcpPoller`; first `poll()` runs one bounded exchange; then renew at T1; conflict → `DhcpPoller::handle_conflict()` (DHCPDECLINE + fresh DISCOVER); a lease aged past validity is dropped. Every address write is routed through `NetInterface::apply()` — `DhcpProtocol` gets a `set_apply()` callback rather than touching `configure_ipv4` itself |
| `Ipv4LinkLocalProvider` | v4 | RFC 3927 FSM (below); no internal clock; the terminal fallback — never reports `failed`, holds link-local and defends it |
| `freertos::FreeRtosObserver` | one per family | observe-only: maps `rosette_platform_get_ipv{4,6}_addr()` into a lease; never fails, never blocks |
| `wiznet::SlaacProvider` | v6 | *future — see below* |

### Declaring it

There is no bring-up wrapper and no `ROSETTE_NET_*` policy macros. Firmware
declares the providers it wants, in fallback order, straight into an
`AddressManager`:

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
    const uint32_t now = rosette_platform_get_ms();
    net_addr.poll(now);
    if (net_addr.has_address()) {
        app.spin_once(now);
    }
    rosette_platform_wait_for_event(app.connected() ? 5u : 200u);
}
```

`main()` never blocks on the network: `poll(now)` runs every iteration and
`app.spin_once()` is gated on `has_address()`. When carrier drops,
`NetworkAddress` releases the address, `has_address()` goes false, the app stops
being serviced; when carrier returns it re-acquires from the top (a fresh DHCP
DISCOVER, since `begin()` — not a renew — is what the link-up edge calls).

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

**Blocking note.** `DhcpAddressProvider`'s first `poll()` and a conflict-recovery
exchange run a blocking DHCP exchange bounded by
`DhcpConfig::exchange_timeout_ms` (10 s here). During that window
`app.spin_once()` can miss a beat — the same characteristic as today's lease
renewal. The FreeRTOS `DhcpTask` path does not block. Everything else in the
address layer is non-blocking.

**Known limitation.** Once zeroconf binds (DHCP was down at boot), the manager
stays on it; it only re-tries DHCP-from-the-top if zeroconf itself later fails.
A periodic "retry the preferred provider while a fallback holds" is a future knob.

## Link failure and re-negotiation

Carrier is a `NetInterface` property (`link_up()` — WIZnet reads PHYSR live).
`NetworkAddress` supervises it, so the per-family managers stay link-agnostic:

* **carrier lost** → `NetworkAddress` calls `stop()` on its managers (which
  release the address and clear the chip via `NetInterface::apply`),
  `has_address()` goes false, `main()` stops calling `app.spin_once()`.
* **carrier back** → `NetworkAddress` calls `begin()` on its managers. Because
  `begin()` — not a renewal — is what runs, DHCP does a fresh **DISCOVER** (the
  old lease is presumed invalid on a possibly-different network), zeroconf
  re-picks a candidate, static re-writes `SIPR`. No provider-specific re-negotiation
  code — the normal `stop()`/`begin()` lifecycle already does the right thing.

`main()` no longer needs a `while (!link.bring_link_up())` wait — chip init
(`verify` + `init_socket_mode`) stays explicit, but the PHY wait folds into the
supervised loop.

## FreeRTOS+TCP path

`freertos::FreeRtosObserver` only *observes* — it maps
`rosette_platform_get_ipv{4,6}_addr()` into an `AddressLease`, classifying
`169.254/16` → `zeroconf_v4` and `fe80::/10` → `link_local_v6`. The stack does
DHCP, AutoIP fallback (`ipconfigDHCP_FALL_BACK_AUTO_IP` = `1`), RA/SLAAC, and DAD
itself. Give the v4 observer to a `v4` manager and the v6 observer to a `v6`
manager (each list is just that one provider), then combine with `NetworkAddress`.

## Future: W6100 / W6300 hardware IPv6

Not implemented — no EVB to verify the SLAAC/DAD register semantics. The seam is
a commented trait block in `w6x00.hpp` plus:

* `wiznet::SlaacProvider<Chip,Bus> : AddressProvider` (family v6) — write the LLA
  via `WiznetInterface`, kick RS + the chip's DAD engine, poll the status
  register, on DAD fail regenerate the IID, promote to `slaac_v6` on a GUA.
* Its own `AddressManager` for the v6 list, passed to `NetworkAddress` as the
  second argument.
* `WiznetInterface::apply(AddressFamily::ipv6, …)` — the v6 publication point.
* Gate `WiznetSocketProvider`'s `SocketDomain::ipv6` rejection on
  `if constexpr (!Chip::has_hw_ipv6())`.

Nothing above changes — hardware v6 is a second provider list.

## Verification

`tests/host/`: `ipv4ll_provider_test` (RFC 3927 FSM against a fake `NetInterface`),
`address_snapshot_test` (`AddressSnapshot` equality + link-local classifiers),
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

Only on real W5100S-EVB-Pico2 hardware: real DHCP + T1 renewal; DHCP unplugged
→ `169.254.x.y` → micro-ROS still reaches an on-link Agent; conflict injection
(a second host on the same IP) → the reaction runs, backs off, re-acquires.
