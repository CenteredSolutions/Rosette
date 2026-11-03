# Security policy

Rosette parses traffic from the network on devices with no memory protection, so a parsing bug can be a remote
code execution bug. Reports are welcome and are handled privately until a fix is available.

## Reporting a vulnerability

Do not open a public issue, pull request, or discussion for a suspected vulnerability.

- Preferred: GitHub's private reporting, from the repository's **Security** tab, **Report a vulnerability**.
- Or email software@centeredsolutions.com with "Rosette security" in the subject.

Include what you can of:

- the affected component (for example the RTPS message parser, SPDP, the DHCP client, `rmw_rosette`);
- the commit or release, the board or host, and the ROS 2 distribution;
- a reproducer: a packet capture, a datagram as hex, or a fuzzer input, and what happens (crash, hang, memory
  corruption, wrong behavior);
- whether you believe it is reachable from the network without any prior exchange.

## What to expect

Rosette is maintained by a small team, so these are targets rather than guarantees:

- an acknowledgement within 5 business days;
- an assessment, and a fix or mitigation plan, within 30 days;
- coordinated disclosure: a GitHub security advisory and a release once a fix is available, normally within 90 days
  of the report. Reporters are credited in the advisory unless they ask not to be.

## Supported versions

Fixes go to `main` and to the latest release. Rosette is pre-1.0; older releases are not patched.

## Scope

In scope: anything that lets traffic on the network crash, hang, corrupt, or take control of a Rosette device or a
host process using `rmw_rosette`. That includes the RTPS stack (`src/rtps`), the address layer (DHCP, link-local,
SLAAC, SNTP, mDNS), the WIZnet driver, `rmw_rosette`, and the micro-ROS agent client. The RTPS message parser, SPDP,
SEDP, the fragment assembler, and the endpoint match table are fuzzed in CI (`tests/spec/fuzz`).

Not a vulnerability: Rosette does not implement DDS-Security. Like a stock ROS 2 system without SROS 2, any
participant on the network can discover a Rosette device, publish to its topics, and call its services. Deploy it
on a network you trust, and isolate that network from untrusted ones. Resource exhaustion by a peer that follows the
protocol (for example, more participants than a board is sized for) is a sizing question, covered in
`docs/mcu_memory_model.md`; a report is still welcome if a board fails to recover once the load stops.
