#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
"""Serve upstream-compatible Micro XRCE-DDS discovery replies over multicast.

Design goal
-----------
Provide a tiny host-side responder that speaks the same DDS-XRCE discovery
request and reply format used by the upstream Micro XRCE-DDS client and agent.

Why this exists
---------------
The upstream helper is documented as Linux-only.  Rosette uses the same wire
format on MCU targets, and this responder gives us a simple reference endpoint
for local testing or for environments where the stock Agent discovery helper is
not available.

References
----------
- eProsima Micro XRCE-DDS Client `discovery.c`: `GET_INFO` probe sent to
  `239.255.0.2:7400`.
- eProsima Micro XRCE-DDS Agent `Processor::process_get_info_packet()`: `INFO`
  reply with agent configuration and transport locator sequence.
"""

from __future__ import annotations

import argparse
import ipaddress
import socket
from dataclasses import dataclass


SESSION_ID_WITHOUT_CLIENT_KEY = 0x80
STREAM_ID_NONE = 0x00
REQUEST_ID = bytes((0x00, 0x09))
OBJECT_ID_AGENT = bytes((0xFF, 0xFD))

SUBMESSAGE_ID_GET_INFO = 0x02
SUBMESSAGE_ID_INFO = 0x06
FLAG_LITTLE_ENDIANNESS = 0x01

OBJECT_KIND_AGENT = 0x0D
ADDRESS_FORMAT_MEDIUM = 0x01

INFO_CONFIGURATION = 0x01 << 0
INFO_ACTIVITY = 0x01 << 1
STATUS_OK = 0x00
IMPLEMENTATION_STATUS_UNKNOWN = 0x00

XRCE_COOKIE = b"XRCE"
XRCE_VERSION = bytes((0x01, 0x00))
XRCE_VENDOR_INVALID = bytes((0x00, 0x00))


class LittleEndianCdrWriter:
    """Minimal CDR writer for the subset used by XRCE discovery packets."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def align(self, alignment: int) -> None:
        if alignment <= 1:
            return
        padding = (alignment - (len(self.buffer) % alignment)) % alignment
        if padding:
            self.buffer.extend(b"\x00" * padding)

    def write_u8(self, value: int) -> None:
        self.buffer.append(value & 0xFF)

    def write_bool(self, value: bool) -> None:
        self.write_u8(1 if value else 0)

    def write_u16(self, value: int) -> None:
        self.align(2)
        self.buffer.extend((value & 0xFF, (value >> 8) & 0xFF))

    def write_i16(self, value: int) -> None:
        if value < 0:
            value = (1 << 16) + value
        self.write_u16(value)

    def write_u32(self, value: int) -> None:
        self.align(4)
        self.buffer.extend(
            (
                value & 0xFF,
                (value >> 8) & 0xFF,
                (value >> 16) & 0xFF,
                (value >> 24) & 0xFF,
            )
        )

    def write_bytes(self, value: bytes) -> None:
        self.buffer.extend(value)

    def to_bytes(self) -> bytes:
        return bytes(self.buffer)


class LittleEndianCdrReader:
    """Minimal CDR reader for the subset used by XRCE discovery packets."""

    def __init__(self, payload: bytes) -> None:
        self.payload = payload
        self.offset = 0

    def align(self, alignment: int) -> None:
        if alignment <= 1:
            return
        self.offset += (alignment - (self.offset % alignment)) % alignment

    def require(self, length: int) -> None:
        if self.offset + length > len(self.payload):
            raise ValueError("truncated packet")

    def read_u8(self) -> int:
        self.require(1)
        value = self.payload[self.offset]
        self.offset += 1
        return value

    def read_u16(self) -> int:
        self.align(2)
        self.require(2)
        value = self.payload[self.offset] | (self.payload[self.offset + 1] << 8)
        self.offset += 2
        return value

    def read_u32(self) -> int:
        self.align(4)
        self.require(4)
        value = (
            self.payload[self.offset]
            | (self.payload[self.offset + 1] << 8)
            | (self.payload[self.offset + 2] << 16)
            | (self.payload[self.offset + 3] << 24)
        )
        self.offset += 4
        return value

    def read_bytes(self, length: int) -> bytes:
        self.require(length)
        value = self.payload[self.offset : self.offset + length]
        self.offset += length
        return value


@dataclass(frozen=True)
class DiscoveryRequest:
    """Structured view of the incoming upstream `GET_INFO` discovery packet."""

    request_id: bytes
    object_id: bytes
    info_mask: int

    @classmethod
    def parse(cls, payload: bytes) -> "DiscoveryRequest | None":
        """Decode a packet if it matches the upstream XRCE `GET_INFO` layout."""
        try:
            reader = LittleEndianCdrReader(payload)
            session_id = reader.read_u8()
            stream_id = reader.read_u8()
            sequence_number = reader.read_u16()
            submessage_id = reader.read_u8()
            flags = reader.read_u8()
            submessage_length = reader.read_u16()
            request_id = reader.read_bytes(2)
            object_id = reader.read_bytes(2)
            info_mask = reader.read_u32()
        except ValueError:
            return None

        if (
            session_id != SESSION_ID_WITHOUT_CLIENT_KEY
            or stream_id != STREAM_ID_NONE
            or sequence_number != 0
            or submessage_id != SUBMESSAGE_ID_GET_INFO
            or (flags & FLAG_LITTLE_ENDIANNESS) == 0
            or submessage_length != 8
            or request_id != REQUEST_ID
            or object_id != OBJECT_ID_AGENT
        ):
            return None

        return cls(request_id=request_id, object_id=object_id, info_mask=info_mask)


def parse_args() -> argparse.Namespace:
    """Parse command-line flags for the responder process."""
    parser = argparse.ArgumentParser(
        description="Respond to upstream Micro XRCE-DDS multicast discovery probes."
    )
    parser.add_argument(
        "--group",
        default="239.255.0.2",
        help="IPv4 multicast group to listen on.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=7400,
        help="UDP port to listen on for discovery probes.",
    )
    parser.add_argument(
        "--agent-port",
        type=int,
        default=8888,
        help="UDP port where the micro-ROS Agent accepts XRCE traffic.",
    )
    parser.add_argument(
        "--agent-address",
        default="",
        help=(
            "Optional explicit IPv4 address to advertise for the Agent. "
            "When omitted, the responder source IP should already be the Agent host."
        ),
    )
    parser.add_argument(
        "--bind",
        default="0.0.0.0",
        help="Local IPv4 address to bind for multicast receive.",
    )
    return parser.parse_args()


def make_socket(bind_address: str, group: str, port: int) -> socket.socket:
    """Create and configure the multicast receive socket."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except OSError:
        pass

    sock.bind((bind_address, port))
    membership = socket.inet_aton(group) + socket.inet_aton("0.0.0.0")
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
    return sock


def encode_info_reply(request: DiscoveryRequest, agent_ipv4: bytes, agent_port: int) -> bytes:
    """Serialize an upstream-compatible XRCE `INFO` reply packet."""
    payload = LittleEndianCdrWriter()
    payload.write_bytes(request.request_id)
    payload.write_bytes(request.object_id)
    payload.write_u8(STATUS_OK)
    payload.write_u8(IMPLEMENTATION_STATUS_UNKNOWN)

    payload.write_bool(True)
    payload.write_u8(OBJECT_KIND_AGENT)
    payload.write_bytes(XRCE_COOKIE)
    payload.write_bytes(XRCE_VERSION)
    payload.write_bytes(XRCE_VENDOR_INVALID)
    payload.write_bool(False)

    payload.write_bool(True)
    payload.write_u8(OBJECT_KIND_AGENT)
    payload.write_i16(1)
    payload.write_u32(1)
    payload.write_u8(ADDRESS_FORMAT_MEDIUM)
    payload.write_bytes(agent_ipv4)
    payload.write_u16(agent_port)

    payload_bytes = payload.to_bytes()

    packet = LittleEndianCdrWriter()
    packet.write_u8(SESSION_ID_WITHOUT_CLIENT_KEY)
    packet.write_u8(STREAM_ID_NONE)
    packet.write_u16(0)
    packet.write_u8(SUBMESSAGE_ID_INFO)
    packet.write_u8(FLAG_LITTLE_ENDIANNESS)
    packet.write_u16(len(payload_bytes))
    packet.write_bytes(payload_bytes)
    return packet.to_bytes()


def resolve_agent_ipv4(agent_address: str, remote_host: str) -> bytes:
    """Resolve the IPv4 address that should be advertised in the INFO reply."""
    if agent_address:
        return ipaddress.IPv4Address(agent_address).packed

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        probe.connect((remote_host, 9))
        local_ip = probe.getsockname()[0]
    finally:
        probe.close()
    return ipaddress.IPv4Address(local_ip).packed


def run() -> int:
    """Run the multicast responder until interrupted."""
    args = parse_args()
    sock = make_socket(args.bind, args.group, args.port)

    print(
        f"[xrce-discovery] listening on {args.group}:{args.port}, "
        f"advertising agent port {args.agent_port}"
    )
    if args.agent_address:
        print(f"[xrce-discovery] advertising explicit agent address {args.agent_address}")

    while True:
        payload, remote = sock.recvfrom(2048)
        request = DiscoveryRequest.parse(payload)
        if request is None:
            continue

        agent_ipv4 = resolve_agent_ipv4(args.agent_address, remote[0])
        reply = encode_info_reply(request, agent_ipv4, args.agent_port)
        sock.sendto(reply, remote)


if __name__ == "__main__":
    raise SystemExit(run())
