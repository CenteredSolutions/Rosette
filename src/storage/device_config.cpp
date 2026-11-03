// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file storage/device_config.cpp
 * @brief Load/store for the versioned device-configuration record.
 *
 * The record on flash is `Header` immediately followed by a byte-for-byte copy
 * of `DeviceConfig`. `load()` reads the whole file, checks the magic, version,
 * and size, then verifies a CRC-32 over the payload before handing the caller a
 * populated struct. A version-1 record (the same layout without the trailing
 * `ntp_server`) is accepted, so boards keep their stored settings across the
 * version change. Any mismatch leaves the caller's struct at its defaults so
 * a corrupt or stale record can never brick startup. `store()` recomputes the
 * header and writes the record through `Volume::write_file()`, which is atomic.
 *
 * References:
 * - CRC-32 (IEEE 802.3), reflected, polynomial 0xEDB88320.
 */

#include "rosette/storage/device_config.hpp"

#include <stddef.h>
#include <string.h>

namespace rosette::storage {

namespace {

constexpr uint32_t kMagic = 0x52434647u;  // "RCFG"
constexpr uint16_t kVersion = 2u;

/// The version-1 payload: every field before `ntp_server`, padded as that struct was.
struct DeviceConfigV1 {
    NetMode net_mode;
    uint8_t static_ip[4];
    uint8_t static_mask[4];
    uint8_t static_gw[4];
    uint8_t static_dns[4];
    char agent_host[64];
    uint16_t agent_port;
    uint32_t client_key;
    uint16_t domain_id;
    char node_namespace[32];
};

static_assert(offsetof(DeviceConfig, ntp_server) <= sizeof(DeviceConfigV1),
              "version 2 must only append to the version-1 layout");

/**
 * @brief On-flash record header preceding the `DeviceConfig` payload.
 */
struct Header {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    uint32_t crc32;
};

/**
 * @brief Computes a reflected CRC-32 (IEEE 802.3) over @p data.
 *
 * A table-free bitwise implementation; the record is a few hundred bytes so the
 * per-byte loop cost is irrelevant and it keeps the footprint small.
 *
 * @param[in] data Input bytes.
 * @param[in] len Number of bytes.
 * @return The CRC-32 of the input.
 */
uint32_t crc32(const void* data, size_t len) noexcept
{
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= p[i];
        for (int bit = 0; bit < 8; ++bit) {
            const uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

}  // namespace

bool load(Volume& volume, DeviceConfig& out) noexcept
{
    out = DeviceConfig{};

    uint8_t buffer[sizeof(Header) + sizeof(DeviceConfig)];
    const size_t n = volume.read_file(kDeviceConfigPath, buffer, sizeof(buffer));
    if (n < sizeof(Header)) {
        return false;
    }

    Header header;
    memcpy(&header, buffer, sizeof(header));
    const bool current = header.version == kVersion && header.payload_size == sizeof(DeviceConfig);
    const bool v1 = header.version == 1u && header.payload_size == sizeof(DeviceConfigV1);
    if (header.magic != kMagic || !(current || v1) || n != sizeof(Header) + header.payload_size) {
        return false;
    }

    const uint8_t* payload = buffer + sizeof(Header);
    if (crc32(payload, header.payload_size) != header.crc32) {
        return false;
    }

    memcpy(&out, payload, header.payload_size);
    if (v1) {
        // The v1 tail padding overlaps the appended field; a v1 record names no NTP server.
        memset(out.ntp_server, 0, sizeof(out.ntp_server));
    }

    return true;
}

bool store(Volume& volume, const DeviceConfig& config) noexcept
{
    uint8_t buffer[sizeof(Header) + sizeof(DeviceConfig)];

    Header header;
    header.magic = kMagic;
    header.version = kVersion;
    header.payload_size = static_cast<uint16_t>(sizeof(DeviceConfig));
    header.crc32 = crc32(&config, sizeof(DeviceConfig));

    memcpy(buffer, &header, sizeof(header));
    memcpy(buffer + sizeof(Header), &config, sizeof(DeviceConfig));

    return volume.write_file(kDeviceConfigPath, buffer, sizeof(buffer));
}

}  // namespace rosette::storage
