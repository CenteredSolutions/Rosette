/**
 * @file storage/device_config.cpp
 * @brief Load/store for the versioned device-configuration record.
 *
 * The record on flash is `Header` immediately followed by a byte-for-byte copy
 * of `DeviceConfig`. `load()` reads the whole file, checks the magic, version,
 * and size, then verifies a CRC-32 over the payload before handing the caller a
 * populated struct. Any mismatch leaves the caller's struct at its defaults so
 * a corrupt or stale record can never brick startup. `store()` recomputes the
 * header and writes the record through `Volume::write_file()`, which is atomic.
 *
 * References:
 * - CRC-32 (IEEE 802.3), reflected, polynomial 0xEDB88320.
 */

#include "rosette/storage/device_config.hpp"

#include <string.h>

namespace rosette::storage {

namespace {

constexpr uint32_t kMagic = 0x52434647u;  // "RCFG"
constexpr uint16_t kVersion = 1u;

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
    if (n != sizeof(buffer)) {
        return false;
    }

    Header header;
    memcpy(&header, buffer, sizeof(header));
    if (header.magic != kMagic ||
        header.version != kVersion ||
        header.payload_size != sizeof(DeviceConfig)) {
        return false;
    }

    const uint8_t* payload = buffer + sizeof(Header);
    if (crc32(payload, sizeof(DeviceConfig)) != header.crc32) {
        return false;
    }

    memcpy(&out, payload, sizeof(DeviceConfig));
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
