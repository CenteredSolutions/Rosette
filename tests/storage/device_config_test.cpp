/**
 * @file tests/storage/device_config_test.cpp
 * @brief DeviceConfig records: a version-2 round trip, and a stored version-1 record still loading.
 *
 * Runs on the host volume (a backing file; `ROSETTE_STORAGE_FILE` picks it).
 *
 * - TC-STO-CFG-001: `store()` then `load()` returns every field, `ntp_server` included.
 * - TC-STO-CFG-002: a version-1 record (the layout before `ntp_server`)
 *   loads with its fields intact and `ntp_server` unset.
 * - TC-STO-CFG-003: a record with a bad CRC loads as defaults.
 */

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "rosette/storage/device_config.hpp"
#include "rosette/storage/volume.hpp"

namespace {

namespace sto = rosette::storage;

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) {
        ++g_failures;
    }
}

/// The version-1 payload layout, as firmware before `ntp_server` wrote it.
struct V1 {
    sto::NetMode net_mode;
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

struct Header {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    uint32_t crc32;
};

uint32_t crc32(const void* data, std::size_t len) {
    const auto* p = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= p[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
        }
    }

    return ~crc;
}

/// @brief Writes a version-1 record naming an agent, with its tail padding set to 0xFF.
bool write_v1(sto::Volume& volume, bool corrupt_crc) {
    uint8_t buffer[sizeof(Header) + sizeof(V1)];
    std::memset(buffer, 0xFF, sizeof(buffer));
    V1 v1{};
    std::memset(&v1, 0xFF, sizeof(v1));
    v1.net_mode = sto::NetMode::dhcp;
    std::snprintf(v1.agent_host, sizeof(v1.agent_host), "%s", "10.1.2.3");
    v1.agent_port = 9999u;
    v1.client_key = 0xCAFEF00Du;
    v1.domain_id = 7u;
    std::snprintf(v1.node_namespace, sizeof(v1.node_namespace), "%s", "unit7");

    const Header header{0x52434647u, 1u, static_cast<uint16_t>(sizeof(V1)),
                        crc32(&v1, sizeof(V1)) ^ (corrupt_crc ? 1u : 0u)};
    std::memcpy(buffer, &header, sizeof(header));
    std::memcpy(buffer + sizeof(header), &v1, sizeof(v1));
    return volume.write_file(sto::kDeviceConfigPath, buffer, sizeof(buffer));
}

}  // namespace

int main() {
    sto::Volume volume;
    if (!volume.ok()) {
        std::printf("could not mount the storage volume\n");
        return 1;
    }

    sto::DeviceConfig stored{};
    stored.client_key = 0x11223344u;
    stored.ntp_server[0] = 192u;
    stored.ntp_server[1] = 168u;
    stored.ntp_server[2] = 1u;
    stored.ntp_server[3] = 2u;
    sto::DeviceConfig loaded{};
    const bool round_trip = sto::store(volume, stored) && sto::load(volume, loaded);
    check(round_trip && loaded.client_key == 0x11223344u && loaded.ntp_server[0] == 192u
              && loaded.ntp_server[3] == 2u,
          "TC-STO-CFG-001 a version-2 record round-trips, ntp_server included");

    sto::DeviceConfig from_v1{};
    const bool v1_loaded = write_v1(volume, false) && sto::load(volume, from_v1);
    const bool no_ntp = from_v1.ntp_server[0] == 0u && from_v1.ntp_server[1] == 0u
                        && from_v1.ntp_server[2] == 0u && from_v1.ntp_server[3] == 0u;
    check(v1_loaded && std::strcmp(from_v1.agent_host, "10.1.2.3") == 0
              && from_v1.agent_port == 9999u && from_v1.client_key == 0xCAFEF00Du
              && from_v1.domain_id == 7u && std::strcmp(from_v1.node_namespace, "unit7") == 0,
          "TC-STO-CFG-002 a version-1 record keeps its fields");
    check(no_ntp, "TC-STO-CFG-002 a version-1 record names no NTP server");

    sto::DeviceConfig corrupt{};
    const bool corrupt_loaded = write_v1(volume, true) && sto::load(volume, corrupt);
    check(!corrupt_loaded && corrupt.agent_port == sto::DeviceConfig{}.agent_port,
          "TC-STO-CFG-003 a record with a bad CRC loads as defaults");

    std::printf(g_failures == 0 ? "ALL PASS\n" : "%d FAILURE(S)\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
