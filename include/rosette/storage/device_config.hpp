/**
 * @file storage/device_config.hpp
 * @brief Persistent device configuration the firmware reads at boot.
 *
 * A single fixed-layout record stored at `/device.cfg` on the board storage
 * volume. It holds the values the micro-ROS client needs before it can reach
 * the Agent, so a deployed board can be re-addressed (new Agent endpoint,
 * static IP, namespace, client key) without recompiling or reflashing the
 * application image.
 *
 * The on-flash form is a small header `{magic, version, payload_size, crc32}`
 * followed by the payload. `load()` returns compiled defaults when the file is
 * missing, the header does not match, or the CRC fails, so the board always
 * boots with a usable configuration. `store()` writes the record atomically
 * through `Volume`. Version 2 appends `ntp_server`; `load()` still accepts a
 * version-1 record and leaves `ntp_server` unset.
 *
 * The firmware hands `ntp_server` to the network, where SNTP uses it:
 *
 *     if (cfg.ntp_server[0] != 0u) {
 *         network.time.server = rosette::net::make_socket_address_v4(
 *             {cfg.ntp_server[0], cfg.ntp_server[1], cfg.ntp_server[2], cfg.ntp_server[3]},
 *             rosette::net::kNtpServerPort);
 *     }
 *
 * This record is Rosette's own client configuration. Application-specific
 * configuration (per-unit parameters, calibration, and so on) belongs in a
 * `FileStore`, not here.
 */

#pragma once

#include "rosette/storage/volume.hpp"

#include <stdint.h>

namespace rosette::storage {

/// How the board obtains its IP address.
enum class NetMode : uint8_t {
    static_ip = 0,
    dhcp = 1,
    slaac = 2,
    zeroconf = 3,
};

/// Path of the device configuration record on the storage volume.
inline constexpr char kDeviceConfigPath[] = "/device.cfg";

/**
 * @brief Boot-time device configuration.
 *
 * Plain data, safe to copy. Field defaults are the values used when no valid
 * record is stored.
 */
struct DeviceConfig {
    NetMode net_mode = NetMode::dhcp;
    uint8_t static_ip[4] = {0, 0, 0, 0};
    uint8_t static_mask[4] = {255, 255, 255, 0};
    uint8_t static_gw[4] = {0, 0, 0, 0};
    uint8_t static_dns[4] = {8, 8, 8, 8};

    char agent_host[64] = {};        ///< Agent IPv4 literal or hostname; empty means "discover".
    uint16_t agent_port = 8888u;

    uint32_t client_key = 0xA1B2C3D4u;  ///< XRCE client key; set this per-board in production.
    uint16_t domain_id = 0u;
    char node_namespace[32] = {};

    /// NTP server IPv4 address for `NetworkConfig::time.server`; all zero for none (record v2).
    uint8_t ntp_server[4] = {0, 0, 0, 0};
};

/**
 * @brief Loads the stored device configuration, or defaults.
 *
 * @param[in] volume Mounted storage volume.
 * @param[out] out Populated from the stored record on success, left at its
 * constructed defaults otherwise.
 * @return `true` when a valid stored record was applied; `false` when defaults
 * were used (missing file, header mismatch, or CRC failure).
 */
bool load(Volume& volume, DeviceConfig& out) noexcept;

/**
 * @brief Stores the device configuration atomically.
 *
 * @param[in] volume Mounted storage volume.
 * @param[in] config Configuration to persist.
 * @return `true` when the record was written and is durable.
 */
bool store(Volume& volume, const DeviceConfig& config) noexcept;

}  // namespace rosette::storage
