// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file examples/storage_demo.cpp
 * @brief Exercises the Rosette storage layer on the host.
 *
 * Mounts the LittleFS volume (a backing file under `ROSETTE_BOARD=posix`),
 * round-trips a `DeviceConfig` record, and puts / gets / lists / erases opaque
 * blobs through a `FileStore`. The blob contents here are just bytes — the
 * store never parses them; a real application would keep its own format
 * (JSON, a packed struct, ...) on top.
 *
 *   ./storage_demo
 *   ROSETTE_STORAGE_FILE=/tmp/rosette.img ./storage_demo   # pick the backing file
 */

#include "rosette/storage/device_config.hpp"
#include "rosette/storage/file_store.hpp"
#include "rosette/storage/volume.hpp"

#include <cstdio>
#include <cstring>

namespace {

int failures = 0;

/**
 * @brief Reports a check result and tracks failures.
 */
void check(const char* what, bool ok) noexcept
{
    std::printf("  [%s] %s\n", ok ? "ok" : "FAIL", what);
    if (!ok) {
        ++failures;
    }
}

}  // namespace

int main()
{
    rosette::storage::Volume volume;
    std::printf("volume mounted: %s\n", volume.ok() ? "yes" : "no");
    if (!volume.ok()) {
        std::printf("could not mount the storage volume\n");
        return 1;
    }

    // --- DeviceConfig round-trip -------------------------------------------
    std::printf("device config:\n");
    {
        rosette::storage::DeviceConfig cfg;
        const bool had_stored = rosette::storage::load(volume, cfg);
        std::printf("  loaded %s\n", had_stored ? "stored record" : "defaults");

        cfg.net_mode = rosette::storage::NetMode::static_ip;
        cfg.static_ip[0] = 192; cfg.static_ip[1] = 168; cfg.static_ip[2] = 1; cfg.static_ip[3] = 50;
        std::snprintf(cfg.agent_host, sizeof(cfg.agent_host), "%s", "192.168.1.10");
        cfg.agent_port = 8888;
        cfg.client_key = 0x11223344u;

        check("store()", rosette::storage::store(volume, cfg));

        rosette::storage::DeviceConfig reloaded;
        const bool ok = rosette::storage::load(volume, reloaded);
        check("load() returns the stored record", ok);
        check("agent_host survived", std::strcmp(reloaded.agent_host, "192.168.1.10") == 0);
        check("client_key survived", reloaded.client_key == 0x11223344u);
        check("static_ip survived", reloaded.static_ip[3] == 50);
    }

    // --- FileStore: opaque blobs keyed by name ---------------------------
    std::printf("file store (/units):\n");
    {
        rosette::storage::FileStore store(volume, "/units", ".cfg");

        const char blob_a[] = "{\"any\":\"opaque payload\",\"n\":1}";
        const char blob_b[] = "second unit payload";

        check("put A", store.put("UNIT-A1", blob_a, std::strlen(blob_a)) ==
                       rosette::storage::FileStoreStatus::ok);
        check("put B", store.put("UNIT-B2", blob_b, std::strlen(blob_b)) ==
                       rosette::storage::FileStoreStatus::ok);
        check("reject bad key", store.put("bad key!", blob_a, 4) ==
                                rosette::storage::FileStoreStatus::bad_key);

        char out[128];
        size_t out_len = 0;
        const auto rc = store.get("UNIT-A1", out, sizeof(out), out_len);
        check("get A", rc == rosette::storage::FileStoreStatus::ok);
        check("get A byte-exact",
              out_len == std::strlen(blob_a) && std::memcmp(out, blob_a, out_len) == 0);

        check("exists B", store.exists("UNIT-B2"));
        check("missing key is not_found",
              store.get("NOPE", out, sizeof(out), out_len) ==
              rosette::storage::FileStoreStatus::not_found);

        char keys[8][rosette::storage::kFileStoreKeyMax + 1];
        const uint8_t n = store.list(keys, 8);
        std::printf("  list -> %u key(s):", n);
        for (uint8_t i = 0; i < n; ++i) {
            std::printf(" %s", keys[i]);
        }
        std::printf("\n");
        check("list sees both units", n == 2);

        check("erase A", store.erase("UNIT-A1") == rosette::storage::FileStoreStatus::ok);
        check("A gone after erase", !store.exists("UNIT-A1"));

        // Leave the volume clean for the next run.
        (void)store.erase("UNIT-B2");
    }

    std::printf("\n%s (%d failure%s)\n",
                failures == 0 ? "PASS" : "FAIL", failures, failures == 1 ? "" : "s");
    return failures == 0 ? 0 : 1;
}
