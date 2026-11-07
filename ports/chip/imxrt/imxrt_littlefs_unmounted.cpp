// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/imxrt/imxrt_littlefs_unmounted.cpp
 * @brief The i.MX RT port's LittleFS hooks: there is no storage volume, so none mounts.
 *
 * The storage half of the i.MX RT chip port, compiled into rosette_storage
 * (the library with littlefs on its include path) through
 * ROSETTE_CHIP_STORAGE_SOURCES. The port reserves no flash region
 * (imxrt_flash_region.c), so rosette::storage::Volume reports the volume
 * unmounted and DeviceConfig keeps its defaults.
 *
 * Verification:
 * - Correct means: littlefs_mount() returns false and leaves `mounted` false.
 * - Covered by: examples/storage_demo on an i.MX RT board (reports no volume).
 */

#include "rosette/platform/littlefs_platform.hpp"

namespace rosette::platform {

bool littlefs_mount(LittleFsVolume& volume) noexcept {
    volume.mounted = false;
    return false;
}

void littlefs_unmount(LittleFsVolume& volume) noexcept {
    volume.mounted = false;
}

}  // namespace rosette::platform
