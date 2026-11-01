// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/stub/stub_littlefs.cpp
 * @brief Compile-only LittleFS block device: the volume never mounts.
 *
 * The storage half of the stub chip port (chip_stub.cpp). It is compiled
 * into rosette_storage, the library that has littlefs on its include path,
 * through ROSETTE_CHIP_STORAGE_SOURCES (port.cmake).
 *
 * Verification:
 * - Correct means: tests/port_stub links rosette_storage whole.
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
