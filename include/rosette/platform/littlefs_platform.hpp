// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file platform/littlefs_platform.hpp
 * @brief Platform-owned LittleFS block-device contract for Rosette.
 *
 * Rosette keeps a small LittleFS volume on the board so device configuration
 * (network mode, static addressing, the micro-ROS Agent endpoint, the XRCE
 * client key, calibration blobs) survives power cycles and reflashing of the
 * application image.
 *
 * The chip port owns the flash geometry and the erase/program/read
 * primitives. It implements `littlefs_mount()` / `littlefs_unmount()` for its
 * MCU (ports/chip/<chip>/, through ROSETTE_CHIP_STORAGE_SOURCES), binding an
 * `lfs_config` to a reserved flash region. Application and Rosette code above
 * this layer only ever touch the mounted `lfs_t`; the block-device details
 * stay in the port.
 *
 * References:
 * - littlefs design: https://github.com/littlefs-project/littlefs/blob/master/DESIGN.md
 */

#pragma once

#include <lfs.h>

#include <stdint.h>

namespace rosette::platform {

/**
 * @brief Handle for one mounted LittleFS volume backed by on-board flash.
 *
 * The board layer fills `cfg` with its erase/program/read hooks and geometry,
 * mounts `lfs`, and sets `mounted`. `context` is an optional board-owned
 * pointer (for example a flash-partition descriptor) passed through to the
 * block-device callbacks.
 */
struct LittleFsVolume {
    lfs_t       lfs = {};
    lfs_config  cfg = {};
    void       *context = nullptr;
    bool        mounted = false;
};

/**
 * @brief Mounts the board's configuration LittleFS volume.
 *
 * Called once during board bring-up, before any Rosette component reads or
 * writes persistent configuration. The board implementation binds `volume.cfg`
 * to its reserved flash region, attempts `lfs_mount()`, and formats the region
 * on first boot or when the on-flash image is unreadable.
 *
 * @param[in,out] volume Volume handle populated and mounted on success.
 * @return `true` when the volume is mounted and ready for file operations.
 */
bool littlefs_mount(LittleFsVolume& volume) noexcept;

/**
 * @brief Unmounts a volume previously mounted by `littlefs_mount()`.
 *
 * Flushes any pending metadata and releases the block device. Safe to call on
 * an unmounted volume.
 *
 * @param[in,out] volume Volume handle to unmount; `mounted` is cleared.
 */
void littlefs_unmount(LittleFsVolume& volume) noexcept;

} // namespace rosette::platform
