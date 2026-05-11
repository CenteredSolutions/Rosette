/**
 * @file platform/littlefs_platform.hpp
 * @brief Platform-owned LittleFS block-device contract for Rosette.
 *
 * Board support code should implement the callbacks described by this header
 * and mount a LittleFS instance before constructing `LittleFsPersistenceBackend`.
 * The RTPS layer only consumes the mounted `lfs_t`; erase/program/read details
 * stay in `platform/`.
 */

#pragma once

#include "../../littlefs/lfs.h"
#include <stdint.h>

namespace rtps::platform {

struct LittleFsVolume {
    lfs_t       lfs = {};
    lfs_config  cfg = {};
    void       *context = nullptr;
    bool        mounted = false;
};

bool littlefs_mount(LittleFsVolume& volume) noexcept;
void littlefs_unmount(LittleFsVolume& volume) noexcept;

} // namespace rtps::platform
