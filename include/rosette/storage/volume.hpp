// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file storage/volume.hpp
 * @brief RAII handle for the board's mounted LittleFS configuration volume.
 *
 * `Volume` mounts the board storage region on construction (formatting a blank
 * region on first boot) and unmounts on destruction. It exposes just enough of
 * the LittleFS file API for the configuration stores layered on top
 * (`DeviceConfig`, `MotorStore`): whole-file read, atomic whole-file write,
 * remove, and directory listing.
 *
 * Writes are atomic at the file level: the new contents go to a temporary file
 * that is renamed over the target, so a power loss mid-write leaves either the
 * old file or the new one, never a truncated file.
 *
 * One `Volume` should exist per process/boot. It is not copyable; move it if
 * ownership must transfer.
 */

#pragma once

#include "rosette/platform/littlefs_platform.hpp"

#include <stddef.h>
#include <stdint.h>

namespace rosette::storage {

/// Longest path the configuration stores use, including the leading slash.
inline constexpr size_t kMaxPath = 48u;

/**
 * @brief A mounted LittleFS volume backed by on-board flash.
 */
class Volume {
public:
    /**
     * @brief Mounts the board storage volume.
     *
     * On a board that reserves no storage region, or when the flash cannot be
     * mounted or formatted, `ok()` is false and every operation fails cleanly.
     */
    Volume() noexcept
    {
        mounted_ = rosette::platform::littlefs_mount(volume_);
    }

    Volume(const Volume&) = delete;
    Volume& operator=(const Volume&) = delete;

    Volume(Volume&& other) noexcept
        : volume_(other.volume_), mounted_(other.mounted_)
    {
        other.volume_ = rosette::platform::LittleFsVolume{};
        other.mounted_ = false;
    }

    Volume& operator=(Volume&& other) noexcept
    {
        if (this != &other) {
            close();
            volume_ = other.volume_;
            mounted_ = other.mounted_;
            other.volume_ = rosette::platform::LittleFsVolume{};
            other.mounted_ = false;
        }
        return *this;
    }

    ~Volume() { close(); }

    /**
     * @brief Reports whether the volume is mounted and usable.
     */
    bool ok() const noexcept { return mounted_; }

    /**
     * @brief Reads an entire file into @p out.
     *
     * @param[in] path Absolute file path (for example `/device.cfg`).
     * @param[out] out Destination buffer.
     * @param[in] cap Capacity of @p out in bytes.
     * @return Number of bytes read, or 0 when the file is missing, empty, or
     * larger than @p cap.
     */
    size_t read_file(const char* path, void* out, size_t cap) noexcept;

    /**
     * @brief Atomically replaces a file with @p data.
     *
     * Writes to `<path>.tmp`, syncs, then renames it over @p path.
     *
     * @param[in] path Absolute file path.
     * @param[in] data Source bytes.
     * @param[in] len Number of bytes to write.
     * @return `true` when the rename completed and the new contents are durable.
     */
    bool write_file(const char* path, const void* data, size_t len) noexcept;

    /**
     * @brief Removes a file if it exists.
     *
     * @param[in] path Absolute file path.
     * @return `true` when the file no longer exists after the call.
     */
    bool remove_file(const char* path) noexcept;

    /**
     * @brief Reports whether a file exists.
     *
     * @param[in] path Absolute file path.
     * @return `true` when @p path names a regular file.
     */
    bool exists(const char* path) noexcept;

    /**
     * @brief Lists the regular-file names in a directory.
     *
     * @param[in] dir Absolute directory path (for example `/motors`).
     * @param[out] names Row-major buffer of `count` name slots.
     * @param[in] name_cap Capacity of each name slot in bytes.
     * @param[in] count Number of name slots available.
     * @return Number of names written (capped at @p count).
     */
    uint8_t list_dir(const char* dir, char* names, size_t name_cap, uint8_t count) noexcept;

    /**
     * @brief Ensures a directory exists, creating it if needed.
     *
     * @param[in] dir Absolute directory path.
     * @return `true` when the directory exists after the call.
     */
    bool make_dir(const char* dir) noexcept;

private:
    void close() noexcept
    {
        if (mounted_) {
            rosette::platform::littlefs_unmount(volume_);
            mounted_ = false;
        }
    }

    rosette::platform::LittleFsVolume volume_{};
    bool mounted_ = false;
};

}  // namespace rosette::storage
