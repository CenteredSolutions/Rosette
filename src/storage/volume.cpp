/**
 * @file storage/volume.cpp
 * @brief LittleFS file operations behind `rosette::storage::Volume`.
 *
 * Implements whole-file read, atomic whole-file write (temp file + rename),
 * remove, existence check, and directory listing over a mounted `lfs_t`. The
 * atomic-write path is what gives `DeviceConfig` and `MotorStore` power-fail
 * safety without either of them re-implementing journaling.
 */

#include "rosette/storage/volume.hpp"

#include <lfs.h>

#include <string.h>

namespace rosette::storage {

namespace {

/**
 * @brief Appends `.tmp` to @p path in @p out.
 *
 * @param[in] path Source path.
 * @param[out] out Destination buffer of at least `kMaxPath + 4` bytes.
 * @param[in] cap Capacity of @p out.
 * @return `true` when the suffixed path fit.
 */
bool temp_path(const char* path, char* out, size_t cap) noexcept
{
    const size_t n = strnlen(path, cap);
    if (n + 5u > cap) {
        return false;
    }
    memcpy(out, path, n);
    memcpy(out + n, ".tmp", 5u);
    return true;
}

}  // namespace

size_t Volume::read_file(const char* path, void* out, size_t cap) noexcept
{
    if (!mounted_) {
        return 0u;
    }

    lfs_file_t file;
    if (lfs_file_open(&volume_.lfs, &file, path, LFS_O_RDONLY) != 0) {
        return 0u;
    }

    const lfs_soff_t size = lfs_file_size(&volume_.lfs, &file);
    if (size <= 0 || static_cast<size_t>(size) > cap) {
        (void)lfs_file_close(&volume_.lfs, &file);
        return 0u;
    }

    const lfs_ssize_t read = lfs_file_read(&volume_.lfs, &file, out, static_cast<lfs_size_t>(size));
    (void)lfs_file_close(&volume_.lfs, &file);
    return (read == size) ? static_cast<size_t>(read) : 0u;
}

bool Volume::write_file(const char* path, const void* data, size_t len) noexcept
{
    if (!mounted_) {
        return false;
    }

    char tmp[kMaxPath + 5u];
    if (!temp_path(path, tmp, sizeof(tmp))) {
        return false;
    }

    lfs_file_t file;
    if (lfs_file_open(&volume_.lfs, &file, tmp,
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) != 0) {
        return false;
    }

    bool ok = true;
    if (len > 0u) {
        const lfs_ssize_t written =
            lfs_file_write(&volume_.lfs, &file, data, static_cast<lfs_size_t>(len));
        ok = (written == static_cast<lfs_ssize_t>(len));
    }
    if (ok) {
        ok = (lfs_file_sync(&volume_.lfs, &file) == 0);
    }
    (void)lfs_file_close(&volume_.lfs, &file);

    if (!ok) {
        (void)lfs_remove(&volume_.lfs, tmp);
        return false;
    }

    // lfs_rename replaces the destination atomically.
    if (lfs_rename(&volume_.lfs, tmp, path) != 0) {
        (void)lfs_remove(&volume_.lfs, tmp);
        return false;
    }
    return true;
}

bool Volume::remove_file(const char* path) noexcept
{
    if (!mounted_) {
        return false;
    }
    const int rc = lfs_remove(&volume_.lfs, path);
    return rc == 0 || rc == LFS_ERR_NOENT;
}

bool Volume::exists(const char* path) noexcept
{
    if (!mounted_) {
        return false;
    }
    struct lfs_info info;
    return lfs_stat(&volume_.lfs, path, &info) == 0 && info.type == LFS_TYPE_REG;
}

bool Volume::make_dir(const char* dir) noexcept
{
    if (!mounted_) {
        return false;
    }
    const int rc = lfs_mkdir(&volume_.lfs, dir);
    return rc == 0 || rc == LFS_ERR_EXIST;
}

uint8_t Volume::list_dir(const char* dir, char* names, size_t name_cap, uint8_t count) noexcept
{
    if (!mounted_ || names == nullptr || name_cap == 0u || count == 0u) {
        return 0u;
    }

    lfs_dir_t handle;
    if (lfs_dir_open(&volume_.lfs, &handle, dir) != 0) {
        return 0u;
    }

    uint8_t found = 0u;
    struct lfs_info info;
    while (found < count && lfs_dir_read(&volume_.lfs, &handle, &info) > 0) {
        if (info.type != LFS_TYPE_REG) {
            continue;
        }
        char* slot = names + static_cast<size_t>(found) * name_cap;
        const size_t n = strnlen(info.name, name_cap - 1u);
        memcpy(slot, info.name, n);
        slot[n] = '\0';
        ++found;
    }

    (void)lfs_dir_close(&volume_.lfs, &handle);
    return found;
}

}  // namespace rosette::storage
