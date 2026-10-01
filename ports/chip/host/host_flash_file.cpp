/**
 * @file platform/posix/host_littlefs.cpp
 * @brief File-backed LittleFS block device for host builds.
 *
 * Gives `ROSETTE_BOARD=posix` builds a real, persistent LittleFS volume so the
 * `rosette::storage` layer and its examples/tests exercise exactly the same
 * code path as the board. The volume is a fixed-size regular file; each
 * LittleFS block maps to a file offset. `ROSETTE_STORAGE_FILE` overrides the
 * path (default `rosette_storage.img` in the working directory), and
 * `ROSETTE_STORAGE_SIZE` overrides the size in bytes.
 *
 * The `rosette_platform_flash_*` primitives are also implemented here against
 * the same file so host tooling that pokes flash directly behaves consistently,
 * though the storage layer itself only ever goes through LittleFS.
 */

#include "rosette/platform/flash.hpp"
#include "rosette/platform/littlefs_platform.hpp"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <unistd.h>

namespace {

constexpr uint32_t kBlockSize = 4096u;
constexpr uint32_t kDefaultSize = 256u * 1024u;  // 64 blocks.

/**
 * @brief Returns the backing-file path, honoring `ROSETTE_STORAGE_FILE`.
 */
const char* backing_path() noexcept
{
    const char* env = std::getenv("ROSETTE_STORAGE_FILE");
    return (env != nullptr && env[0] != '\0') ? env : "rosette_storage.img";
}

/**
 * @brief Returns the backing-volume size, honoring `ROSETTE_STORAGE_SIZE`.
 */
uint32_t backing_size() noexcept
{
    if (const char* env = std::getenv("ROSETTE_STORAGE_SIZE")) {
        const unsigned long v = std::strtoul(env, nullptr, 0);
        if (v >= 2u * kBlockSize && (v % kBlockSize) == 0u) {
            return static_cast<uint32_t>(v);
        }
    }
    return kDefaultSize;
}

/**
 * @brief Opens (creating and zero-filling if needed) the backing file.
 *
 * @param[out] out_size Receives the volume size in bytes.
 * @return An open writable file descriptor, or -1 on failure.
 */
int open_backing(uint32_t& out_size) noexcept
{
    out_size = backing_size();
    const int fd = ::open(backing_path(), O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
        return -1;
    }
    if (::lseek(fd, 0, SEEK_END) < static_cast<off_t>(out_size)) {
        (void)::ftruncate(fd, static_cast<off_t>(out_size));
    }
    return fd;
}

int lfs_file_read(const struct lfs_config* c, lfs_block_t block, lfs_off_t off,
                  void* buffer, lfs_size_t size)
{
    const int fd = static_cast<int>(reinterpret_cast<intptr_t>(c->context));
    const off_t pos = static_cast<off_t>(block) * c->block_size + off;
    return (::pread(fd, buffer, size, pos) == static_cast<ssize_t>(size)) ? 0 : LFS_ERR_IO;
}

int lfs_file_prog(const struct lfs_config* c, lfs_block_t block, lfs_off_t off,
                  const void* buffer, lfs_size_t size)
{
    const int fd = static_cast<int>(reinterpret_cast<intptr_t>(c->context));
    const off_t pos = static_cast<off_t>(block) * c->block_size + off;
    return (::pwrite(fd, buffer, size, pos) == static_cast<ssize_t>(size)) ? 0 : LFS_ERR_IO;
}

int lfs_file_erase(const struct lfs_config* c, lfs_block_t block)
{
    const int fd = static_cast<int>(reinterpret_cast<intptr_t>(c->context));
    static uint8_t ones[kBlockSize];
    std::memset(ones, 0xFF, sizeof(ones));
    const off_t pos = static_cast<off_t>(block) * c->block_size;
    return (::pwrite(fd, ones, c->block_size, pos) == static_cast<ssize_t>(c->block_size))
               ? 0 : LFS_ERR_IO;
}

int lfs_file_sync(const struct lfs_config* c)
{
    return (::fsync(static_cast<int>(reinterpret_cast<intptr_t>(c->context))) == 0) ? 0 : LFS_ERR_IO;
}

}  // namespace

extern "C" rosette_flash_region rosette_platform_flash_region(void)
{
    rosette_flash_region region;
    region.offset = 0u;
    region.size = backing_size();
    return region;
}

extern "C" uint32_t rosette_platform_flash_sector_size(void) { return kBlockSize; }
extern "C" uint32_t rosette_platform_flash_page_size(void) { return 256u; }

extern "C" void rosette_platform_flash_erase(uint32_t addr, uint32_t size)
{
    uint32_t total = 0u;
    const int fd = open_backing(total);
    if (fd < 0) {
        return;
    }
    static uint8_t ones[kBlockSize];
    std::memset(ones, 0xFF, sizeof(ones));
    for (uint32_t done = 0u; done < size; done += kBlockSize) {
        (void)::pwrite(fd, ones, kBlockSize, static_cast<off_t>(addr + done));
    }
    (void)::close(fd);
}

extern "C" void rosette_platform_flash_write(uint32_t addr, const void* data, uint32_t size)
{
    uint32_t total = 0u;
    const int fd = open_backing(total);
    if (fd < 0) {
        return;
    }
    (void)::pwrite(fd, data, size, static_cast<off_t>(addr));
    (void)::close(fd);
}

extern "C" void rosette_platform_flash_read(uint32_t addr, void* data, uint32_t size)
{
    uint32_t total = 0u;
    const int fd = open_backing(total);
    if (fd < 0) {
        std::memset(data, 0xFF, size);
        return;
    }
    if (::pread(fd, data, size, static_cast<off_t>(addr)) != static_cast<ssize_t>(size)) {
        std::memset(data, 0xFF, size);
    }
    (void)::close(fd);
}

namespace rosette::platform {

bool littlefs_mount(LittleFsVolume& volume) noexcept
{
    if (volume.mounted) {
        return true;
    }

    uint32_t size = 0u;
    const int fd = open_backing(size);
    if (fd < 0) {
        return false;
    }
    volume.context = reinterpret_cast<void*>(static_cast<intptr_t>(fd));

    lfs_config& cfg = volume.cfg;
    cfg = lfs_config{};
    cfg.context = volume.context;
    cfg.read = &lfs_file_read;
    cfg.prog = &lfs_file_prog;
    cfg.erase = &lfs_file_erase;
    cfg.sync = &lfs_file_sync;
    cfg.read_size = 64u;
    cfg.prog_size = 256u;
    cfg.block_size = kBlockSize;
    cfg.block_count = size / kBlockSize;
    cfg.cache_size = 256u;
    cfg.lookahead_size = 32u;
    cfg.block_cycles = 500;

    if (lfs_mount(&volume.lfs, &cfg) == 0) {
        volume.mounted = true;
        return true;
    }
    if (lfs_format(&volume.lfs, &cfg) != 0 || lfs_mount(&volume.lfs, &cfg) != 0) {
        (void)::close(fd);
        return false;
    }
    volume.mounted = true;
    return true;
}

void littlefs_unmount(LittleFsVolume& volume) noexcept
{
    if (volume.mounted) {
        (void)lfs_unmount(&volume.lfs);
        (void)::close(static_cast<int>(reinterpret_cast<intptr_t>(volume.context)));
        volume.mounted = false;
    }
}

}  // namespace rosette::platform
