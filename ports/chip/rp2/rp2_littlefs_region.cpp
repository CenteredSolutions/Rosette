/**
 * @file ports/chip/rp2/rp2_littlefs_region.cpp
 * @brief LittleFS block device for the RP2040 / RP2350 on-board flash.
 *
 * Binds a LittleFS `lfs_config` to the raw flash primitives in
 * `platform/flash.hpp` and mounts the volume that `rosette::storage` uses for
 * device and motor configuration. The volume lives in the flash region the
 * board reserves through `ROSETTE_STORAGE_FLASH_OFFSET` /
 * `ROSETTE_STORAGE_FLASH_SIZE` in `board_config.h`; the application linker
 * script must keep the image out of that range.
 *
 * `littlefs_mount()` attempts `lfs_mount()` once and, on failure (blank flash
 * on a new board, or an unreadable image), formats the region and retries. That
 * is the intended first-boot path: a factory-fresh board comes up with an empty
 * but valid volume.
 *
 * References:
 * - littlefs README, "Block device API": https://github.com/littlefs-project/littlefs
 * - RP2350 datasheet §5.1 (flash), Pico SDK `hardware/flash.h`
 */

#include "rosette/platform/flash.hpp"
#include "rosette/platform/littlefs_platform.hpp"

#include "board_config.h"

#include <string.h>

#ifndef ROSETTE_STORAGE_FLASH_SIZE
#define ROSETTE_STORAGE_FLASH_SIZE 0u
#endif
#ifndef ROSETTE_STORAGE_FLASH_OFFSET
#define ROSETTE_STORAGE_FLASH_OFFSET 0u
#endif

// LittleFS working-buffer sizes. cache_size trades RAM for fewer flash reads;
// one erase block per cache is a good balance on a controller board.
#define ROSETTE_LFS_READ_SIZE       64u
#define ROSETTE_LFS_PROG_SIZE       256u   ///< RP2040/RP2350 flash page.
#define ROSETTE_LFS_CACHE_SIZE      256u
#define ROSETTE_LFS_LOOKAHEAD_SIZE  32u
#define ROSETTE_LFS_BLOCK_CYCLES    500    ///< Wear-leveling erase cycles before relocation.

extern "C" rosette_flash_region rosette_platform_flash_region(void)
{
    rosette_flash_region region;
    region.offset = (uint32_t)ROSETTE_STORAGE_FLASH_OFFSET;
    region.size = (uint32_t)ROSETTE_STORAGE_FLASH_SIZE;
    return region;
}

extern "C" uint32_t rosette_platform_flash_sector_size(void) { return 4096u; }
extern "C" uint32_t rosette_platform_flash_page_size(void) { return 256u; }

/**
 * @brief LittleFS read callback: copies from mapped flash.
 *
 * @param[in] c Active LittleFS configuration; `context` holds the region base.
 * @param[in] block Erase block index within the volume.
 * @param[in] off Byte offset within the block.
 * @param[out] buffer Destination for @p size bytes.
 * @param[in] size Number of bytes to read.
 * @return `0` on success (flash reads do not fail).
 */
static int lfs_flash_read(const struct lfs_config* c, lfs_block_t block,
                          lfs_off_t off, void* buffer, lfs_size_t size)
{
    const uint32_t base = (uint32_t)(uintptr_t)c->context;
    rosette_platform_flash_read(base + block * c->block_size + off, buffer, size);
    return 0;
}

/**
 * @brief LittleFS program callback: writes one page-aligned span to flash.
 *
 * LittleFS only ever calls this with @p off and @p size that are multiples of
 * `prog_size`, so the underlying page-aligned flash program is always valid.
 *
 * @param[in] c Active LittleFS configuration.
 * @param[in] block Erase block index within the volume.
 * @param[in] off Page-aligned byte offset within the block.
 * @param[in] buffer Source bytes.
 * @param[in] size Page-aligned length.
 * @return `0` on success.
 */
static int lfs_flash_prog(const struct lfs_config* c, lfs_block_t block,
                          lfs_off_t off, const void* buffer, lfs_size_t size)
{
    const uint32_t base = (uint32_t)(uintptr_t)c->context;
    rosette_platform_flash_write(base + block * c->block_size + off, buffer, size);
    return 0;
}

/**
 * @brief LittleFS erase callback: erases one volume block.
 *
 * @param[in] c Active LittleFS configuration.
 * @param[in] block Erase block index within the volume.
 * @return `0` on success.
 */
static int lfs_flash_erase(const struct lfs_config* c, lfs_block_t block)
{
    const uint32_t base = (uint32_t)(uintptr_t)c->context;
    rosette_platform_flash_erase(base + block * c->block_size, c->block_size);
    return 0;
}

/**
 * @brief LittleFS sync callback.
 *
 * Flash programming is synchronous on the Pico, so there is nothing to flush.
 *
 * @param[in] c Active LittleFS configuration (unused).
 * @return `0` always.
 */
static int lfs_flash_sync(const struct lfs_config* c)
{
    (void)c;
    return 0;
}

namespace rosette::platform {

/**
 * @brief Fills @p volume's `lfs_config` for the board storage region.
 *
 * @param[in,out] volume Volume whose `cfg` and `context` are populated.
 * @return `true` when the board reserves a non-empty, block-aligned region.
 */
static bool configure(LittleFsVolume& volume) noexcept
{
    const rosette_flash_region region = rosette_platform_flash_region();
    const uint32_t block = rosette_platform_flash_sector_size();
    if (region.size < 2u * block || (region.size % block) != 0u) {
        return false;
    }

    volume.context = reinterpret_cast<void*>(static_cast<uintptr_t>(region.offset));

    lfs_config& cfg = volume.cfg;
    cfg = lfs_config{};
    cfg.context = volume.context;
    cfg.read = &lfs_flash_read;
    cfg.prog = &lfs_flash_prog;
    cfg.erase = &lfs_flash_erase;
    cfg.sync = &lfs_flash_sync;

    cfg.read_size = ROSETTE_LFS_READ_SIZE;
    cfg.prog_size = ROSETTE_LFS_PROG_SIZE;
    cfg.block_size = block;
    cfg.block_count = region.size / block;
    cfg.cache_size = ROSETTE_LFS_CACHE_SIZE;
    cfg.lookahead_size = ROSETTE_LFS_LOOKAHEAD_SIZE;
    cfg.block_cycles = ROSETTE_LFS_BLOCK_CYCLES;
    return true;
}

bool littlefs_mount(LittleFsVolume& volume) noexcept
{
    if (volume.mounted) {
        return true;
    }
    if (!configure(volume)) {
        return false;
    }

    if (lfs_mount(&volume.lfs, &volume.cfg) == 0) {
        volume.mounted = true;
        return true;
    }

    // Blank or corrupt region: format once and retry. This is the first-boot
    // path on a factory-fresh board.
    if (lfs_format(&volume.lfs, &volume.cfg) != 0) {
        return false;
    }
    if (lfs_mount(&volume.lfs, &volume.cfg) != 0) {
        return false;
    }
    volume.mounted = true;
    return true;
}

void littlefs_unmount(LittleFsVolume& volume) noexcept
{
    if (volume.mounted) {
        (void)lfs_unmount(&volume.lfs);
        volume.mounted = false;
    }
}

}  // namespace rosette::platform
