// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/persistence.cpp
 * @brief LittleFS-backed persistence backend implementation.
 */

#include "persistence.hpp"
#include <string.h>

namespace rosette::rtps {

static bool ensure_parent_dirs(lfs_t *lfs, const char *path) noexcept
{
    if (!lfs || !path || path[0] == '\0') {
        return false;
    }

    char tmp[128];
    strncpy(tmp, path, sizeof(tmp) - 1u);
    tmp[sizeof(tmp) - 1u] = '\0';

    for (char *p = tmp + 1; *p; ++p) {
        if (*p != '/') {
            continue;
        }
        *p = '\0';
        int err = lfs_mkdir(lfs, tmp);
        if (err != 0 && err != LFS_ERR_EXIST) {
            return false;
        }
        *p = '/';
    }

    return true;
}

uint32_t LittleFsPersistenceBackend::crc32(const void* data, uint32_t len) noexcept
{
    const auto* bytes = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFF'FFFFu;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= bytes[i];
        for (int b = 0; b < 8; ++b) {
            const uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1u)));
            crc = (crc >> 1u) ^ (0xEDB8'8320u & mask);
        }
    }
    return ~crc;
}

static uint32_t crc32_extend(uint32_t seed, const void* data, uint32_t len) noexcept
{
    const auto* bytes = static_cast<const uint8_t*>(data);
    uint32_t crc = ~seed;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= bytes[i];
        for (int b = 0; b < 8; ++b) {
            const uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1u)));
            crc = (crc >> 1u) ^ (0xEDB8'8320u & mask);
        }
    }
    return ~crc;
}

bool LittleFsPersistenceBackend::load(const Guid& writer_guid,
                                      CacheChange& out) noexcept
{
    if (!lfs_) {
        return false;
    }

    const WriterMapping* m = find_mapping(writer_guid.entityId);
    if (!m || !m->path) {
        return false;
    }

    lfs_file_t file;
    int err = lfs_file_open(lfs_, &file, m->path, LFS_O_RDONLY);
    if (err < 0) {
        return false;
    }

    LittleFsPersistenceHeader hdr{};
    lfs_ssize_t n = lfs_file_read(lfs_, &file, &hdr, sizeof(hdr));
    if (n != static_cast<lfs_ssize_t>(sizeof(hdr))) {
        (void)lfs_file_close(lfs_, &file);
        return false;
    }

    if (hdr.magic != LittleFsPersistenceHeader::kMagic ||
        hdr.version != LittleFsPersistenceHeader::kVersion) {
        (void)lfs_file_close(lfs_, &file);
        return false;
    }

    if (memcmp(hdr.entityId, &writer_guid.entityId, sizeof(hdr.entityId)) != 0) {
        (void)lfs_file_close(lfs_, &file);
        return false;
    }

    if (hdr.payloadSize > kPoolMaxBlockSize) {
        (void)lfs_file_close(lfs_, &file);
        return false;
    }

    BufRef payload = buf_alloc(hdr.payloadSize);
    if (!payload.valid()) {
        (void)lfs_file_close(lfs_, &file);
        return false;
    }

    if (hdr.payloadSize > 0u) {
        n = lfs_file_read(lfs_, &file, const_cast<Octet*>(payload.data), hdr.payloadSize);
        if (n != static_cast<lfs_ssize_t>(hdr.payloadSize)) {
            payload.release();
            (void)lfs_file_close(lfs_, &file);
            return false;
        }
    }
    (void)lfs_file_close(lfs_, &file);

    payload.size = hdr.payloadSize;

    LittleFsPersistenceHeader check = hdr;
    check.crc32 = 0u;
    uint32_t computed = LittleFsPersistenceBackend::crc32(&check, sizeof(check));
    if (payload.size > 0u) {
        computed = crc32_extend(computed, payload.data, payload.size);
    }
    if (computed != hdr.crc32) {
        payload.release();
        return false;
    }

    out.kind = ChangeKind::ALIVE;
    out.writerGuid = writer_guid;
    out.sequenceNumber = SequenceNumber{hdr.snHigh, hdr.snLow};
    out.sourceTimestamp = TIME_INVALID;
    out.payload = static_cast<BufRef&&>(payload);
    return true;
}

bool LittleFsPersistenceBackend::store(const CacheChange& change) noexcept
{
    if (!lfs_ || !change.payload.valid()) {
        return false;
    }

    const WriterMapping* m = find_mapping(change.writerGuid.entityId);
    if (!m || !m->path) {
        return false;
    }

    const uint32_t payload_size = change.payload.size;
    if (payload_size > kPoolMaxBlockSize) {
        return false;
    }

    if (!ensure_parent_dirs(lfs_, m->path)) {
        return false;
    }

    LittleFsPersistenceHeader hdr{};
    hdr.magic = LittleFsPersistenceHeader::kMagic;
    hdr.version = LittleFsPersistenceHeader::kVersion;
    memcpy(hdr.entityId, &change.writerGuid.entityId, sizeof(hdr.entityId));
    hdr.snHigh = change.sequenceNumber.high;
    hdr.snLow = change.sequenceNumber.low;
    hdr.payloadSize = payload_size;
    hdr.crc32 = 0u;
    hdr.crc32 = crc32(&hdr, sizeof(hdr));
    if (payload_size > 0u) {
        hdr.crc32 = crc32_extend(hdr.crc32, change.payload.data, payload_size);
    }

    lfs_file_t file;
    int err = lfs_file_open(lfs_, &file, m->path,
                            LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (err < 0) {
        return false;
    }

    lfs_ssize_t written = lfs_file_write(lfs_, &file, &hdr, sizeof(hdr));
    if (written != static_cast<lfs_ssize_t>(sizeof(hdr))) {
        (void)lfs_file_close(lfs_, &file);
        return false;
    }

    if (payload_size > 0u) {
        written = lfs_file_write(lfs_, &file, change.payload.data, payload_size);
        if (written != static_cast<lfs_ssize_t>(payload_size)) {
            (void)lfs_file_close(lfs_, &file);
            return false;
        }
    }

    if (lfs_file_sync(lfs_, &file) < 0) {
        (void)lfs_file_close(lfs_, &file);
        return false;
    }

    return lfs_file_close(lfs_, &file) == 0;
}

}  // namespace rosette::rtps
