// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/persistence.hpp
 * @brief LittleFS-backed persistence backend for durable RTPS history.
 *
 * Implements persistent storage for KEEP_LAST(1) writer history caches
 * with TRANSIENT_LOCAL or higher durability (§8.7.2.2, p. 196).
 *
 * Only user-data changes persist: a CDR body written by write_async(), which
 * the writer frames with its encapsulation header at send time. Preframed
 * changes are SEDP announcements; they carry this boot's GUIDs and locators
 * and are rebuilt from the endpoint table on every boot, so they are never
 * stored.
 *
 * File layout
 * -----------
 * One LittleFS file per writer EntityId: a fixed header followed by exactly
 * `payload_size` octets of CDR body.
 *
 * @verbatim
 *   0               4       6       8              12
 *   +---------------+-------+-------+---------------+
 *   | magic 'RTPS'  | ver 1 | rsvd  | entity_id     |
 *   +---------------+-------+-------+---------------+
 *   | seq_num_high  | seq_num_low   | payload_size  |
 *   +---------------+---------------+---------------+
 *   | crc32: header with crc32 = 0, then payload    |
 *   +-----------------------------------------------+
 *   | payload_size octets of CDR body               |
 *   +-----------------------------------------------+
 * @endverbatim
 *
 * load() rejects a file whose magic, version, entity_id or CRC-32 does not
 * match. store() rewrites the mapped file through LittleFS, whose
 * copy-on-write commit leaves the previous file intact if power fails.
 *
 * Platform hook
 * -------------
 * The RTPS layer only depends on a mounted `lfs_t` instance. Platform code in
 * `platform/` is responsible for providing the LittleFS block device and mount
 * lifecycle, typically by implementing the callbacks declared in
 * `platform/littlefs_platform.hpp`.
 */

#pragma once

#include <lfs.h>

#include "buf_pool.hpp"
#include "history_cache.hpp"
#include "types.hpp"

namespace rosette::rtps {

struct LittleFsPersistenceHeader {
    static constexpr uint32_t kMagic   = 0x52545053u;
    static constexpr uint16_t kVersion = 0x0001u;

    uint32_t magic       = 0;
    uint16_t version     = 0;
    uint16_t reserved    = 0;
    Octet    entityId[4] = {};
    Long     snHigh      = 0;
    ULong    snLow       = 0;
    uint32_t payloadSize = 0;
    uint32_t crc32       = 0;
};

static_assert(sizeof(LittleFsPersistenceHeader) <= 32u,
              "LittleFsPersistenceHeader unexpectedly large");

/**
 * @brief Durable storage for the last change of a TRANSIENT_LOCAL user-data writer.
 *
 * A writer's history holds HistoryChange slots, which carry no writer GUID,
 * so the writer names itself by EntityId. load() yields a change that
 * add_change() takes back unaltered, so a writer restores its last sample
 * after a reboot and replays it to late-joining readers.
 *
 * Verification:
 * - Correct means: store() then load() for one writer yields a change with
 *   the stored sequence number and payload octets, kind ALIVE and preframed
 *   false; store() refuses a preframed or non-ALIVE change; a corrupted file, a file
 *   stored for another writer, or a writer with no file loads nothing.
 * - Covered by: TC-PER-8.7.2.2-001 to TC-PER-8.7.2.2-004.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.7.2.2 (TRANSIENT_LOCAL and PERSISTENT durability)
 */
struct IPersistenceBackend {
    virtual ~IPersistenceBackend() = default;

    /**
     * @brief Reads `writer`'s stored change.
     *
     * @param[in]  writer EntityId of the writer whose change to read.
     * @param[out] out    On success, the stored change: kind ALIVE, its
     *   sequence number, preframed false, and a pool block holding the CDR
     *   body. Unchanged on failure.
     * @return true when a stored change was read; false when the writer has
     *   none, the file fails its checks, or the pool has no block for it.
     */
    virtual bool load(const EntityId& writer, HistoryChange& out) noexcept = 0;

    /**
     * @brief Replaces `writer`'s stored change with `change`.
     *
     * @param[in] writer EntityId of the writer that owns the change.
     * @param[in] change The change to keep: an ALIVE change with a CDR body
     *   payload, not preframed.
     * @return true when the change is on storage; false when it is preframed,
     *   not ALIVE, or has no payload, the writer has no file, or the write fails. A
     *   failed write may leave a file that load() rejects.
     */
    virtual bool store(const EntityId& writer, const HistoryChange& change) noexcept = 0;
};

#ifndef RTPS_MAX_PERSISTENT_WRITERS
#  define RTPS_MAX_PERSISTENT_WRITERS 4
#endif

/**
 * @brief IPersistenceBackend on a mounted LittleFS volume, one file per writer.
 *
 * Example:
 * @code{.cpp}
 * static const LittleFsPersistenceBackend::WriterMapping kFiles[] = {
 *     {map_writer_id, "/rtps/map.bin"},
 * };
 * LittleFsPersistenceBackend backend(&lfs, kFiles, 1u);
 * HistoryChange last{};
 * if (backend.load(map_writer_id, last)) {
 *     writer.add_change(static_cast<HistoryChange&&>(last));
 * }
 * @endcode
 * Full program: tests/spec/src/test_persistence_8_7_2_2.cpp
 */
class LittleFsPersistenceBackend : public IPersistenceBackend {
public:
    /// Which file holds which writer's change.
    struct WriterMapping {
        EntityId    entityId = {};
        const char* path = nullptr;  ///< Absolute path; parent directories are created.
    };

    /**
     * @brief Binds the backend to a mounted volume and its writer-to-file map.
     *
     * @param[in] lfs      Mounted LittleFS volume; must outlive the backend.
     * @param[in] mappings Writer-to-file map, copied; its paths must outlive
     *   the backend.
     * @param[in] count    Entries in `mappings`; those past
     *   RTPS_MAX_PERSISTENT_WRITERS are ignored.
     */
    LittleFsPersistenceBackend(lfs_t                *lfs,
                               const WriterMapping  *mappings,
                               uint8_t               count) noexcept
        : lfs_(lfs)
    {
        uint8_t n = count < RTPS_MAX_PERSISTENT_WRITERS
                    ? count : RTPS_MAX_PERSISTENT_WRITERS;
        for (uint8_t i = 0; i < n; ++i) {
            mappings_[i] = mappings[i];
        }

        count_ = n;
    }

    bool load(const EntityId& writer, HistoryChange& out) noexcept override;
    bool store(const EntityId& writer, const HistoryChange& change) noexcept override;

private:
    lfs_t         *lfs_ = nullptr;
    WriterMapping  mappings_[RTPS_MAX_PERSISTENT_WRITERS] = {};
    uint8_t        count_ = 0;

    const WriterMapping* find_mapping(const EntityId& eid) const noexcept {
        for (uint8_t i = 0; i < count_; ++i) {
            if (mappings_[i].entityId == eid) {
                return &mappings_[i];
            }
        }

        return nullptr;
    }

    static uint32_t crc32(const void* data, uint32_t len) noexcept;
};

using FlashPersistenceBackend = LittleFsPersistenceBackend;

}  // namespace rosette::rtps
