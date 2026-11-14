// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/persistence.hpp
 * @brief LittleFS-backed persistence backend for durable RTPS history.
 *
 * Implements persistent storage for KEEP_LAST(1) writer history caches
 * with TRANSIENT_LOCAL or higher durability (§8.7.2.2, p. 196).
 *
 * Design
 * ------
 *  - One LittleFS file per writer EntityId.
 *  - The file starts with a fixed-size header followed by exactly
 *    `payload_size` bytes of serialized payload:
 *      magic[4]      = 0x52545053 ('RTPS')
 *      version[2]    = 0x0001
 *      flags[2]      = bit 0: the payload is preframed (HistoryChange::preframed)
 *      entity_id[4]  = writer EntityId bytes
 *      seq_num_high[4]
 *      seq_num_low[4]
 *      payload_size[4]
 *      crc32[4]      = CRC-32 of header-with-zero-crc + payload bytes
 *  - On load(): verify magic, version, and CRC-32. If invalid, return false.
 *  - On store(): rewrite the mapped file atomically from userspace through
 *    LittleFS. The board-specific erase/program/read hooks live behind the
 *    platform LittleFS block-device implementation.
 *
 * Platform hook
 * -------------
 * The RTPS layer only depends on a mounted `lfs_t` instance. Platform code in
 * `platform/` is responsible for providing the LittleFS block device and mount
 * lifecycle, typically by implementing the callbacks declared in
 * `platform/littlefs_platform.hpp`.
 */

#pragma once

#include "../platform/littlefs_platform.hpp"
#include "../../littlefs/lfs.h"
#include "buf_pool.hpp"
#include "history_cache.hpp"
#include "types.hpp"

namespace rosette::rtps {

struct LittleFsPersistenceHeader {
    static constexpr uint32_t kMagic   = 0x52545053u;
    static constexpr uint16_t kVersion = 0x0001u;
    static constexpr uint16_t kFlagPreframed = 0x0001u;

    uint32_t magic       = 0;
    uint16_t version     = 0;
    uint16_t flags = 0;  ///< kFlagPreframed, or 0 for a CDR body.
    Octet    entityId[4] = {};
    Long     snHigh      = 0;
    ULong    snLow       = 0;
    uint32_t payloadSize = 0;
    uint32_t crc32       = 0;
};

static_assert(sizeof(LittleFsPersistenceHeader) <= 32u,
              "LittleFsPersistenceHeader unexpectedly large");

/**
 * @brief Durable storage for the last change of a TRANSIENT_LOCAL writer.
 *
 * A writer's history holds HistoryChange slots, which carry no writer GUID,
 * so the writer names itself by EntityId and the backend stores and returns
 * the slot as the history holds it: load() yields a change that add_change()
 * takes back unaltered.
 *
 * Verification:
 * - Correct means: store() then load() for one writer yields a change with
 *   the stored sequence number, payload octets, kind ALIVE and preframed
 *   flag; a corrupted or foreign file loads nothing.
 * - Covered by: no test yet; a LittleFS RAM block device round trip would
 *   check it.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.7.2.2 (TRANSIENT_LOCAL and PERSISTENT durability)
 */
struct IPersistenceBackend {
    virtual ~IPersistenceBackend() = default;

    /// @brief Reads `writer`'s stored change into `out`; false when there is none or it is corrupt.
    virtual bool load(const EntityId& writer, HistoryChange& out) noexcept = 0;

    /// @brief Replaces `writer`'s stored change with `change`; false when it cannot be written.
    virtual bool store(const EntityId& writer, const HistoryChange& change) noexcept = 0;
};

#ifndef RTPS_MAX_PERSISTENT_WRITERS
#  define RTPS_MAX_PERSISTENT_WRITERS 4
#endif

class LittleFsPersistenceBackend : public IPersistenceBackend {
public:
    struct WriterMapping {
        EntityId    entityId = {};
        const char *path     = nullptr;
    };

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
