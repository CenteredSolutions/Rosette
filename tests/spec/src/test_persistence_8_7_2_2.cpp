// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_persistence_8_7_2_2.cpp
 * @brief LittleFsPersistenceBackend on a RAM block device: TC-PER-8.7.2.2-001 to -004.
 *
 * A TRANSIENT_LOCAL writer keeps its last change across a reboot (§8.7.2.2).
 * Each case formats a 16 KB LittleFS volume in RAM and unmounts and remounts
 * it between store() and load(), so what load() returns came from the volume,
 * not from LittleFS's caches.
 *
 *   ./persistence_spec_tests
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/persistence.hpp"

#include <string.h>

#include <string>

namespace rtps = rosette::rtps;

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

constexpr lfs_size_t kBlockSize = 256u;
constexpr lfs_size_t kBlockCount = 64u;

/// A LittleFS volume in RAM, formatted and mounted on construction.
struct RamVolume {
    uint8_t    mem[kBlockSize * kBlockCount] = {};
    lfs_config cfg{};
    lfs_t      lfs{};
    bool       mounted = false;

    RamVolume() {
        cfg.context = this;
        cfg.read = &read;
        cfg.prog = &prog;
        cfg.erase = &erase;
        cfg.sync = &sync;
        cfg.read_size = 16u;
        cfg.prog_size = 16u;
        cfg.block_size = kBlockSize;
        cfg.block_count = kBlockCount;
        cfg.block_cycles = 500;
        cfg.cache_size = 64u;
        cfg.lookahead_size = 16u;
        memset(mem, 0xFF, sizeof(mem));
        mounted = lfs_format(&lfs, &cfg) == 0 && lfs_mount(&lfs, &cfg) == 0;
    }

    ~RamVolume() {
        if (mounted) {
            (void)lfs_unmount(&lfs);
        }
    }

    /// @brief Unmounts and mounts again, as a reboot does.
    bool remount() {
        mounted = lfs_unmount(&lfs) == 0 && lfs_mount(&lfs, &cfg) == 0;
        return mounted;
    }

    static uint8_t* at(const lfs_config* c, lfs_block_t block, lfs_off_t off) {
        return static_cast<RamVolume*>(c->context)->mem + block * kBlockSize + off;
    }

    static int read(const lfs_config* c, lfs_block_t b, lfs_off_t off, void* buf, lfs_size_t n) {
        memcpy(buf, at(c, b, off), n);
        return 0;
    }

    static int prog(const lfs_config* c, lfs_block_t b, lfs_off_t off, const void* buf,
                    lfs_size_t n) {
        memcpy(at(c, b, off), buf, n);
        return 0;
    }

    static int erase(const lfs_config* c, lfs_block_t b) {
        memset(at(c, b, 0u), 0xFF, kBlockSize);
        return 0;
    }

    static int sync(const lfs_config*) { return 0; }
};

const rtps::EntityId kMapWriter = spec_test::make_entity(0x00u, 0x01u, 0x00u, 0x02u);
const rtps::EntityId kParamWriter = spec_test::make_entity(0x00u, 0x02u, 0x00u, 0x02u);
const rtps::LittleFsPersistenceBackend::WriterMapping kFiles[] = {
    {kMapWriter, "/rtps/map.bin"},
    {kParamWriter, "/rtps/param.bin"},
};

const rtps::Octet kBody[5] = {0x11u, 0x22u, 0x33u, 0x44u, 0x55u};

/// @brief A change as write_async() stores it: ALIVE, CDR body, not preframed.
rtps::HistoryChange user_change(int64_t sn) {
    rtps::HistoryChange ch{};
    ch.sequenceNumber = rtps::SequenceNumber::from_int64(sn);
    ch.payload = rtps::buf_alloc(sizeof(kBody));
    if (ch.payload.valid()) {
        memcpy(ch.payload.data, kBody, sizeof(kBody));
        ch.payload.size = sizeof(kBody);
    }

    return ch;
}

bool holds_body(const rtps::HistoryChange& ch) {
    return ch.payload.valid() && ch.payload.size == sizeof(kBody)
           && memcmp(ch.payload.data, kBody, sizeof(kBody)) == 0;
}

}  // namespace

// §8.7.2.2: a user-data change stored before a reboot loads after it with
// its sequence number and body, as an ALIVE change that is not preframed.
static bool tc_per_8_7_2_2_001(std::string& err) {
    RamVolume vol;
    CHECK(vol.mounted, "mount");
    rtps::HistoryChange stored = user_change(7);
    {
        rtps::LittleFsPersistenceBackend backend(&vol.lfs, kFiles, 2u);
        CHECK(backend.store(kMapWriter, stored), "store");
    }

    stored.clear();
    CHECK(vol.remount(), "remount");

    rtps::LittleFsPersistenceBackend backend(&vol.lfs, kFiles, 2u);
    rtps::HistoryChange              loaded{};
    CHECK(backend.load(kMapWriter, loaded), "load");
    CHECK(loaded.kind == rtps::ChangeKind::ALIVE, "kind ALIVE");
    CHECK(loaded.sequenceNumber == rtps::SequenceNumber::from_int64(7), "sequence number");
    CHECK(!loaded.preframed, "not preframed");
    CHECK(holds_body(loaded), "body octets");
    loaded.clear();
    return true;
}

// A preframed change (an SEDP announcement, valid for one boot) and a
// disposal (which load() would return as ALIVE) are refused, and the change
// stored before them stays the one that loads.
static bool tc_per_8_7_2_2_002(std::string& err) {
    RamVolume vol;
    CHECK(vol.mounted, "mount");
    rtps::LittleFsPersistenceBackend backend(&vol.lfs, kFiles, 2u);
    rtps::HistoryChange              first = user_change(3);
    CHECK(backend.store(kMapWriter, first), "store the user-data change");

    rtps::HistoryChange preframed = user_change(4);
    preframed.preframed = true;
    CHECK(!backend.store(kMapWriter, preframed), "a preframed change is refused");

    rtps::HistoryChange disposed = user_change(5);
    disposed.kind = rtps::ChangeKind::NOT_ALIVE_DISPOSED;
    CHECK(!backend.store(kMapWriter, disposed), "a disposal is refused");

    CHECK(vol.remount(), "remount");
    rtps::HistoryChange loaded{};
    CHECK(backend.load(kMapWriter, loaded), "load");
    CHECK(loaded.sequenceNumber == rtps::SequenceNumber::from_int64(3) && holds_body(loaded),
          "the change stored before the refusals loads");
    first.clear();
    preframed.clear();
    disposed.clear();
    loaded.clear();
    return true;
}

// A file whose payload changed after it was stored fails its CRC-32 and
// loads nothing.
static bool tc_per_8_7_2_2_003(std::string& err) {
    RamVolume vol;
    CHECK(vol.mounted, "mount");
    rtps::LittleFsPersistenceBackend backend(&vol.lfs, kFiles, 2u);
    rtps::HistoryChange              stored = user_change(9);
    CHECK(backend.store(kMapWriter, stored), "store");
    stored.clear();

    lfs_file_t file{};
    CHECK(lfs_file_open(&vol.lfs, &file, "/rtps/map.bin", LFS_O_RDWR) == 0, "open");
    const uint8_t flipped = 0xEEu;
    const auto    at = static_cast<lfs_soff_t>(sizeof(rtps::LittleFsPersistenceHeader) + 2u);
    const bool    written = lfs_file_seek(&vol.lfs, &file, at, LFS_SEEK_SET) == at
                         && lfs_file_write(&vol.lfs, &file, &flipped, 1u) == 1;
    CHECK(lfs_file_close(&vol.lfs, &file) == 0 && written, "corrupt one payload octet");

    rtps::HistoryChange loaded{};
    CHECK(!backend.load(kMapWriter, loaded), "a corrupted file loads nothing");
    CHECK(!loaded.payload.valid(), "out is untouched");
    return true;
}

// A file stored for one writer does not load for another mapped to it, and
// a writer with no mapping or no file loads nothing.
static bool tc_per_8_7_2_2_004(std::string& err) {
    RamVolume vol;
    CHECK(vol.mounted, "mount");
    rtps::LittleFsPersistenceBackend backend(&vol.lfs, kFiles, 2u);
    rtps::HistoryChange              stored = user_change(2);
    CHECK(backend.store(kMapWriter, stored), "store");
    stored.clear();

    rtps::HistoryChange loaded{};
    CHECK(!backend.load(kParamWriter, loaded), "a writer with no file loads nothing");

    const rtps::LittleFsPersistenceBackend::WriterMapping swapped[] = {
        {kParamWriter, "/rtps/map.bin"},
    };
    rtps::LittleFsPersistenceBackend crossed(&vol.lfs, swapped, 1u);
    CHECK(!crossed.load(kParamWriter, loaded), "another writer's file loads nothing");
    CHECK(!crossed.load(kMapWriter, loaded), "an unmapped writer loads nothing");
    CHECK(!loaded.payload.valid(), "out is untouched");
    return true;
}

int main(int argc, char** argv) {
    const spec_test::TestCase tests[] = {
        {"TC-PER-8.7.2.2-001",
         "§8.7.2.2: a stored user-data change loads after a remount with its sequence number and "
         "body, ALIVE and not preframed.",
         tc_per_8_7_2_2_001},
        {"TC-PER-8.7.2.2-002",
         "store() refuses preframed changes and disposals, keeping the change stored before them.",
         tc_per_8_7_2_2_002},
        {"TC-PER-8.7.2.2-003",
         "A file whose payload changed after store() fails its CRC-32 and loads nothing.",
         tc_per_8_7_2_2_003},
        {"TC-PER-8.7.2.2-004",
         "Another writer's file, a writer with no file, and an unmapped writer load nothing.",
         tc_per_8_7_2_2_004},
    };

    return spec_test::run_standalone_tests(tests, sizeof(tests) / sizeof(tests[0]), argc, argv);
}
