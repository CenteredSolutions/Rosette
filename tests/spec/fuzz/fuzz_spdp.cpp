#include "rtps/spdp.hpp"

#include <stddef.h>
#include <stdint.h>

using namespace rtps;

namespace {

struct Obs {
    int new_calls = 0;
    int removed_calls = 0;
};

static void on_new_peer(const PeerEntry&, void* user) noexcept {
    auto* o = static_cast<Obs*>(user);
    o->new_calls++;
}

static void on_removed(const PeerEntry&, void* user) noexcept {
    auto* o = static_cast<Obs*>(user);
    o->removed_calls++;
}

uint32_t rd_u32(const uint8_t* data, size_t size, size_t& off) {
    if (off + 4u > size) return 0u;
    const uint32_t v = static_cast<uint32_t>(
        static_cast<uint32_t>(data[off]) |
        (static_cast<uint32_t>(data[off + 1u]) << 8u) |
        (static_cast<uint32_t>(data[off + 2u]) << 16u) |
        (static_cast<uint32_t>(data[off + 3u]) << 24u));
    off += 4u;
    return v;
}

void fuzz_spdp_one_input(const uint8_t* data, size_t size) {
    if (!data || size < 8u) return;

    Obs obs{};
    SpdpReader reader(on_new_peer, on_removed, &obs);

    size_t off = 0u;
    GuidPrefix sender{};
    for (size_t i = 0; i < 12u && off < size; ++i) {
        sender.value[i] = static_cast<Octet>(data[off++]);
    }

    uint32_t now_ms = rd_u32(data, size, off);
    if (now_ms > 120000u) now_ms %= 120001u;

    uint32_t tick_delta = rd_u32(data, size, off);
    if (tick_delta > 120000u) tick_delta %= 120001u;

    const Octet* payload = reinterpret_cast<const Octet*>(data + off);
    const uint32_t payload_len = static_cast<uint32_t>((off < size) ? (size - off) : 0u);

    (void)reader.on_data(sender, payload, payload_len, now_ms);

    // Exercise lease/rediscovery state transitions in one input.
    if ((size > 0u) && ((data[0] & 0x1u) != 0u)) {
        reader.mark_sedp_announced(sender);
        (void)reader.on_data(sender, payload, payload_len, now_ms + 1u);
    }

    reader.tick(now_ms + tick_delta);

    // Walk discovered peers to force traversal code paths.
    reader.for_each_peer([](const PeerEntry& peer) noexcept {
        (void)peer.data.availableBuiltinEndpoints;
    });
}

} // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    fuzz_spdp_one_input(data, size);
    return 0;
}
