#include "rosette/rtps/spdp.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

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

void replay_one_input(const uint8_t* data, size_t size) {
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
    if ((size > 0u) && ((data[0] & 0x1u) != 0u)) {
        if (PeerEntry* known = reader.find_peer(sender)) {
            (void)reader.activate(*known);
        }

        (void)reader.on_data(sender, payload, payload_len, now_ms + 1u);
    }
    reader.tick(now_ms + tick_delta);
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " <corpus-dir>\n";
        return 2;
    }

    const std::filesystem::path corpus_dir = argv[1];
    if (!std::filesystem::exists(corpus_dir) || !std::filesystem::is_directory(corpus_dir)) {
        std::cerr << "corpus directory not found: " << corpus_dir << "\n";
        return 2;
    }

    size_t files = 0;
    for (const auto& entry : std::filesystem::directory_iterator(corpus_dir)) {
        if (!entry.is_regular_file()) continue;
        std::ifstream in(entry.path(), std::ios::binary);
        if (!in) continue;

        std::vector<uint8_t> data((std::istreambuf_iterator<char>(in)),
                                  std::istreambuf_iterator<char>());
        replay_one_input(data.data(), data.size());
        ++files;
    }

    std::cout << "Replayed " << files << " SPDP corpus files from " << corpus_dir << "\n";
    return 0;
}
