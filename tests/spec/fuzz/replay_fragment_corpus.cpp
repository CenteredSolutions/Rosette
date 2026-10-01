#include "rosette/rtps/fragment_assembler.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace rtps;

namespace {

uint16_t rd_u16(const uint8_t* data, size_t size, size_t& off) {
    if (off + 2u > size) return 0u;
    const uint16_t v = static_cast<uint16_t>(
        static_cast<uint16_t>(data[off]) |
        (static_cast<uint16_t>(data[off + 1u]) << 8u));
    off += 2u;
    return v;
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
    if (!data || size < 24u) return;

    FragmentAssembler<4> assembler{};
    CacheChange out{};

    Guid writer{};
    size_t off = 0u;
    for (size_t i = 0; i < 12u && off < size; ++i) {
        writer.prefix.value[i] = static_cast<Octet>(data[off++]);
    }
    writer.entityId.entityKey[0] = static_cast<Octet>(off < size ? data[off++] : 0u);
    writer.entityId.entityKey[1] = static_cast<Octet>(off < size ? data[off++] : 0u);
    writer.entityId.entityKey[2] = static_cast<Octet>(off < size ? data[off++] : 0u);
    writer.entityId.entityKind   = static_cast<Octet>(off < size ? data[off++] : 0xC2u);

    const uint32_t sn_hi_u = rd_u32(data, size, off);
    const uint32_t sn_lo_u = rd_u32(data, size, off);
    const SequenceNumber sn{
        static_cast<int32_t>(sn_hi_u),
        static_cast<uint32_t>(sn_lo_u)
    };

    uint32_t frag_start = rd_u32(data, size, off);
    uint16_t frags_in_submsg = rd_u16(data, size, off);
    uint16_t frag_size = rd_u16(data, size, off);
    uint32_t data_size = rd_u32(data, size, off);
    const uint32_t last_known = rd_u32(data, size, off);

    if (frag_start > 600u) frag_start %= 600u;
    frags_in_submsg = static_cast<uint16_t>((frags_in_submsg % 16u) + 1u);
    frag_size = static_cast<uint16_t>(frag_size % 512u);
    data_size %= 8192u;

    const Octet* payload = reinterpret_cast<const Octet*>(data + off);
    const uint32_t payload_len = static_cast<uint32_t>((off < size) ? (size - off) : 0u);
    const Timestamp ts{
        static_cast<int32_t>(sn_hi_u),
        sn_lo_u
    };

    const bool complete = assembler.on_data_frag(
        writer,
        sn,
        frag_start,
        frags_in_submsg,
        frag_size,
        data_size,
        payload,
        payload_len,
        ts,
        out);

    FragmentNumberSet missing{};
    const uint32_t bounded_last_known = (last_known > 1024u) ? (last_known % 1025u) : last_known;
    (void)assembler.missing_fragments(writer, sn, bounded_last_known, missing);

    if (complete) out.clear();
    assembler.discard(writer, sn);
    assembler.discard_writer(writer);
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

    std::cout << "Replayed " << files << " fragment corpus files from " << corpus_dir << "\n";
    return 0;
}
