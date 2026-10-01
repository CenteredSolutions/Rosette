#include "rosette/rtps/sedp.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace rtps;

namespace {

struct MatchObs {
    int match_calls = 0;
    int unmatch_calls = 0;
};

static void on_match(const DiscoveredWriterData&,
                     const DiscoveredReaderData&,
                     void* user) noexcept {
    auto* o = static_cast<MatchObs*>(user);
    o->match_calls++;
}

static void on_unmatch(const Guid&, const Guid&, void* user) noexcept {
    auto* o = static_cast<MatchObs*>(user);
    o->unmatch_calls++;
}

static Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

static void seed_local_endpoints(EndpointMatchTable& table) {
    DiscoveredWriterData w{};
    w.writerGuid = make_guid(0x11u, 0x22u, 0x33u, entity_id::PARTICIPANT);
    w.topicName.assign("scan");
    w.typeName.assign("sensor_msgs::msg::LaserScan");
    w.reliabilityKind = ReliabilityKind::RELIABLE;
    w.durabilityKind = DurabilityKind::VOLATILE;
    table.add_writer(w);

    DiscoveredReaderData r{};
    r.readerGuid = make_guid(0x44u, 0x55u, 0x66u, entity_id::PARTICIPANT);
    r.topicName.assign("scan");
    r.typeName.assign("sensor_msgs::msg::LaserScan");
    r.reliabilityKind = ReliabilityKind::RELIABLE;
    r.durabilityKind = DurabilityKind::VOLATILE;
    table.add_reader(r);
}

void replay_one_input(const uint8_t* data, size_t size) {
    if (!data || size < 4u) return;

    MatchObs obs{};
    EndpointMatchTable table(on_match, on_unmatch, &obs);
    seed_local_endpoints(table);
    SedpReader sedp_reader(table);

    size_t off = 0u;
    GuidPrefix sender{};
    for (size_t i = 0; i < 12u && off < size; ++i) {
        sender.value[i] = static_cast<Octet>(data[off++]);
    }

    while (off < size) {
        const uint8_t op = data[off++] & 0x3u;
        const uint8_t len_hint = (off < size) ? data[off++] : 0u;
        size_t chunk_len = static_cast<size_t>(len_hint);
        if (chunk_len > (size - off)) chunk_len = size - off;

        const Octet* payload = reinterpret_cast<const Octet*>(data + off);
        const uint32_t payload_len = static_cast<uint32_t>(chunk_len);

        if (op == 0u) {
            sedp_reader.on_publication_data(sender, payload, payload_len);
        } else if (op == 1u) {
            sedp_reader.on_subscription_data(sender, payload, payload_len);
        } else if (op == 2u) {
            table.remove_participant(sender);
        } else {
            sender.value[0] ^= static_cast<Octet>(len_hint);
        }

        off += chunk_len;
    }
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

    std::cout << "Replayed " << files << " SEDP corpus files from " << corpus_dir << "\n";
    return 0;
}
