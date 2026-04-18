#include "rtps/endpoint_match_table.hpp"

#include <stddef.h>
#include <stdint.h>

using namespace rtps;

namespace {

struct Obs {
    int match_calls = 0;
    int unmatch_calls = 0;
};

static void on_match(const DiscoveredWriterData&,
                     const DiscoveredReaderData&,
                     void* user) noexcept {
    auto* o = static_cast<Obs*>(user);
    o->match_calls++;
}

static void on_unmatch(const Guid&, const Guid&, void* user) noexcept {
    auto* o = static_cast<Obs*>(user);
    o->unmatch_calls++;
}

static const char* kTopics[] = {
    "scan", "odom", "tf", "clock", "imu", "pose", "twist", "chatter"
};
static constexpr uint8_t kTopicCount = static_cast<uint8_t>(sizeof(kTopics) / sizeof(kTopics[0]));

static Guid make_guid(uint8_t a, uint8_t b, uint8_t c, bool writer) {
    Guid g{};
    g.prefix.value[0] = a;
    g.prefix.value[1] = b;
    g.prefix.value[2] = c;
    g.entityId.entityKey[0] = a;
    g.entityId.entityKey[1] = b;
    g.entityId.entityKey[2] = c;
    g.entityId.entityKind = writer ? 0xC2u : 0xC7u;
    return g;
}

void fuzz_match_table(const uint8_t* data, size_t size) {
    if (!data || size == 0u) return;

    Obs obs{};
    EndpointMatchTable t(on_match, on_unmatch, &obs);

    size_t off = 0u;
    while (off < size) {
        const uint8_t op = data[off++] % 6u;
        const uint8_t a = (off < size) ? data[off++] : 0u;
        const uint8_t b = (off < size) ? data[off++] : 0u;
        const uint8_t c = (off < size) ? data[off++] : 0u;
        const uint8_t topic_idx = (off < size) ? (data[off++] % kTopicCount) : 0u;
        const uint8_t type_idx = (off < size) ? (data[off++] % kTopicCount) : 0u;
        const uint8_t rel = (off < size) ? (data[off++] % 2u) : 0u;
        const uint8_t dur = (off < size) ? (data[off++] % 2u) : 0u;

        if (op == 0u || op == 1u) {
            DiscoveredWriterData wd{};
            wd.writerGuid = make_guid(a, b, c, true);
            wd.topicName.assign(kTopics[topic_idx]);
            wd.typeName.assign(kTopics[type_idx]);
            wd.reliabilityKind = (rel == 0u) ? ReliabilityKind::BEST_EFFORT : ReliabilityKind::RELIABLE;
            wd.durabilityKind = (dur == 0u) ? DurabilityKind::VOLATILE : DurabilityKind::TRANSIENT_LOCAL;
            t.add_writer(wd);
        } else if (op == 2u || op == 3u) {
            DiscoveredReaderData rd{};
            rd.readerGuid = make_guid(a, b, c, false);
            rd.topicName.assign(kTopics[topic_idx]);
            rd.typeName.assign(kTopics[type_idx]);
            rd.reliabilityKind = (rel == 0u) ? ReliabilityKind::BEST_EFFORT : ReliabilityKind::RELIABLE;
            rd.durabilityKind = (dur == 0u) ? DurabilityKind::VOLATILE : DurabilityKind::TRANSIENT_LOCAL;
            t.add_reader(rd);
        } else if (op == 4u) {
            t.remove_participant(make_guid(a, b, c, true).prefix);
        } else {
            t.remove_writer(make_guid(a, b, c, true));
            t.remove_reader(make_guid(a, b, c, false));
        }
    }
}

} // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    fuzz_match_table(data, size);
    return 0;
}
