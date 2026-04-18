#include "spec_test_common.hpp"

#include <fstream>

namespace spec_test {

static void on_data_cb(const ParseContext& ctx, const ParsedData& d) noexcept {
    auto* cap = static_cast<TestCapture*>(ctx.user_data);
    cap->data_calls++;
    cap->data = d;
    cap->data_ctx = ctx;
    cap->data_payload.assign(d.serializedPayload, d.serializedPayload + d.serializedPayloadSize);
}

static void on_data_frag_cb(const ParseContext& ctx, const ParsedDataFrag& d) noexcept {
    auto* cap = static_cast<TestCapture*>(ctx.user_data);
    cap->data_frag_calls++;
    cap->data_frag = d;
}

static void on_gap_cb(const ParseContext& ctx, const ParsedGap& g) noexcept {
    auto* cap = static_cast<TestCapture*>(ctx.user_data);
    cap->gap_calls++;
    cap->gap = g;
}

static void on_unknown_cb(const ParseContext& ctx,
                          const SubmessageHeader&,
                          const Octet*,
                          uint16_t raw_len) noexcept {
    auto* cap = static_cast<TestCapture*>(ctx.user_data);
    cap->unknown_calls++;
    cap->unknown_raw_len = raw_len;
}

EntityId make_entity(Octet k0, Octet k1, Octet k2, Octet kind) {
    EntityId e{};
    e.entityKey[0] = k0;
    e.entityKey[1] = k1;
    e.entityKey[2] = k2;
    e.entityKind = kind;
    return e;
}

void write_u16(std::vector<Octet>& out, uint16_t v, bool little) {
    if (little) {
        out.push_back(static_cast<Octet>(v & 0xFFu));
        out.push_back(static_cast<Octet>((v >> 8u) & 0xFFu));
    } else {
        out.push_back(static_cast<Octet>((v >> 8u) & 0xFFu));
        out.push_back(static_cast<Octet>(v & 0xFFu));
    }
}

void write_u32(std::vector<Octet>& out, uint32_t v, bool little) {
    if (little) {
        out.push_back(static_cast<Octet>(v & 0xFFu));
        out.push_back(static_cast<Octet>((v >> 8u) & 0xFFu));
        out.push_back(static_cast<Octet>((v >> 16u) & 0xFFu));
        out.push_back(static_cast<Octet>((v >> 24u) & 0xFFu));
    } else {
        out.push_back(static_cast<Octet>((v >> 24u) & 0xFFu));
        out.push_back(static_cast<Octet>((v >> 16u) & 0xFFu));
        out.push_back(static_cast<Octet>((v >> 8u) & 0xFFu));
        out.push_back(static_cast<Octet>(v & 0xFFu));
    }
}

void write_i32(std::vector<Octet>& out, int32_t v, bool little) {
    write_u32(out, static_cast<uint32_t>(v), little);
}

void append_header(std::vector<Octet>& out) {
    RTPSMessageHeader hdr{};
    hdr.guidPrefix.value[0] = 0xA1u;
    hdr.guidPrefix.value[1] = 0xB2u;
    out.insert(out.end(),
               reinterpret_cast<const Octet*>(&hdr),
               reinterpret_cast<const Octet*>(&hdr) + sizeof(hdr));
}

void append_submessage(std::vector<Octet>& out,
                       SubmessageKind kind,
                       Octet flags,
                       const std::vector<Octet>& payload,
                       bool wire_little_endian,
                       uint16_t override_len) {
    const uint16_t len = (override_len == 0xFFFFu)
                       ? static_cast<uint16_t>(payload.size())
                       : override_len;
    out.push_back(static_cast<Octet>(kind));
    out.push_back(flags);
    write_u16(out, len, wire_little_endian);
    out.insert(out.end(), payload.begin(), payload.end());
}

void append_entity_id(std::vector<Octet>& out, const EntityId& e) {
    out.push_back(e.entityKey[0]);
    out.push_back(e.entityKey[1]);
    out.push_back(e.entityKey[2]);
    out.push_back(e.entityKind);
}

std::vector<Octet> make_data_payload(bool little_endian,
                                     const EntityId& reader_id,
                                     const EntityId& writer_id,
                                     const SequenceNumber& writer_sn,
                                     uint16_t octets_to_inline_qos,
                                     const std::vector<Octet>& inline_qos,
                                     const std::vector<Octet>& serialized_payload) {
    std::vector<Octet> p{};
    write_u16(p, 0u, little_endian);  // extraFlags
    write_u16(p, octets_to_inline_qos, little_endian);
    append_entity_id(p, reader_id);
    append_entity_id(p, writer_id);
    write_i32(p, writer_sn.high, little_endian);
    write_u32(p, writer_sn.low, little_endian);
    if (octets_to_inline_qos > 16u) {
        const uint16_t ext = static_cast<uint16_t>(octets_to_inline_qos - 16u);
        for (uint16_t i = 0; i < ext; ++i) p.push_back(0xA5u);
    }
    p.insert(p.end(), inline_qos.begin(), inline_qos.end());
    p.insert(p.end(), serialized_payload.begin(), serialized_payload.end());
    return p;
}

std::vector<Octet> make_data_frag_payload(bool little_endian,
                                          const EntityId& reader_id,
                                          const EntityId& writer_id,
                                          const SequenceNumber& writer_sn,
                                          uint32_t frag_start,
                                          uint16_t frags_in_submsg,
                                          uint16_t frag_size,
                                          uint32_t data_size,
                                          const std::vector<Octet>& frag_payload) {
    std::vector<Octet> p{};
    write_u16(p, 0u, little_endian);   // extraFlags
    write_u16(p, 28u, little_endian);  // octetsToInlineQos (no extension)
    append_entity_id(p, reader_id);
    append_entity_id(p, writer_id);
    write_i32(p, writer_sn.high, little_endian);
    write_u32(p, writer_sn.low, little_endian);
    write_u32(p, frag_start, little_endian);
    write_u16(p, frags_in_submsg, little_endian);
    write_u16(p, frag_size, little_endian);
    write_u32(p, data_size, little_endian);
    p.insert(p.end(), frag_payload.begin(), frag_payload.end());
    return p;
}

std::vector<Octet> make_related_sample_identity_qos(const SampleIdentity& sid, bool little_endian) {
    std::vector<Octet> q{};
    write_u16(q, inline_qos_pid::RELATED_SAMPLE_IDENTITY, little_endian);
    write_u16(q, sample_identity_wire_size(), little_endian);
    q.insert(q.end(), sid.writerGuid.prefix.value, sid.writerGuid.prefix.value + 12u);
    q.push_back(sid.writerGuid.entityId.entityKey[0]);
    q.push_back(sid.writerGuid.entityId.entityKey[1]);
    q.push_back(sid.writerGuid.entityId.entityKey[2]);
    q.push_back(sid.writerGuid.entityId.entityKind);
    write_i32(q, sid.sequenceNumber.high, little_endian);
    write_u32(q, sid.sequenceNumber.low, little_endian);
    write_u16(q, inline_qos_pid::SENTINEL, little_endian);
    write_u16(q, 0u, little_endian);
    return q;
}

std::vector<Octet> make_sentinel_only_qos(bool little_endian) {
    std::vector<Octet> q{};
    write_u16(q, inline_qos_pid::SENTINEL, little_endian);
    write_u16(q, 0u, little_endian);
    return q;
}

ParseOps make_ops_all(TestCapture& cap) {
    ParseOps ops{};
    ops.on_data = on_data_cb;
    ops.on_data_frag = on_data_frag_cb;
    ops.on_gap = on_gap_cb;
    ops.on_unknown = on_unknown_cb;
    (void)cap;
    return ops;
}

bool write_passed_json(const std::string& path, const std::vector<std::string>& passed_ids) {
    std::ofstream out(path);
    if (!out) return false;
    out << "{\n  \"passed_test_ids\": [";
    for (size_t i = 0; i < passed_ids.size(); ++i) {
        out << (i == 0 ? "" : ", ") << "\"" << passed_ids[i] << "\"";
    }
    out << "]\n}\n";
    return true;
}

} // namespace spec_test

