#pragma once

#include "rtps/message_parser.hpp"

#include <string>
#include <vector>

namespace spec_test {

using namespace rtps;

struct TestCapture {
    int data_calls = 0;
    int data_frag_calls = 0;
    int gap_calls = 0;
    int unknown_calls = 0;
    ParsedData data{};
    ParsedDataFrag data_frag{};
    ParsedGap gap{};
    ParseContext data_ctx{};
    std::vector<Octet> data_payload{};
    uint16_t unknown_raw_len = 0;
};

struct TestCase {
    const char* id;
    const char* doc;
    bool (*fn)(std::string&);
};

EntityId make_entity(Octet k0, Octet k1, Octet k2, Octet kind);

void write_u16(std::vector<Octet>& out, uint16_t v, bool little);
void write_u32(std::vector<Octet>& out, uint32_t v, bool little);
void write_i32(std::vector<Octet>& out, int32_t v, bool little);
void append_header(std::vector<Octet>& out);
void append_submessage(std::vector<Octet>& out,
                       SubmessageKind kind,
                       Octet flags,
                       const std::vector<Octet>& payload,
                       bool wire_little_endian = true,
                       uint16_t override_len = 0xFFFFu);
void append_entity_id(std::vector<Octet>& out, const EntityId& e);

std::vector<Octet> make_data_payload(bool little_endian,
                                     const EntityId& reader_id,
                                     const EntityId& writer_id,
                                     const SequenceNumber& writer_sn,
                                     uint16_t octets_to_inline_qos,
                                     const std::vector<Octet>& inline_qos,
                                     const std::vector<Octet>& serialized_payload);

std::vector<Octet> make_data_frag_payload(bool little_endian,
                                          const EntityId& reader_id,
                                          const EntityId& writer_id,
                                          const SequenceNumber& writer_sn,
                                          uint32_t frag_start,
                                          uint16_t frags_in_submsg,
                                          uint16_t frag_size,
                                          uint32_t data_size,
                                          const std::vector<Octet>& frag_payload);

std::vector<Octet> make_related_sample_identity_qos(const SampleIdentity& sid,
                                                    bool little_endian);
std::vector<Octet> make_sentinel_only_qos(bool little_endian);

ParseOps make_ops_all(TestCapture& cap);

bool write_passed_json(const std::string& path, const std::vector<std::string>& passed_ids);

} // namespace spec_test
