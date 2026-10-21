/**
 * @file fuzz_dns_wire.cpp
 * @brief Fuzz harness for the DNS/mDNS reader that agent discovery uses.
 *
 * mDNS is a shared bus, so every host on the link can feed the reader. The
 * harness runs a datagram through each consumer (response matching, first A
 * record, first SRV record, and a PTR read of every record) and requires only
 * that none reads outside the datagram or leaves a name unterminated.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <array>

#include "rosette/net/mdns.hpp"

using namespace rosette::net;

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    if (data == nullptr) {
        return 0;
    }

    mdns_detail::response_answers(data, size, "_microrosagent._udp.local");

    std::array<uint8_t, 4> address{};
    mdns_detail::find_first_a(data, size, address);

    dns::Srv srv{};
    if (mdns_detail::find_first_srv(data, size, srv)
        && memchr(srv.target, '\0', sizeof(srv.target)) == nullptr) {
        __builtin_trap();
    }

    dns::Reader reader(data, size);
    dns::Header header{};
    if (!reader.read_header(header) || !reader.skip_questions(header.question_count)) {
        return 0;
    }

    for (uint16_t i = 0u; i < header.record_count(); ++i) {
        dns::Record record{};
        if (!reader.read_record(record)) {
            break;
        }

        char name[dns::kMaxNameLength];
        reader.read_ptr(record, name, sizeof(name));
    }

    return 0;
}
